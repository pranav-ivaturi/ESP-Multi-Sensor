/*
 * ============================================================
 *  ESP32 "SLAVE 2" - ADS1113 16-BIT ADC + VL53L4CD LIDAR, SENT OVER CAN
 * ============================================================
 *
 *  Board : ESP32-WROOM-32 DevKit
 *  SDK   : ESP-IDF v5.5+ / v6.x
 *
 *  main/CMakeLists.txt must contain:
 *
 *      idf_component_register(SRCS "main.c"
 *                             INCLUDE_DIRS "."
 *                             PRIV_REQUIRES esp_driver_i2c
 *                                           esp_driver_gpio
 *                                           esp_driver_twai)
 *
 *  SENSOR POWER
 *    The ADS1113 and VL53L4CD are powered by the external regulated 3.3 V
 *    sensor supply shown in the system diagram, not by an ESP32 GPIO or its
 *    3V3 pin.  The external supply ground and ESP32 ground must be common.
 *
 *  WIRING
 *    ADS1113:  VDD -> external 3.3 V, GND -> common GND, SDA -> GPIO 21, SCL -> GPIO 22
 *              ADDR -> GND (address 0x48)
 *              AIN1 -> GND (the ADS1113 measures AIN0 - AIN1)
 *              AIN0 -> divider output
 *
 *    VL53L4CD: VDD -> external 3.3 V, GND -> common GND,
 *              SDA -> GPIO 21, SCL -> GPIO 22, address 0x29
 *              shared with the ADS1113 on I2C controller 0
 *
 *    Input divider:
 *      INPUT ---- 10k ----+---- ADS1113 AIN0
 *                         |
 *                        20k
 *                         |
 *                        GND
 *
 *    CAN (needs an external 3.3 V CAN transceiver):
 *      GPIO 5 -> transceiver TXD
 *      GPIO 4 -> transceiver RXD
 *      CANH / CANL -> bus
 *      120 ohm termination at each END of the bus
 *
 * ------------------------------------------------------------
 *  CAN FRAMES 0x200 / 0x201
 * ------------------------------------------------------------
 *
 *  Standard 11-bit CAN ID
 *  8-byte payload
 *  BIG-ENDIAN
 *
 *      bytes 0-1  raw ADC value (int16)
 *      bytes 2-3  ADC pin voltage in mV (uint16)
 *      bytes 4-5  original input voltage in mV (uint16)
 *      bytes 6-7  packet counter (uint16)
 *
 *  0x201 (LiDAR):
 *      bytes 0-1  signed distance in mm; -1 when unavailable
 *      bytes 2-3  packet counter (uint16)
 *      byte 4     status; bit 0 = valid distance
 *      bytes 5-7  reserved
 *
 *  IMPORTANT:
 *    All ESP Multi Sensor slaves now use BIG-ENDIAN encoding.
 *
 * ============================================================
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_err.h"

#include "driver/gpio.h"
#include "driver/i2c_master.h"

#include "esp_twai.h"
#include "esp_twai_onchip.h"

#include "vl53l4cd.h"


/* ============================================================
 * TAG
 * ============================================================ */

static const char *TAG = "SLAVE_2";


/* ============================================================
 * I2C CONFIGURATION
 * ============================================================ */

#define I2C_PORT                I2C_NUM_0

#define I2C_SDA_GPIO            GPIO_NUM_21
#define I2C_SCL_GPIO            GPIO_NUM_22

#define I2C_FREQUENCY           400000
#define I2C_TIMEOUT_MS          100

#define ADS1113_ADDRESS         0x48

/* The ADS1113 and VL53L4CD have different I2C addresses and can share the
 * existing 400 kHz bus without changing Slave 2's physical I2C wiring. */
#define VL53L4CD_ADDRESS        0x29


/* ============================================================
 * ADS1113 REGISTERS
 * ============================================================ */

#define ADS1113_REG_CONVERSION  0x00
#define ADS1113_REG_CONFIG      0x01


/* ============================================================
 * ADS1113 CONFIGURATION
 *
 * Differential input AIN0 - AIN1
 * Full scale = +/-2.048 V
 * Single-shot conversion
 * 128 SPS
 * Comparator disabled
 * ============================================================ */

#define ADS1113_CONFIG          0x8583


/* ============================================================
 * ADS1113 SCALING
 *
 * Full scale = +/-2.048 V, 16 bits
 *
 * 1 LSB = 2.048 / 32768
 *       = 0.0000625 V
 *       = 62.5 uV
 *
 * External voltage divider:
 *
 * V_ADC = V_INPUT * 20 / 30
 *       = V_INPUT / 1.5
 *
 * Therefore:
 *
 * V_INPUT = V_ADC * 1.5
 *
 * Maximum measurable input = 2.048 V * 1.5 = 3.072 V
 * ============================================================ */

#define ADS1113_LSB_V           0.0000625f
#define DIVIDER_RATIO           1.5f


/* ============================================================
 * CAN / TWAI CONFIGURATION
 * ============================================================ */

#define CAN_TX_GPIO             GPIO_NUM_5
#define CAN_RX_GPIO             GPIO_NUM_4

#define CAN_BITRATE             500000

#define CAN_ID_ADS1113          0x200
#define CAN_ID_LIDAR            0x201
#define CAN_ID_RELAY_STATUS     0x202
#define CAN_ID_RELAY_COMMAND    0x610

/*
 * GPIO25 drives an external, active-high relay/contactor driver only. It must
 * never be connected directly to a coil. GPIO26 is an isolated auxiliary
 * contact feedback input; the relay is fail-safe OFF if it does not confirm.
 */
#define RELAY_DRIVER_GPIO        GPIO_NUM_25
#define RELAY_FEEDBACK_GPIO      GPIO_NUM_26
#define RELAY_ACTIVE_LEVEL       1
#define RELAY_COMMAND_TIMEOUT_MS 750
#define RELAY_FEEDBACK_SETTLE_MS 500
#define RELAY_STATUS_PERIOD_MS   100
#define RELAY_COMMAND_MAGIC      0xA5
#define RELAY_COMMAND_TRAILER    0x5A

#define CAN_TX_QUEUE_DEPTH      5

#define CAN_TX_TIMEOUT_MS       100


/* ============================================================
 * HANDLES
 * ============================================================ */

static i2c_master_bus_handle_t i2c_bus = NULL;
static i2c_master_dev_handle_t ads_dev = NULL;
static i2c_master_dev_handle_t lidar_dev = NULL;

static twai_node_handle_t can_node = NULL;

static vl53l4cd_t lidar;
static bool lidar_ready = false;

typedef struct {
    uint8_t data[8];
} relay_command_t;

static QueueHandle_t relay_command_queue = NULL;
static volatile bool relay_requested = false;
static volatile bool relay_output_on = false;
static volatile uint8_t relay_faults = 0;
static volatile uint16_t relay_sequence = 0;
static volatile TickType_t relay_last_command = 0;
static TickType_t relay_energized_at = 0;

#define RELAY_FAULT_TIMEOUT  0x01
#define RELAY_FAULT_FEEDBACK 0x02
#define RELAY_FAULT_COMMAND  0x04


/* ============================================================
 * I2C INITIALIZATION
 * ============================================================ */

static void i2c_init(void)
{
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_PORT,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    ESP_ERROR_CHECK(
        i2c_new_master_bus(
            &bus_config,
            &i2c_bus
        )
    );


    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = ADS1113_ADDRESS,
        .scl_speed_hz = I2C_FREQUENCY,
    };


    ESP_ERROR_CHECK(
        i2c_master_bus_add_device(
            i2c_bus,
            &dev_config,
            &ads_dev
        )
    );


    ESP_LOGI(
        TAG,
        "I2C initialized: SDA=%d SCL=%d",
        (int)I2C_SDA_GPIO,
        (int)I2C_SCL_GPIO
    );


    /* Report whether the ADS1113 answers,
       but keep running either way. */

    if (
        i2c_master_probe(
            i2c_bus,
            ADS1113_ADDRESS,
            I2C_TIMEOUT_MS
        ) == ESP_OK
    )
    {
        ESP_LOGI(
            TAG,
            "ADS1113 detected at 0x%02X",
            ADS1113_ADDRESS
        );
    }
    else
    {
        ESP_LOGE(
            TAG,
            "ADS1113 NOT detected at 0x%02X - check wiring and ADDR pin",
            ADS1113_ADDRESS
        );
    }
}


/* ============================================================
 * VL53L4CD I2C INTERFACE
 *
 * The LiDAR shares the existing ADS1113 bus (GPIO21/GPIO22).  The distinct
 * addresses, 0x48 and 0x29, make both devices independently addressable.
 * ============================================================ */

static void lidar_i2c_init(void)
{
    i2c_device_config_t device_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = VL53L4CD_ADDRESS,
        .scl_speed_hz = I2C_FREQUENCY,
    };

    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus, &device_config,
                                               &lidar_dev));

    esp_err_t err = vl53l4cd_init(&lidar, lidar_dev);
    if (err == ESP_OK) {
        lidar_ready = true;
        ESP_LOGI(TAG, "VL53L4CD ranging at 0x%02X on the ADS1113 I2C bus",
                 VL53L4CD_ADDRESS);
    } else {
        ESP_LOGE(TAG, "VL53L4CD setup failed: %s", esp_err_to_name(err));
    }
}


/* Returns a new millimetre measurement, or -1 if ranging is unavailable. */
static int lidar_read_distance(void)
{
    int distance_mm = -1;
    if (!lidar_ready)
        return -1;

    esp_err_t err = vl53l4cd_read_distance_mm(&lidar, &distance_mm, 60);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "VL53L4CD ranging failed: %s", esp_err_to_name(err));
        return -1;
    }

    return distance_mm;
}

/* Relay command payload: A5, requested-state, sequence-be16, 'M', '2', XOR, 5A. */
static uint8_t relay_command_checksum(const uint8_t data[8])
{
    uint8_t checksum = 0;
    for (int i = 0; i < 6; ++i)
        checksum ^= data[i];
    return checksum;
}

static void relay_set_output(bool on)
{
    gpio_set_level(RELAY_DRIVER_GPIO,
                   on ? RELAY_ACTIVE_LEVEL : !RELAY_ACTIVE_LEVEL);
    relay_output_on = on;
    if (on)
        relay_energized_at = xTaskGetTickCount();
}

static bool relay_feedback_closed(void)
{
    return gpio_get_level(RELAY_FEEDBACK_GPIO) == RELAY_ACTIVE_LEVEL;
}

static void relay_init(void)
{
    const gpio_config_t driver_cfg = {
        .pin_bit_mask = 1ULL << RELAY_DRIVER_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    const gpio_config_t feedback_cfg = {
        .pin_bit_mask = 1ULL << RELAY_FEEDBACK_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&driver_cfg));
    ESP_ERROR_CHECK(gpio_config(&feedback_cfg));
    relay_set_output(false); /* Safe state before CAN is initialized. */
    relay_command_queue = xQueueCreate(8, sizeof(relay_command_t));
    if (relay_command_queue == NULL)
        abort();
}

static bool relay_can_rx_callback(twai_node_handle_t handle,
                                  const twai_rx_done_event_data_t *edata,
                                  void *ctx)
{
    (void)edata;
    (void)ctx;
    relay_command_t command = {0};
    twai_frame_t frame = {.buffer = command.data, .buffer_len = sizeof(command.data)};
    if (twai_node_receive_from_isr(handle, &frame) != ESP_OK)
        return false;
    if (frame.header.ide || frame.header.id != CAN_ID_RELAY_COMMAND)
        return false;
    BaseType_t woken = pdFALSE;
    (void)xQueueSendFromISR(relay_command_queue, &command, &woken);
    return woken == pdTRUE;
}

static void relay_accept_command(const relay_command_t *command)
{
    const uint8_t *data = command->data;
    uint16_t sequence = ((uint16_t)data[2] << 8) | data[3];
    uint16_t delta = (uint16_t)(sequence - relay_sequence);
    bool valid = data[0] == RELAY_COMMAND_MAGIC && data[1] <= 1 &&
                 data[4] == 'M' && data[5] == '2' &&
                 data[6] == relay_command_checksum(data) &&
                 data[7] == RELAY_COMMAND_TRAILER &&
                 (relay_last_command == 0 || delta == 0 || delta < 0x8000);
    if (!valid) {
        relay_faults |= RELAY_FAULT_COMMAND;
        relay_set_output(false);
        return;
    }

    relay_requested = data[1] == 1;
    relay_sequence = sequence;
    relay_last_command = xTaskGetTickCount();
    if (!relay_requested) {
        relay_faults = 0; /* An explicit OFF is the only software fault reset. */
        relay_set_output(false);
    } else {
        relay_faults &= (uint8_t)~RELAY_FAULT_TIMEOUT;
    }
}

static esp_err_t can_send_relay_status(void)
{
    uint8_t data[8] = {0};
    bool feedback = relay_feedback_closed();
    TickType_t now = xTaskGetTickCount();
    uint32_t age_ms = relay_last_command == 0 ? UINT16_MAX :
        (uint32_t)pdTICKS_TO_MS(now - relay_last_command);
    if (age_ms > UINT16_MAX) age_ms = UINT16_MAX;

    data[0] = (relay_requested ? 0x01 : 0) |
              (relay_output_on ? 0x02 : 0) |
              (feedback ? 0x04 : 0) |
              0x08 |
              ((age_ms <= RELAY_COMMAND_TIMEOUT_MS) ? 0x10 : 0) |
              (relay_faults ? 0x20 : 0);
    data[1] = relay_faults;
    data[2] = (uint8_t)(relay_sequence >> 8);
    data[3] = (uint8_t)relay_sequence;
    data[4] = (uint8_t)(age_ms >> 8);
    data[5] = (uint8_t)age_ms;

    twai_frame_t frame = {.header.id = CAN_ID_RELAY_STATUS, .header.ide = false,
                          .buffer = data, .buffer_len = sizeof(data)};
    return twai_node_transmit(can_node, &frame, CAN_TX_TIMEOUT_MS);
}

static void relay_control_task(void *arg)
{
    (void)arg;
    TickType_t last_status = 0;
    while (1) {
        relay_command_t command;
        while (xQueueReceive(relay_command_queue, &command, 0) == pdTRUE)
            relay_accept_command(&command);

        TickType_t now = xTaskGetTickCount();
        bool fresh = relay_last_command != 0 &&
            (now - relay_last_command) <= pdMS_TO_TICKS(RELAY_COMMAND_TIMEOUT_MS);
        if (!fresh) {
            relay_faults |= RELAY_FAULT_TIMEOUT;
            relay_requested = false;
            relay_set_output(false);
        } else if (relay_requested && relay_faults == 0 && !relay_output_on) {
            relay_set_output(true);
        }

        if (relay_output_on &&
            (now - relay_energized_at) >= pdMS_TO_TICKS(RELAY_FEEDBACK_SETTLE_MS) &&
            !relay_feedback_closed()) {
            relay_faults |= RELAY_FAULT_FEEDBACK;
            relay_set_output(false);
        }

        if ((now - last_status) >= pdMS_TO_TICKS(RELAY_STATUS_PERIOD_MS)) {
            (void)can_send_relay_status();
            last_status = now;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}


/* ============================================================
 * ADS1113 WRITE CONFIG REGISTER
 * ============================================================ */

static esp_err_t ads1113_write_config(
    uint16_t config_value
)
{
    uint8_t data[3];

    data[0] = ADS1113_REG_CONFIG;

    data[1] =
        (uint8_t)(config_value >> 8);

    data[2] =
        (uint8_t)(config_value & 0xFF);


    return i2c_master_transmit(
        ads_dev,
        data,
        sizeof(data),
        I2C_TIMEOUT_MS
    );
}


/* ============================================================
 * ADS1113 READ CONVERSION
 * ============================================================ */

static esp_err_t ads1113_read_conversion(
    int16_t *raw
)
{
    uint8_t reg =
        ADS1113_REG_CONVERSION;

    uint8_t data[2];


    esp_err_t err =
        i2c_master_transmit_receive(
            ads_dev,
            &reg,
            1,
            data,
            sizeof(data),
            I2C_TIMEOUT_MS
        );


    if (err != ESP_OK) {
        return err;
    }


    *raw = (int16_t)(
        ((uint16_t)data[0] << 8) |
        data[1]
    );


    return ESP_OK;
}


/* ============================================================
 * ADS1113 READ VOLTAGE
 * ============================================================ */

static esp_err_t ads1113_read_voltage(
    int16_t *raw,
    float *adc_voltage,
    float *input_voltage
)
{
    esp_err_t err;


    /*
     * Start a single conversion.
     */

    err =
        ads1113_write_config(
            ADS1113_CONFIG
        );


    if (err != ESP_OK) {
        return err;
    }


    /*
     * At 128 SPS a conversion takes
     * approximately 7.8 ms.
     */

    vTaskDelay(
        pdMS_TO_TICKS(10)
    );


    /*
     * Read conversion result.
     */

    err =
        ads1113_read_conversion(
            raw
        );


    if (err != ESP_OK) {
        return err;
    }


    /*
     * Convert raw ADC value to voltage.
     */

    *adc_voltage =
        (float)(*raw) *
        ADS1113_LSB_V;


    /*
     * Reverse the external voltage divider.
     */

    *input_voltage =
        (*adc_voltage) *
        DIVIDER_RATIO;


    /*
     * Prevent tiny negative values around zero.
     */

    if (
        *adc_voltage < 0.0f &&
        *adc_voltage > -0.001f
    )
    {
        *adc_voltage = 0.0f;
    }


    if (
        *input_voltage < 0.0f &&
        *input_voltage > -0.001f
    )
    {
        *input_voltage = 0.0f;
    }


    return ESP_OK;
}


/* ============================================================
 * CAN INITIALIZATION
 * ============================================================ */

static void can_init(void)
{
    twai_onchip_node_config_t config = {
        .io_cfg.tx = CAN_TX_GPIO,
        .io_cfg.rx = CAN_RX_GPIO,

        .bit_timing.bitrate = CAN_BITRATE,

        .tx_queue_depth = CAN_TX_QUEUE_DEPTH,
    };


    ESP_ERROR_CHECK(
        twai_new_node_onchip(
            &config,
            &can_node
        )
    );

    twai_event_callbacks_t callbacks = {
        .on_rx_done = relay_can_rx_callback,
    };
    ESP_ERROR_CHECK(twai_node_register_event_callbacks(can_node, &callbacks, NULL));


    ESP_ERROR_CHECK(
        twai_node_enable(
            can_node
        )
    );


    ESP_LOGI(
        TAG,
        "CAN initialized"
    );

    ESP_LOGI(
        TAG,
        "CAN TX GPIO : %d",
        (int)CAN_TX_GPIO
    );

    ESP_LOGI(
        TAG,
        "CAN RX GPIO : %d",
        (int)CAN_RX_GPIO
    );

    ESP_LOGI(
        TAG,
        "CAN bitrate : %d bit/s",
        CAN_BITRATE
    );

    ESP_LOGI(
        TAG,
        "CAN ID      : 0x%03X",
        CAN_ID_ADS1113
    );

    ESP_LOGI(
        TAG,
        "CAN encoding: BIG-ENDIAN"
    );
}


/* ============================================================
 * BIG-ENDIAN HELPER
 * ============================================================ */

static void write_be16(
    uint8_t *data,
    uint16_t value
)
{
    data[0] =
        (uint8_t)(value >> 8);

    data[1] =
        (uint8_t)(value & 0xFF);
}


/* ============================================================
 * SEND ADS1113 CAN FRAME
 *
 * CAN DATA - BIG-ENDIAN
 *
 * Byte 0-1:
 *     Raw ADC value
 *
 * Byte 2-3:
 *     ADC voltage in mV
 *
 * Byte 4-5:
 *     Original input voltage in mV
 *
 * Byte 6-7:
 *     Packet counter
 * ============================================================ */

static esp_err_t can_send_ads1113(
    int16_t raw,
    float adc_voltage,
    float input_voltage,
    uint16_t packet_counter
)
{
    uint16_t adc_mv;
    uint16_t input_mv;


    /*
     * Convert ADC voltage to millivolts.
     */

    if (adc_voltage <= 0.0f)
    {
        adc_mv = 0;
    }
    else if (adc_voltage >= 65.535f)
    {
        adc_mv = 65535;
    }
    else
    {
        adc_mv =
            (uint16_t)(
                adc_voltage * 1000.0f
            );
    }


    /*
     * Convert original input voltage
     * to millivolts.
     */

    if (input_voltage <= 0.0f)
    {
        input_mv = 0;
    }
    else if (input_voltage >= 65.535f)
    {
        input_mv = 65535;
    }
    else
    {
        input_mv =
            (uint16_t)(
                input_voltage * 1000.0f
            );
    }


    /*
     * 8-byte CAN payload.
     *
     * Static so it remains valid while
     * the frame is queued.
     */

    static uint8_t data[8];


    /*
     * BIG-ENDIAN
     *
     * Raw ADC
     */

    write_be16(
        &data[0],
        (uint16_t)raw
    );


    /*
     * ADC voltage in mV
     */

    write_be16(
        &data[2],
        adc_mv
    );


    /*
     * Original input voltage in mV
     */

    write_be16(
        &data[4],
        input_mv
    );


    /*
     * Packet counter
     */

    write_be16(
        &data[6],
        packet_counter
    );


    /*
     * Standard 11-bit CAN frame.
     */

    twai_frame_t frame = {
        .header.id = CAN_ID_ADS1113,
        .header.ide = false,

        .buffer = data,
        .buffer_len = sizeof(data),
    };


    /*
     * Queue CAN frame.
     */

    esp_err_t err =
        twai_node_transmit(
            can_node,
            &frame,
            CAN_TX_TIMEOUT_MS
        );


    if (err != ESP_OK)
    {
        ESP_LOGE(
            TAG,
            "CAN transmit queue failed: %s",
            esp_err_to_name(err)
        );

        return err;
    }


    /*
     * Wait until transmission completes.
     */

    err =
        twai_node_transmit_wait_all_done(
            can_node,
            CAN_TX_TIMEOUT_MS
        );


    if (err != ESP_OK)
    {
        ESP_LOGE(
            TAG,
            "CAN transmission failed: %s",
            esp_err_to_name(err)
        );

        return err;
    }


    return ESP_OK;
}


/* ============================================================
 * SEND VL53L4CD CAN FRAME (ID 0x201)
 *
 * Byte 0-1: signed distance in mm; -1 means unavailable
 * Byte 2-3: packet counter
 * Byte 4:   status bit 0 = valid measurement
 * Byte 5-7: reserved
 * ============================================================ */

static esp_err_t can_send_lidar(int distance_mm, uint16_t packet_counter)
{
    static uint8_t data[8];

    write_be16(&data[0], (uint16_t)(int16_t)distance_mm);
    write_be16(&data[2], packet_counter);
    data[4] = distance_mm >= 0 ? 0x01 : 0x00;
    data[5] = 0;
    data[6] = 0;
    data[7] = 0;

    twai_frame_t frame = {
        .header.id = CAN_ID_LIDAR,
        .header.ide = false,
        .buffer = data,
        .buffer_len = sizeof(data),
    };

    return twai_node_transmit(can_node, &frame, CAN_TX_TIMEOUT_MS);
}


/* ============================================================
 * MAIN
 * ============================================================ */

void app_main(void)
{
    ESP_LOGI(
        TAG,
        "========================================"
    );

    ESP_LOGI(
        TAG,
        "ESP32 SLAVE 2"
    );

    ESP_LOGI(
        TAG,
        "ADS1113 + CAN"
    );

    ESP_LOGI(
        TAG,
        "========================================"
    );


    /* The relay driver is held OFF before any communications start. */
    relay_init();

    /* --------------------------------------------------------
     * Initialize I2C
     * -------------------------------------------------------- */

    i2c_init();
    lidar_i2c_init();


    /* --------------------------------------------------------
     * Initialize CAN
     * -------------------------------------------------------- */

    can_init();

    if (xTaskCreate(relay_control_task, "relay_control", 3072, NULL, 9, NULL) != pdPASS)
        abort();


    int16_t raw = 0;

    float adc_voltage = 0.0f;

    float input_voltage = 0.0f;

    uint16_t packet_counter = 0;


    ESP_LOGI(
        TAG,
        "System initialized"
    );


    /* --------------------------------------------------------
     * Main measurement loop
     * -------------------------------------------------------- */

    while (1)
    {
        esp_err_t adc_result =
            ads1113_read_voltage(
                &raw,
                &adc_voltage,
                &input_voltage
            );


        if (adc_result != ESP_OK)
        {
            ESP_LOGE(
                TAG,
                "ADS1113 read failed: %s",
                esp_err_to_name(adc_result)
            );


            raw = 0;
            adc_voltage = 0.0f;
            input_voltage = 0.0f;
        }


        /* ----------------------------------------------------
         * Send measurement over CAN
         * ---------------------------------------------------- */

        esp_err_t can_result =
            adc_result == ESP_OK
            ? can_send_ads1113(raw, adc_voltage, input_voltage, packet_counter)
            : ESP_ERR_INVALID_STATE;

        int distance_mm = lidar_read_distance();
        esp_err_t lidar_can_result =
            can_send_lidar(distance_mm, packet_counter);


        /* ----------------------------------------------------
         * Print measurement
         * ---------------------------------------------------- */

        ESP_LOGI(
            TAG,
            "RAW=%d | ADC=%.6f V | INPUT=%.6f V | LIDAR=%d mm | CAN=%s | CNT=%u",
            raw,
            adc_voltage,
            input_voltage,
            distance_mm,
            (can_result == ESP_OK && lidar_can_result == ESP_OK)
                ? "OK"
                : "ERROR",
            packet_counter
        );


        packet_counter++;


        /* ----------------------------------------------------
         * Send every 100 ms
         * ---------------------------------------------------- */

        vTaskDelay(
            pdMS_TO_TICKS(100)
        );
    }
}

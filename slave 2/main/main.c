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
 *  WIRING
 *    ADS1113:  VDD -> 3V3, GND -> GND, SDA -> GPIO 21, SCL -> GPIO 22
 *              ADDR -> GND (address 0x48)
 *              AIN1 -> GND (the ADS1113 measures AIN0 - AIN1)
 *              AIN0 -> divider output
 *
 *    VL53L4CD: SDA -> GPIO 21, SCL -> GPIO 22, address 0x29
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


    /* --------------------------------------------------------
     * Initialize I2C
     * -------------------------------------------------------- */

    i2c_init();
    lidar_i2c_init();


    /* --------------------------------------------------------
     * Initialize CAN
     * -------------------------------------------------------- */

    can_init();


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

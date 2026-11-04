/*
 * ============================================================
 *  ESP32 "SLAVE 3" - 2x IR TEMPERATURE + LIDAR + PRESSURE, OVER CAN
 * ============================================================
 *
 *  Board : ESP32-WROOM-32 DevKit
 *  SDK   : ESP-IDF v5.5+ / v6.x
 *
 *  SENSORS
 *    1. GY-906 (MLX90614) #1   - IR temperature   - I2C bus 0 - 0x5A
 *    2. VL53L0X                - laser distance   - I2C bus 0 - 0x29
 *    3. GY-906 (MLX90614) #2   - IR temperature   - I2C bus 1 - 0x5A
 *    4. Hydraulic pressure sensor (0.5-4.5 V)     - ADC1_CH6
 *
 *  OUTPUT
 *    - Three CAN frames every 100 ms (10 Hz), 500 kbit/s
 *    - The same values printed on the serial monitor
 *
 * ------------------------------------------------------------
 *  FIRST-TIME PROJECT SETUP
 * ------------------------------------------------------------
 *  This is a single-file project: main.c is the only source file.
 *
 *  main/CMakeLists.txt must contain (note the added esp_driver_twai):
 *
 *      idf_component_register(SRCS "main.c"
 *                             INCLUDE_DIRS "."
 *                             PRIV_REQUIRES esp_driver_i2c esp_driver_gpio
 *                                           esp_driver_twai esp_adc)
 *
 *  Then:  idf.py set-target esp32
 *         idf.py build
 *         idf.py -p <PORT> flash monitor
 *
 * ------------------------------------------------------------
 *  WHY TWO I2C BUSES?
 * ------------------------------------------------------------
 *  Both GY-906 modules ship with address 0x5A. Putting each one on
 *  its own bus lets both keep the default address, so no sensor
 *  needs to be reprogrammed.
 *
 * ------------------------------------------------------------
 *  WIRING
 * ------------------------------------------------------------
 *  I2C bus 0 : GPIO21 = SDA, GPIO22 = SCL  -> GY-906 #1, VL53L0X
 *  I2C bus 1 : GPIO25 = SDA, GPIO26 = SCL  -> GY-906 #2
 *  Pressure  : GPIO34 (ADC1_CH6, input only)
 *  CAN       : GPIO5 = TX, GPIO4 = RX      -> CAN transceiver
 *
 *  Power for all sensors: external regulated sensor supply and common GND.
 *  Do not power sensor VCC/VIN from the ESP32 3V3 pin or a GPIO; this
 *  firmware only reads sensor signals and never controls sensor power.
 *
 *  Pressure sensor:
 *    VCC -> external 5 V supply
 *    GND -> ESP32 GND  (MUST be common ground)
 *    OUT -> 10k -> GPIO34 -> 20k -> GND
 *
 *    ADC voltage = sensor voltage x 20k / (10k + 20k) = sensor / 1.5
 *    4.5 V at the sensor -> 3.0 V at the ESP32 pin.
 *
 *  CAN (TWAI) - needs an external 3.3 V CAN transceiver
 *    GPIO 5 -> transceiver TXD
 *    GPIO 4 -> transceiver RXD
 *    CANH / CANL -> bus, with 120 ohm termination at each END of the bus
 *
 * ------------------------------------------------------------
 *  CAN FRAMES (all values big-endian, standard 11-bit IDs, 8 bytes)
 * ------------------------------------------------------------
 *  0x300  bytes 0-1 T1 ambient x100 (int16) | 2-3 T1 object x100 (int16)
 *         bytes 4-5 T2 ambient x100 (int16) | 6-7 T2 object x100 (int16)
 *         Invalid temperature = -32768 (0x8000). Values are clamped to
 *         +-327.67 C.
 *
 *  0x301  bytes 0-1 LiDAR distance in mm (uint16, 0xFFFF = invalid)
 *         bytes 2-5 pressure in Pa (uint32, 0xFFFFFFFF = invalid)
 *         bytes 6-7 sequence number (uint16)
 *
 *  0x302  byte 0 status: bit0 = GY-906 #1 OK, bit1 = GY-906 #2 OK,
 *                        bit2 = LiDAR OK,     bit3 = pressure ADC OK
 *         bytes 1-7 reserved (0)
 *
 * ------------------------------------------------------------
 *  SERIAL OUTPUT (one line every 100 ms)
 * ------------------------------------------------------------
 *  PACKET | T1_AMB=.. C | T1_OBJ=.. C | T2_AMB=.. C | T2_OBJ=.. C |
 *           LIDAR=.. mm | PRESSURE=.. Pa | CAN=ok/ERR
 *
 *  Error values:  temperature = -999.00   lidar = -1   pressure = -1
 * ============================================================
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#include "esp_log.h"

/* ============================================================
 * CONFIGURATION
 * ============================================================ */

/* I2C bus 0: GY-906 #1 + VL53L0X */
#define I2C0_SDA_PIN            21
#define I2C0_SCL_PIN            22

/* I2C bus 1: GY-906 #2 */
#define I2C1_SDA_PIN            25
#define I2C1_SCL_PIN            26

/* Pressure ADC */
#define PRESSURE_ADC_GPIO       34
#define PRESSURE_ADC_CHANNEL    ADC_CHANNEL_6
#define ADC_SAMPLES             16

/* I2C addresses */
#define TEMP1_ADDR              0x5A    /* bus 0 */
#define TEMP2_ADDR              0x5A    /* bus 1 (same address, different bus) */
#define VL53_ADDR               0x29    /* bus 0 */

/* MLX90614 registers */
#define MLX_REG_AMBIENT         0x06
#define MLX_REG_OBJECT1         0x07

/* Pressure sensor: 0.5 V -> 0 Pa, 4.5 V -> 1 MPa */
#define SENSOR_MIN_V            0.5f
#define SENSOR_MAX_V            4.5f
#define MAX_PRESSURE_PA         1000000.0f

/* Divider: 10k (top) + 20k (bottom) -> ratio 1.5 */
#define DIVIDER_RATIO           1.5f

/* CAN (TWAI) */
#define CAN_TX_GPIO             GPIO_NUM_5
#define CAN_RX_GPIO             GPIO_NUM_4
#define CAN_BITRATE             500000

#define CAN_ID_TEMPERATURES     0x300
#define CAN_ID_DIST_PRESSURE    0x301
#define CAN_ID_STATUS           0x302

#define TEMP_INVALID            ((int16_t)-32768)
#define DIST_INVALID            0xFFFF
#define PRESSURE_INVALID        0xFFFFFFFFu

#define I2C_TIMEOUT_MS          100
#define LOOP_PERIOD_MS          100     /* 10 Hz */

#define CHECK(x)                                    \
    do {                                            \
        esp_err_t _e = (x);                         \
        if (_e != ESP_OK) {                         \
            return _e;                              \
        }                                           \
    } while (0)

/* ============================================================
 * GLOBALS
 * ============================================================ */

static const char *TAG = "MULTI_SENSOR";

static i2c_master_bus_handle_t i2c_bus0;
static i2c_master_bus_handle_t i2c_bus1;

static i2c_master_dev_handle_t temp1_handle;
static i2c_master_dev_handle_t temp2_handle;
static i2c_master_dev_handle_t lidar_handle;

static adc_oneshot_unit_handle_t adc_handle;
static adc_cali_handle_t         adc_cali_handle;
static bool                      adc_calibrated;

static uint8_t lidar_stop_var;
static bool    lidar_ready;

static twai_node_handle_t can_node;
static bool               can_ok = false;

/* ============================================================
 * I2C SETUP
 * ============================================================ */

static void i2c_init(void)
{
    /* ---------- Bus 0 ---------- */
    i2c_master_bus_config_t bus0_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C0_SDA_PIN,
        .scl_io_num = I2C0_SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus0_cfg, &i2c_bus0));

    /* ---------- Bus 1 ---------- */
    i2c_master_bus_config_t bus1_cfg = {
        .i2c_port = I2C_NUM_1,
        .sda_io_num = I2C1_SDA_PIN,
        .scl_io_num = I2C1_SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus1_cfg, &i2c_bus1));

    /* ---------- GY-906 #1 on bus 0 ---------- */
    i2c_device_config_t t1_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = TEMP1_ADDR,
        .scl_speed_hz = 100000,     /* MLX90614 max is 100 kHz */
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus0, &t1_cfg, &temp1_handle));

    /* ---------- VL53L0X on bus 0 ---------- */
    i2c_device_config_t lidar_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = VL53_ADDR,
        .scl_speed_hz = 100000,     /* kept at 100 kHz to match the slower MLX on the same bus */
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus0, &lidar_cfg, &lidar_handle));

    /* ---------- GY-906 #2 on bus 1 ---------- */
    i2c_device_config_t t2_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = TEMP2_ADDR,
        .scl_speed_hz = 100000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus1, &t2_cfg, &temp2_handle));
}

/* ============================================================
 * MLX90614 (GY-906)
 * ============================================================ */

/* Reads one 16-bit register. The sensor returns: low byte, high byte, PEC. */
static esp_err_t mlx_read_register(i2c_master_dev_handle_t dev,
                                   uint8_t reg, uint16_t *value)
{
    uint8_t data[3];

    CHECK(i2c_master_transmit_receive(dev, &reg, 1, data, 3, I2C_TIMEOUT_MS));

    uint16_t raw = ((uint16_t)data[1] << 8) | data[0];

    /* Bit 15 set means the sensor flagged an error. */
    if (raw & 0x8000) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    *value = raw;
    return ESP_OK;
}

static float mlx_to_celsius(uint16_t raw)
{
    return ((float)raw * 0.02f) - 273.15f;
}

static bool read_temperature(i2c_master_dev_handle_t dev,
                             float *ambient, float *object)
{
    uint16_t amb_raw, obj_raw;

    if (mlx_read_register(dev, MLX_REG_AMBIENT, &amb_raw) != ESP_OK) {
        return false;
    }
    if (mlx_read_register(dev, MLX_REG_OBJECT1, &obj_raw) != ESP_OK) {
        return false;
    }

    *ambient = mlx_to_celsius(amb_raw);
    *object  = mlx_to_celsius(obj_raw);
    return true;
}

/* ============================================================
 * VL53L0X (simplified single-shot driver)
 *
 * NOTE: this skips ST's full calibration / tuning sequence, so
 * absolute accuracy is limited. Check against a ruler, and use a
 * full driver (e.g. a port of Pololu's or ST's API) if you need
 * precise readings.
 * ============================================================ */

static esp_err_t lidar_write(uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = { reg, value };
    return i2c_master_transmit(lidar_handle, buf, 2, I2C_TIMEOUT_MS);
}

static esp_err_t lidar_read8(uint8_t reg, uint8_t *value)
{
    return i2c_master_transmit_receive(lidar_handle, &reg, 1, value, 1,
                                       I2C_TIMEOUT_MS);
}

static esp_err_t lidar_read16(uint8_t reg, uint16_t *value)
{
    uint8_t data[2];

    CHECK(i2c_master_transmit_receive(lidar_handle, &reg, 1, data, 2,
                                      I2C_TIMEOUT_MS));
    *value = ((uint16_t)data[0] << 8) | data[1];
    return ESP_OK;
}

/* Reads (or writes) the internal "stop variable" needed to start ranging. */
static esp_err_t lidar_stop_var_access(bool write)
{
    CHECK(lidar_write(0x80, 0x01));
    CHECK(lidar_write(0xFF, 0x01));
    CHECK(lidar_write(0x00, 0x00));

    if (write) {
        CHECK(lidar_write(0x91, lidar_stop_var));
    } else {
        CHECK(lidar_read8(0x91, &lidar_stop_var));
    }

    CHECK(lidar_write(0x00, 0x01));
    CHECK(lidar_write(0xFF, 0x00));
    CHECK(lidar_write(0x80, 0x00));
    return ESP_OK;
}

static esp_err_t lidar_init(void)
{
    uint8_t v;

    /* Use 2.8 V I/O mode */
    CHECK(lidar_read8(0x89, &v));
    CHECK(lidar_write(0x89, v | 0x01));

    /* Standard I2C mode */
    CHECK(lidar_write(0x88, 0x00));

    CHECK(lidar_stop_var_access(false));

    /* Disable MSRC and TCC limit checks */
    CHECK(lidar_read8(0x60, &v));
    CHECK(lidar_write(0x60, v | 0x12));

    /* Signal rate limit = 0.25 MCPS (Q9.7 fixed point = 0x0020) */
    CHECK(lidar_write(0x44, 0x00));
    CHECK(lidar_write(0x45, 0x20));

    /* Sequence config */
    CHECK(lidar_write(0x01, 0xFF));

    /* Interrupt on "new sample ready", active low */
    CHECK(lidar_write(0x0A, 0x04));
    CHECK(lidar_read8(0x84, &v));
    CHECK(lidar_write(0x84, v & ~0x10));
    CHECK(lidar_write(0x0B, 0x01));

    return ESP_OK;
}

/* Returns distance in mm, or -1 on error / timeout. */
static int lidar_read_distance(void)
{
    uint8_t status;
    uint16_t distance;

    if (!lidar_ready) {
        return -1;
    }

    if (lidar_stop_var_access(true) != ESP_OK) return -1;
    if (lidar_write(0x00, 0x01) != ESP_OK)     return -1;   /* start single shot */

    /* Wait for the start bit to clear */
    for (int i = 0; i < 100; i++) {
        if (lidar_read8(0x00, &status) != ESP_OK) return -1;
        if (!(status & 0x01)) break;
        vTaskDelay(pdMS_TO_TICKS(1));
        if (i == 99) return -1;
    }

    /* Wait for the measurement-ready flag */
    for (int i = 0; i < 100; i++) {
        if (lidar_read8(0x13, &status) != ESP_OK) return -1;
        if (status & 0x07) break;
        vTaskDelay(pdMS_TO_TICKS(5));
        if (i == 99) return -1;
    }

    if (lidar_read16(0x1E, &distance) != ESP_OK) return -1;
    if (lidar_write(0x0B, 0x01) != ESP_OK)       return -1;   /* clear interrupt */

    return (int)distance;
}

/* ============================================================
 * PRESSURE SENSOR (ADC)
 * ============================================================ */

static void pressure_init(void)
{
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &adc_handle));

    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle,
                                               PRESSURE_ADC_CHANNEL,
                                               &chan_cfg));

    /* Factory calibration improves accuracy, especially near 3 V. */
    adc_cali_line_fitting_config_t cali_cfg = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    if (adc_cali_create_scheme_line_fitting(&cali_cfg, &adc_cali_handle) == ESP_OK) {
        adc_calibrated = true;
        ESP_LOGI(TAG, "ADC calibration enabled");
    } else {
        adc_calibrated = false;
        ESP_LOGW(TAG, "ADC calibration unavailable, using raw conversion");
    }
}

/* Returns pressure in Pa, or -1 on ADC error. */
static float pressure_read(void)
{
    int sum = 0;

    /* Average several samples to reduce ADC noise */
    for (int i = 0; i < ADC_SAMPLES; i++) {
        int raw;
        if (adc_oneshot_read(adc_handle, PRESSURE_ADC_CHANNEL, &raw) != ESP_OK) {
            return -1.0f;
        }
        sum += raw;
    }
    int raw_avg = sum / ADC_SAMPLES;

    float adc_voltage;
    if (adc_calibrated) {
        int mv = 0;
        adc_cali_raw_to_voltage(adc_cali_handle, raw_avg, &mv);
        adc_voltage = mv / 1000.0f;
    } else {
        adc_voltage = ((float)raw_avg / 4095.0f) * 3.3f;
    }

    /* Undo the voltage divider to get the real sensor voltage */
    float sensor_voltage = adc_voltage * DIVIDER_RATIO;

    /* 0.5-4.5 V -> 0-1,000,000 Pa */
    float pressure = ((sensor_voltage - SENSOR_MIN_V) /
                      (SENSOR_MAX_V - SENSOR_MIN_V)) * MAX_PRESSURE_PA;

    if (pressure < 0)               pressure = 0;
    if (pressure > MAX_PRESSURE_PA) pressure = MAX_PRESSURE_PA;

    return pressure;
}

/* ============================================================
 * SENSOR DETECTION
 * ============================================================ */

static void probe_device(i2c_master_bus_handle_t bus, uint8_t addr,
                         const char *name)
{
    if (i2c_master_probe(bus, addr, I2C_TIMEOUT_MS) == ESP_OK) {
        ESP_LOGI(TAG, "%s detected at 0x%02X", name, addr);
    } else {
        ESP_LOGE(TAG, "%s NOT detected (0x%02X) - check wiring", name, addr);
    }
}

static void check_sensors(void)
{
    ESP_LOGI(TAG, "Checking sensors...");
    probe_device(i2c_bus0, TEMP1_ADDR, "GY-906 #1 (bus 0)");
    probe_device(i2c_bus1, TEMP2_ADDR, "GY-906 #2 (bus 1)");
    probe_device(i2c_bus0, VL53_ADDR,  "VL53L0X   (bus 0)");
}

/* ============================================================
 * CAN (TWAI)
 * ============================================================ */

static void put_be16(uint8_t *dst, uint16_t value)
{
    dst[0] = (uint8_t)(value >> 8);
    dst[1] = (uint8_t)(value & 0xFF);
}

static void put_be32(uint8_t *dst, uint32_t value)
{
    dst[0] = (uint8_t)(value >> 24);
    dst[1] = (uint8_t)(value >> 16);
    dst[2] = (uint8_t)(value >> 8);
    dst[3] = (uint8_t)(value & 0xFF);
}

/* Temperature in C -> int16 in hundredths of a degree.
 * Clamped to +-327.67 C so it never collides with TEMP_INVALID. */
static int16_t temp_x100(float celsius)
{
    float scaled = roundf(celsius * 100.0f);
    if (scaled > 32767.0f)  return 32767;
    if (scaled < -32767.0f) return -32767;
    return (int16_t)scaled;
}

static bool init_can(void)
{
    twai_onchip_node_config_t node_cfg = {
        .io_cfg.tx = CAN_TX_GPIO,
        .io_cfg.rx = CAN_RX_GPIO,
        .bit_timing.bitrate = CAN_BITRATE,
        .tx_queue_depth = 6,
    };

    esp_err_t err = twai_new_node_onchip(&node_cfg, &can_node);
    if (err == ESP_OK) {
        err = twai_node_enable(can_node);
    }

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "CAN init failed: %s", esp_err_to_name(err));
        return false;
    }

    ESP_LOGI(TAG, "CAN started at %d kbit/s", CAN_BITRATE / 1000);
    return true;
}

/* 'data' must stay valid until the frame is sent, so callers pass static buffers. */
static esp_err_t can_send(uint32_t id, uint8_t *data)
{
    twai_frame_t frame = {
        .header.id = id,
        .buffer = data,
        .buffer_len = 8,
    };
    return twai_node_transmit(can_node, &frame, 0);
}

/* ============================================================
 * MAIN
 * ============================================================ */

void app_main(void)
{
    printf("\n============================================================\n");
    printf("               ESP32 MULTI-SENSOR SYSTEM\n");
    printf("============================================================\n\n");
    printf("I2C bus 0 : SDA=GPIO%d  SCL=GPIO%d  (GY-906 #1, VL53L0X)\n",
           I2C0_SDA_PIN, I2C0_SCL_PIN);
    printf("I2C bus 1 : SDA=GPIO%d  SCL=GPIO%d  (GY-906 #2)\n",
           I2C1_SDA_PIN, I2C1_SCL_PIN);
    printf("Pressure  : GPIO%d (ADC1_CH6)\n", PRESSURE_ADC_GPIO);
    printf("CAN       : TX=GPIO%d  RX=GPIO%d  %d kbit/s\n\n",
           (int)CAN_TX_GPIO, (int)CAN_RX_GPIO, CAN_BITRATE / 1000);

    i2c_init();
    pressure_init();
    vTaskDelay(pdMS_TO_TICKS(100));

    check_sensors();

    /* VL53L0X: verify model ID, then initialise */
    uint8_t model_id = 0;
    if (lidar_read8(0xC0, &model_id) == ESP_OK && model_id == 0xEE) {
        if (lidar_init() == ESP_OK) {
            lidar_ready = true;
            ESP_LOGI(TAG, "VL53L0X initialised");
        } else {
            ESP_LOGE(TAG, "VL53L0X init failed");
        }
    } else {
        ESP_LOGE(TAG, "VL53L0X model ID incorrect or unreadable (0x%02X)",
                 model_id);
    }

    can_ok = init_can();

    printf("\n============================================================\n");
    printf("                      SENSOR PACKETS\n");
    printf("============================================================\n");

    uint16_t seq = 0;

    /* Static so they stay valid while queued for transmission */
    static uint8_t frame_temps[8];
    static uint8_t frame_dist_press[8];
    static uint8_t frame_status[8];

    while (1) {
        seq++;

        float t1_amb = 0, t1_obj = 0;
        float t2_amb = 0, t2_obj = 0;

        bool t1_ok = read_temperature(temp1_handle, &t1_amb, &t1_obj);
        bool t2_ok = read_temperature(temp2_handle, &t2_amb, &t2_obj);

        int   distance_mm = lidar_read_distance();
        float pressure_pa = pressure_read();

        bool lidar_valid    = (distance_mm >= 0);
        bool pressure_valid = (pressure_pa >= 0.0f);

        /* ----- Frame 0x300: temperatures ----- */
        put_be16(&frame_temps[0], (uint16_t)(t1_ok ? temp_x100(t1_amb) : TEMP_INVALID));
        put_be16(&frame_temps[2], (uint16_t)(t1_ok ? temp_x100(t1_obj) : TEMP_INVALID));
        put_be16(&frame_temps[4], (uint16_t)(t2_ok ? temp_x100(t2_amb) : TEMP_INVALID));
        put_be16(&frame_temps[6], (uint16_t)(t2_ok ? temp_x100(t2_obj) : TEMP_INVALID));

        /* ----- Frame 0x301: distance + pressure + sequence ----- */
        put_be16(&frame_dist_press[0],
                 lidar_valid ? (uint16_t)distance_mm : (uint16_t)DIST_INVALID);
        put_be32(&frame_dist_press[2],
                 pressure_valid ? (uint32_t)(pressure_pa + 0.5f) : PRESSURE_INVALID);
        put_be16(&frame_dist_press[6], seq);

        /* ----- Frame 0x302: status ----- */
        frame_status[0] = (t1_ok ? 0x01 : 0x00) |
                          (t2_ok ? 0x02 : 0x00) |
                          (lidar_valid ? 0x04 : 0x00) |
                          (pressure_valid ? 0x08 : 0x00);
        for (int i = 1; i < 8; i++) {
            frame_status[i] = 0;
        }

        /* ----- Transmit ----- */
        bool can_sent = false;
        if (can_ok) {
            esp_err_t e1 = can_send(CAN_ID_TEMPERATURES, frame_temps);
            esp_err_t e2 = can_send(CAN_ID_DIST_PRESSURE, frame_dist_press);
            esp_err_t e3 = can_send(CAN_ID_STATUS, frame_status);
            can_sent = (e1 == ESP_OK) && (e2 == ESP_OK) && (e3 == ESP_OK);
        }

        printf("PACKET | "
               "T1_AMB=%.2f C | T1_OBJ=%.2f C | "
               "T2_AMB=%.2f C | T2_OBJ=%.2f C | "
               "LIDAR=%d mm | PRESSURE=%.0f Pa | CAN=%s\n",
               t1_ok ? t1_amb : -999.0f, t1_ok ? t1_obj : -999.0f,
               t2_ok ? t2_amb : -999.0f, t2_ok ? t2_obj : -999.0f,
               distance_mm,
               pressure_pa,
               can_sent ? "ok" : "ERR");

        vTaskDelay(pdMS_TO_TICKS(LOOP_PERIOD_MS));
    }
}

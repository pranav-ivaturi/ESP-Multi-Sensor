/*
 * ============================================================
 *  ESP32 "SLAVE 1" - IMU + HALL SENSOR + 2x PT100, SENT OVER CAN
 * ============================================================
 *
 *  Board : ESP32-WROOM-32 DevKit
 *  SDK   : ESP-IDF v5.5+ / v6.x
 *
 *  SENSORS
 *    1. ICM-42688 IMU (accel + gyro)   - I2C, address 0x68
 *    2. HW-484 Hall sensor module      - ADC1_CH6 (A0) + digital (D0)
 *    3. PT100 RTD #1 via MAX31865      - shared SPI2 bus, CS = GPIO 15
 *    4. PT100 RTD #2 via MAX31865      - shared SPI2 bus, CS = GPIO 13
 *
 *  OUTPUT
 *    - Three CAN frames every 50 ms (20 Hz), 500 kbit/s
 *    - The same values printed on the serial monitor
 *
 * ------------------------------------------------------------
 *  FIRST-TIME PROJECT SETUP
 * ------------------------------------------------------------
 *  This is a SINGLE-FILE project: main.c is the only source file.
 *  The MAX31865 driver is built into it (see the "MAX31865 DRIVER"
 *  section). Do NOT add the esp-idf-lib max31865 component here.
 *
 *  main/CMakeLists.txt must contain:
 *
 *      idf_component_register(SRCS "main.c"
 *                             INCLUDE_DIRS "."
 *                             PRIV_REQUIRES esp_driver_i2c esp_driver_spi
 *                                           esp_driver_gpio esp_driver_twai
 *                                           esp_adc)
 *
 *  Then:  idf.py set-target esp32
 *         idf.py build
 *         idf.py -p <PORT> flash monitor
 *
 * ------------------------------------------------------------
 *  WIRING
 * ------------------------------------------------------------
 *  SENSOR POWER (external supply; not ESP32 3V3)
 *    Power the IMU, Hall module, and MAX31865 boards from the external
 *    regulated sensor supply shown in the system diagram.  Join its ground
 *    to ESP32 GND.  This firmware never drives a GPIO as a sensor supply.
 *
 *  IMU (ICM-42688, I2C)
 *    VCC -> external 3.3 V sensor supply, GND -> common GND
 *    SDA -> GPIO 21    SCL -> GPIO 22
 *    AD0 -> GND  (address 0x68; tie to 3V3 for 0x69 and change ICM_ADDR)
 *
 *  Hall sensor (HW-484)
 *    +  -> external sensor supply, G -> common GND
 *    A0 -> GPIO 34     D0 -> GPIO 27
 *
 *  MAX31865 boards (both share the same SPI lines)
 *    VIN -> external 3.3 V sensor supply, GND -> common GND
 *    CLK -> GPIO 18    SDI (MOSI) -> GPIO 23    SDO (MISO) -> GPIO 19
 *    CS of board 1 -> GPIO 15
 *    CS of board 2 -> GPIO 13
 *    The PT100 wiring mode (2/3/4-wire) must match RTD_WIRING below
 *    and the solder jumpers on the board.
 *
 *  CAN (TWAI) - needs an external 3.3 V CAN transceiver
 *    GPIO 5 -> transceiver TXD
 *    GPIO 4 -> transceiver RXD
 *    CANH / CANL -> bus, with 120 ohm termination at each end
 *
 * ------------------------------------------------------------
 *  CAN FRAMES (all values big-endian, standard 11-bit IDs)
 * ------------------------------------------------------------
 *  0x100  bytes 0-1 AX | 2-3 AY | 4-5 AZ | 6-7 sequence number (uint16)
 *  0x101  bytes 0-1 GX | 2-3 GY | 4-5 GZ | 6-7 Hall A0 raw (uint16)
 *  0x102  bytes 0-1 Temp1 x100 (int16) | 2-3 Temp2 x100 (int16)
 *         byte 4 Hall D0 | byte 5 RTD1 fault | byte 6 RTD2 fault
 *         byte 7 status: bit0 = IMU OK, bit1 = Hall ADC OK
 *
 *  IMU raw scale (power-on defaults): accel 2048 LSB/g (+-16 g),
 *  gyro 16.4 LSB/(deg/s) (+-2000 dps).
 * ============================================================
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"

/* ============================================================
 * MAX31865 DRIVER (built into this file - PT100 / PT1000 RTDs)
 *
 * Set MAX31865_MAINS_50HZ to 1 for 50 Hz mains (India, Europe)
 * or 0 for 60 Hz mains. This selects the chip's noise filter.
 * ============================================================ */

#define MAX31865_MAINS_50HZ           1

#define MAX31865_CONFIG_BIAS          0x80
#define MAX31865_CONFIG_MODEAUTO      0x40
#define MAX31865_CONFIG_3WIRE         0x10
#define MAX31865_CONFIG_FAULTSTAT     0x02
#define MAX31865_CONFIG_FILT50HZ      0x01

#define MAX31865_REG_CONFIG_READ      0x00
#define MAX31865_REG_CONFIG_WRITE     0x80
#define MAX31865_REG_RTD_MSB          0x01
#define MAX31865_REG_FAULT_STATUS     0x07

typedef enum {
    MAX31865_2WIRE = 0,
    MAX31865_3WIRE = 1,
    MAX31865_4WIRE = 0      /* 2-wire and 4-wire use the same config bit */
} max31865_numwires_t;

typedef struct {
    spi_device_handle_t spi_dev;
    float rref;             /* 430.0 for PT100, 4300.0 for PT1000 */
    float rnominal;         /* 100.0 for PT100, 1000.0 for PT1000 */
    max31865_numwires_t wires;
} max31865_t;

static uint8_t max31865_read_reg8(max31865_t *dev, uint8_t addr)
{
    uint8_t tx_data[2] = { addr & 0x7F, 0xFF };
    uint8_t rx_data[2] = { 0 };

    spi_transaction_t t = {
        .length = 16,
        .tx_buffer = tx_data,
        .rx_buffer = rx_data,
    };
    spi_device_polling_transmit(dev->spi_dev, &t);
    return rx_data[1];
}

static void max31865_write_reg8(max31865_t *dev, uint8_t addr, uint8_t data)
{
    uint8_t tx_data[2] = { addr | 0x80, data };

    spi_transaction_t t = {
        .length = 16,
        .tx_buffer = tx_data,
    };
    spi_device_polling_transmit(dev->spi_dev, &t);
}

/* Starts continuous conversions. Call after spi_bus_add_device(). */
static void max31865_init(max31865_t *dev)
{
    uint8_t config = MAX31865_CONFIG_BIAS |
                     MAX31865_CONFIG_MODEAUTO |
                     MAX31865_CONFIG_FAULTSTAT;

    if (dev->wires == MAX31865_3WIRE) {
        config |= MAX31865_CONFIG_3WIRE;
    }
#if MAX31865_MAINS_50HZ
    config |= MAX31865_CONFIG_FILT50HZ;
#endif

    max31865_write_reg8(dev, MAX31865_REG_CONFIG_WRITE, config);
}

/* Clears the fault status so new faults can be detected. */
static void max31865_clear_fault(max31865_t *dev)
{
    uint8_t cfg = max31865_read_reg8(dev, MAX31865_REG_CONFIG_READ);
    cfg |= MAX31865_CONFIG_FAULTSTAT;
    max31865_write_reg8(dev, MAX31865_REG_CONFIG_WRITE, cfg);
}

/* Fault status register (0 = no fault). */
static uint8_t max31865_read_fault(max31865_t *dev)
{
    return max31865_read_reg8(dev, MAX31865_REG_FAULT_STATUS);
}

/* Raw 15-bit RTD value. Does NOT clear faults. */
static uint16_t max31865_read_rtd_raw(max31865_t *dev)
{
    uint8_t tx_data[3] = { MAX31865_REG_RTD_MSB & 0x7F, 0xFF, 0xFF };
    uint8_t rx_data[3] = { 0 };

    spi_transaction_t t = {
        .length = 24,
        .tx_buffer = tx_data,
        .rx_buffer = rx_data,
    };
    spi_device_polling_transmit(dev->spi_dev, &t);

    uint16_t rtd = ((uint16_t)rx_data[1] << 8) | rx_data[2];
    rtd >>= 1;      /* drop the fault flag (bit 0) */
    return rtd;
}

/* Temperature in degrees C. */
static float max31865_read_temperature(max31865_t *dev)
{
    float raw = (float)max31865_read_rtd_raw(dev);
    float Rt = (raw * dev->rref) / 32768.0f;

    /* Callendar-Van Dusen coefficients for PT100 / PT1000 */
    const float A = 3.90830e-3f;
    const float B = -5.77500e-7f;

    float Z1 = -A;
    float Z2 = A * A - 4.0f * B;
    float Z3 = (4.0f * B) / dev->rnominal;
    float Z4 = 2.0f * B;

    float discriminant = Z2 + (Z3 * Rt);
    if (discriminant < 0.0f) {
        discriminant = 0.0f;
    }

    float temp = (sqrtf(discriminant) + Z1) / Z4;
    if (temp >= 0.0f) {
        return temp;
    }

    /* Below 0 C the quadratic is not accurate; use the polynomial fit.
     * The polynomial is written for a 100 ohm element, so normalise first. */
    float r = (Rt / dev->rnominal) * 100.0f;
    float rpoly = r;

    temp = -242.02f;
    temp += 2.2228f * rpoly;
    rpoly *= r;
    temp += 2.5859e-3f * rpoly;
    rpoly *= r;
    temp -= 4.8260e-6f * rpoly;
    rpoly *= r;
    temp -= 2.8183e-8f * rpoly;
    rpoly *= r;
    temp += 1.5243e-10f * rpoly;

    return temp;
}

/* ============================================================
 * CONFIGURATION
 * ============================================================ */

/* ---------- CAN (TWAI) ---------- */
#define CAN_TX_GPIO             GPIO_NUM_5
#define CAN_RX_GPIO             GPIO_NUM_4
#define CAN_BITRATE             500000

#define CAN_ID_IMU_ACCEL        0x100
#define CAN_ID_IMU_GYRO_HALL    0x101
#define CAN_ID_TEMPERATURES     0x102

/* ---------- ICM-42688 (I2C) ---------- */
#define I2C_SDA_IO              21
#define I2C_SCL_IO              22
#define I2C_FREQ_HZ             400000
#define ICM_ADDR                0x68
#define ICM_REG_WHO_AM_I        0x75
#define ICM_WHO_AM_I_VALUE      0x47
#define ICM_REG_PWR_MGMT0       0x4E
#define ICM_REG_DATA_START      0x1D    /* TEMP_DATA1: temp(2) accel(6) gyro(6) = 14 bytes */
#define ICM_DATA_LEN            14
#define I2C_TIMEOUT_MS          50

/* ---------- HW-484 Hall sensor ---------- */
#define HALL_ADC_UNIT           ADC_UNIT_1
#define HALL_ADC_CHANNEL        ADC_CHANNEL_6       /* GPIO 34 */
#define HALL_DIGITAL_GPIO       GPIO_NUM_27
#define HALL_BASELINE           1900                /* approx. zero-field ADC value (log only) */

/* ---------- MAX31865 (shared SPI) ---------- */
#define SPI_HOST_ID             SPI2_HOST
#define PIN_NUM_MISO            GPIO_NUM_19
#define PIN_NUM_MOSI            GPIO_NUM_23
#define PIN_NUM_CLK             GPIO_NUM_18
#define PIN_NUM_CS1             GPIO_NUM_15
#define PIN_NUM_CS2             GPIO_NUM_13
#define SPI_CLOCK_HZ            1000000

#define RTD_REF_OHMS            430.0f      /* check the resistor on your board */
#define RTD_NOMINAL_OHMS        100.0f      /* PT100 */
#define RTD_WIRING              MAX31865_3WIRE  /* or MAX31865_2WIRE / MAX31865_4WIRE */

/* ---------- Loop ---------- */
#define LOOP_PERIOD_MS          50          /* 20 Hz */

/* ============================================================
 * GLOBALS
 * ============================================================ */

static const char *TAG = "SLAVE1";

static i2c_master_bus_handle_t i2c_bus;
static i2c_master_dev_handle_t imu_dev;
static bool imu_ok = false;

static max31865_t rtd1;
static max31865_t rtd2;

static adc_oneshot_unit_handle_t adc_handle;
static bool hall_ok = false;

static twai_node_handle_t can_node;
static bool can_ok = false;

/* ============================================================
 * SMALL HELPERS
 * ============================================================ */

static void put_be16(uint8_t *dst, uint16_t value)
{
    dst[0] = (uint8_t)(value >> 8);
    dst[1] = (uint8_t)(value & 0xFF);
}

/* Temperature in C -> int16 in hundredths of a degree, clamped. */
static int16_t temp_x100(float celsius)
{
    float scaled = roundf(celsius * 100.0f);
    if (scaled > 32767.0f)  return 32767;
    if (scaled < -32768.0f) return -32768;
    return (int16_t)scaled;
}

/* ============================================================
 * IMU (ICM-42688 over I2C)
 * ============================================================ */

static esp_err_t icm_write(uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = { reg, value };
    return i2c_master_transmit(imu_dev, buf, 2, I2C_TIMEOUT_MS);
}

static esp_err_t icm_read(uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(imu_dev, &reg, 1, data, len,
                                       I2C_TIMEOUT_MS);
}

static bool init_imu(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_SDA_IO,
        .scl_io_num = I2C_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &i2c_bus));

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = ICM_ADDR,
        .scl_speed_hz = I2C_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus, &dev_cfg, &imu_dev));

    uint8_t whoami = 0;
    if (icm_read(ICM_REG_WHO_AM_I, &whoami, 1) != ESP_OK ||
        whoami != ICM_WHO_AM_I_VALUE) {
        ESP_LOGE(TAG, "ICM-42688 check failed. Expected 0x%02X, got 0x%02X. "
                      "Check wiring and the AD0 address pin.",
                 ICM_WHO_AM_I_VALUE, whoami);
        return false;
    }

    /* Accel + gyro in low-noise mode */
    if (icm_write(ICM_REG_PWR_MGMT0, 0x0F) != ESP_OK) {
        ESP_LOGE(TAG, "ICM-42688 power-up write failed");
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(50));

    ESP_LOGI(TAG, "ICM-42688 ready");
    return true;
}

static bool imu_read(int16_t *ax, int16_t *ay, int16_t *az,
                     int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint8_t raw[ICM_DATA_LEN];

    if (icm_read(ICM_REG_DATA_START, raw, ICM_DATA_LEN) != ESP_OK) {
        return false;
    }

    /* raw[0..1] = temperature (not used) */
    *ax = (int16_t)(((uint16_t)raw[2]  << 8) | raw[3]);
    *ay = (int16_t)(((uint16_t)raw[4]  << 8) | raw[5]);
    *az = (int16_t)(((uint16_t)raw[6]  << 8) | raw[7]);
    *gx = (int16_t)(((uint16_t)raw[8]  << 8) | raw[9]);
    *gy = (int16_t)(((uint16_t)raw[10] << 8) | raw[11]);
    *gz = (int16_t)(((uint16_t)raw[12] << 8) | raw[13]);
    return true;
}

/* ============================================================
 * RTDs (2x MAX31865 on one SPI bus)
 * ============================================================ */

static void init_rtd(max31865_t *dev, int cs_pin)
{
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = SPI_CLOCK_HZ,
        .mode = 1,                          /* MAX31865 uses SPI mode 1 or 3 */
        .spics_io_num = cs_pin,
        .queue_size = 1,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(SPI_HOST_ID, &devcfg, &dev->spi_dev));

    dev->rref = RTD_REF_OHMS;
    dev->rnominal = RTD_NOMINAL_OHMS;
    dev->wires = RTD_WIRING;
    max31865_init(dev);
}

static void init_rtds(void)
{
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI_HOST_ID, &buscfg, SPI_DMA_CH_AUTO));

    init_rtd(&rtd1, PIN_NUM_CS1);
    init_rtd(&rtd2, PIN_NUM_CS2);

    ESP_LOGI(TAG, "Dual MAX31865 initialised on shared SPI bus");
}

/* Reads one RTD and its fault register. A non-zero fault is reported once
 * and then cleared so the chip can recover when the wiring is fixed. */
static float read_rtd(max31865_t *dev, uint8_t *fault)
{
    float temperature = max31865_read_temperature(dev);

    *fault = max31865_read_fault(dev);
    if (*fault) {
        max31865_clear_fault(dev);
    }
    return temperature;
}

/* ============================================================
 * HALL SENSOR (HW-484)
 * ============================================================ */

static void init_hall(void)
{
    adc_oneshot_unit_init_cfg_t unit_cfg = { .unit_id = HALL_ADC_UNIT };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &adc_handle));

    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, HALL_ADC_CHANNEL,
                                               &chan_cfg));

    gpio_config_t digital_cfg = {
        .pin_bit_mask = (1ULL << HALL_DIGITAL_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&digital_cfg));

    hall_ok = true;
    ESP_LOGI(TAG, "Hall sensor ready (A0 on GPIO34, D0 on GPIO27)");
}

/* ============================================================
 * CAN (TWAI)
 * ============================================================ */

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
    printf("                      ESP32 SLAVE 1\n");
    printf("============================================================\n\n");

    imu_ok = init_imu();
    init_rtds();
    init_hall();
    can_ok = init_can();

    /* Let the MAX31865 finish its first conversions */
    vTaskDelay(pdMS_TO_TICKS(100));

    uint16_t seq = 0;

    /* Static so they stay valid while queued for transmission */
    static uint8_t frame1[8];
    static uint8_t frame2[8];
    static uint8_t frame3[8];

    while (1) {
        seq++;

        /* ----- 1. IMU ----- */
        int16_t ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
        bool imu_valid = imu_ok && imu_read(&ax, &ay, &az, &gx, &gy, &gz);

        /* ----- 2. Hall sensor ----- */
        int hall_raw = 0;
        bool hall_valid = hall_ok &&
            (adc_oneshot_read(adc_handle, HALL_ADC_CHANNEL, &hall_raw) == ESP_OK);
        if (!hall_valid) {
            hall_raw = 0;
        }
        uint8_t hall_d0 = (uint8_t)gpio_get_level(HALL_DIGITAL_GPIO);

        /* ----- 3. Temperatures ----- */
        uint8_t fault1 = 0, fault2 = 0;
        float t1 = read_rtd(&rtd1, &fault1);
        float t2 = read_rtd(&rtd2, &fault2);

        /* ----- Build the CAN frames ----- */
        put_be16(&frame1[0], (uint16_t)ax);
        put_be16(&frame1[2], (uint16_t)ay);
        put_be16(&frame1[4], (uint16_t)az);
        put_be16(&frame1[6], seq);

        put_be16(&frame2[0], (uint16_t)gx);
        put_be16(&frame2[2], (uint16_t)gy);
        put_be16(&frame2[4], (uint16_t)gz);
        put_be16(&frame2[6], (uint16_t)hall_raw);

        put_be16(&frame3[0], (uint16_t)temp_x100(t1));
        put_be16(&frame3[2], (uint16_t)temp_x100(t2));
        frame3[4] = hall_d0;
        frame3[5] = fault1;
        frame3[6] = fault2;
        frame3[7] = (imu_valid ? 0x01 : 0x00) | (hall_valid ? 0x02 : 0x00);

        /* ----- Transmit ----- */
        bool can_sent = false;
        if (can_ok) {
            can_sent = (can_send(CAN_ID_IMU_ACCEL, frame1) == ESP_OK) &&
                       (can_send(CAN_ID_IMU_GYRO_HALL, frame2) == ESP_OK) &&
                       (can_send(CAN_ID_TEMPERATURES, frame3) == ESP_OK);
        }

        ESP_LOGI(TAG,
                 "#%u | A:%d,%d,%d | G:%d,%d,%d | T1:%.2f C (F:0x%02X) | "
                 "T2:%.2f C (F:0x%02X) | Hall:%d (%+d) D0:%d | IMU:%s CAN:%s",
                 seq, ax, ay, az, gx, gy, gz,
                 t1, fault1, t2, fault2,
                 hall_raw, hall_raw - HALL_BASELINE, hall_d0,
                 imu_valid ? "ok" : "ERR",
                 can_sent ? "ok" : "ERR");

        vTaskDelay(pdMS_TO_TICKS(LOOP_PERIOD_MS));
    }
}

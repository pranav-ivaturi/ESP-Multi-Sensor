/*
 * ============================================================
 *  ESP32 "SLAVE 4" - DUAL PT100 RTD + HALL SENSOR
 * ============================================================
 *
 *  Board : ESP32-WROOM-32 DevKit
 *  SDK   : ESP-IDF v5.2+ / v6.x
 *
 *  SENSORS
 *    1. PT100 RTD #1 via MAX31865 board  - SPI2 (own bus)
 *    2. PT100 RTD #2 via MAX31865 board  - SPI3 (own bus)
 *    3. Analog Hall-effect sensor        - ADC1_CH6 (GPIO34)
 *
 * ------------------------------------------------------------
 *  FIRST-TIME PROJECT SETUP
 * ------------------------------------------------------------
 *  This code needs the "max31865" library from esp-idf-lib.
 *
 *  1. Create main/idf_component.yml containing:
 *
 *       dependencies:
 *         esp-idf-lib/max31865: "*"
 *
 *  2. Create main/CMakeLists.txt containing:
 *
 *       idf_component_register(
 *           SRCS "main.c"
 *           INCLUDE_DIRS "."
 *           PRIV_REQUIRES
 *               esp_driver_spi
 *               esp_driver_gpio
 *               esp_adc
 *               esp_driver_twai
 *       )
 *
 * ------------------------------------------------------------
 *  WIRING
 * ------------------------------------------------------------
 *
 *  MAX31865 board       RTD #1       RTD #2
 *  ------------------------------------------
 *  VIN                   3V3          3V3
 *  GND                   GND          GND
 *  CLK (SCK)             GPIO18       GPIO14
 *  SDI (MOSI)            GPIO23       GPIO13
 *  SDO (MISO)            GPIO19       GPIO27
 *  CS                    GPIO25       GPIO26
 *
 *  Hall sensor:
 *    VCC -> supply required by your sensor
 *    GND -> ESP32 GND
 *    OUT -> GPIO34
 *
 *  CAN:
 *    ESP32 GPIO5 -> CAN transceiver TXD
 *    ESP32 GPIO4 -> CAN transceiver RXD
 *    CANH / CANL -> CAN bus
 *
 *  IMPORTANT:
 *    GPIO4/5 are used ONLY for CAN.
 *
 * ------------------------------------------------------------
 *  CAN
 * ------------------------------------------------------------
 *
 *  Bitrate:
 *    500 kbit/s
 *
 *  Slave 4 CAN IDs:
 *
 *    0x400 -> PT100 temperatures + status
 *    0x401 -> Hall sensor data
 *
 *  All values below use BIG-ENDIAN encoding, matching
 *  Slave 1 and Slave 3.
 *
 *  0x400:
 *    Byte 0-1 : RTD1 temperature x100, signed
 *    Byte 2-3 : RTD2 temperature x100, signed
 *    Byte 4   : status
 *               bit 0 = RTD1 valid
 *               bit 1 = RTD2 valid
 *    Byte 5-6 : packet counter
 *    Byte 7   : reserved
 *
 *  0x401:
 *    Byte 0-1 : Hall raw ADC, unsigned
 *    Byte 2-3 : Hall voltage in mV, unsigned
 *    Byte 4   : status
 *               bit 0 = Hall valid
 *    Byte 5-6 : packet counter
 *    Byte 7   : reserved
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

#include "driver/spi_master.h"
#include "driver/gpio.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#include "esp_twai.h"
#include "esp_twai_onchip.h"

#include "max31865.h"


/* ============================================================
 * CONFIGURATION
 * ============================================================ */

/* ---------- RTD 1: SPI2 ---------- */

#define RTD1_SPI_HOST       SPI2_HOST
#define RTD1_SCK            18
#define RTD1_MOSI           23
#define RTD1_MISO           19
#define RTD1_CS             25


/* ---------- RTD 2: SPI3 ---------- */

#define RTD2_SPI_HOST       SPI3_HOST
#define RTD2_SCK            14
#define RTD2_MOSI           13
#define RTD2_MISO           27
#define RTD2_CS             26


/* ---------- RTD settings ---------- */

#define RTD_NOMINAL_OHMS    100.0f
#define RTD_REFERENCE_OHMS  430.0f

#define RTD_WIRING          MAX31865_4WIRE
#define RTD_FILTER          MAX31865_FILTER_50HZ

#define SPI_CLOCK_HZ        1000000


/* ---------- Hall sensor ---------- */

#define HALL_ADC_GPIO       34
#define HALL_CHANNEL        ADC_CHANNEL_6
#define HALL_SAMPLES        16
#define HALL_DIVIDER_RATIO  1.0f


/* ---------- General ---------- */

#define LOOP_PERIOD_MS      500


/* ============================================================
 * CAN CONFIGURATION
 * ============================================================ */

#define CAN_TX_GPIO         GPIO_NUM_5
#define CAN_RX_GPIO         GPIO_NUM_4

#define CAN_BITRATE         500000

#define CAN_ID_RTDS         0x400
#define CAN_ID_HALL         0x401

#define CAN_TX_QUEUE_DEPTH  6


/* ============================================================
 * GLOBALS
 * ============================================================ */

static const char *TAG = "SLAVE4";

static max31865_t rtd1;
static max31865_t rtd2;

static bool rtd1_ok = false;
static bool rtd2_ok = false;

static adc_oneshot_unit_handle_t adc_handle;
static adc_cali_handle_t         adc_cali_handle;

static bool adc_calibrated = false;
static bool hall_ok = false;

static twai_node_handle_t can_node = NULL;
static bool can_ok = false;


/* ============================================================
 * SPI + RTD SETUP
 * ============================================================ */

static esp_err_t init_spi_bus(spi_host_device_t host,
                              int sck_pin,
                              int mosi_pin,
                              int miso_pin)
{
    spi_bus_config_t bus_config = {
        .mosi_io_num = mosi_pin,
        .miso_io_num = miso_pin,
        .sclk_io_num = sck_pin,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 16,
    };

    return spi_bus_initialize(host, &bus_config, SPI_DMA_DISABLED);
}


static esp_err_t init_rtd(max31865_t *dev,
                          spi_host_device_t host,
                          int cs_pin,
                          const char *name)
{
    esp_err_t err =
        max31865_init_desc(dev, host, SPI_CLOCK_HZ, cs_pin);

    if (err != ESP_OK) {
        ESP_LOGE(TAG,
                 "%s: MAX31865 init failed (%s)",
                 name,
                 esp_err_to_name(err));

        return err;
    }

    dev->rtd_nominal = RTD_NOMINAL_OHMS;
    dev->r_ref       = RTD_REFERENCE_OHMS;
    dev->standard    = MAX31865_ITS90;

    max31865_config_t config = {
        .v_bias     = true,
        .connection = RTD_WIRING,
        .mode       = MAX31865_MODE_SINGLE,
        .filter     = RTD_FILTER,
    };

    err = max31865_set_config(dev, &config);

    if (err != ESP_OK) {
        ESP_LOGE(TAG,
                 "%s: configuration failed (%s)",
                 name,
                 esp_err_to_name(err));

        return err;
    }

    ESP_LOGI(TAG, "%s ready", name);

    return ESP_OK;
}


static void init_rtds(void)
{
    /* ----- RTD 1 on SPI2 ----- */

    esp_err_t err =
        init_spi_bus(RTD1_SPI_HOST,
                     RTD1_SCK,
                     RTD1_MOSI,
                     RTD1_MISO);

    if (err == ESP_OK) {
        rtd1_ok =
            (init_rtd(&rtd1,
                      RTD1_SPI_HOST,
                      RTD1_CS,
                      "RTD 1") == ESP_OK);
    }
    else {
        ESP_LOGE(TAG,
                 "RTD 1: SPI bus init failed (%s)",
                 esp_err_to_name(err));
    }


    /* ----- RTD 2 on SPI3 ----- */

    err =
        init_spi_bus(RTD2_SPI_HOST,
                     RTD2_SCK,
                     RTD2_MOSI,
                     RTD2_MISO);

    if (err == ESP_OK) {
        rtd2_ok =
            (init_rtd(&rtd2,
                      RTD2_SPI_HOST,
                      RTD2_CS,
                      "RTD 2") == ESP_OK);
    }
    else {
        ESP_LOGE(TAG,
                 "RTD 2: SPI bus init failed (%s)",
                 esp_err_to_name(err));
    }
}


/* ============================================================
 * HALL SENSOR
 * ============================================================ */

static esp_err_t init_hall(void)
{
    adc_oneshot_unit_init_cfg_t unit_config = {
        .unit_id = ADC_UNIT_1,
    };

    esp_err_t err =
        adc_oneshot_new_unit(&unit_config, &adc_handle);

    if (err != ESP_OK) {
        return err;
    }


    adc_oneshot_chan_cfg_t channel_config = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = ADC_ATTEN_DB_12,
    };

    err =
        adc_oneshot_config_channel(adc_handle,
                                   HALL_CHANNEL,
                                   &channel_config);

    if (err != ESP_OK) {
        return err;
    }


    /* Factory calibration improves voltage accuracy. */

    adc_cali_line_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };

    if (adc_cali_create_scheme_line_fitting(
            &cali_config,
            &adc_cali_handle) == ESP_OK)
    {
        adc_calibrated = true;

        ESP_LOGI(TAG,
                 "Hall ADC calibration enabled");
    }
    else
    {
        ESP_LOGW(TAG,
                 "Hall ADC calibration unavailable, using raw conversion");
    }

    return ESP_OK;
}


/*
 * Averages several readings.
 *
 * Returns:
 *   raw_avg = averaged ADC count
 *   volts   = sensor-output voltage
 */
static bool hall_read(int *raw_avg, float *volts)
{
    int sum = 0;

    for (int i = 0; i < HALL_SAMPLES; i++) {

        int raw;

        if (adc_oneshot_read(
                adc_handle,
                HALL_CHANNEL,
                &raw) != ESP_OK)
        {
            return false;
        }

        sum += raw;
    }


    *raw_avg = sum / HALL_SAMPLES;


    float adc_voltage;

    int mv = 0;

    if (adc_calibrated &&
        adc_cali_raw_to_voltage(
            adc_cali_handle,
            *raw_avg,
            &mv) == ESP_OK)
    {
        adc_voltage = mv / 1000.0f;
    }
    else
    {
        adc_voltage =
            ((float)*raw_avg / 4095.0f) * 3.3f;
    }


    *volts =
        adc_voltage * HALL_DIVIDER_RATIO;

    return true;
}


/* ============================================================
 * CAN / TWAI
 * ============================================================ */

static bool init_can(void)
{
    twai_onchip_node_config_t node_cfg = {
        .io_cfg.tx = CAN_TX_GPIO,
        .io_cfg.rx = CAN_RX_GPIO,

        .bit_timing.bitrate = CAN_BITRATE,

        .tx_queue_depth = CAN_TX_QUEUE_DEPTH,
    };


    esp_err_t err =
        twai_new_node_onchip(
            &node_cfg,
            &can_node);


    if (err == ESP_OK) {

        err =
            twai_node_enable(can_node);
    }


    if (err != ESP_OK) {

        ESP_LOGE(TAG,
                 "CAN init failed: %s",
                 esp_err_to_name(err));

        return false;
    }


    ESP_LOGI(TAG,
             "CAN started at %d kbit/s",
             CAN_BITRATE / 1000);

    ESP_LOGI(TAG,
             "CAN TX = GPIO%d",
             CAN_TX_GPIO);

    ESP_LOGI(TAG,
             "CAN RX = GPIO%d",
             CAN_RX_GPIO);

    ESP_LOGI(TAG,
             "Slave 4 CAN IDs: 0x%03X / 0x%03X",
             CAN_ID_RTDS,
              CAN_ID_HALL);

    return true;
}


/* ============================================================
 * CAN HELPERS
 * ============================================================ */

static void put_be16(uint8_t *dst, uint16_t value)
{
    dst[0] = (uint8_t)(value >> 8);
    dst[1] = (uint8_t)(value & 0xFF);
}


static void put_be_signed16(uint8_t *dst, int16_t value)
{
    put_be16(dst, (uint16_t)value);
}


static int16_t temperature_x100(float temperature)
{
    float scaled = temperature * 100.0f;

    if (scaled > 32767.0f) {
        return 32767;
    }

    if (scaled < -32767.0f) {
        return -32767;
    }

    return (int16_t)scaled;
}


/* ============================================================
 * SEND RTD CAN FRAME
 *
 * ID: 0x400
 *
 * Byte 0-1 : RTD1 temperature x100
 * Byte 2-3 : RTD2 temperature x100
 * Byte 4   : status
 * Byte 5-6 : packet counter
 * Byte 7   : reserved
 * ============================================================ */

static esp_err_t can_send_rtds(float temp1,
                               float temp2,
                               bool t1_valid,
                               bool t2_valid,
                               uint16_t sequence)
{
    static uint8_t data[8];

    int16_t t1 = t1_valid
                 ? temperature_x100(temp1)
                 : INT16_MIN;

    int16_t t2 = t2_valid
                 ? temperature_x100(temp2)
                 : INT16_MIN;


    put_be_signed16(&data[0], t1);
    put_be_signed16(&data[2], t2);


    data[4] = 0;

    if (t1_valid) {
        data[4] |= 0x01;
    }

    if (t2_valid) {
        data[4] |= 0x02;
    }


    put_be16(&data[5], sequence);

    data[7] = 0;


    twai_frame_t frame = {
        .header.id = CAN_ID_RTDS,
        .header.ide = false,

        .buffer = data,
        .buffer_len = sizeof(data),
    };


    return twai_node_transmit(
        can_node,
        &frame,
        0);
}


/* ============================================================
 * SEND HALL CAN FRAME
 *
 * ID: 0x401
 *
 * Byte 0-1 : Hall raw ADC
 * Byte 2-3 : Hall voltage in mV
 * Byte 4   : status
 * Byte 5-6 : packet counter
 * Byte 7   : reserved
 * ============================================================ */

static esp_err_t can_send_hall(int hall_raw,
                               float hall_voltage,
                               bool hall_valid,
                               uint16_t sequence)
{
    static uint8_t data[8];


    uint16_t hall_mv = 0;

    if (hall_voltage > 0.0f) {

        float mv = hall_voltage * 1000.0f;

        if (mv > 65535.0f) {
            mv = 65535.0f;
        }

        hall_mv = (uint16_t)mv;
    }


    if (hall_raw < 0) {
        hall_raw = 0;
    }

    if (hall_raw > 4095) {
        hall_raw = 4095;
    }


    put_be16(&data[0],
             (uint16_t)hall_raw);

    put_be16(&data[2],
             hall_mv);


    data[4] = hall_valid ? 0x01 : 0x00;


    put_be16(&data[5],
             sequence);

    data[7] = 0;


    twai_frame_t frame = {
        .header.id = CAN_ID_HALL,
        .header.ide = false,

        .buffer = data,
        .buffer_len = sizeof(data),
    };


    return twai_node_transmit(
        can_node,
        &frame,
        0);
}


/* ============================================================
 * MAIN
 * ============================================================ */

void app_main(void)
{
    printf("\n============================================================\n");
    printf("                    ESP32 SLAVE 4\n");
    printf("============================================================\n\n");


    printf("RTD 1 (SPI2): SCK=%d MOSI=%d MISO=%d CS=%d\n",
           RTD1_SCK,
           RTD1_MOSI,
           RTD1_MISO,
           RTD1_CS);


    printf("RTD 2 (SPI3): SCK=%d MOSI=%d MISO=%d CS=%d\n",
           RTD2_SCK,
           RTD2_MOSI,
           RTD2_MISO,
           RTD2_CS);


    printf("Hall ADC    : GPIO%d (ADC1_CH6)\n",
           HALL_ADC_GPIO);


    printf("CAN         : TX=GPIO%d RX=GPIO%d %d kbit/s\n\n",
           CAN_TX_GPIO,
           CAN_RX_GPIO,
           CAN_BITRATE / 1000);


    /* Existing sensor initialization */

    init_rtds();


    esp_err_t hall_err =
        init_hall();

    hall_ok =
        (hall_err == ESP_OK);


    if (!hall_ok) {

        ESP_LOGE(TAG,
                 "Hall setup failed: %s",
                 esp_err_to_name(hall_err));
    }


    /* New CAN initialization */

    can_ok =
        init_can();


    printf("\n============================================================\n");
    printf("                      SENSOR PACKETS\n");
    printf("============================================================\n");


    uint16_t sequence = 0;


    /*
     * Let the MAX31865 finish its first conversions.
     */

    vTaskDelay(
        pdMS_TO_TICKS(100));


    while (1)
    {
        sequence++;


        /* ----------------------------------------------------
         * Existing RTD readings
         * ---------------------------------------------------- */

        float temp1 = 0.0f;
        float temp2 = 0.0f;


        bool t1_valid =
            rtd1_ok &&
            (max31865_measure(
                &rtd1,
                &temp1) == ESP_OK);


        bool t2_valid =
            rtd2_ok &&
            (max31865_measure(
                &rtd2,
                &temp2) == ESP_OK);


        /* ----------------------------------------------------
         * Existing Hall reading
         * ---------------------------------------------------- */

        int hall_raw = 0;

        float hall_volts = 0.0f;


        bool h_valid =
            hall_ok &&
            hall_read(
                &hall_raw,
                &hall_volts);


        /* ----------------------------------------------------
         * Existing Serial output
         * ---------------------------------------------------- */

        printf("PACKET | "
               "RTD1=%.2f C | "
               "RTD2=%.2f C | "
               "HALL_RAW=%d | "
               "HALL_V=%.3f V\n",

               t1_valid ? temp1 : -999.0f,

               t2_valid ? temp2 : -999.0f,

               h_valid ? hall_raw : -1,

                h_valid ? hall_volts : -1.0f);


        /* ----------------------------------------------------
         * CAN transmission
         * ---------------------------------------------------- */

        if (can_ok)
        {
            esp_err_t can_err;


            can_err =
                can_send_rtds(
                    temp1,
                    temp2,
                    t1_valid,
                    t2_valid,
                    sequence);


            if (can_err != ESP_OK) {

                ESP_LOGE(TAG,
                         "CAN RTD transmit failed: %s",
                         esp_err_to_name(can_err));
            }


            can_err =
                can_send_hall(
                    hall_raw,
                    hall_volts,
                    h_valid,
                    sequence);


            if (can_err != ESP_OK) {

                ESP_LOGE(TAG,
                         "CAN Hall transmit failed: %s",
                         esp_err_to_name(can_err));
            }

        }


        vTaskDelay(
            pdMS_TO_TICKS(LOOP_PERIOD_MS));
    }
}

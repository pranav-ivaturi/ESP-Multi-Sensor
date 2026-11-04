/*
 * SPDX-License-Identifier: MIT
 *
 * Copyright (c) 2022 Carter Nelson for Adafruit Industries
 *
 * The default configuration values and sensor startup sequence are derived
 * from the MIT-licensed Adafruit CircuitPython VL53L4CD driver, which in turn
 * follows ST's VL53L4CD Ultra Lite Driver (STSW-IMG026 / UM2931).
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include "vl53l4cd.h"

#include <stdint.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define VL53L4CD_MODEL_ID_REG                 0x010F
#define VL53L4CD_EXPECTED_MODEL_ID             0xEB
#define VL53L4CD_EXPECTED_MODULE_TYPE          0xAA
#define VL53L4CD_FIRMWARE_SYSTEM_STATUS_REG    0x00E5
#define VL53L4CD_GPIO_HV_MUX_CTRL_REG          0x0030
#define VL53L4CD_GPIO_TIO_HV_STATUS_REG        0x0031
#define VL53L4CD_SYSTEM_INTERRUPT_CLEAR_REG    0x0086
#define VL53L4CD_SYSTEM_START_REG              0x0087
#define VL53L4CD_RESULT_RANGE_STATUS_REG       0x0089
#define VL53L4CD_RESULT_DISTANCE_REG           0x0096
#define VL53L4CD_OSC_FREQ_REG                  0x0006
#define VL53L4CD_RANGE_CONFIG_A_REG            0x005E
#define VL53L4CD_RANGE_CONFIG_B_REG            0x0061
#define VL53L4CD_INTERMEASUREMENT_REG          0x006C

#define VL53L4CD_CONFIGURATION_REG             0x002D
#define VL53L4CD_CONFIGURATION_SIZE             91
#define VL53L4CD_BOOT_TIMEOUT_MS                500
#define VL53L4CD_VHV_TIMEOUT_MS                 1000
#define VL53L4CD_I2C_TIMEOUT_MS                 100
#define VL53L4CD_TIMING_BUDGET_US               50000U

/* 0x002D through 0x0087. Fast-mode-plus is disabled because Slave 2 runs
 * the shared ADS1113/LiDAR bus at 400 kHz. */
static const uint8_t default_configuration[VL53L4CD_CONFIGURATION_SIZE] = {
    0x00, 0x00, 0x00, 0x11, 0x02, 0x00, 0x02, 0x08, 0x00, 0x08,
    0x10, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x0F,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x0B, 0x00, 0x00, 0x02,
    0x14, 0x21, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0xC8,
    0x00, 0x00, 0x38, 0xFF, 0x01, 0x00, 0x08, 0x00, 0x00, 0x01,
    0xCC, 0x07, 0x01, 0xF1, 0x05, 0x00, 0xA0, 0x00, 0x80, 0x08,
    0x38, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x89, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x07, 0x05, 0x06, 0x06,
    0x00, 0x00, 0x02, 0xC7, 0xFF, 0x9B, 0x00, 0x00, 0x00, 0x01,
    0x00,
};

static esp_err_t vl53l4cd_write(vl53l4cd_t *sensor, uint16_t reg,
                                 const uint8_t *data, size_t length)
{
    if (!sensor || !sensor->i2c || !data || length > VL53L4CD_CONFIGURATION_SIZE)
        return ESP_ERR_INVALID_ARG;

    uint8_t buffer[2 + VL53L4CD_CONFIGURATION_SIZE];
    buffer[0] = (uint8_t)(reg >> 8);
    buffer[1] = (uint8_t)reg;
    memcpy(&buffer[2], data, length);
    return i2c_master_transmit(sensor->i2c, buffer, length + 2,
                               VL53L4CD_I2C_TIMEOUT_MS);
}

static esp_err_t vl53l4cd_read(vl53l4cd_t *sensor, uint16_t reg,
                                uint8_t *data, size_t length)
{
    if (!sensor || !sensor->i2c || !data || !length)
        return ESP_ERR_INVALID_ARG;

    uint8_t address[2] = {(uint8_t)(reg >> 8), (uint8_t)reg};
    return i2c_master_transmit_receive(sensor->i2c, address, sizeof(address),
                                        data, length, VL53L4CD_I2C_TIMEOUT_MS);
}

static esp_err_t vl53l4cd_write_byte(vl53l4cd_t *sensor, uint16_t reg,
                                      uint8_t value)
{
    return vl53l4cd_write(sensor, reg, &value, 1);
}

static esp_err_t vl53l4cd_read_byte(vl53l4cd_t *sensor, uint16_t reg,
                                     uint8_t *value)
{
    return vl53l4cd_read(sensor, reg, value, 1);
}

static esp_err_t vl53l4cd_write_word(vl53l4cd_t *sensor, uint16_t reg,
                                      uint16_t value)
{
    uint8_t data[2] = {(uint8_t)(value >> 8), (uint8_t)value};
    return vl53l4cd_write(sensor, reg, data, sizeof(data));
}

static esp_err_t vl53l4cd_read_word(vl53l4cd_t *sensor, uint16_t reg,
                                     uint16_t *value)
{
    uint8_t data[2];
    esp_err_t err = vl53l4cd_read(sensor, reg, data, sizeof(data));
    if (err == ESP_OK)
        *value = ((uint16_t)data[0] << 8) | data[1];
    return err;
}

static esp_err_t vl53l4cd_clear_interrupt(vl53l4cd_t *sensor)
{
    return vl53l4cd_write_byte(sensor, VL53L4CD_SYSTEM_INTERRUPT_CLEAR_REG, 0x01);
}

static esp_err_t vl53l4cd_wait_for_boot(vl53l4cd_t *sensor)
{
    TickType_t start = xTaskGetTickCount();
    while ((xTaskGetTickCount() - start) < pdMS_TO_TICKS(VL53L4CD_BOOT_TIMEOUT_MS)) {
        uint8_t status;
        esp_err_t err = vl53l4cd_read_byte(sensor,
                                            VL53L4CD_FIRMWARE_SYSTEM_STATUS_REG,
                                            &status);
        if (err != ESP_OK)
            return err;
        if (status == 0x03)
            return ESP_OK;
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    return ESP_ERR_TIMEOUT;
}

static esp_err_t vl53l4cd_wait_for_data_ready(vl53l4cd_t *sensor,
                                               uint32_t timeout_ms)
{
    TickType_t start = xTaskGetTickCount();
    while ((xTaskGetTickCount() - start) < pdMS_TO_TICKS(timeout_ms)) {
        uint8_t mux_control;
        uint8_t gpio_status;

        esp_err_t err = vl53l4cd_read_byte(sensor, VL53L4CD_GPIO_HV_MUX_CTRL_REG,
                                            &mux_control);
        if (err != ESP_OK)
            return err;
        err = vl53l4cd_read_byte(sensor, VL53L4CD_GPIO_TIO_HV_STATUS_REG,
                                 &gpio_status);
        if (err != ESP_OK)
            return err;

        uint8_t active_state = (mux_control & 0x10) ? 0 : 1;
        if ((gpio_status & 0x01) == active_state)
            return ESP_OK;

        vTaskDelay(pdMS_TO_TICKS(2));
    }
    return ESP_ERR_TIMEOUT;
}

static esp_err_t vl53l4cd_set_timing_budget(vl53l4cd_t *sensor,
                                             uint32_t timing_budget_us)
{
    uint16_t osc_frequency;
    esp_err_t err = vl53l4cd_read_word(sensor, VL53L4CD_OSC_FREQ_REG,
                                       &osc_frequency);
    if (err != ESP_OK)
        return err;
    if (osc_frequency == 0 || timing_budget_us <= 2500)
        return ESP_ERR_INVALID_RESPONSE;

    uint64_t macro_period_us =
        ((uint64_t)2304 * (0x40000000ULL / osc_frequency)) >> 6;
    uint64_t timing_budget_scaled = ((uint64_t)timing_budget_us - 2500U) << 12;
    uint64_t denominator_a = (macro_period_us * 16U) >> 6;
    uint64_t denominator_b = (macro_period_us * 12U) >> 6;
    if (denominator_a == 0 || denominator_b == 0)
        return ESP_ERR_INVALID_RESPONSE;

    uint32_t ls_byte =
        (uint32_t)((timing_budget_scaled + (denominator_a / 2U)) / denominator_a) - 1U;
    uint16_t ms_byte = 0;
    while ((ls_byte & 0xFFFFFF00U) != 0) {
        ls_byte >>= 1;
        ms_byte++;
    }
    err = vl53l4cd_write_word(sensor, VL53L4CD_RANGE_CONFIG_A_REG,
                               (uint16_t)((ms_byte << 8) | ls_byte));
    if (err != ESP_OK)
        return err;

    ls_byte =
        (uint32_t)((timing_budget_scaled + (denominator_b / 2U)) / denominator_b) - 1U;
    ms_byte = 0;
    while ((ls_byte & 0xFFFFFF00U) != 0) {
        ls_byte >>= 1;
        ms_byte++;
    }
    return vl53l4cd_write_word(sensor, VL53L4CD_RANGE_CONFIG_B_REG,
                                (uint16_t)((ms_byte << 8) | ls_byte));
}

esp_err_t vl53l4cd_init(vl53l4cd_t *sensor, i2c_master_dev_handle_t i2c)
{
    if (!sensor || !i2c)
        return ESP_ERR_INVALID_ARG;

    memset(sensor, 0, sizeof(*sensor));
    sensor->i2c = i2c;

    uint8_t model[2];
    esp_err_t err = vl53l4cd_read(sensor, VL53L4CD_MODEL_ID_REG, model,
                                   sizeof(model));
    if (err != ESP_OK)
        return err;
    if (model[0] != VL53L4CD_EXPECTED_MODEL_ID ||
        model[1] != VL53L4CD_EXPECTED_MODULE_TYPE)
        return ESP_ERR_NOT_FOUND;

    err = vl53l4cd_wait_for_boot(sensor);
    if (err != ESP_OK)
        return err;

    err = vl53l4cd_write(sensor, VL53L4CD_CONFIGURATION_REG,
                          default_configuration, sizeof(default_configuration));
    if (err != ESP_OK)
        return err;

    /* Run the mandatory VHV sequence before applying the final timing setup. */
    err = vl53l4cd_write_byte(sensor, VL53L4CD_SYSTEM_START_REG, 0x40);
    if (err != ESP_OK)
        return err;
    err = vl53l4cd_wait_for_data_ready(sensor, VL53L4CD_VHV_TIMEOUT_MS);
    if (err != ESP_OK)
        return err;
    err = vl53l4cd_clear_interrupt(sensor);
    if (err != ESP_OK)
        return err;
    err = vl53l4cd_write_byte(sensor, VL53L4CD_SYSTEM_START_REG, 0x00);
    if (err != ESP_OK)
        return err;

    const uint8_t intermeasurement[4] = {0, 0, 0, 0};
    const uint8_t vhv_settings[2] = {0x05, 0x00};
    err = vl53l4cd_write_byte(sensor, 0x0008, 0x09);
    if (err != ESP_OK)
        return err;
    err = vl53l4cd_write_byte(sensor, 0x000B, 0x00);
    if (err != ESP_OK)
        return err;
    err = vl53l4cd_write(sensor, 0x0024, vhv_settings, sizeof(vhv_settings));
    if (err != ESP_OK)
        return err;
    err = vl53l4cd_write(sensor, VL53L4CD_INTERMEASUREMENT_REG,
                          intermeasurement, sizeof(intermeasurement));
    if (err != ESP_OK)
        return err;
    err = vl53l4cd_set_timing_budget(sensor, VL53L4CD_TIMING_BUDGET_US);
    if (err != ESP_OK)
        return err;

    err = vl53l4cd_write_byte(sensor, VL53L4CD_SYSTEM_START_REG, 0x21);
    if (err == ESP_OK)
        sensor->ranging = true;
    return err;
}

esp_err_t vl53l4cd_read_distance_mm(vl53l4cd_t *sensor,
                                    int *distance_mm,
                                    uint32_t timeout_ms)
{
    if (!sensor || !distance_mm || !sensor->ranging)
        return ESP_ERR_INVALID_STATE;

    esp_err_t err = vl53l4cd_wait_for_data_ready(sensor, timeout_ms);
    if (err != ESP_OK)
        return err;

    uint8_t range_status;
    uint16_t raw_distance;
    err = vl53l4cd_read_byte(sensor, VL53L4CD_RESULT_RANGE_STATUS_REG,
                             &range_status);
    if (err == ESP_OK)
        err = vl53l4cd_read_word(sensor, VL53L4CD_RESULT_DISTANCE_REG,
                                 &raw_distance);

    esp_err_t clear_err = vl53l4cd_clear_interrupt(sensor);
    if (err != ESP_OK)
        return err;
    if (clear_err != ESP_OK)
        return clear_err;

    if ((range_status & 0x1F) != 0)
        return ESP_ERR_INVALID_RESPONSE;

    *distance_mm = (int)raw_distance;
    return ESP_OK;
}

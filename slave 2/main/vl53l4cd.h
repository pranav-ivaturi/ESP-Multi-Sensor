/*
 * SPDX-License-Identifier: MIT
 *
 * Minimal ESP-IDF VL53L4CD ranging driver. The initialization register
 * sequence follows ST's VL53L4CD Ultra Lite Driver (STSW-IMG026 / UM2931).
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/i2c_master.h"
#include "esp_err.h"

typedef struct {
    i2c_master_dev_handle_t i2c;
    bool ranging;
} vl53l4cd_t;

/** Initialise the sensor and start continuous ranging with a 50 ms budget. */
esp_err_t vl53l4cd_init(vl53l4cd_t *sensor, i2c_master_dev_handle_t i2c);

/** Wait for a valid measurement and return its distance in millimetres. */
esp_err_t vl53l4cd_read_distance_mm(vl53l4cd_t *sensor,
                                    int *distance_mm,
                                    uint32_t timeout_ms);

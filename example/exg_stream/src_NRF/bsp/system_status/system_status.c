/*
 * ----------------------------------------------------------------------
 *
 * File: system_status.c
 *
 * Last edited: 05.01.2026
 *
 * Copyright (C) 2026, ETH Zurich and University of Bologna.
 *
 * ----------------------------------------------------------------------
 * SPDX-License-Identifier: Apache-2.0
 */

#include "bsp/system_status/system_status.h"
#include "bsp/battery/battery.h"
#include "sensors/imu/imu_appl.h"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(system_status, LOG_LEVEL_INF);

// BLE Command for battery state (must match ble_appl.h / protocol)
#define REQUEST_BATTERY_STATE 0x01

int system_status_init(void) {
    LOG_INF("System Status aggregator initialized");
    return 0;
}

int system_status_build_ble_packet(uint8_t *buffer, size_t buf_size, size_t *out_len) {
    if (buf_size < 7) {
        return -1; // Buffer too small for standard 7-byte status packet
    }

    // Get battery status
    battery_status_t bat_status;
    battery_get_status(&bat_status);

    // Get temperature from IMU
    float temp_celsius = 0.0f;
    int temp_ret = imu_read_temperature(&temp_celsius);

    // Build packet matching original format
    buffer[0] = REQUEST_BATTERY_STATE;
    buffer[1] = bat_status.is_charging ? 1 : 0;
    buffer[2] = 0; // Reserved/Padding
    buffer[3] = (uint8_t)bat_status.power_mw;
    buffer[4] = bat_status.soc_percent;
    buffer[5] = (uint8_t)bat_status.voltage_mv; // Note: This truncates, matching original behavior
    
    if (temp_ret == 0) {
        buffer[6] = (uint8_t)temp_celsius;
    } else {
        buffer[6] = 0;
    }

    *out_len = 7;
    return 0;
}

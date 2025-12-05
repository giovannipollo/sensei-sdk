/*
 * ----------------------------------------------------------------------
 *
 * File: ads_spi.h
 *
 * Last edited: 23.07.2025
 *
 * Copyright (C) 2025, ETH Zurich
 *
 * Authors:
 * - Sebastian Frey (sefrey@iis.ee.ethz.ch), ETH Zurich
 *
 * ----------------------------------------------------------------------
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef ADS_SPI_H
#define ADS_SPI_H

#include "ads_appl.h"
#include "ads_defs.h"
#include <stdbool.h>
#include <stdint.h>

/*==============================================================================
 * Global Variables
 *============================================================================*/

/**
 * @brief SPI command/data buffer
 *
 * Used for constructing SPI transactions. First byte typically contains
 * the command, followed by register addresses and data.
 */
extern uint8_t pr_word[10];

/**
 * @brief ADS1298 initialization status flag
 *
 * Set to true after both ADS1298 devices have been successfully initialized
 * and verified. Used to prevent data acquisition before initialization.
 */
extern bool ads_initialized;

/*==============================================================================
 * Function Declarations - Initialization
 *============================================================================*/

/**
 * @brief Initialize SPI peripheral and GPIO pins
 *
 * Configures the SPIM instance with appropriate settings for ADS1298 communication:
 * - SPI Mode 1 (CPOL=0, CPHA=1)
 * - 4 MHz clock frequency
 * - MSB first bit order
 * - Separate CS pins for each ADS1298 device
 * - START pin for synchronized acquisition
 *
 * @note Must be called before any other ADS functions
 */
void init_SPI();

/*==============================================================================
 * Function Declarations - Data Acquisition Control
 *============================================================================*/

/**
 * @brief Start data acquisition on both ADS1298 devices
 *
 * Sends START and RDATAC commands to both devices to begin continuous
 * data conversion and streaming. The devices are synchronized via the
 * shared START pin.
 */
void ADS_Start();

/**
 * @brief Stop data acquisition on both ADS1298 devices
 *
 * Sends SDATAC command to both devices to stop continuous data mode.
 * Sets skip_reads flag to discard initial samples after restart.
 */
void ADS_Stop();

/**
 * @brief Clear the skip_reads flag and reset skipped sample counter
 *
 * Call this after manually waiting for ADC settling (e.g., during
 * synchronized streaming) to prevent additional sample skipping
 * in process_ads_data().
 */
void ADS_clear_skip_reads();

/**
 * @brief Initialize an ADS1298 device with specified parameters
 *
 * Configures the ADS1298 registers for data acquisition:
 * - Sample rate configuration
 * - Channel gain and input settings
 * - Reference buffer settings
 * - Test signal configuration (if enabled)
 *
 * @param InitParams Pointer to initialization parameter array:
 *                   [0]: Sample rate code (0-6, see datasheet)
 *                   [1]: Channel input configuration
 *                   [2]: Reserved
 *                   [3]: Reserved
 *                   [4]: Channel gain code
 * @param ads_id Device identifier (ADS1298_A or ADS1298_B)
 *
 * @note Device must pass ID check before initialization proceeds
 */
void ADS_Init(uint8_t *InitParams, enum ADS_id_t ads_id);

/**
 * @brief Verify ADS1298 device ID
 *
 * Reads the device ID register and verifies it matches the expected value (0xD2).
 * Enters infinite loop with error logging if ID check fails.
 *
 * @param ads_id Device identifier (ADS1298_A or ADS1298_B)
 */
void ADS_check_ID(enum ADS_id_t ads_id);


/*==============================================================================
 * Function Declarations - Data Ready Handling
 *============================================================================*/

/**
 * @brief Initialize data ready (DRDY) interrupt
 *
 * Configures GPIO interrupt for the ADS1298_A DRDY pin. This pin signals
 * when new conversion data is available. Both devices share the same DRDY
 * signal due to synchronized operation.
 *
 * @return 0 on success, negative error code on failure
 */
int ADS_dr_init();

/**
 * @brief Read current state of DRDY pin
 *
 * @return 1 if data ready, 0 if not ready
 */
int ADS_dr_read();

/**
 * @brief Process ADS1298 data when DRDY interrupt occurs
 *
 * Called from main loop to handle new data availability:
 * 1. Reads data from both ADS1298 devices via SPI
 * 2. Combines with PPG data if enabled
 * 3. Appends IMU accelerometer data
 * 4. Constructs BLE packets for transmission
 * 5. Manages packet counters and triggers
 *
 * @note This function checks the ads_data_ready flag set by the DRDY interrupt
 */
void process_ads_data();

#endif // ADS_SPI_H
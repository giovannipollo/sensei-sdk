/*
 * ----------------------------------------------------------------------
 *
 * File: ads_appl.c
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

/**
 * @file ads_appl.c
 * @brief ADS1298 Application Layer Implementation
 *
 * Implements the application-level state machine and control logic for the
 * ADS1298 bio-potential AFE. This module acts as a bridge between the BLE
 * command interface and the low-level SPI driver.
 */

#include "afe/ads_appl.h"

#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>

/* Initialize the logging module */
LOG_MODULE_REGISTER(ads_appl, LOG_LEVEL_INF);

/*==============================================================================
 * Module Variables
 *============================================================================*/

/**
 * @brief Current application state
 *
 * Initialized to STILL (idle) state. Modified by BLE commands and error handlers.
 */
enum ADS_function_t ADS_function = STILL;

/**
 * @brief Trigger value for event marking
 *
 * This value is embedded in each data packet to mark specific events or stimuli.
 * Initialized to 0x00 (no trigger active).
 */
static int trigger_value = 0x00;

/*==============================================================================
 * Public Functions
 *============================================================================*/

/**
 * @brief Get the current ADS application function state
 *
 * Thread-safe getter for the global application state.
 *
 * @return Current ADS_function_t state
 */
enum ADS_function_t Get_ADS_Function() { return (ADS_function); }

/**
 * @brief Set the ADS application function state
 *
 * Updates the global state machine. This function is called from:
 * - BLE command handlers to change operating mode
 * - Error handlers to stop acquisition
 * - Main application logic for state transitions
 *
 * @param f New function state to set
 *
 * @note This function does not perform the state transition itself,
 *       it only updates the state variable. The actual transition is
 *       handled by the main loop or interrupt handlers.
 */
void Set_ADS_Function(enum ADS_function_t f) { 
    LOG_DBG("Setting ADS function to %d", f);
    ADS_function = f; 
}

/**
 * @brief Set the trigger value for sample synchronization
 *
 * The trigger value is used to mark specific samples in the data stream.
 * Typical use cases include:
 * - Event-related potential (ERP) studies
 * - Stimulus presentation marking
 * - Synchronization with external devices
 *
 * @param value Trigger value to set (0x00-0xFF)
 *
 * @note The trigger value is automatically embedded in each BLE packet
 *       and reset after transmission (if needed by application logic).
 */
void set_trigger(uint8_t value) { 
    LOG_DBG("Setting trigger value to %d", value);
    trigger_value = value; 
}

/**
 * @brief Get the current trigger value
 *
 * Retrieves the trigger value that will be embedded in the next data packet.
 * This function is called by the data packaging routine in ads_spi.c.
 *
 * @return Current trigger value (0x00-0xFF)
 */
uint8_t get_trigger() { return trigger_value; }

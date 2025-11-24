/*
 * ----------------------------------------------------------------------
 *
 * File: ads_appl.h
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
 * @file ads_appl.h
 * @brief ADS1298 Application Layer Interface
 *
 * This module provides high-level application control for the ADS1298 bio-potential
 * analog front-end (AFE) devices. It manages the application state machine and
 * trigger synchronization for EEG/EMG/ExG data acquisition.
 */

#ifndef ADS_APPL_H
#define ADS_APPL_H

#include <stdbool.h>
#include <stdint.h>

/*==============================================================================
 * Type Definitions
 *============================================================================*/

/**
 * @enum ADS_function_t
 * @brief Application state machine functions for ADS1298 control
 *
 * Defines the various operational states and commands that control the
 * behavior of the ADS1298 devices and the overall system.
 */

enum ADS_function_t {
  READ,           /**< Continuous data reading mode */
  START,          /**< Start data acquisition */
  STOP,           /**< Stop data acquisition */
  STILL,          /**< Idle state, no active operation */
  INIT_GAP9_CTRL, /**< Initialize GAP9 processor control */
  WOLF_CTRL,      /**< Biowolf system control */
  CONNECT,        /**< Establish connection */
  RESTART_WOLF,   /**< Restart Biowolf system */
  TOGGLE_DRDY,    /**< Toggle data ready pin monitoring */
  READ_BATTERY,   /**< Read battery status */
  PROGRAM_WOLF,   /**< Program Biowolf firmware */
  ES_QUALITY      /**< Electrode-skin quality measurement */
};

/**
 * @enum ADS_id_t
 * @brief ADS1298 device identifier
 *
 * Used to distinguish between the two ADS1298 chips in the dual-AFE configuration.
 */
enum ADS_id_t { ADS1298_A, ADS1298_B };

/*==============================================================================
 * Global Variables
 *============================================================================*/

/**
 * @brief Current application state for ADS1298 control
 *
 * This variable tracks the current operational state of the system.
 * It is used by various modules to coordinate data acquisition and processing.
 */
extern enum ADS_function_t ADS_function;

/*==============================================================================
 * Function Declarations
 *============================================================================*/

/**
 * @brief Get the current ADS application function state
 *
 * @return Current ADS_function_t state
 */
enum ADS_function_t Get_ADS_Function();

/**
 * @brief Set the ADS application function state
 *
 * Updates the global state machine to control system behavior.
 * This function is typically called from BLE command handlers or
 * error handling routines.
 *
 * @param f New function state to set
 */
void Set_ADS_Function(enum ADS_function_t f);

/**
 * @brief Set the trigger value for sample synchronization
 *
 * The trigger value is embedded in each data packet and can be used to
 * mark specific events or stimuli for time-locked analysis.
 *
 * @param value Trigger value to set (0x00-0xFF)
 */
void set_trigger(uint8_t value);

/**
 * @brief Get the current trigger value
 *
 * Retrieves the trigger value that will be embedded in the next data packet.
 *
 * @return Current trigger value (0x00-0xFF)
 */
uint8_t get_trigger();

#endif // ADS_APPL_H
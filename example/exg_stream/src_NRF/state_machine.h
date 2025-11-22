/*
 * ----------------------------------------------------------------------
 *
 * File: state_machine.h
 *
 * Last edited: 20.11.2025
 *
 * Copyright (c) 2024 ETH Zurich and University of Bologna
 *
 * Authors:
 * - Philip Wiese (wiesep@iis.ee.ethz.ch), ETH Zurich
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
 * @file state_machine.h
 * @brief State machine management for ExG streaming application
 *
 * This module implements a finite state machine that manages different
 * operational modes of the device, including power states, streaming modes,
 * and BLE control states.
 *
 * The state machine runs in a dedicated thread and provides thread-safe
 * state transitions through setter/getter functions. Each state implements
 * three callbacks (entry, run, exit) that are automatically invoked during
 * state transitions and operation.
 */

#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

/*===========================================================================*/
/* Includes                                                                  */
/*===========================================================================*/

#include <stdint.h>

/*===========================================================================*/
/* State Definitions                                                         */
/*===========================================================================*/

/**
 * @brief State machine states enumeration
 *
 * Defines all possible states for the device state machine. States are
 * ordered by power consumption level (highest to lowest power).
 */
typedef enum {
  /** @brief System shutdown state - complete power down */
  S_SHUTDOWN,
  
  /** @brief Deep sleep state - minimal power consumption, wake on interrupt */
  S_DEEPSLEEP,
  
  /** @brief Low power connected state - BLE connection maintained with minimal power */
  S_LOW_POWER_CONNECTED,
  
  /** @brief Nordic streaming state - Active ExG data acquisition and BLE transmission */
  S_NORDIC_STREAM,
  
  /** @brief GAP control state - BLE Generic Access Profile management */
  S_GAP_CTRL,
  
  /** @brief Maximum number of states (used for array sizing and validation) */
  S_MAX_STATES
} State_t;

/*===========================================================================*/
/* Configuration                                                             */
/*===========================================================================*/

/** @brief Stack size for the state machine thread in bytes */
#define STATE_MACHINE_STACK_SIZE 2048

/** @brief Priority level for the state machine thread (cooperative scheduling) */
#define STATE_MACHINE_PRIORITY 7

/*===========================================================================*/
/* Public API - Initialization                                               */
/*===========================================================================*/

/**
 * @brief Initialize the state machine
 *
 * Sets up the initial state to S_LOW_POWER_CONNECTED and prepares the
 * state machine for operation. This function must be called before
 * state_machine_start().
 *
 * @note This does not start the state machine thread execution.
 *       Call state_machine_start() after all system initialization is complete.
 */
void state_machine_init(void);

/**
 * @brief Start the state machine thread execution
 *
 * Signals the state machine thread to begin running the state loop.
 * The thread will continuously execute the run() function of the current
 * state with a 1us sleep between iterations.
 *
 * @note Call this function only after state_machine_init() and all other
 *       required system initialization is complete.
 */
void state_machine_start(void);

/*===========================================================================*/
/* Public API - State Control                                                */
/*===========================================================================*/

/**
 * @brief Transition to a new state
 *
 * Performs a state transition by calling the exit callback of the current
 * state, updating the state variable, and calling the entry callback of
 * the new state.
 *
 * @param new_state The target state to transition to
 *
 * @warning State transitions are not atomic. Ensure proper synchronization
 *          if called from multiple threads.
 */
void set_SM_state(State_t new_state);

/**
 * @brief Get the current state
 *
 * Returns the currently active state of the state machine.
 *
 * @return Current State_t value
 */
State_t get_SM_state(void);

#endif // STATE_MACHINE_H

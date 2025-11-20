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

#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include "common.h"

// State machine configuration
#define STATE_MACHINE_STACK_SIZE 2048
#define STATE_MACHINE_PRIORITY 7

/**
 * @brief Initialize the state machine
 *
 * Sets up the initial state and prepares the state machine thread.
 */
void state_machine_init(void);

/**
 * @brief Start the state machine thread
 *
 * Signals the state machine thread to begin execution.
 */
void state_machine_start(void);

#endif // STATE_MACHINE_H

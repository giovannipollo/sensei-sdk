/*
 * ----------------------------------------------------------------------
 *
 * File: state_machine.c
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

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "state_machine.h"
#include "board_streaming.h"
#include "bsp/pwr_bsp.h"
#include "pwr/pwr.h"

LOG_MODULE_REGISTER(state_machine, LOG_LEVEL_INF);

// Function pointers for state behavior
typedef void (*StateFuncEntry)(void);
typedef void (*StateFuncRun)(void);
typedef void (*StateFuncExit)(void);

// State structure
typedef struct {
  StateFuncEntry entry;
  StateFuncRun run;
  StateFuncExit exit;
} StateMachine_t;

// Global variable to track the current state
static State_t current_state = S_LOW_POWER_CONNECTED;

// State machine synchronization
static K_SEM_DEFINE(state_machine_ready_sem, 0, 1);

// ==================== State Functions ====================

// S_SHUTDOWN
static void s_shutdown_entry(void) { LOG_INF("Entering SHUTDOWN state"); }
static void s_shutdown_run(void) { /* Logic for shutdown */ }
static void s_shutdown_exit(void) { LOG_INF("Exiting SHUTDOWN state"); }

// S_DEEPSLEEP
static void s_deepsleep_entry(void) { LOG_INF("Entering DEEPSLEEP state"); }
static void s_deepsleep_run(void) { /* Logic for deep sleep */ }
static void s_deepsleep_exit(void) { LOG_INF("Exiting DEEPSLEEP state"); }

// S_LOW_POWER_CONNECTED
static void s_low_power_connected_entry(void) { LOG_INF("Entering LOW POWER CONNECTED state"); }
static void s_low_power_connected_run(void) { /* Logic for low power */ }
static void s_low_power_connected_exit(void) { LOG_INF("Exiting LOW POWER CONNECTED state"); }

// S_NORDIC_STREAM
static void s_nordic_stream_entry(void) {
  // Depending on the ExG shield to use...
  // pwr_ads_on_bipolar();
  pwr_ads_on_unipolar();
}
static void s_nordic_stream_run(void) { loop_streaming(); }
static void s_nordic_stream_exit(void) {
  // LOG_INF("Exiting NORDIC STREAM state");
  pwr_ads_off();
}

// S_GAP_CTRL
static void s_gap_ctrl_entry(void) { LOG_INF("Entering GAP CTRL state"); }
static void s_gap_ctrl_run(void) { /* BLE control logic */ }
static void s_gap_ctrl_exit(void) { LOG_INF("Exiting GAP CTRL state"); }

// ==================== State Machine Definition ====================
static const StateMachine_t state_machine[S_MAX_STATES] = {
    {s_shutdown_entry, s_shutdown_run, s_shutdown_exit},
    {s_deepsleep_entry, s_deepsleep_run, s_deepsleep_exit},
    {s_low_power_connected_entry, s_low_power_connected_run, s_low_power_connected_exit},
    {s_nordic_stream_entry, s_nordic_stream_run, s_nordic_stream_exit},
    {s_gap_ctrl_entry, s_gap_ctrl_run, s_gap_ctrl_exit}};

// ==================== Public Functions ====================

void set_SM_state(State_t new_state) {
  if (new_state < S_MAX_STATES) {
    state_machine[current_state].exit();  // Call exit function of current state
    current_state = new_state;            // Update current state
    state_machine[current_state].entry(); // Call entry function of new state
  } else {
    LOG_ERR("Invalid state: %d", new_state);
  }
}

State_t get_SM_state(void) {
  return current_state; // Return the current state
}

// ==================== State Machine Thread ====================

static void state_machine_thread() {
  // Wait for initialization to complete
  LOG_INF("State machine thread wants to start");
  k_sem_take(&state_machine_ready_sem, K_FOREVER);
  LOG_INF("State machine thread started");

  while (1) {
    state_machine[current_state].run();
    // k_cpu_idle(); // Use idle instead of sleep to stay responsive
    // Sleep for 1us to yield CPU
    k_sleep(K_USEC(1));
  }
}

K_THREAD_DEFINE(state_machine_thread_id, STATE_MACHINE_STACK_SIZE, state_machine_thread, NULL, NULL, NULL,
                STATE_MACHINE_PRIORITY, 0, 0);

// ==================== Initialization ====================

void state_machine_init(void) {
  // Set initial state
  current_state = S_LOW_POWER_CONNECTED;
  LOG_INF("State machine initialized to state: %d", current_state);
}

void state_machine_start(void) {
  // Signal state machine can start
  k_sem_give(&state_machine_ready_sem);
  LOG_INF("State machine started");
}

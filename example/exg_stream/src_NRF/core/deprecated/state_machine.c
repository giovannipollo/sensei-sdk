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

/**
 * @file state_machine.c
 * @brief Implementation of the finite state machine for ExG streaming
 *
 * This file implements a thread-based state machine that manages device
 * operation modes. Each state has three callbacks: entry, run, and exit.
 * The state machine thread continuously executes the run() callback of
 * the current state.
 *
 * @see state_machine.h for state definitions and public API
 */

/*===========================================================================*/
/* Includes                                                                  */
/*===========================================================================*/

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "bsp/pwr_bsp.h"
#include "core/board_streaming.h"
#include "core/state_machine.h"

/*===========================================================================*/
/* Module Configuration                                                      */
/*===========================================================================*/

LOG_MODULE_REGISTER(state_machine, LOG_LEVEL_INF);

/*===========================================================================*/
/* Type Definitions                                                          */
/*===========================================================================*/

/** @brief Function pointer type for state entry callback */
typedef void (*StateFuncEntry)(void);

/** @brief Function pointer type for state run callback (executed continuously) */
typedef void (*StateFuncRun)(void);

/** @brief Function pointer type for state exit callback */
typedef void (*StateFuncExit)(void);

/**
 * @brief State machine structure defining state behavior
 *
 * Each state has three callbacks:
 * - entry: Called once when entering the state
 * - run: Called continuously while in the state
 * - exit: Called once when leaving the state
 */
typedef struct {
  StateFuncEntry entry; /**< Entry callback function */
  StateFuncRun run;     /**< Run callback function */
  StateFuncExit exit;   /**< Exit callback function */
} StateMachine_t;

/*===========================================================================*/
/* Private Variables                                                         */
/*===========================================================================*/

/** @brief Current active state of the state machine */
static State_t current_state = S_LOW_POWER_CONNECTED;

/** @brief Semaphore for synchronizing state machine thread startup */
static K_SEM_DEFINE(state_machine_ready_sem, 0, 1);

/*===========================================================================*/
/* State Callback Functions                                                  */
/*===========================================================================*/

/*---------------------------------------------------------------------------*/
/* S_SHUTDOWN State                                                          */
/*---------------------------------------------------------------------------*/

/**
 * @brief Entry callback for SHUTDOWN state
 *
 * Logs entry into shutdown state. System prepares for complete power down.
 */
static void s_shutdown_entry(void) { LOG_INF("Entering SHUTDOWN state"); }

/**
 * @brief Run callback for SHUTDOWN state
 *
 * Placeholder for shutdown logic. Could include final cleanup operations.
 */
static void s_shutdown_run(void) { LOG_DBG("Running SHUTDOWN state"); }

/**
 * @brief Exit callback for SHUTDOWN state
 *
 * Logs exit from shutdown state (typically not reached).
 */
static void s_shutdown_exit(void) { LOG_INF("Exiting SHUTDOWN state"); }

/*---------------------------------------------------------------------------*/
/* S_DEEPSLEEP State                                                         */
/*---------------------------------------------------------------------------*/

/**
 * @brief Entry callback for DEEPSLEEP state
 *
 * Placesholder for deep sleep setup logic.
 */
static void s_deepsleep_entry(void) { LOG_INF("Entering DEEPSLEEP state"); }

/**
 * @brief Run callback for DEEPSLEEP state
 *
 * Placeholder for deep sleep logic. Could implement actual sleep routines.
 */
static void s_deepsleep_run(void) { LOG_DBG("Running DEEPSLEEP state"); }

/**
 * @brief Exit callback for DEEPSLEEP state
 *
 * Placesholder for exit logic from deep sleep state.
 */
static void s_deepsleep_exit(void) { LOG_INF("Exiting DEEPSLEEP state"); }

/*---------------------------------------------------------------------------*/
/* S_LOW_POWER_CONNECTED State                                               */
/*---------------------------------------------------------------------------*/

/**
 * @brief Entry callback for LOW_POWER_CONNECTED state
 *
 * Placeholder for low power connection setup logic.
 */
static void s_low_power_connected_entry(void) { LOG_INF("Entering LOW POWER CONNECTED state"); }

/**
 * @brief Run callback for LOW_POWER_CONNECTED state
 *
 * Placeholder for low power connection maintenance logic.
 */
static void s_low_power_connected_run(void) { LOG_DBG("Running LOW POWER CONNECTED state"); }

/**
 * @brief Exit callback for LOW_POWER_CONNECTED state
 *
 * Placeholder for exit logic from low power connected state.
 */
static void s_low_power_connected_exit(void) { LOG_INF("Exiting LOW POWER CONNECTED state"); }

/*---------------------------------------------------------------------------*/
/* S_NORDIC_STREAM State                                                     */
/*---------------------------------------------------------------------------*/

/**
 * @brief Entry callback for NORDIC_STREAM state
 *
 * Powers on the ADS (Analog-to-Digital Sensor) for ExG signal acquisition.
 * The mode (bipolar or unipolar) is determined by CONFIG_ADS_USE_BIPOLAR_MODE.
 *
 * - Bipolar mode (EMG): Differential measurement between two electrodes
 * - Unipolar mode (EEG): Single-ended measurement referenced to ground
 */
static void s_nordic_stream_entry(void) { LOG_INF("Entering NORDIC STREAM state"); }

/**
 * @brief Run callback for NORDIC_STREAM state
 *
 * Continuously executes the streaming loop to acquire and transmit ExG data.
 */
static void s_nordic_stream_run(void) { LOG_INF("Running NORDIC STREAM state"); }

/**
 * @brief Exit callback for NORDIC_STREAM state
 *
 * Powers off the ADS to conserve energy when streaming is no longer needed.
 */
static void s_nordic_stream_exit(void) { LOG_INF("Exiting NORDIC STREAM state"); }

/*---------------------------------------------------------------------------*/
/* S_GAP_CTRL State                                                          */
/*---------------------------------------------------------------------------*/

/**
 * @brief Entry callback for GAP_CTRL state
 *
 * Placeholder for BLE GAP control setup logic.
 */
static void s_gap_ctrl_entry(void) { LOG_INF("Entering GAP CTRL state"); }

/**
 * @brief Run callback for GAP_CTRL state
 *
 * Placeholder for BLE GAP control logic (advertising, connections, etc.).
 */
static void s_gap_ctrl_run(void) { LOG_DBG("Running GAP CTRL state"); }

/**
 * @brief Exit callback for GAP_CTRL state
 *
 * Placeholder for cleanup logic when exiting GAP control state.
 */
static void s_gap_ctrl_exit(void) { LOG_INF("Exiting GAP CTRL state"); }

/*===========================================================================*/
/* State Machine Definition                                                  */
/*===========================================================================*/

/**
 * @brief State machine lookup table
 *
 * Maps each state enum to its corresponding entry, run, and exit callbacks.
 * The array index corresponds to the State_t enum value.
 */
static const StateMachine_t state_machine[S_MAX_STATES] = {
    {s_shutdown_entry, s_shutdown_run, s_shutdown_exit},
    {s_deepsleep_entry, s_deepsleep_run, s_deepsleep_exit},
    {s_low_power_connected_entry, s_low_power_connected_run, s_low_power_connected_exit},
    {s_nordic_stream_entry, s_nordic_stream_run, s_nordic_stream_exit},
    {s_gap_ctrl_entry, s_gap_ctrl_run, s_gap_ctrl_exit}};

/*===========================================================================*/
/* Public API Implementation                                                 */
/*===========================================================================*/

/**
 * @brief Transition to a new state
 *
 * Performs a thread-safe state transition by calling the exit callback of
 * the current state, updating the state variable, and calling the entry
 * callback of the new state.
 *
 * @param new_state The target state to transition to
 *
 * @note This function is declared in common.h for access by other modules.
 *       State transitions are not atomic - ensure proper synchronization
 *       if called from multiple threads.
 */
void set_SM_state(State_t new_state) {
  // if (new_state < S_MAX_STATES) {
  //   LOG_INF("Transitioning from state %d to state %d", current_state, new_state);
  //   state_machine[current_state].exit();  /* Call exit function of current state */
  //   current_state = new_state;            /* Update current state */
  //   state_machine[current_state].entry(); /* Call entry function of new state */
  // } else {
  //   LOG_ERR("Invalid state: %d", new_state);
  // }
}

/**
 * @brief Get the current state
 *
 * Returns the currently active state of the state machine.
 *
 * @return Current State_t value
 *
 * @note This function is declared in common.h for access by other modules.
 */
State_t get_SM_state(void) { return current_state; }

/*===========================================================================*/
/* State Machine Thread                                                      */
/*===========================================================================*/

/**
 * @brief State machine thread function
 *
 * Implements the main state machine loop. Waits for initialization signal,
 * then continuously executes the run() callback of the current state.
 *
 * Between iterations, the behavior depends on CONFIG_STATE_MACHINE_USE_CPU_IDLE:
 * - If enabled: Uses k_cpu_idle() for maximum responsiveness (higher power)
 * - If disabled: Uses k_sleep(1us) for better power efficiency
 *
 * @param unused1 Unused parameter (required by thread signature)
 * @param unused2 Unused parameter (required by thread signature)
 * @param unused3 Unused parameter (required by thread signature)
 */
// static void state_machine_thread(void *unused1, void *unused2, void *unused3) {
//   ARG_UNUSED(unused1);
//   ARG_UNUSED(unused2);
//   ARG_UNUSED(unused3);

//   /* Wait for initialization to complete */
//   LOG_INF("State machine thread wants to start");
//   k_sem_take(&state_machine_ready_sem, K_FOREVER);
//   LOG_INF("State machine thread started");

//   /* Main state machine loop */
//   while (1) {
//     state_machine[current_state].run();

// #ifdef CONFIG_STATE_MACHINE_USE_CPU_IDLE
//     /* Use CPU idle - more responsive but higher power consumption */
//     k_cpu_idle();
// #else
//     /* Use sleep - better power efficiency, yields CPU to other threads */
//     k_sleep(K_USEC(1));
// #endif
//   }
// }

/**
 * @brief Define and create the state machine thread
 *
 * Creates a Zephyr thread with specified stack size and priority.
 * The thread is created at boot but waits on the semaphore before starting.
 */
// K_THREAD_DEFINE(state_machine_thread_id, STATE_MACHINE_STACK_SIZE, state_machine_thread, NULL, NULL, NULL,
//                 STATE_MACHINE_PRIORITY, 0, 0);

/*===========================================================================*/
/* Initialization Functions                                                  */
/*===========================================================================*/

/**
 * @brief Initialize the state machine
 *
 * Sets the initial state to S_LOW_POWER_CONNECTED. Must be called before
 * state_machine_start() to properly initialize the system.
 */
void state_machine_init(void) {
  current_state = S_LOW_POWER_CONNECTED;
  LOG_INF("State machine initialized to state: %d", current_state);
}

/**
 * @brief Start the state machine thread
 *
 * Signals the state machine thread to begin execution by releasing the
 * startup semaphore.
 */
void state_machine_start(void) {
  k_sem_give(&state_machine_ready_sem);
  LOG_INF("State machine started");
}

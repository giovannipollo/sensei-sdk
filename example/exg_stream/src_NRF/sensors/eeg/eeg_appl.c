/*
 * ----------------------------------------------------------------------
 *
 * File: eeg_appl.c
 *
 * Last edited: 09.12.2025
 *
 * Copyright (C) 2025, ETH Zurich and University of Bologna
 *
 * Authors:
 * - Philip Wiese (wiesep@iis.ee.ethz.ch), ETH Zurich
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
 * @file eeg_appl.c
 * @brief EEG Application Layer Implementation
 *
 * This module implements the high-level application control for the ADS1298 EEG
 * sensor, managing data acquisition, streaming, and BLE packet formatting.
 */

#include "eeg_appl.h"
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>

// Include ADS1298 driver headers
#include "afe/ads_appl.h"
#include "afe/ads_defs.h"
#include "afe/ads_spi.h"

// Include BLE application header for packet transmission
#include "ble/ble_appl.h"

// Include sync streaming for synchronized start/stop
#include "core/sync_streaming.h"

LOG_MODULE_REGISTER(eeg_appl, LOG_LEVEL_INF);

/*==============================================================================
 * Static Variables
 *============================================================================*/

static volatile eeg_state_t eeg_state = EEG_STATE_IDLE;
static volatile bool eeg_keep_running = false;
static K_SEM_DEFINE(eeg_start_sem, 0, 1);
static uint8_t eeg_tx_buf[EEG_PCKT_SIZE];
static uint8_t eeg_buf_idx = 0;
static uint8_t eeg_pkt_counter = 0;
static uint8_t eeg_trigger_value = 0;
uint8_t init_params[5] = {6, 0, 2, 4, 0}; // SAMPLE RATE 1KSPS, CHANNEL FUNCTION SHORT, NC, NC, GAIN = 6
static bool first_run = true;

/*==============================================================================
 * Static Function Declarations
 *============================================================================*/

/**
 * @brief EEG streaming thread function
 *
 * Main thread that handles EEG data acquisition and BLE transmission.
 * Runs continuously while eeg_keep_running is true.
 *
 * @param arg1 Unused
 * @param arg2 Unused
 * @param arg3 Unused
 */
static void eeg_streaming_thread(void *arg1, void *arg2, void *arg3);

/*==============================================================================
 * Thread Definition
 *============================================================================*/

K_THREAD_DEFINE(eeg_thread_id, 2048, eeg_streaming_thread, NULL, NULL, NULL, 7, 0, 0);

/*==============================================================================
 * Public Function Implementations
 *============================================================================*/

int eeg_init(void) {

  LOG_INF("Initializing EEG subsystem");

  // Set initial state
  eeg_state = EEG_STATE_IDLE;
  eeg_keep_running = false;
  eeg_buf_idx = 0;
  eeg_pkt_counter = 0;
  eeg_trigger_value = 0;

  LOG_INF("EEG subsystem initialized successfully");
  return 0;
}

int eeg_start_streaming(void) {
  if (eeg_state != EEG_STATE_IDLE) {
    LOG_ERR("EEG not in idle state, current state: %d", eeg_state);
    return -EBUSY;
  }

  LOG_INF("Starting EEG streaming");

  eeg_state = EEG_STATE_STARTING;
  eeg_keep_running = true;
  eeg_buf_idx = 0;
  eeg_pkt_counter = 0;

  LOG_INF("Powering ADS unipolar");
  pwr_ads_on_unipolar();
  k_msleep(300);

  if (first_run) {
    first_run = false;
    LOG_INF("Checking ADS1298 device IDs");
    ads_check_id(ADS1298_A);
    ads_check_id(ADS1298_B);

    LOG_INF("Initializing ADS1298 devices with provided parameters");
    ads_init(init_params, ADS1298_A);
    ads_init(init_params, ADS1298_B);

    LOG_INF("Starting ADS1298 data acquisition");
    ADS_Start();
    LOG_INF("ADS1298 started");

    // Signal thread to begin
    k_sem_give(&eeg_start_sem);
    LOG_INF("Signaled EEG streaming thread to start");

    eeg_state = EEG_STATE_STREAMING;
    LOG_INF("EEG streaming started");

  } else {
    LOG_INF("Re-initializing ADS1298 devices with provided parameters");
    ads_init(init_params, ADS1298_A);
    ads_init(init_params, ADS1298_B);
    LOG_INF("Starting ADS1298 data acquisition");
    ADS_Start();
    LOG_INF("ADS1298 started");
    // Signal thread to begin
    k_sem_give(&eeg_start_sem);
    LOG_INF("Signaled EEG streaming thread to start");
    eeg_state = EEG_STATE_STREAMING;
    LOG_INF("EEG streaming started");
  }

  return 0;
}

int eeg_stop_streaming(void) {
  if (eeg_state != EEG_STATE_STREAMING) {
    LOG_ERR("EEG not currently streaming, current state: %d", eeg_state);
    return -EINVAL;
  }

  LOG_INF("Stopping EEG streaming");

  eeg_state = EEG_STATE_STOPPING;
  eeg_keep_running = false;

  LOG_INF("EEG streaming stopped");

  return 0;
}

eeg_state_t eeg_get_state(void) { return eeg_state; }

bool eeg_is_streaming(void) { return (eeg_state == EEG_STATE_STREAMING); }

void eeg_set_trigger(uint8_t value) {
  eeg_trigger_value = value;
  // Also set in ADS application layer
  set_trigger(value);
}

uint8_t eeg_get_trigger(void) { return eeg_trigger_value; }

/*==============================================================================
 * Static Function Implementations
 *============================================================================*/

static void eeg_streaming_thread(void *arg1, void *arg2, void *arg3) {
  ARG_UNUSED(arg1);
  ARG_UNUSED(arg2);
  ARG_UNUSED(arg3);

  LOG_INF("EEG streaming thread started");

  while (1) {
    // Wait for start signal
    k_sem_take(&eeg_start_sem, K_FOREVER);

    LOG_INF("EEG streaming thread running");
    ADS_clear_skip_reads();
    Set_ADS_Function(READ);
    while (eeg_keep_running) {
      process_ads_data();
    }
    LOG_INF("EEG streaming thread stopping");
    Set_ADS_Function(STILL);
    LOG_INF("ADS set to STILL");
    ADS_Stop(); // Stop ADS
    LOG_INF("ADS stopped");
    k_msleep(100);
    eeg_state = EEG_STATE_IDLE;
    LOG_INF("EEG state set to IDLE");
    pwr_ads_off();
    LOG_INF("Powering off ADS devices");
  }

  LOG_INF("EEG streaming thread exiting");
}
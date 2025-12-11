/*
 * ----------------------------------------------------------------------
 *
 * File: board_streaming.c.h
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
#include <stdio.h>
#include <string.h>

#include "bsp/pwr_bsp.h"
#include "pwr/pwr.h"
#include "pwr/pwr_common.h"
#include "pwr/thread_pwr.h"

#include "afe/ads_appl.h"
#include "afe/ads_spi.h"
#include "ble/ble_appl.h"
#include "core/common.h"
#include "core/sync_streaming.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(board_streaming, LOG_LEVEL_DBG);

bool first_run = true;

uint8_t InitParams[5] = {2, 1, 0, 0, 0}; // SAMPLE RATE 1KSPS, CHANNEL FUNCTION SHORT, NC, NC, GAIN = 6

// void loop_streaming() {
//   switch (Get_ADS_Function()) {
//   case START:
//     // LOG_INF("loop_streaming: START command received");
//     // if (first_run) {
//     //   LOG_INF("First run initialization");
//     //   GetConfigParam(InitParams); // Send "Ready" to host and gets configuration parameters
//     //   k_msleep(200);
//     //   ads_check_id(ADS1298_A);
//     //   ads_check_id(ADS1298_B);
//     //   ads_init(InitParams, ADS1298_A); // Initialize ADS
//     //   ads_init(InitParams, ADS1298_B); // Initialize ADS

//     //   ADS_Start(); // Start ADS

//     //   /* Wait for ADC settling before sync barrier (500 samples @ 1kSPS = 500ms)
//     //    * This ensures EXG data starts at the same time as MIC after the barrier */
//     //   if (sync_is_active()) {
//     //     LOG_INF("EXG ready, waiting at sync barrier...");
//     //     int ret = sync_wait(SYNC_SUBSYSTEM_EXG, 5000); /* 5 second timeout */
//     //     if (ret != 0) {
//     //       LOG_ERR("Sync wait failed: %d", ret);
//     //       Set_ADS_Function(STILL);
//     //       break;
//     //     }
//     //   }

//     //   ADS_clear_skip_reads(); // Clear the skip flag since we waited manually
//     //   Set_ADS_Function(READ);
//     //   first_run = false;

//     // } else {
//     //   LOG_INF("Subsequent run initialization");
//     //   GetConfigParam(InitParams); // Send "Ready" to host and gets configuration parameters
//     //   k_msleep(200);
//     //   ads_init(InitParams, ADS1298_A); // Initialize ADS
//     //   ads_init(InitParams, ADS1298_B); // Initialize ADS

//     //   ADS_Start(); // Start ADS

//     //   /* Wait for ADC settling before sync barrier (500 samples @ 1kSPS = 500ms)
//     //    * This ensures EXG data starts at the same time as MIC after the barrier */
//     //   if (sync_is_active()) {
//     //     LOG_INF("EXG ready, waiting at sync barrier...");
//     //     int ret = sync_wait(SYNC_SUBSYSTEM_EXG, 5000); /* 5 second timeout */
//     //     if (ret != 0) {
//     //       LOG_ERR("Sync wait failed: %d", ret);
//     //       Set_ADS_Function(STILL);
//     //       break;
//     //     }
//     //   }

//     //   ADS_clear_skip_reads(); // Clear the skip flag since we waited manually
//     //   Set_ADS_Function(READ);
//     // }

//     break;

//   case STOP:
//     // if (!first_run) {
//     //   Set_ADS_Function(STILL);
//     //   ADS_Stop();   // Stop ADS
//     //   sync_reset(); // Reset sync state for next session
//     //   k_msleep(100);
//     // }
//     break;

//   case READ:
//     // process_ads_data();
//     break;

//   case STILL:
//     // set_trigger(0);
//     break;

//   default:
//     break;
//   }
// }

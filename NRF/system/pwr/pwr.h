/*
 * ----------------------------------------------------------------------
 *
 * File: pwr.h
 *
 * Last edited: 30.10.2025
 *
 * Copyright (c) 2024 ETH Zurich and University of Bologna
 *
 * Authors:
 * - Philipp Schilk (schilkp@ethz.ch), ETH Zurich
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

#ifndef PWR_H_
#define PWR_H_

#include <stdint.h>
#include <zephyr/kernel.h>

#define PWR_BAT_CRITICAL_MV (2900)
#define PWR_BAT_LOW_MV (3100)
#define PWR_BAT_OK_MV (3300)

/**
 * @brief Initialise the power sub-system
 * @return negative on error, 0 otherwise.
 */
int pwr_init();

/**
 * @brief Start the power sub-system
 * @note May only be called once.
 * @return negative on error, 0 otherwise.
 */
int pwr_start();

// note: mutex lock dies
void pwr_kill();

// Note must be active or init?
uint32_t pwr_bat_perc();

// Note must be active or init?
uint32_t pwr_bat_mV();

// Configure GAP9 power supply
void gap9_pwr(bool);

#endif /* PWR_H_ */

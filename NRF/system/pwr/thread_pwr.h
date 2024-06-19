/*
 * ----------------------------------------------------------------------
 *
 * File: thread_pwr.h
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

#ifndef THREAD_PWR_H_
#define THREAD_PWR_H_

#include "zephyr/kernel.h"

/**
 * @brief Initialise the power thread
 * @note pwr mutex must be held when this function is called.
 * @return negative on error, 0 otherwise.
 */
int thread_pwr_init();

/**
 * @brief Start the pwr thread.
 * @note May only be called once.
 */
void thread_pwr_start();

/**
 * @brief Measure currents
 * @note pwr mutex must be held when this function is called.
 * @return negative on error, 0 otherwise.
 */
int measure_currents();

/**
 * @brief Measure battery voltage and percentage.
 * @note pwr mutex must be held when this function is called.
 * @return negative on error, 0 otherwise.
 */
int measure_voltages();

/**
 * @brief Current battery charge state, in percent.
 * @note 0 before first read, which is performed by thread_pwr_init.
 */
extern atomic_t battery_perc;

/**
 * @brief Current battery voltage, in millivolts
 * @note 0 before first read, which is performed by thread_pwr_init.
 */
extern atomic_t battery_mV;

/**
 * @brief Current GAP9 power state.
 * @note 0 before first read, which is performed by thread_pwr_init.
 */
extern atomic_t gap9_pwr_state;

/** @brief Power thread ID */
extern k_tid_t pwr_t;

#endif /* THREAD_PWR_H_ */

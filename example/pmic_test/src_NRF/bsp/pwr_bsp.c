/*
 * ----------------------------------------------------------------------
 *
 * File: pwr_bsp.c
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

#include "bsp/pwr_bsp.h"

#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

// ======== Defines/Variables ======================================================================

LOG_MODULE_REGISTER(pwr_bsp);

static const struct device *pwr_i2c = DEVICE_DT_GET(DT_ALIAS(i2ca));

// ======== Functions ==============================================================================

int pwr_bsp_init() {
  if (!device_is_ready(pwr_i2c)) {
    LOG_ERR("PWR I2C not ready!");
    return -1;
  }

  return 0;
}

int pwr_bsp_start() { return 0; }

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

#include "pwr/pwr.h"

// ======== Defines/Variables ======================================================================

LOG_MODULE_REGISTER(pwr_bsp);

static const struct device *pwr_i2c = DEVICE_DT_GET(DT_ALIAS(i2ca));

/* The devicetree node identifier */
#define GPIO_NODE_led_pwr DT_NODELABEL(gpio_led_pwr)
#define GPIO_NODE_debug_led_pwr DT_NODELABEL(gpio_debug_led_pwr)

/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
static const struct gpio_dt_spec gpio_led_pwr = GPIO_DT_SPEC_GET(GPIO_NODE_led_pwr, gpios);
static const struct gpio_dt_spec gpio_led_debug_pwr = GPIO_DT_SPEC_GET(GPIO_NODE_debug_led_pwr, gpios);

// ======== Functions ==============================================================================

int pwr_bsp_init() {
  if (!device_is_ready(pwr_i2c)) {
    LOG_ERR("PWR I2C not ready!");
    return -1;
  }

  return 0;
}

int pwr_bsp_start() {
  if (gpio_pin_set_dt(&gpio_led_pwr, 1) < 0) {
    LOG_ERR("LED EN GPIO configuration error");
    return -1;
  }

  if (gpio_pin_set_dt(&gpio_led_debug_pwr, 1) < 0) {
    LOG_ERR("DEBUG LED EN GPIO configuration error");
    return 0;
  }

  return 0;
}

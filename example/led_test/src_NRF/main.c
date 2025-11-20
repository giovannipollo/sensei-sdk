/*
 * ----------------------------------------------------------------------
 *
 * File: main.c
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

#include <stdio.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/ring_buffer.h>

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/led.h>
#include <zephyr/drivers/uart.h>

#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>

#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/usbd.h>

#include "bsp/pwr_bsp.h"
#include "pwr/pwr.h"

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS 3000

/* Structure describing a color by its component values and name */
struct color_data {
  uint8_t r, g, b;
  const char *name;
};

/* The sequence of colors the RGB LED will display */
static const struct color_data color_sequence[] = {
    {0xFF, 0x00, 0x00, "Red"},   {0x00, 0xFF, 0x00, "Green"},  {0x00, 0x00, 0xFF, "Blue"},
    {0xFF, 0xFF, 0xFF, "White"}, {0xFF, 0xFF, 0x00, "Yellow"}, {0xFF, 0x00, 0xFF, "Purple"},
    {0x00, 0xFF, 0xFF, "Cyan"},  {0xF4, 0x79, 0x20, "Orange"},
};

#define GPIO_NODE_gap9_i2c_ctrl DT_NODELABEL(gpio_gap9_i2c_ctrl)

/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
static const struct device *const uart_dev = DEVICE_DT_GET_ONE(zephyr_cdc_acm_uart);
static const struct device *const led = DEVICE_DT_GET_ANY(issi_is31fl3194);
static const struct gpio_dt_spec gpio_p0_6_gap9_i2c_ctrl = GPIO_DT_SPEC_GET(GPIO_NODE_gap9_i2c_ctrl, gpios);

LOG_MODULE_REGISTER(main);

int main(void) {
  int ret = 0;

  LOG_INIT();

  LOG_INF("LED Test on %s", CONFIG_BOARD);

  // Initialize power management
  // WIESEP: We do not start the power management task as it requires access to the I2C_A bus,
  // which will be used by the GAP9.
  if (pwr_init()) {
    LOG_ERR("PWR Init failed!");
  }
  pwr_start();

  if (!device_is_ready(led)) {
    LOG_ERR("LED device not ready");
    return 0;
  }

  if (!device_is_ready(uart_dev)) {
    LOG_ERR("CDC ACM device not ready");
    return 0;
  }

  if (usb_enable(NULL)) {
    return 0;
  }
  LOG_INF("USB enabled");

  for (size_t i = 0; i < ARRAY_SIZE(color_sequence); i++) {
    ret = led_set_color(led, 0, 3, &(color_sequence[i].r));
    if (ret) {
      LOG_ERR("Failed to set color");
      return 0;
    }
    k_msleep(333);
  }

  k_msleep(100);
  if (gpio_pin_set_dt(&gpio_p0_6_gap9_i2c_ctrl, 1) < 0) {
    LOG_ERR("GAP9 I2C GPIO configuration error");
    return 0;
  }

  gap9_pwr(true);
  LOG_INF("GAP9 powered up");

  while (1) {
    k_msleep(SLEEP_TIME_MS);
    printf("Sleeping..\r\n");
  }
  return 0;
}

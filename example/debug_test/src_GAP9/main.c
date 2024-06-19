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

/***Include****/
#include "pmsis.h"
#include <stdint.h>
#include <stdio.h>

#define PAD_GPIO_46 (PI_PAD_046)
#define PAD_GPIO_47 (PI_PAD_047)
#define PAD_GPIO_48 (PI_PAD_048)
#define PAD_GPIO_49 (PI_PAD_049)
#define PAD_GPIO_50 (PI_PAD_050)
#define PAD_GPIO_51 (PI_PAD_051)

#define DEBUG_BUTTON_3 (PAD_GPIO_46)
#define DEBUG_LED_RED (PAD_GPIO_47)

#define APP_DELAY_US 1000000 // 1s delay

pi_evt_t cb_gpio_task;
pi_err_t err = PI_OK;

static void __pi_gpio_cb() { pi_gpio_pin_toggle(DEBUG_LED_RED); }

int main(void) {

  // Configure the LED
  pi_pad_function_set(DEBUG_LED_RED, PI_PAD_FUNC1);
  pi_gpio_pin_configure(DEBUG_LED_RED, PI_GPIO_OUTPUT | PI_GPIO_PULL_DISABLE | PI_GPIO_DRIVE_STRENGTH_LOW);

  // Configure the button
  pi_pad_function_set(DEBUG_BUTTON_3, PI_PAD_FUNC1);
  pi_gpio_pin_configure(DEBUG_BUTTON_3, PI_GPIO_INPUT | PI_GPIO_PULL_UP | PI_GPIO_DRIVE_STRENGTH_HIGH);

  // Initialize and attach the callback
  pi_evt_callback_no_irq_init(&cb_gpio_task, __pi_gpio_cb, NULL);

  if (pi_gpio_pin_task_add(DEBUG_BUTTON_3, &cb_gpio_task, PI_GPIO_NOTIF_FALL)) {
    err = PI_FAIL;
  }
  pi_time_wait_us(10000);

  // Toggle the LED
  for (int i = 0; i < 10; i++) {
    pi_gpio_pin_toggle(DEBUG_LED_RED);
    pi_time_wait_us(500000);
  }

  // Print "Hello World" every second
  while (1) {
    printf("Hello World\r\n");
    pi_time_wait_us(APP_DELAY_US);
  }
  return 0;
}

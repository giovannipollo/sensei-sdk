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
#define SLEEP_TIME_MS 500

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

// Required to allow overwriting the boot mode of GAP9 via the debug PCB
#define GPIO_NODE_gap9_spi_ctrl DT_NODELABEL(gpio_gap9_spi_ctrl)
#define LED_NODE DT_NODELABEL(led_0)
#define LED_DEBUG_NODE DT_NODELABEL(debug_led)

#define GPIO_NODE_debug_green DT_NODELABEL(gpio_debug_led_green)
#define GPIO_NODE_debug_yellow DT_NODELABEL(gpio_debug_led_yellow)

#define GPIO_NODE_debug_button1 DT_NODELABEL(debug_button1)
#define GPIO_NODE_debug_button2 DT_NODELABEL(debug_button2)

static const struct device *const uart_dev = DEVICE_DT_GET_ONE(zephyr_cdc_acm_uart);
static const struct device *const led = DEVICE_DT_GET(LED_NODE);
static const struct device *const led_debug = DEVICE_DT_GET(LED_DEBUG_NODE);
static const struct gpio_dt_spec gpio_led_debug_green = GPIO_DT_SPEC_GET(GPIO_NODE_debug_green, gpios);
static const struct gpio_dt_spec gpio_led_debug_yellow = GPIO_DT_SPEC_GET(GPIO_NODE_debug_yellow, gpios);
static const struct gpio_dt_spec gpio_gap9_spi_ctrl = GPIO_DT_SPEC_GET(GPIO_NODE_gap9_spi_ctrl, gpios);
static const struct gpio_dt_spec gpio_debug_button1 = GPIO_DT_SPEC_GET(GPIO_NODE_debug_button1, gpios);
static const struct gpio_dt_spec gpio_debug_button2 = GPIO_DT_SPEC_GET(GPIO_NODE_debug_button2, gpios);

static struct gpio_callback debug_button1_cb_data;
static struct gpio_callback debug_button2_cb_data;

K_SEM_DEFINE(show_led_pattern_sem, 0, 1);

LOG_MODULE_REGISTER(main);

void show_led_pattern(const struct device *dev, const struct color_data *color, size_t size) {
  int ret = 0;
  for (size_t i = 0; i < size; i++) {
    ret = led_set_color(dev, 0, 3, &(color[i].r));
    if (ret) {
      LOG_ERR("Failed to set color on LED");
      return;
    }
    k_msleep(333);
  }
  led_set_color(dev, 0, 3, (uint8_t[]){0, 0, 0});
}

void debug_button1_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
  printf("Button1 %d pressed at %" PRIu32 "\n", pins, k_cycle_get_32());

  // Signal the main thread to show the LED pattern
  k_sem_give(&show_led_pattern_sem);
}

void debug_button2_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
  printf("Button2 %d pressed at %" PRIu32 "\n", pins, k_cycle_get_32());

  if (gpio_pin_toggle_dt(&gpio_led_debug_yellow) < 0) {
    printf("Yellow Debug LED GPIO configuration error");
    return;
  }
}

int main(void) {
  int ret = 0;

  LOG_INIT();

  LOG_INF("DEBUG Board Test on %s", CONFIG_BOARD);

  // Initialize power management
  if (pwr_init()) {
    LOG_ERR("PWR Init failed!");
  }
  pwr_start();

  if (!device_is_ready(led)) {
    LOG_ERR("LED device not ready");
    return 0;
  }

  if (!device_is_ready(led_debug)) {
    LOG_ERR("Debug LED device not ready");
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

  show_led_pattern(led, color_sequence, ARRAY_SIZE(color_sequence));
  show_led_pattern(led_debug, color_sequence, ARRAY_SIZE(color_sequence));

  k_msleep(100);

  ret = gpio_pin_configure_dt(&gpio_debug_button1, GPIO_INPUT);
  if (ret != 0) {
    LOG_ERR("Error %d: failed to configure %s pin %d\n", ret, gpio_debug_button1.port->name, gpio_debug_button1.pin);
    return 0;
  }

  ret = gpio_pin_interrupt_configure_dt(&gpio_debug_button1, GPIO_INT_EDGE_TO_ACTIVE);
  if (ret != 0) {
    LOG_ERR("Error %d: failed to configure interrupt on %s pin %d\n", ret, gpio_debug_button1.port->name,
            gpio_debug_button1.pin);
    return 0;
  }

  gpio_init_callback(&debug_button1_cb_data, debug_button1_pressed, BIT(gpio_debug_button1.pin));
  gpio_add_callback(gpio_debug_button1.port, &debug_button1_cb_data);

  ret = gpio_pin_configure_dt(&gpio_debug_button2, GPIO_INPUT);
  if (ret != 0) {
    LOG_ERR("Error %d: failed to configure %s pin %d\n", ret, gpio_debug_button2.port->name, gpio_debug_button2.pin);
    return 0;
  }

  ret = gpio_pin_interrupt_configure_dt(&gpio_debug_button2, GPIO_INT_EDGE_TO_ACTIVE);
  if (ret != 0) {
    LOG_ERR("Error %d: failed to configure interrupt on %s pin %d\n", ret, gpio_debug_button2.port->name,
            gpio_debug_button2.pin);
    return 0;
  }

  gpio_init_callback(&debug_button2_cb_data, debug_button2_pressed, BIT(gpio_debug_button2.pin));
  gpio_add_callback(gpio_debug_button2.port, &debug_button2_cb_data);

  k_msleep(100);

  // Required to allow overwriting the boot mode of GAP9 via the debug PCB
  if (gpio_pin_set_dt(&gpio_gap9_spi_ctrl, 1) < 0) {
    LOG_ERR("GAP9 SPI GPIO configuration error");
    return 0;
  }

  gap9_pwr(true);
  LOG_INF("GAP9 powered up");

  if (gpio_pin_configure_dt(&gpio_led_debug_green, GPIO_OUTPUT) < 0) {
    LOG_ERR("Green Debug LED GPIO configuration error");
    return 0;
  }

  if (gpio_pin_configure_dt(&gpio_led_debug_yellow, GPIO_OUTPUT) < 0) {
    LOG_ERR("Yellow Debug LED GPIO configuration error");
    return 0;
  }

  while (1) {
    // Toggle the green debug LED
    if (gpio_pin_toggle_dt(&gpio_led_debug_green) < 0) {
      LOG_ERR("Green Debug LED GPIO configuration error");
      return 0;
    }

    // Check if the button was pressed and show the LED pattern
    if (k_sem_take(&show_led_pattern_sem, K_MSEC(SLEEP_TIME_MS)) == 0) {
      show_led_pattern(led_debug, color_sequence, ARRAY_SIZE(color_sequence));
    }
  }
  return 0;
}

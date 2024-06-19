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

#include "pmsis.h"
#include <stdint.h>
#include <stdio.h>

#define IS31FL3194_PRODUCT_ID 0x00 // should return 0xCE
#define IS31FL3194_OP_CONFIG 0x01
#define IS31FL3194_OUT_CONFIG 0x02
#define IS31FL3194_CURRENT_BAND 0x03
#define IS31FL3194_HOLD_FUNCTION 0x04

// Current Mode
#define IS31FL3194_OUT1 0x10
#define IS31FL3194_OUT2 0x21
#define IS31FL3194_OUT3 0x32

#define IS31FL3194_COLOR_UPDATE 0x40
#define IS31FL3194_RESET 0x4F

// Global variable of the I2C device
static struct pi_device i2c_device;

void show_rainbow();

// Initialize I2C configuration and open the device
static int i2c_setup(uint8_t itf, uint8_t addr) {
  pi_pad_function_set(PI_PAD_042, PI_PAD_FUNC0); // SDA
  pi_pad_function_set(PI_PAD_043, PI_PAD_FUNC0); // SCL

  pi_i2c_conf_t i2c_conf;
  pi_i2c_conf_init(&i2c_conf);
  i2c_conf.itf = itf;
  i2c_conf.max_baudrate = 100000;
  pi_i2c_conf_set_slave_addr(&i2c_conf, addr << 1, 0);

  pi_open_from_conf(&i2c_device, &i2c_conf);
  if (pi_i2c_open(&i2c_device)) {
    printf("I2C open failed\r\n");
    return -1;
  }
  return 0;
}

// Write data to a specific register
static int i2c_reg_write(uint8_t reg, uint8_t value) {
  uint8_t data[2] = {reg, value};
  // pi_i2c_write signature according to your SDK's expectations
  if (pi_i2c_write(&i2c_device, data, sizeof(data), PI_I2C_XFER_START | PI_I2C_XFER_STOP) != PI_OK) {
    printf("I2C write failed\r\n");
    return -1;
  }
  pi_time_wait_us(1000); // wait for the device to process the data
  return 0;
}

// Write data to a specific register
static int i2c_reg_read(uint8_t reg, uint8_t *value) {
  // pi_i2c_write signature according to your SDK's expectations
  if (pi_i2c_write(&i2c_device, &reg, 1, PI_I2C_XFER_START) != PI_OK) {
    printf("I2C write failed\n");
    return -1;
  }
  pi_time_wait_us(1000); // wait for the device to process the data
  // pi_i2c_read signature according to your SDK's expectations
  if (pi_i2c_read(&i2c_device, value, 1, PI_I2C_XFER_STOP) != PI_OK) {
    printf("I2C read failed\r\n");
    return -1;
  }
  pi_time_wait_us(1000); // wait for the device to process the data
  return 0;
}

// Function to turn on the LED
static void turn_on_led(void) {
  i2c_setup(1, 0x53); // Setup I2C on interface 1

  i2c_reg_write(IS31FL3194_RESET, 0xC5); // start by resetting device
  pi_time_wait_us(10000);

  i2c_reg_write(IS31FL3194_OP_CONFIG, 0x01);     // normal operation in current mode
  i2c_reg_write(IS31FL3194_OUT_CONFIG, 0x07);    // enable all three ouputs
  i2c_reg_write(IS31FL3194_CURRENT_BAND, 0x00);  // 10 mA max current
  i2c_reg_write(IS31FL3194_HOLD_FUNCTION, 0x00); // hold function disable

  // set rgb led current
  i2c_reg_write(IS31FL3194_OUT1, 0xFF); // maximum current
  i2c_reg_write(IS31FL3194_OUT2, 0x00);
  i2c_reg_write(IS31FL3194_OUT3, 0x00);
  i2c_reg_write(IS31FL3194_COLOR_UPDATE, 0xC5); // write to color update register for changes to take effect
  pi_time_wait_us(1000000);

  // set rgb led current
  i2c_reg_write(IS31FL3194_OUT1, 0x00); // maximum current
  i2c_reg_write(IS31FL3194_OUT2, 0xFF);
  i2c_reg_write(IS31FL3194_OUT3, 0x00);
  i2c_reg_write(IS31FL3194_COLOR_UPDATE, 0xC5); // write to color update register for changes to take effect
  pi_time_wait_us(1000000);

  // set rgb led current
  i2c_reg_write(IS31FL3194_OUT1, 0x00); // maximum current
  i2c_reg_write(IS31FL3194_OUT2, 0x00);
  i2c_reg_write(IS31FL3194_OUT3, 0xFF);
  i2c_reg_write(IS31FL3194_COLOR_UPDATE, 0xC5); // write to color update register for changes to take effect
  pi_time_wait_us(1000000);
}

// Function to set the RGB LED to a specific color
void set_rgb_color(uint8_t red, uint8_t green, uint8_t blue) {
  i2c_reg_write(IS31FL3194_OUT1, red);          // Red channel
  i2c_reg_write(IS31FL3194_OUT2, green);        // Green channel
  i2c_reg_write(IS31FL3194_OUT3, blue);         // Blue channel
  i2c_reg_write(IS31FL3194_COLOR_UPDATE, 0xC5); // Apply color update
}

// Function to gradually change the color from start to end values
void fade_color(uint8_t start, uint8_t end, uint8_t *color, int increment) {
  if (increment > 0) {
    for (int i = start; i <= end; i += increment) {
      *color = i;
      set_rgb_color(color[0], color[1], color[2]);
      pi_time_wait_us(50000); // Adjust delay for smoother transition
    }
  } else {
    for (int i = start; i >= end; i += increment) {
      *color = i;
      set_rgb_color(color[0], color[1], color[2]);
      pi_time_wait_us(50000); // Adjust delay for smoother transition
    }
  }
}

// Function to show rainbow sequence
void show_rainbow() {
  // WIESEP: Hack to reset I2C device as nRF communication causes bus conflicts
  i2c_setup(1, 0x53); // Setup I2C on interface 1

  uint8_t colors[3] = {0xFF, 0x00, 0x00}; // Start with red

  // Red to yellow (increase green)
  fade_color(0, 0xFF, &colors[1], 5); // Gradually increase green

  // Yellow to green (decrease red)
  fade_color(0xFF, 0, &colors[0], -5); // Gradually decrease red

  // Green to cyan (increase blue)
  fade_color(0, 0xFF, &colors[2], 5); // Gradually increase blue

  // Cyan to blue (decrease green)
  fade_color(0xFF, 0, &colors[1], -5); // Gradually decrease green

  // Blue to magenta (increase red)
  fade_color(0, 0xFF, &colors[0], 5); // Gradually increase red

  // Magenta to red (decrease blue)
  fade_color(0xFF, 0, &colors[2], -5); // Gradually decrease blue
}

int main(void) {
  pi_time_wait_us(1000000);
  turn_on_led();
  while (1) {
    show_rainbow();
  }
  return 0;
}

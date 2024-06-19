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

#include <zephyr/device.h>
#include <zephyr/kernel.h>

#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/uart.h>

#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>

#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/usbd.h>

#include "i2c_helpers.h"
#include "pwr/pwr.h"

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS 3000

static const struct device *i2c_a = DEVICE_DT_GET(DT_ALIAS(i2ca));
static const struct device *i2c_b = DEVICE_DT_GET(DT_ALIAS(i2cb));
static const struct device *const uart_dev = DEVICE_DT_GET_ONE(zephyr_cdc_acm_uart);

LOG_MODULE_REGISTER(main);

#define GAP9_I2C_SLAVE 0
#define I2C_SLAVE_L2_TEST_ADDRESS (0x1c019000)
#define I2C_SLAVE_L2_TEST_SIZE (BUFF_SIZE * 4)
#define BUFF_SIZE (16)

#define GAP9_I2C_SLAVE_ADDR (0x0A)

/* Buffer to write in EEPROM : BUF_SIZE + 2, for the memory address. */
uint32_t write_buff[BUFF_SIZE + 2];
/* Buffer to read from EEPROM : BUF_SIZE. */
uint32_t read_buff[BUFF_SIZE];

/* BUffer holding the memory address & size for read transactions. */
uint32_t addr_buff[2];

void data_init(int nb) {
  addr_buff[0] = I2C_SLAVE_L2_TEST_ADDRESS;
  addr_buff[1] = I2C_SLAVE_L2_TEST_SIZE;

  write_buff[0] = I2C_SLAVE_L2_TEST_ADDRESS;
  write_buff[1] = I2C_SLAVE_L2_TEST_SIZE;
  for (int i = 0; i < nb; i++) {
    write_buff[i + 2] = i + 1UL;
  }
}

int main(void) {

  LOG_INIT();

  LOG_INF("Sensor Shield Scan Test on %s", CONFIG_BOARD);

  // Initialize and start power management
  pwr_init();
  pwr_start();

  k_msleep(100);

  if (!device_is_ready(uart_dev)) {
    LOG_ERR("CDC ACM device not ready");
    return 0;
  }

  if (usb_enable(NULL)) {
    return 0;
  }
  LOG_INF("USB enabled");

  data_init(BUFF_SIZE);
  int ret = 0;

  while (1) {
    k_msleep(SLEEP_TIME_MS);

    LOG_INF("Scanning I2C A Interface");
    i2c_scan(i2c_a);

    LOG_INF("Scanning I2C B Interface");
    i2c_scan(i2c_b);

#if GAP9_I2C_SLAVE
    k_msleep(100);
    printf("> Testing I2C communication with GAP9 I2C slave\r\n");

    memset(read_buff, 0, BUFF_SIZE * 4);
    memset(addr_buff, 0, 2 * 4);

    ret = i2c_write(i2c_a, (uint8_t *)write_buff, (BUFF_SIZE + 2) * 4, GAP9_I2C_SLAVE_ADDR);
    ret += i2c_read(i2c_a, (uint8_t *)addr_buff, 2 * 4, GAP9_I2C_SLAVE_ADDR);
    ret += i2c_read(i2c_a, (uint8_t *)read_buff, BUFF_SIZE * 4, GAP9_I2C_SLAVE_ADDR);
    if (ret) {
      LOG_ERR("Failed to communicate with GAP9 I2C slave\r\n");
    }

    printf("  Addr buffer: 0x%08X, 0x%08X\r\n", addr_buff[0], addr_buff[1]);
    printf("  Read buffer: ");
    for (int i = 0; i < 15; i++) {
      printf("0x%02X, ", read_buff[i]);
    }
    printf("\r\n");
#endif
  }
  return 0;
}

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
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/usbd.h>

#include "bsp/pwr_bsp.h"
#include "pwr/pwr.h"
#include "pwr/pwr_common.h"
#include "pwr/thread_pwr.h"

#include "ads_appl.h"
#include "ads_spi.h"
#include "ble_appl.h"
#include "board_streaming.h"
#include "common.h"
#include "lis2duxs12_sensor.h"
#include "ppg_appl.h"
#include "state_machine.h"

static const struct device *const uart_dev = DEVICE_DT_GET_ONE(zephyr_cdc_acm_uart);

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

#define UART_BUF_SIZE 40
#define MINIMAL_STACK_SIZE 1024

struct uart_data_t {
  void *fifo_reserved;
  uint8_t data[UART_BUF_SIZE];
  uint16_t len;
};

void z_fatal_error(unsigned int reason, const z_arch_esf_t *esf) {
  LOG_INF("Fatal error occurred: %d", reason);
  while (1) {
    // Halt here for debugging
  }
}

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
  // pwr_start();
  if (pwr_bsp_start()) {
    LOG_ERR("PWR BSP Start failed!");
  }

  if (!device_is_ready(uart_dev)) {
    LOG_ERR("CDC ACM device not ready");
    return 0;
  }

  if (usb_enable(NULL)) {
    return 0;
  }
  LOG_INF("USB enabled");

  // LOG_INF("Initializing LIS2DUXS12...");
  // init_lis2duxs12();
  LOG_INF("Enabling charge...");
  pwr_charge_enable();
  LOG_INF("Initializing ADS...");
  ret = ADS_dr_init();
  LOG_INF("Powering ADS unipolar...");
  pwr_ads_on_unipolar();

  LOG_INF("Initializing SPI...");
  init_SPI();

  LOG_INF("Powering GAP9...");
  gap9_pwr(true);
  LOG_INF("GAP9 powered up");

  struct uart_data_t *buf = k_malloc(sizeof(*buf));
  LOG_INF("Initializing BLE comm...");
  init_ble_comm();
  LOG_INF("Starting BLE adverts...");
  start_bluetooth_adverts();

  // Initialize and start state machine
  LOG_INF("Initializing state machine...");
  state_machine_init();
  LOG_INF("Starting state machine...");
  state_machine_start();
  LOG_INF("State machine initialization complete");

  while (1) {
    k_msleep(1000); // Main thread can sleep now, all the work is handeled by other threads
    if (flag_isr_soft_reset) {
      // Soft reset the device
      // Put PMIC into factory reset
      // do a nop in a busy for loop for 100ms to allow the PMIC to process the command
      volatile int i;
      for (i = 0; i < 10000000; i++) {
        __asm__ volatile("nop");
      }

      int ret = max77654_factory_ship_mode(&pmic_h);
      if (ret != 0) {
        LOG_ERR("Failed to soft reset PMIC (error %d)", ret);
      } else {
        LOG_INF("PMIC soft reset triggered");
      }
      flag_isr_soft_reset = 0;
      for (i = 0; i < 10000000; i++) {
        __asm__ volatile("nop");
      }
    }
  }
  return 0;
}

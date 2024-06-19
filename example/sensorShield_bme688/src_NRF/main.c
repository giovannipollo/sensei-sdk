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

#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/uart.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/uuid.h>

#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>

#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/usbd.h>

#include <bluetooth/services/nus.h>

#include "pwr/pwr.h"
#include "pwr/pwr_common.h"
#include "pwr/thread_pwr.h"

#include "common.h"

#define UART_BUF_SIZE 40

LOG_MODULE_REGISTER(main);

static const struct device *const bme_dev = DEVICE_DT_GET_ONE(bosch_bme680);

struct uart_data_t {
  void *fifo_reserved;
  uint8_t data[UART_BUF_SIZE];
  uint16_t len;
};

static void bt_receive_cb(struct bt_conn *conn, const uint8_t *const data, uint16_t len) {
  char addr[BT_ADDR_LE_STR_LEN] = {0};

  bt_addr_le_to_str(bt_conn_get_dst(conn), addr, ARRAY_SIZE(addr));

  LOG_INF("Received data from: %s", addr);

  // Printf the received data
  printf("[BLE] %.*s\n", len, data);
}

static struct bt_nus_cb nus_cb = {
    .received = bt_receive_cb,
    .sent = NULL,
    .send_enabled = NULL,
};

int main(void) {

  LOG_INIT();

  int rc = 0;
  if (IS_ENABLED(CONFIG_USB_DEVICE_STACK)) {
    rc = usb_enable(NULL);

    /* Ignore EALREADY error as USB CDC is likely already initialised */
    if (rc != 0 && rc != -EALREADY) {
      LOG_ERR("Failed to enable USB");
      return 0;
    }
  }

  LOG_INF("Sensor Shield Readout Example on %s V1", CONFIG_BOARD);

  struct sensor_value temp, press, humidity, gas_res;

  if (!device_is_ready(bme_dev)) {
    LOG_ERR("BME688 not not ready.\n");
    return 0;
  }

  printf("Device %p name is %s\n", bme_dev, bme_dev->name);

  rc = bt_nus_init(&nus_cb);
  if (rc) {
    LOG_ERR("Failed to initialize UART service (err: %d)", rc);
    return 0;
  }

  // Initialize and start power management
  pwr_init();
  pwr_start();

  start_bluetooth_adverts();

  /* using __TIME__ ensure that a new binary will be built on every
   * compile which is convenient when testing firmware upgrade.
   */
  LOG_INF("build time: " __DATE__ " " __TIME__);

  k_msleep(100);

  struct uart_data_t *buf = k_malloc(sizeof(*buf));

  while (1) {
    k_sleep(K_MSEC(1000));
    // Write current time in the UART buffer
    buf->len =
        snprintf(buf->data, sizeof(buf->data), "Uptime: %d ms\n", k_ticks_to_ms_near32((uint32_t)k_uptime_ticks()));

    rc = bt_nus_send(NULL, buf->data, buf->len);
    if (!(rc == 0 || rc == -ENOTCONN)) {
      LOG_WRN("Failed to send data over BLE (err: %d)", rc);
    }

    sensor_sample_fetch(bme_dev);
    sensor_channel_get(bme_dev, SENSOR_CHAN_AMBIENT_TEMP, &temp);
    sensor_channel_get(bme_dev, SENSOR_CHAN_PRESS, &press);
    sensor_channel_get(bme_dev, SENSOR_CHAN_HUMIDITY, &humidity);
    sensor_channel_get(bme_dev, SENSOR_CHAN_GAS_RES, &gas_res);

    update_status(&temp, &press, &humidity, &gas_res);
  }
  return 0;
}

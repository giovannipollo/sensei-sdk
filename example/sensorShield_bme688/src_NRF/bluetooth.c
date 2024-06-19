/*
 * ----------------------------------------------------------------------
 *
 * File: bluetooth.c
 *
 * Last edited: 30.10.2025
 *
 * Copyright (c) 2025 ETH Zurich and University of Bologna
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

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>

#include <zephyr/drivers/sensor.h>

#include <bluetooth/services/nsms.h>

#define LOG_LEVEL LOG_LEVEL_DBG
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main_bluetooth);

#include <zephyr/settings/settings.h>

static struct k_work advertise_work;

static uint8_t device_name[] = {'U', 'T', '0', '0', '0', '0', '0', '1'};

BT_NSMS_DEF(nsms_bme_temp, "Temperature", false, "Unknown", 20);
BT_NSMS_DEF(nsms_bme_pres, "Pressure", false, "Unknown", 20);
BT_NSMS_DEF(nsms_bme_hum, "Humidity", false, "Unknown", 20);
BT_NSMS_DEF(nsms_bme_gas, "Gas", false, "Unknown", 20);

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_NAME_COMPLETE, device_name, sizeof(device_name)),
};

static const struct bt_data sd[] = {
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NSMS_VAL),
};

static void advertise(struct k_work *work) {
  int rc;

  bt_le_adv_stop();

  if (IS_ENABLED(CONFIG_SETTINGS)) {
    settings_load();
  }

  rc = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
  if (rc) {
    LOG_ERR("Advertising failed to start (rc %d)", rc);
    return;
  }

  LOG_INF("Advertising successfully started");
}

static void connected(struct bt_conn *conn, uint8_t err) {
  char addr[BT_ADDR_LE_STR_LEN] = {0};

  if (err) {
    LOG_ERR("Connection failed (err 0x%02x)", err);
  } else {
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, ARRAY_SIZE(addr));
    LOG_INF("Connected to %s", addr);
  }
}

static void disconnected(struct bt_conn *conn, uint8_t reason) {
  char addr[BT_ADDR_LE_STR_LEN] = {0};
  bt_addr_le_to_str(bt_conn_get_dst(conn), addr, ARRAY_SIZE(addr));

  LOG_INF("Disconnected from %s (reason 0x%02x)", addr, reason);
  k_work_submit(&advertise_work);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = connected,
    .disconnected = disconnected,
};

static void bt_ready(int err) {
  if (err != 0) {
    LOG_ERR("Bluetooth failed to initialise: %d", err);
  } else {
    k_work_submit(&advertise_work);
  }
}

void start_bluetooth_adverts(void) {
  int rc;

  k_work_init(&advertise_work, advertise);
  rc = bt_enable(bt_ready);

  if (rc != 0) {
    LOG_ERR("Bluetooth enable failed: %d", rc);
  }
}

void update_status(struct sensor_value *temp, struct sensor_value *press, struct sensor_value *humidity,
                   struct sensor_value *gas_res) {
  char temp_str[20];
  char press_str[20];
  char humidity_str[20];
  char gas_res_str[20];

  snprintf(temp_str, sizeof(temp_str), "%d.%06d", temp->val1, temp->val2);
  snprintf(press_str, sizeof(press_str), "%d.%06d", press->val1, press->val2);
  snprintf(humidity_str, sizeof(humidity_str), "%d.%06d", humidity->val1, humidity->val2);
  snprintf(gas_res_str, sizeof(gas_res_str), "%d.%06d", gas_res->val1, gas_res->val2);

  bt_nsms_set_status(&nsms_bme_temp, temp_str);
  bt_nsms_set_status(&nsms_bme_pres, press_str);
  bt_nsms_set_status(&nsms_bme_hum, humidity_str);
  bt_nsms_set_status(&nsms_bme_gas, gas_res_str);
}

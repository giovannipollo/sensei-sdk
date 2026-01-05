/*
 * ----------------------------------------------------------------------
 *
 * File: ble_appl.c
 *
 * Last edited: 23.07.2025
 *
 * Copyright (C) 2025, ETH Zurich
 *
 * Authors:
 * - Sebastian Frey (sefrey@iis.ee.ethz.ch), ETH Zurich
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
#include "ble/ble_appl.h"
#include "afe/ads_appl.h"
#include "afe/ads_spi.h"
#include "core/common.h"
#include "sensors/imu/imu_appl.h"
#include "sensors/imu/lis2duxs12_sensor.h"
#include "sensors/mic/mic_appl.h"
#include "core/sync_streaming.h"
#include "sensors/eeg/eeg_appl.h"
#include "bsp/battery/battery.h"


#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>

/* Initialize the logging module */
LOG_MODULE_REGISTER(ble_appl, LOG_LEVEL_INF);

// #define PRINT_RECEIVED_DATA

/* Define message queues */
K_MSGQ_DEFINE(send_msgq, sizeof(ble_packet_t), SEND_QUEUE_SIZE, 4);
K_MSGQ_DEFINE(receive_msgq, BLE_PCKT_RECEIVE_SIZE, RECEIVE_QUEUE_SIZE, 1);

/* Define stack sizes and priorities */
#define BLE_SEND_STACK_SIZE 2048
#define BLE_SEND_PRIORITY 5

#define BLE_RECEIVE_STACK_SIZE 2048
#define BLE_RECEIVE_PRIORITY 4

BLE_nus_data ble_data_available;

uint8_t WaitingForConfig = 0;
uint8_t ConfigParams[5] = {6, 0, 2, 4, 96}; // [SAMPLE_RATE ADS_MODE 2 4 GAIN]

int8_t biowolf_current_state = STATE_STREAMING_NORDIC;
uint8_t bat_data[7];

uart_to_pulp_data pck_uart_wolf;

K_SEM_DEFINE(ble_data_received, 0, 1);
K_SEM_DEFINE(config_received_sem, 0, 1);

/**
 * @brief BLE Send Thread
 *
 * This thread continuously retrieves data from the send message queue
 * and transmits it over BLE. Supports variable packet sizes.
 *
 * @param arg1 Unused
 * @param arg2 Unused
 * @param arg3 Unused
 */
void ble_send_thread(void *arg1, void *arg2, void *arg3) {
  ble_packet_t packet;
  int ret;

  LOG_INF("BLE send thread started");

  while (1) {
    // Retrieve data to send from the send_msgq
    ret = k_msgq_get(&send_msgq, &packet, K_FOREVER);
    if (ret == 0) {
      // Send with actual packet size
      send_data_ble(packet.data, packet.size);
    } else {
      LOG_ERR("Failed to get data from send_msgq (err %d)", ret);
    }
  }
}

/**
 * @brief Handle Configuration Data Reception
 *
 * Processes configuration parameters received from BLE after a
 * START_EEG_STREAMING command. The configuration data contains
 * 5 bytes: [SAMPLE_RATE, ADS_MODE, reserved, reserved, GAIN].
 * After processing, signals the config_received_sem semaphore to
 * unblock GetConfigParam().
 */
static void handle_config_reception(void) {
  LOG_INF("Config received");
  ConfigParams[0] = ble_data_available.data[0];
  ConfigParams[1] = ble_data_available.data[1];
  ConfigParams[2] = ble_data_available.data[2];
  ConfigParams[3] = ble_data_available.data[3];
  ConfigParams[4] = ble_data_available.data[4];
  WaitingForConfig = 0;
  k_sem_give(&config_received_sem);
}

/**
 * @brief Forward BLE Data to GAP9 via UART
 *
 * When the board is in STATE_GAP9_MASTER mode, all received BLE data
 * is forwarded to the GAP9 processor through UART. The data is copied
 * to the UART packet buffer and marked as available for transmission.
 */
static void forward_to_gap9(void) {
  LOG_DBG("Forwarding to GAP9");
  pck_uart_wolf.data_len = ble_data_available.size;
  memcpy(pck_uart_wolf.p_data, ble_data_available.data, pck_uart_wolf.data_len);
  pck_uart_wolf.is_data_available = true;
}

/**
 * @brief Process BLE Command
 *
 * Handles all BLE commands received from the connected device.
 * Commands include battery state requests, device settings, streaming
 * control (Nordic and microphone), board state management, and more.
 *
 * @param cmd The command byte received from BLE (first byte of packet)
 */
static void handle_ble_command(uint8_t cmd) {
  switch (cmd) {
  case REQUEST_BATTERY_STATE:
    LOG_DBG("Ping REQUEST_BATTERY_STATE");
    bat_data[0] = REQUEST_BATTERY_STATE;
    bat_data[1] = bsp_is_charging();
    bat_data[2] = 0;
    bat_data[3] = (uint8_t)bsp_get_total_power_mw();
    bat_data[4] = (uint8_t)bsp_get_battery_soc();
    bat_data[5] = (uint8_t)bsp_get_battery_voltage();
    /* Get temperature from IMU sensor if available */
    float temp_celsius = 0.0f;
    if (imu_read_temperature(&temp_celsius) == 0) {
      bat_data[6] = (uint8_t)temp_celsius;
    } else {
      bat_data[6] = 0;
    }
    send_data_ble(bat_data, 7);
    break;

  case GET_DEVICE_SETTINGS:
    LOG_DBG("Ping GET_DEVICE_SETTINGS");
    SendDeviceSettings();
    break;

  case SET_DEVICE_SETTINGS:
    LOG_DBG("Ping SET_DEVICE_SETTINGS");
    break;

  case REQUEST_CONNECTING_STRING:
    LOG_DBG("Ping REQUEST_CONNECTING_STRING");
    SendReady_BLE();
    break;

  case REQUEST_HARDWARE_VERSION:
    LOG_DBG("Ping REQUEST_HARDWARE_VERSION");
    SendHardwareVersion();
    break;

  case REQUEST_FIRMWARE_VERSION:
    LOG_DBG("Ping REQUEST_FIRMWARE_VERSION");
    SendFirmwareVersion();
    break;

  case REQUEST_AVAILABLE_SENSORS:
    LOG_DBG("Ping REQUEST_AVAILABLE_SENSORS");
    SendAvailableSensors();
    break;

  case GET_BOARD_STATE:
    LOG_DBG("Ping GET_BOARD_STATE");
    LOG_DBG("Sending current state: %d", biowolf_current_state);
    send_data_ble(&biowolf_current_state, 1);
    break;

  case SET_BOARD_STATE:
    LOG_DBG("Ping SET_BOARD_STATE");
    LOG_DBG(".data[1], %d", ble_data_available.data[1]);
    LOG_DBG(".data[2], %d", ble_data_available.data[2]);

    if (ble_data_available.data[1] == 1) {
      set_state_biogap(STATE_STREAMING_NORDIC);
    } else {
      set_state_biogap(STATE_GAP9_MASTER);
    }

    if (get_state_biogap() == STATE_STREAMING_NORDIC) {
      Set_ADS_Function(STILL);
    } else {
      Set_ADS_Function(INIT_GAP9_CTRL);
    }
    break;

  case RESET_GAP9:
    LOG_DBG("Ping RESET_GAP9");
    break;

  case RESET_BOARD:
    LOG_DBG("Ping RESET_BOARD");
    break;

  case SET_TRIGGER_STATE:
    LOG_DBG("Ping SET_TRIGGER_STATE");
    set_trigger(ble_data_available.data[1]);
    break;

  case ENTER_BOOTLOADERT_MODE:
    LOG_DBG("Ping ENTER_BOOTLOADER_MODE");
    break;

  case GO_TO_SLEEP:
    LOG_DBG("Ping GO_TO_SLEEP");
    break;

  case START_EEG_STREAMING:
    LOG_INF("Ping START_EEG_STREAMING");
    ble_reset_packet_counters(); /* Reset packet counters for new session */
    eeg_start_streaming();
    break;

  case STOP_EEG_STREAMING:
    LOG_INF("Ping STOP_EEG_STREAMING");
    eeg_stop_streaming();
    ble_print_packet_stats(); /* Print BLE packet stats */
    ResetConfigState(); /* Reset config state for next session */
    break;

  case START_MIC_STREAMING:
    LOG_INF("Ping START_MIC_STREAMING");
    mic_start_streaming();
    break;

  case STOP_MIC_STREAMING:
    LOG_INF("Ping STOP_MIC_STREAMING");
    mic_stop_streaming();
    break;
  case START_COMBINED_STREAMING:
    LOG_DBG("Ping START_COMBINED_STREAMING");
    ble_reset_packet_counters(); /* Reset packet counters for new session */
    // set_SM_state(S_NORDIC_STREAM);
    sync_begin(2); /* Setup sync barrier for 2 subsystems (EXG + MIC) */
    mic_start_streaming();
    WaitingForConfig = 1;
    Set_ADS_Function(START);
    break;
  case STOP_COMBINED_STREAMING:
    LOG_DBG("Ping STOP_COMBINED_STREAMING");
    // set_SM_state(S_LOW_POWER_CONNECTED);
    mic_stop_streaming();
    Set_ADS_Function(STOP);
    ble_print_packet_stats(); /* Print BLE packet stats */
    sync_reset();       /* Clean up sync state */
    ResetConfigState(); /* Reset config state for next session */
    break;

  case START_IMU_STREAMING:
    LOG_DBG("Ping START_IMU_STREAMING");
    imu_start_streaming();
    break;

  case STOP_IMU_STREAMING:
    LOG_DBG("Ping STOP_IMU_STREAMING");
    imu_stop_streaming();
    break;
  }
}

/**
 * @brief BLE Process Received Data Thread
 *
 * Main thread for processing data received over BLE. This thread waits
 * on a semaphore for incoming data and handles it based on the current
 * board state:
 * - STATE_PROGRAM_WOLF: Data is ignored (reserved for future DFU support)
 * - WaitingForConfig: Data is treated as configuration parameters
 * - STATE_GAP9_MASTER: Data is forwarded to GAP9 via UART
 * - Otherwise: Data is processed as a BLE command
 *
 * @param arg1 Unused
 * @param arg2 Unused
 * @param arg3 Unused
 */
void process_received_data_thread(void *arg1, void *arg2, void *arg3) {
  LOG_INF("BLE receive thread started");

  while (1) {
    k_sem_take(&ble_data_received, K_FOREVER);

    if (!ble_data_available.available)
      continue;

    ble_data_available.available = false;
    LOG_INF("Received data from BLE");

    // Skip processing in programming mode
    if (get_state_biogap() == STATE_PROGRAM_WOLF)
      continue;

    // Forward to GAP9 if in master mode
    if (get_state_biogap() == STATE_GAP9_MASTER) {
      forward_to_gap9();
      continue;
    }

    // Process BLE command
    uint8_t cmd = ble_data_available.data[0];
    for (int k = 0; k < ble_data_available.size; k++) {
      LOG_DBG("Data[%d]: %d", k, ble_data_available.data[k]);
    }
    handle_ble_command(cmd);
  }
}

/**
 * @brief Get Configuration Parameters
 *
 * Blocks until configuration parameters are received from BLE using
 * a semaphore (no busy-waiting). This function is typically called
 * after START_EEG_STREAMING to wait for the streaming configuration.
 *
 * @param InitParams Pointer to array where configuration will be copied.
 *                   Must have space for at least 5 bytes:
 *                   [SAMPLE_RATE, ADS_MODE, reserved, reserved, GAIN]
 *
 * @return 0 on success, -EAGAIN on timeout
 */
uint32_t GetConfigParam(uint8_t *InitParams) {
  LOG_INF("Waiting for configuration parameters from BLE...");

  /* Block until config is received (with 30 second timeout) */
  int ret = k_sem_take(&config_received_sem, K_SECONDS(30));
  LOG_INF("Semaphore taken, ret = %d", ret);
  if (ret != 0) {
    LOG_ERR("Timeout waiting for configuration parameters");
    return -EAGAIN;
  }

  LOG_INF("Configuration parameters received from BLE.");
  for (int i = 0; i < 5; i++) {
    LOG_INF("ConfigParams[%d]: %d", i, ConfigParams[i]);
    InitParams[i] = ConfigParams[i];
  }
  return 0;
}

/**
 * @brief Reset Configuration State
 *
 * Resets the configuration reception state. Should be called when
 * stopping streaming to ensure clean state for next session.
 * Clears WaitingForConfig flag and resets the semaphore.
 */
void ResetConfigState(void) {
  WaitingForConfig = 0;
  k_sem_reset(&config_received_sem);
}

// Funtion to put data into receive buffer
void add_data_to_receive_buffer(uint8_t *data) {
  int ret;

  ret = k_msgq_put(&receive_msgq, data, K_NO_WAIT);
  if (ret != 0) {
    LOG_ERR("Receive message queue overflow! Data not enqueued: %d (err %d)", data, ret);
  } else {
    LOG_DBG("Data enqueued for receiving: %d", data);
  }
}

/**
 * @brief Add Data to Send Buffer
 *
 * Enqueues data into the send message queue for transmission.
 * Supports variable packet sizes.
 *
 * @param data The pointer to the byte array to be sent over BLE.
 * @param size The size of the data to send in bytes.
 */
void add_data_to_send_buffer(uint8_t *data, uint16_t size) {
  int ret;
  ble_packet_t packet;

  // Validate size
  if (size > BLE_PCKT_MAX_SIZE) {
    LOG_ERR("Packet size %d exceeds max %d", size, BLE_PCKT_MAX_SIZE);
    return;
  }

  packet.size = size;
  memcpy(packet.data, data, size);

  ret = k_msgq_put(&send_msgq, &packet, K_NO_WAIT);
  if (ret != 0) {
    LOG_ERR("Send message queue overflow! Data not sent: %d (err %d)", data, ret);
  } else {
    LOG_DBG("Data enqueued for sending: %d", data);
  }
}

/**
 * @brief Initialize BLE Communication
 *
 * Creates the BLE send and receive threads.
 */
void init_ble_comm() { LOG_INF("Initializing BLE communication"); }

/**
 * @brief Set Board State
 *
 * Updates the current biowolf/biogap board state.
 *
 * @param state The new state to set (e.g., STATE_STREAMING_NORDIC, STATE_GAP9_MASTER)
 */
void set_state_biogap(int8_t state) { biowolf_current_state = state; }

/**
 * @brief Get Board State
 *
 * Returns the current biowolf/biogap board state.
 *
 * @return The current board state
 */
int8_t get_state_biogap(void) { return biowolf_current_state; }

/**
 * @brief Send Hardware Version
 *
 * Sends the hardware version and revision over BLE.
 * Packet format: [REQUEST_HARDWARE_VERSION, HARDWARE_VERSION, HARDWARE_REVISION, BLE_PCK_TAILER]
 */
void SendHardwareVersion() {

  uint8_t hardware_version_data[4];
  hardware_version_data[0] = REQUEST_HARDWARE_VERSION;
  hardware_version_data[1] = HARDWARE_VERSION;
  hardware_version_data[2] = HARDWARE_REVISION;
  hardware_version_data[3] = BLE_PCK_TAILER;

  return (send_data_ble(&hardware_version_data, sizeof(hardware_version_data)));
}

/**
 * @brief Send Firmware Version
 *
 * Sends the firmware version and revision over BLE.
 * Packet format: [REQUEST_FIRMWARE_VERSION, FIRMWARE_VERSION, FIRMWARE_REVISION, BLE_PCK_TAILER]
 */
void SendFirmwareVersion() {
  uint8_t firmware_version_data[4];
  firmware_version_data[0] = REQUEST_FIRMWARE_VERSION;
  firmware_version_data[1] = FIRMWARE_VERSION;
  firmware_version_data[2] = FIRMWARE_REVISION;
  firmware_version_data[3] = BLE_PCK_TAILER;

  return (send_data_ble(&firmware_version_data, sizeof(firmware_version_data)));
}

/**
 * @brief Send Available Sensors
 *
 * Sends information about available sensors over BLE.
 * Packet format: [REQUEST_AVAILABLE_SENSORS, available_flag, reserved, BLE_PCK_TAILER]
 */
void SendAvailableSensors() {
  uint8_t available_sensors[4];
  available_sensors[0] = REQUEST_AVAILABLE_SENSORS;
  available_sensors[1] = true;
  available_sensors[2] = 0;
  available_sensors[3] = BLE_PCK_TAILER;

  return (send_data_ble(&available_sensors, sizeof(available_sensors)));
}

/**
 * @brief Send Device Settings
 *
 * Sends the current device settings over BLE.
 * Currently not implemented.
 */
void SendDeviceSettings() {}

/**
 * @brief Send Ready String
 *
 * Sends the ready/connection confirmation string "BWF16" over BLE.
 * This is used to confirm successful connection to the client.
 */
void SendReady_BLE() {
  uint8_t ready[5] = {'B', 'W', 'F', '1', '6'};
  return (send_data_ble(&ready[0], 5));
}

/* BLE Send Thread Definition */
K_THREAD_DEFINE(ble_send_tid, BLE_SEND_STACK_SIZE, ble_send_thread, NULL, NULL, NULL, BLE_SEND_PRIORITY, 0, 0);

/* BLE Receive Thread Definition */
K_THREAD_DEFINE(ble_receive_tid, BLE_RECEIVE_STACK_SIZE, process_received_data_thread, NULL, NULL, NULL,
                BLE_RECEIVE_PRIORITY, 0, 0);

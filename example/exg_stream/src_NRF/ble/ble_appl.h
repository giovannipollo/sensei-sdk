/*
 * ----------------------------------------------------------------------
 *
 * File: ble_appl.h
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
#ifndef BLE_APPL_H
#define BLE_APPL_H

#include <stdbool.h>
#include <stdint.h>

// BLE application definitions
/* Define message queue sizes (number of messages) */
#define SEND_QUEUE_SIZE 64    // Number of BLE packets that can be queued for sending
#define RECEIVE_QUEUE_SIZE 16 // Number of BLE packets that can be queued for receiving

#define BLE_PCKT_MAX_SIZE 244     // Maximum BLE packet size (must fit within MTU)
#define BLE_PCKT_RECEIVE_SIZE 234 // Size of each received packet in bytes

/**
 * @brief BLE packet structure for variable-size packet support
 * 
 * Allows sending packets of different sizes through the same queue.
 * Used for both EEG (234 bytes) and MIC (131 bytes) packets.
 */
typedef struct {
  uint16_t size;                   // Actual size of data in this packet
  uint8_t data[BLE_PCKT_MAX_SIZE]; // Packet data buffer
} ble_packet_t;

/* Commands over BLE */
#define GET_BOARD_STATE 15
#define SET_BOARD_STATE 20
#define REQUEST_CONNECTING_STRING 30
#define REQUEST_FIRMWARE_VERSION 29
#define REQUEST_HARDWARE_VERSION 14
#define REQUEST_AVAILABLE_SENSORS 28
#define REQUEST_BATTERY_STATE 17
#define START_EEG_STREAMING 18
#define STOP_EEG_STREAMING 19
#define START_MIC_STREAMING 26
#define STOP_MIC_STREAMING 27
#define START_STREAMING_ALL 31
#define STOP_STREAMING_ALL 32
#define START_IMU_STREAMING 33
#define STOP_IMU_STREAMING 34
#define START_EEG_MIC_STREAMING 35
#define STOP_EEG_MIC_STREAMING 36
#define RESET_GAP9 25
#define RESET_BOARD 21
#define ENTER_BOOTLOADERT_MODE 22
#define GO_TO_SLEEP 24
#define GET_DEVICE_SETTINGS 13
#define SET_DEVICE_SETTINGS 12

/*Trigger state*/
#define SET_TRIGGER_STATE 23
#define TRIGGER_ON 1
#define TRIGGER_OFF 0

typedef struct {
  bool available;
  int size;
  uint8_t data[BLE_PCKT_RECEIVE_SIZE];
} BLE_nus_data;

typedef struct {
  bool is_data_available;
  uint8_t data_len;
  uint8_t p_data[250];
} uart_to_pulp_data;

extern BLE_nus_data ble_data_available;
extern struct k_sem ble_data_received;

// Function prototypes
void init_ble_comm(void);
void add_data_to_send_buffer(uint8_t *data, uint16_t size);
void add_data_to_receive_buffer(uint8_t *data);

void set_state_biogap(int8_t state);
int8_t get_state_biogap(void);

uint32_t GetConfigParam(uint8_t *InitParams);
void ResetConfigState(void);

void SendHardwareVersion();
void SendFirmwareVersion();
void SendAvailableSensors();

void SendDeviceSettings();
void SendReady_BLE();

#endif // BLE_APPL_H
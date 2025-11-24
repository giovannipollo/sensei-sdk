/*
 * ----------------------------------------------------------------------
 *
 * File: ads_spi.h
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
#ifndef ADS_SPI_H
#define ADS_SPI_H

#include "ads_appl.h"
#include <stdbool.h>
#include <stdint.h>

#define BLE_PCK_TAILER 0xAA
#define BLE_PCK_HEADER 0x55
#define PCK_LNGTH 234

/** @brief Symbol specifying SPIM instance to be used. */
#define SPIM_INST_IDX 2
/** @brief Symbol specifying pin number for MOSI. */
#define MOSI_PIN 9
/** @brief Symbol specifying pin number for MISO. */
#define MISO_PIN 10
/** @brief Symbol specifying pin number for SCK. */
#define SCK_PIN 8
///** @brief Symbol specifying pin number for CS. */
// #define ADS_A_CS_PIN 11

// ADS1298 SPI Command Definition Byte Assignments
#define _WAKEUP 0x02  // Wake-up from standby mode
#define _STANDBY 0x04 // Enter Standby mode
#define _RESET 0x06   // Reset the device registers to default
#define _START 0x08   // Start and restart (synchronize) conversions
#define _STOP 0x0A    // Stop conversion
#define _RDATAC 0x10  // Enable Read Data Continuous mode (default mode at power-up)
#define _SDATAC 0x11  // Stop Read Data Continuous mode
#define _RDATA 0x12   // Read data by command; supports multiple read back
#define _RREG 0x20    // Read n nnnn registers starting at address rrrr
#define _WREG 0x40    // Write n nnnn registers starting at address rrrr

// ASD1298 Register Addresses
#define ID 0x00
#define CONFIG1 0x01
#define CONFIG2 0x02
#define CONFIG3 0x03
#define LOFF 0x04
#define CH1SET 0x05
#define CH2SET 0x06
#define CH3SET 0x07
#define CH4SET 0x08
#define CH5SET 0x09
#define CH6SET 0x0A
#define CH7SET 0x0B
#define CH8SET 0x0C
#define RLD_SENSP 0x0D
#define RLD_SENSN 0x0E
#define LOFF_SENSP 0x0F
#define LOFF_SENSN 0x10
#define LOFF_FLIP 0x11
#define LOFF_STATP 0x12
#define LOFF_STATN 0x13
#define GPIO 0x14
#define PACE 0x15
#define RESP 0x16
#define CONFIG4 0x17
#define WCT1 0x18
#define WCT2 0x19

/*==============================================================================
 * Global Variables
 *============================================================================*/

/**
 * @brief SPI command/data buffer
 *
 * Used for constructing SPI transactions. First byte typically contains
 * the command, followed by register addresses and data.
 */
extern uint8_t pr_word[10];

/**
 * @brief ADS1298 initialization status flag
 *
 * Set to true after both ADS1298 devices have been successfully initialized
 * and verified. Used to prevent data acquisition before initialization.
 */
extern bool ads_initialized;

/*==============================================================================
 * Function Declarations - Initialization
 *============================================================================*/

/**
 * @brief Initialize SPI peripheral and GPIO pins
 *
 * Configures the SPIM instance with appropriate settings for ADS1298 communication:
 * - SPI Mode 1 (CPOL=0, CPHA=1)
 * - 4 MHz clock frequency
 * - MSB first bit order
 * - Separate CS pins for each ADS1298 device
 * - START pin for synchronized acquisition
 *
 * @note Must be called before any other ADS functions
 */
void init_SPI();

/*==============================================================================
 * Function Declarations - Data Acquisition Control
 *============================================================================*/

/**
 * @brief Start data acquisition on both ADS1298 devices
 *
 * Sends START and RDATAC commands to both devices to begin continuous
 * data conversion and streaming. The devices are synchronized via the
 * shared START pin.
 */
void ADS_Start();

/**
 * @brief Stop data acquisition on both ADS1298 devices
 *
 * Sends SDATAC command to both devices to stop continuous data mode.
 * Sets skip_reads flag to discard initial samples after restart.
 */
void ADS_Stop();

/**
 * @brief Initialize an ADS1298 device with specified parameters
 *
 * Configures the ADS1298 registers for data acquisition:
 * - Sample rate configuration
 * - Channel gain and input settings
 * - Reference buffer settings
 * - Test signal configuration (if enabled)
 *
 * @param InitParams Pointer to initialization parameter array:
 *                   [0]: Sample rate code (0-6, see datasheet)
 *                   [1]: Channel input configuration
 *                   [2]: Reserved
 *                   [3]: Reserved
 *                   [4]: Channel gain code
 * @param ads_id Device identifier (ADS1298_A or ADS1298_B)
 *
 * @note Device must pass ID check before initialization proceeds
 */
void ADS_Init(uint8_t *InitParams, enum ADS_id_t ads_id);

/**
 * @brief Verify ADS1298 device ID
 *
 * Reads the device ID register and verifies it matches the expected value (0xD2).
 * Enters infinite loop with error logging if ID check fails.
 *
 * @param ads_id Device identifier (ADS1298_A or ADS1298_B)
 */
void ADS_check_ID(enum ADS_id_t ads_id);


/*==============================================================================
 * Function Declarations - Data Ready Handling
 *============================================================================*/

/**
 * @brief Initialize data ready (DRDY) interrupt
 *
 * Configures GPIO interrupt for the ADS1298_A DRDY pin. This pin signals
 * when new conversion data is available. Both devices share the same DRDY
 * signal due to synchronized operation.
 *
 * @return 0 on success, negative error code on failure
 */
int ADS_dr_init();

/**
 * @brief Read current state of DRDY pin
 *
 * @return 1 if data ready, 0 if not ready
 */
int ADS_dr_read();

/**
 * @brief Process ADS1298 data when DRDY interrupt occurs
 *
 * Called from main loop to handle new data availability:
 * 1. Reads data from both ADS1298 devices via SPI
 * 2. Combines with PPG data if enabled
 * 3. Appends IMU accelerometer data
 * 4. Constructs BLE packets for transmission
 * 5. Manages packet counters and triggers
 *
 * @note This function checks the ads_data_ready flag set by the DRDY interrupt
 */
void process_ads_data();

#endif // ADS_SPI_H
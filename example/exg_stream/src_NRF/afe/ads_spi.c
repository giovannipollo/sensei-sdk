/*
 * ----------------------------------------------------------------------
 *
 * File: ads_spi.c
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

/**
 * @file ads_spi.c
 * @brief ADS1298 SPI Driver Implementation
 *
 * This module implements low-level SPI communication with dual ADS1298 devices
 * and constructs BLE data packets for streaming bio-potential signals.
 *
 * @section data_flow Data Flow
 * 1. DRDY interrupt triggers when ADS1298 has new data
 * 2. process_ads_data() reads from both ADS1298 devices
 * 3. Data is combined with PPG (if active) - IMU is now independent
 * 4. Four samples are accumulated into a 210-byte BLE packet
 * 5. Packet is sent via BLE to connected device
 * 6. IMU data is sent separately by lis2duxs12_sensor.c at 400 Hz
 *
 * @section timing Timing Considerations
 * - At 1 kSPS sampling rate, DRDY occurs every 1 ms
 * - BLE packet is sent every 7 ms (7 samples)
 * - SPI transfers must complete within 1 ms to avoid data loss
 */

/* Zephyr RTOS headers */
#include <nrfx_spim.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>

/* Application headers */
#include "afe/ads_appl.h"
#include "afe/ads_spi.h"
#include "ble/ble_appl.h"
#include "sensors/imu/lis2duxs12_sensor.h"
#include "sensors/ppg/ppg_appl.h"

/* BSP headers */
#include "bsp/pwr_bsp.h"
#include "pwr/pwr.h"
#include "pwr/pwr_common.h"

LOG_MODULE_REGISTER(ads_spi, LOG_LEVEL_INF);

/*==============================================================================
 * Device Tree Node References
 *============================================================================*/

/** @brief Data Ready pin for ADS1298_A */
#define ADS_A_DR_NODE DT_NODELABEL(gpio_ads1298_a_dr)

/** @brief Chip Select pin for ADS1298_A */
#define CS_A_NODE DT_NODELABEL(gpio_ads1298_a_spi_cs)

/** @brief Chip Select pin for ADS1298_B */
#define CS_B_NODE DT_NODELABEL(gpio_ads1298_b_spi_cs)

/** @brief Shared START pin for synchronized acquisition */
#define ADS_START_NODE DT_NODELABEL(gpio_ads1298_start_pin)

/*==============================================================================
 * GPIO Device Specifications
 *============================================================================*/

/** @brief ADS1298_A chip select GPIO spec */
static const struct gpio_dt_spec gpio_dt_ads1298_a_cs = GPIO_DT_SPEC_GET(CS_A_NODE, gpios);

/** @brief ADS1298_B chip select GPIO spec */
static const struct gpio_dt_spec gpio_dt_ads1298_b_cs = GPIO_DT_SPEC_GET(CS_B_NODE, gpios);

/** @brief ADS1298 START pin GPIO spec (shared by both devices) */
static const struct gpio_dt_spec gpio_dt_ads1298_start_pin = GPIO_DT_SPEC_GET(ADS_START_NODE, gpios);

/** @brief ADS1298_A data ready pin GPIO spec */
static const struct gpio_dt_spec gpio_dt_ads1298_a_dr = GPIO_DT_SPEC_GET(ADS_A_DR_NODE, gpios);

/*==============================================================================
 * SPI Configuration
 *============================================================================*/

/** @brief SPI interrupt priority level */
#define SPI_INT_PRIO 1

/** @brief SPI operation flags: 8-bit words, MSB first */
#define SPIOP SPI_WORD_SET(8) | SPI_TRANSFER_MSB

/** @brief SPI device tree spec for ADS1298_A */
struct spi_dt_spec spispec = SPI_DT_SPEC_GET(DT_NODELABEL(ads1298_a), SPIOP, 0);

/*==============================================================================
 * Module Variables - SPI Buffers
 *============================================================================*/

/**
 * @brief SPI command/data word buffer
 *
 * Used to construct SPI commands and register writes. Initialized with
 * RESET command. Maximum size is 10 bytes to accommodate register writes.
 */
uint8_t pr_word[10] = {_RESET, 0, 0, 0, 0, 0, 0, 0, 0, 0};

/**
 * @brief Empty buffer for SPI reads
 *
 * Transmitted during data read operations. Size is 27 bytes:
 * - 3 status bytes
 * - 24 data bytes (8 channels × 3 bytes each)
 */
static uint8_t empty_buffer[27] = {0};

/**
 * @brief SPI receive buffer for ADS1298 data
 *
 * Receives data from ADS1298 during SPI transfers. Oversized to 40 bytes
 * to accommodate potential extended transfers.
 */
static uint8_t ads_rx_buf[40];

/*==============================================================================
 * Module Variables - BLE Packet Construction
 *============================================================================*/

/**
 * @brief BLE transmission packet buffer
 *
 * Accumulates 7 samples into a complete 234-byte packet before transmission.
 * Format documented in ads_spi.h header.
 */
uint8_t ble_tx_buf[PCK_LNGTH] = {0};

/**
 * @brief Current write index in BLE packet buffer
 *
 * Tracks the next position to write sample data. Reset to 0 when packet
 * is complete and ready for transmission.
 */
static uint32_t tx_buf_inx = 0;

/**
 * @brief BLE packet counter
 *
 * Increments with each transmitted packet. Wraps around at 255. Used by
 * receiver to detect packet loss.
 */
uint8_t counter = 0;

/**
 * @brief Extra counter for debugging/custom data
 *
 * Increments on each DRDY interrupt. Can be read out by MATLAB conversion
 * scripts for timing analysis.
 */
uint8_t counter_extra = 0;

/*==============================================================================
 * Module Variables - State Flags
 *============================================================================*/

/**
 * @brief ADS1298 initialization status
 *
 * Set to true after both devices pass ID check and register configuration.
 * Prevents data acquisition attempts before proper initialization.
 */
bool ads_initialized = false;

/**
 * @brief Skip initial samples flag
 *
 * Set to true when starting acquisition. First 500 samples are discarded
 * to allow the ADC and analog front-end to settle.
 */
bool skip_reads = true;

/**
 * @brief Count of skipped samples during startup
 *
 * Incremented until reaching 500, at which point skip_reads is cleared
 * and normal data acquisition begins.
 */
int skiped_samples = 0;

/**
 * @brief SPI transfer complete flag
 *
 * Set by SPI interrupt handler when transfer completes. Polled by
 * blocking code to wait for completion.
 */
static volatile bool spi_xfer_done = true;

/**
 * @brief Data ready interrupt flag
 *
 * Set by DRDY GPIO interrupt when new ADC data is available. Cleared
 * by process_ads_data() after reading.
 */
static volatile bool ads_data_ready = false;

/**
 * @brief DRDY serviced flag
 *
 * Tracks whether the previous DRDY interrupt was serviced. If false when
 * new DRDY arrives, indicates data overrun and acquisition is stopped.
 */
bool drdy_served = true;

/**
 * @brief BLE packet ready flag
 *
 * Indicates previous packet has not been transmitted yet. If true when
 * completing a new packet, acquisition is stopped to prevent buffer overflow.
 */
static bool pck_ble_ready = false;

/**
 * @brief Current ADS device being read
 *
 * Tracks which device (A or B) is currently being read in the SPI handler.
 * Used to properly route data in the interrupt callback.
 */
static volatile bool ads_to_read = ADS1298_A;

/*==============================================================================
 * Module Variables - Synchronization
 *============================================================================*/

/**
 * @brief SPI bus mutex
 *
 * Protects SPI bus access from concurrent threads. Locked during SPI
 * transfers, unlocked when complete.
 */
static struct k_mutex spi_mutex;

/**
 * @brief SPIM driver instance
 *
 * nrfx SPIM peripheral instance for ADS1298 communication.
 */
nrfx_spim_t spim_inst = NRFX_SPIM_INSTANCE(SPIM_INST_IDX);

/*==============================================================================
 * External Variables
 *============================================================================*/

/**
 * @brief PPG sensor data structure
 *
 * Defined in ppg_appl.c. Contains circular buffer of red and IR LED readings.
 */
extern sense_struct sense;

/*==============================================================================
 * Private Function Declarations
 *============================================================================*/

static int ads1298_read_spi(uint8_t *data, uint8_t size, enum ADS_id_t ads_id);
static int ads1298_read_samples(uint8_t *data, uint8_t size, enum ADS_id_t ads_id);
static int ads1298_write_spi(uint8_t size, enum ADS_id_t ads_id);
static void cb_ads_a_dr(const struct device *dev, struct gpio_callback *cb, uint32_t pins);

/*==============================================================================
 * SPI Interrupt Handler
 *============================================================================*/

/**
 * @brief SPIM driver interrupt handler
 *
 * Called by nrfx SPIM driver when SPI transfer completes. This handler:
 * 1. Deasserts chip select for both ADS devices
 * 2. Processes received data if in READ mode
 * 3. Constructs BLE packets from ADS/PPG/IMU data
 * 4. Sets flags to indicate transfer completion
 *
 * @param[in] p_event   Pointer to SPIM event structure
 * @param[in] p_context User context (unused)
 *
 * @note This function runs in interrupt context. Keep processing minimal.
 */
static void spim_handler(nrfx_spim_evt_t const *p_event, void *p_context) {

  LOG_DBG("spim_handler called, event type: %d", p_event->type);
  
  if (p_event->type == NRFX_SPIM_EVENT_DONE) {
    if (gpio_pin_set_dt(&gpio_dt_ads1298_a_cs, 0) < 0) { // Set CS pin to disable
      LOG_ERR("ADS1298 power GPIO set error");
      return;
    }
    if (gpio_pin_set_dt(&gpio_dt_ads1298_b_cs, 0) < 0) { // Set CS pin to disable
      LOG_ERR("ADS1298 power GPIO set error");
      return;
    }

    if (Get_ADS_Function() == READ) {
      memcpy(&ble_tx_buf[tx_buf_inx], &ads_rx_buf[3], 24);
      tx_buf_inx += 24;

      if (Get_PPG_Function() == PPG_ACTIVE) {
        // Replace last two EEG channels with PPG data (IR and red LED)
        tx_buf_inx -= 6;

        // Get the pointers to the start of the most recent PPG data
        uint8_t *address_red = (uint8_t *)&sense.red[sense.head];
        uint8_t *address_IR = (uint8_t *)&sense.IR[sense.head];
        // Add the PPG data such that the Biowolf GUI interprets it correctly
        memcpy(&ble_tx_buf[tx_buf_inx], address_red + 2, 1);
        tx_buf_inx++;
        memcpy(&ble_tx_buf[tx_buf_inx], address_red + 1, 1);
        tx_buf_inx++;
        memcpy(&ble_tx_buf[tx_buf_inx], address_red + 0, 1);
        tx_buf_inx++;

        memcpy(&ble_tx_buf[tx_buf_inx], address_IR + 2, 1);
        tx_buf_inx++;
        memcpy(&ble_tx_buf[tx_buf_inx], address_IR + 1, 1);
        tx_buf_inx++;
        memcpy(&ble_tx_buf[tx_buf_inx], address_IR + 0, 1);
        tx_buf_inx++;
      }

      if (ads_to_read == ADS1298_B) {
        /* IMU data is now sent independently via lis2duxs12_sensor.c */

        // Attach other data
        //  Set packet identifier to EEG all channels + PPG inactive
        ble_tx_buf[tx_buf_inx++] = counter_extra; // add here your custom data for each sample.
        ble_tx_buf[tx_buf_inx++] = get_trigger(); // this will capture the tigger per sample send throught UART BLE
      }

      if (tx_buf_inx == EEG_SAMPLE_DATA_END) {
        // Check if the last pck was handled and reset flag

        if (pck_ble_ready == true) {
          Set_ADS_Function(STOP);
          pck_ble_ready = false;
          LOG_INF("Data packet not processed -- stop ADS");
        } else {

          // Reset and condition BLE buffers
          tx_buf_inx = 0;

          // Prepare the next buffer with header, counter, and timestamp
          ble_tx_buf[tx_buf_inx++] = BLE_PCK_HEADER;
          ble_tx_buf[tx_buf_inx++] = ++counter;

          // Add timestamp (microseconds) for cross-packet synchronization
          uint32_t timestamp_us = k_cyc_to_us_floor32(k_cycle_get_32());
          ble_tx_buf[tx_buf_inx++] = (uint8_t)(timestamp_us & 0xFF);
          ble_tx_buf[tx_buf_inx++] = (uint8_t)((timestamp_us >> 8) & 0xFF);
          ble_tx_buf[tx_buf_inx++] = (uint8_t)((timestamp_us >> 16) & 0xFF);
          ble_tx_buf[tx_buf_inx++] = (uint8_t)((timestamp_us >> 24) & 0xFF);

          // Finish up and send
          // Get the index to write metadata
          int buf_current_size = EEG_SAMPLE_DATA_END;

          // Metadata bytes (3 bytes reserved for future use)
          ble_tx_buf[buf_current_size++] = 0x00;
          ble_tx_buf[buf_current_size++] = 0x00;
          ble_tx_buf[buf_current_size++] = 0x00;

          // BLE PCK tail
          ble_tx_buf[buf_current_size++] = BLE_PCK_TAILER;

          add_data_to_send_buffer(ble_tx_buf, EEG_PCK_LNGTH);
        }
      }

      drdy_served = true;
    }
    spi_xfer_done = true;
    LOG_DBG("Setting spi_xfer_done to true");
  }
}

/*==============================================================================
 * Public Functions - Initialization
 *============================================================================*/

/**
 * @brief Initialize SPI peripheral and GPIO pins for ADS1298 communication
 *
 * Performs complete hardware initialization:
 * 1. Connects SPI interrupt handler
 * 2. Configures SPIM peripheral (4 MHz, Mode 1, MSB first)
 * 3. Initializes chip select pins for both ADS devices
 * 4. Initializes START pin for synchronized acquisition
 * 5. Creates SPI mutex for thread-safe access
 *
 * SPI Mode 1 timing:
 * - CPOL = 0 (clock idle low)
 * - CPHA = 1 (data sampled on rising edge, shifted on falling edge)
 *
 * @note This function must be called before any other ADS functions.
 *       Errors are logged but not returned to allow graceful degradation.
 */
void init_SPI() {
  nrfx_err_t status;
  (void)status;

#if defined(__ZEPHYR__)
  IRQ_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_SPIM_INST_GET(SPIM_INST_IDX)), SPI_INT_PRIO,
              NRFX_SPIM_INST_HANDLER_GET(SPIM_INST_IDX), 0, 0);
#endif

  // ADS_A_CS_PIN
  nrfx_spim_config_t spim_config = NRFX_SPIM_DEFAULT_CONFIG(SCK_PIN, MOSI_PIN, MISO_PIN, NRF_SPIM_PIN_NOT_CONNECTED);

  spim_config.frequency = NRFX_MHZ_TO_HZ(4);
  spim_config.mode = NRF_SPIM_MODE_1;
  spim_config.bit_order = NRF_SPIM_BIT_ORDER_MSB_FIRST;
  spim_config.irq_priority = SPI_INT_PRIO;

  void *p_context = "Some context";
  status = nrfx_spim_init(&spim_inst, &spim_config, spim_handler, p_context);
  NRFX_ASSERT(status == NRFX_SUCCESS);

  // Initialize SPI CS pin for ADS A
  if (!device_is_ready(gpio_dt_ads1298_a_cs.port)) {
    LOG_ERR("ADS1298 power GPIO port not ready");
    return;
  }
  if (gpio_pin_configure_dt(&gpio_dt_ads1298_a_cs, GPIO_OUTPUT_INACTIVE) < 0) {
    LOG_ERR("ADS pwr GPIO init error");
    return;
  }

  // Initialize SPI CS pin for ADS B
  if (!device_is_ready(gpio_dt_ads1298_b_cs.port)) {
    LOG_ERR("ADS1298 power GPIO port not ready");
    return;
  }
  if (gpio_pin_configure_dt(&gpio_dt_ads1298_b_cs, GPIO_OUTPUT_INACTIVE) < 0) {
    LOG_ERR("ADS pwr GPIO init error");
    return;
  }

  // Initialize SPI START pin for synchronized start of ADS A and B
  if (!device_is_ready(gpio_dt_ads1298_start_pin.port)) {
    LOG_ERR("ADS1298 power GPIO port not ready");
    return;
  }
  if (gpio_pin_configure_dt(&gpio_dt_ads1298_start_pin, GPIO_OUTPUT_INACTIVE) < 0) {
    LOG_ERR("ADS pwr GPIO init error");
    return;
  }

  k_mutex_init(&spi_mutex);
}

/*==============================================================================
 * Private Functions - Low-Level SPI Transfers
 *============================================================================*/

/**
 * @brief Perform SPI read transaction with command
 *
 * Executes a full-duplex SPI transfer, transmitting command bytes from pr_word
 * and receiving response into ads_rx_buf. Used for register reads and device
 * identification.
 *
 * @param[in] data    Unused parameter (data comes from pr_word buffer)
 * @param[in] size    Number of bytes to transfer
 * @param[in] ads_id  Target device (ADS1298_A or ADS1298_B)
 *
 * @return 0 on success, -1 on GPIO error
 *
 * @note This function disables interrupts during transfer for timing accuracy
 * @note CS is asserted by this function and deasserted by interrupt handler
 */
static int ads1298_read_spi(uint8_t *data, uint8_t size, enum ADS_id_t ads_id) {

  unsigned int key = irq_lock(); // Disable all interrupts
  k_mutex_lock(&spi_mutex, K_FOREVER);
  nrfx_spim_xfer_desc_t spim_xfer_desc = NRFX_SPIM_XFER_TRX(pr_word, size, ads_rx_buf, size);

  if (ads_id == ADS1298_A) {
    if (gpio_pin_set_dt(&gpio_dt_ads1298_a_cs, 1) < 0) { // Set pin to enable
      LOG_ERR("ADS1298 power GPIO set error");
      return -1;
    }
  } else {
    if (gpio_pin_set_dt(&gpio_dt_ads1298_b_cs, 1) < 0) { // Set pin to enable
      LOG_ERR("ADS1298 power GPIO set error");
      return -1;
    }
  }
  nrfx_err_t status;
  status = nrfx_spim_xfer(&spim_inst, &spim_xfer_desc, 0);
  NRFX_ASSERT(status == NRFX_SUCCESS);

  k_mutex_unlock(&spi_mutex);
  irq_unlock(key); // Restore interrupts

  return 0;
}

/**
 * @brief Read ADC sample data in continuous mode
 *
 * Reads conversion data when in RDATAC (Read Data Continuous) mode.
 * Transmits dummy bytes (zeros) and receives 27 bytes of data:
 * - 3 status bytes (24-bit status word)
 * - 24 data bytes (8 channels × 3 bytes per channel)
 *
 * @param[in] data    Unused parameter (data received into ads_rx_buf)
 * @param[in] size    Number of bytes to transfer (typically 27)
 * @param[in] ads_id  Target device (ADS1298_A or ADS1298_B)
 *
 * @return 0 on success, -1 on GPIO error
 *
 * @note Called from DRDY interrupt context via process_ads_data()
 * @note empty_buffer contains zeros for dummy TX bytes
 */
static int ads1298_read_samples(uint8_t *data, uint8_t size, enum ADS_id_t ads_id) {
  unsigned int key = irq_lock(); // Disable all interrupts
  k_mutex_lock(&spi_mutex, K_FOREVER);

  nrfx_spim_xfer_desc_t spim_xfer_desc = NRFX_SPIM_XFER_TRX(empty_buffer, size, ads_rx_buf, size);
  if (ads_id == ADS1298_A) {
    if (gpio_pin_set_dt(&gpio_dt_ads1298_a_cs, 1) < 0) { // Set pin to enable
      LOG_ERR("ADS1298 power GPIO set error");
      return -1;
    }
  } else {
    if (gpio_pin_set_dt(&gpio_dt_ads1298_b_cs, 1) < 0) { // Set pin to enable
      LOG_ERR("ADS1298 power GPIO set error");
      return -1;
    }
  }
  nrfx_err_t status;
  status = nrfx_spim_xfer(&spim_inst, &spim_xfer_desc, 0);
  NRFX_ASSERT(status == NRFX_SUCCESS);
  k_mutex_unlock(&spi_mutex);
  irq_unlock(key); // Restore interrupts

  return 0;
}

/**
 * @brief Write command or register data to ADS1298
 *
 * Transmits command bytes from pr_word buffer to configure the device.
 * Used for:
 * - Single-byte commands (RESET, START, STOP, etc.)
 * - Register writes (WREG followed by address and data)
 *
 * @param[in] size    Number of bytes to write from pr_word
 * @param[in] ads_id  Target device (ADS1298_A or ADS1298_B)
 *
 * @return 0 on success, -1 on GPIO error
 *
 * @note pr_word buffer must be populated before calling this function
 * @note Transfer is non-blocking; poll spi_xfer_done for completion
 */
static int ads1298_write_spi(uint8_t size, enum ADS_id_t ads_id) {
  nrfx_spim_xfer_desc_t spim_xfer_desc = NRFX_SPIM_XFER_TRX(pr_word, sizeof(pr_word), ads_rx_buf, sizeof(pr_word));
  nrfx_err_t status;
  if (ads_id == ADS1298_A) {
    if (gpio_pin_set_dt(&gpio_dt_ads1298_a_cs, 1) < 0) { // Set pin to enable
      LOG_ERR("ADS1298 power GPIO set error");
      return -1;
    }
  } else {
    if (gpio_pin_set_dt(&gpio_dt_ads1298_b_cs, 1) < 0) { // Set pin to enable
      LOG_ERR("ADS1298 power GPIO set error");
      return -1;
    }
  }
  LOG_DBG("Starting SPI write transfer");
  status = nrfx_spim_xfer(&spim_inst, &spim_xfer_desc, 0);
  LOG_DBG("Status: %d", status);
  LOG_DBG("SPI write transfer started");
  NRFX_ASSERT(status == NRFX_SUCCESS);
  LOG_DBG("SPI write transfer asserted CS");

  return 0;
}

/*==============================================================================
 * Public Functions - Device Verification
 *============================================================================*/

/**
 * @brief Verify ADS1298 device ID
 *
 * Performs device identification to ensure proper SPI communication and
 * correct device population. The ADS1298 ID register should read 0xD2.
 *
 * Sequence:
 * 1. Send RESET command to restore default registers
 * 2. Send SDATAC to stop continuous data mode
 * 3. Read ID register (address 0x00)
 * 4. Verify ID value is 0xD2
 *
 * @param[in] ads_id Device to verify (ADS1298_A or ADS1298_B)
 *
 * @note If ID check fails, function enters infinite loop with error logging
 * @note 30ms delays allow device to complete operations per datasheet timing
 */
void ads_check_id(enum ADS_id_t ads_id) {

  // RESET DEVICE
  pr_word[0] = _RESET;
  ads1298_write_spi(1, ads_id);
  while (spi_xfer_done == false)
    ;
  k_msleep(30);

  // STOP DEVICE
  pr_word[0] = _SDATAC;
  ads1298_write_spi(1, ads_id);
  while (spi_xfer_done == false)
    ;
  k_msleep(30);

  // Read out device ID
  pr_word[0] = _RREG | ID;
  pr_word[1] = 0;
  ads1298_read_spi(ads_rx_buf, 3, ads_id);
  k_msleep(30);

  if (ads_rx_buf[2] != 0xd2) {
    // Wait here if ID is not correct
    while (1) {
      LOG_ERR("ADS1298 ID not correct");
    }
  }
  LOG_DBG("ADS1298 id checked");
}

/*==============================================================================
 * Interrupt Callbacks
 *============================================================================*/

/** @brief GPIO callback data structure for DRDY interrupt */
static struct gpio_callback ads1298_a_dr_cb_data;

/**
 * @brief GPIO callback for ADS1298 data ready interrupt
 *
 * Invoked by GPIO subsystem when DRDY pin goes active. Sets flag for
 * main loop processing and increments debug counter.
 *
 * @param[in] dev   GPIO device (unused)
 * @param[in] cb    Callback structure (unused)
 * @param[in] pins  Pin mask that triggered interrupt (unused)
 *
 * @note Runs in interrupt context - keep processing minimal
 * @note Both ADS devices share this DRDY signal (synchronized)
 */
static void cb_ads_a_dr(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
  /* Signal that new data is available */
  ads_data_ready = true;
  /* Increment debug counter for timing analysis */
  counter_extra = counter_extra + 1;
  // LOG_INF("ADS DRDY interrupt");
}

/*==============================================================================
 * Public Functions - Data Ready Signal
 *============================================================================*/

/**
 * @brief Read current state of DRDY pin
 *
 * @return 1 if data ready (pin active low on ADS1298), 0 if not ready
 */
int ADS_dr_read() { return gpio_pin_get_dt(&gpio_dt_ads1298_a_dr); }

/**
 * @brief Initialize data ready (DRDY) GPIO interrupt
 *
 * Configures the DRDY pin for interrupt-driven data acquisition.
 * The DRDY pin is shared between both ADS1298 devices since they
 * are synchronized via the START pin.
 *
 * DRDY characteristics:
 * - Active low signal
 * - Pulses high when new data is ready
 * - Interrupt on low-to-high edge (EDGE_TO_ACTIVE)
 * - At 1 kSPS, occurs every 1 ms
 *
 * @return 0 on success, -1 on error
 */
int ADS_dr_init() {
  // Initialize the data ready pin
  if (!device_is_ready(gpio_dt_ads1298_a_dr.port)) {
    LOG_ERR("ADS1298 DRDY GPIO port not ready");
    return -1;
  }
  if (gpio_pin_configure_dt(&gpio_dt_ads1298_a_dr, GPIO_INPUT) < 0) {
    LOG_ERR("ADS1298 DRDY GPIO init error");
    return -1;
  }
  // Enable interrrupt
  int ret;
  ret = gpio_pin_interrupt_configure_dt(&gpio_dt_ads1298_a_dr, GPIO_INT_EDGE_TO_ACTIVE);
  if (ret < 0) {
    LOG_ERR("ADS1298 DRDY GPIO interrupt enable error");
    return -1;
  }

  // Initialize the static struct gpio_callback variable
  gpio_init_callback(&ads1298_a_dr_cb_data, cb_ads_a_dr, BIT(gpio_dt_ads1298_a_dr.pin));
  // Add the callback function by calling gpio_add_callback()
  ret = gpio_add_callback(gpio_dt_ads1298_a_dr.port, &ads1298_a_dr_cb_data);
  if (ret < 0) {
    LOG_ERR("ADS1298 DRDY GPIO interrupt enable error");
    return -1;
  }

  return 0;
}

/*==============================================================================
 * Public Functions - Device Configuration
 *============================================================================*/

/**
 * @brief Initialize ADS1298 device with specified parameters
 *
 * Performs complete device initialization sequence:
 * 1. Reset device to default state
 * 2. Stop continuous data mode
 * 3. Configure data rate and reference settings (CONFIG1-CONFIG3)
 * 4. Configure all 8 channel settings (gain, input type)
 * 5. Initialize BLE packet buffer
 *
 * @param[in] InitParams Configuration parameter array:
 *   - [0]: Data rate code (0=16kSPS to 6=250SPS)
 *        Common values: 2=1kSPS, 3=500SPS
 *   - [1]: Channel input configuration
 *        0x00=Normal electrode input
 *        0x01=Shorted input (offset calibration)
 *        0x05=Test signal
 *   - [2]: Reserved (not used)
 *   - [3]: Reserved (not used)
 *   - [4]: Channel gain code
 *        0x00=Gain 6, 0x10=Gain 1, 0x20=Gain 2, etc.
 *
 * @param[in] ads_id Device to initialize (ADS1298_A or ADS1298_B)
 *
 * Register Configuration Details:
 * - CONFIG1: Data rate, CLK connection, daisy-chain disable
 * - CONFIG2: Test signal configuration, reference buffer
 * - CONFIG3: Internal reference, RLD buffer, bias settings
 * - CH1SET-CH8SET: Per-channel power-down, gain, and input MUX
 *
 * @note Function blocks waiting for SPI transfers to complete
 * @note 30ms delays allow device to complete configuration per datasheet
 * @note Sets ads_initialized flag when complete
 */
void ads_init(uint8_t *InitParams, enum ADS_id_t ads_id) {

  // buffer_counter = 0;
  tx_buf_inx = 0;
  ble_tx_buf[tx_buf_inx++] = BLE_PCK_HEADER;
  ble_tx_buf[tx_buf_inx++] = ++counter;
  // Reserve 4 bytes for timestamp (will be filled when packet is complete)
  tx_buf_inx += 4;

  // RESET DEVICE
  pr_word[0] = _RESET;
  spi_xfer_done = false;
  ads1298_write_spi(1, ads_id);
  while (spi_xfer_done == false)
    ;
  k_msleep(30);

  // STOP DEVICE
  pr_word[0] = _SDATAC;
  spi_xfer_done = false;
  ads1298_write_spi(1, ads_id);
  while (spi_xfer_done == false)
    ;
  k_msleep(30);

  pr_word[0] = _WREG | CONFIG1;
  pr_word[1] = 2;
  pr_word[2] = 0xC0 + InitParams[0];
  pr_word[3] = 0x55;
  pr_word[4] = 0xC0;

  spi_xfer_done = false;
  ads1298_write_spi(5, ads_id);
  while (spi_xfer_done == false)
    ;

  // SET CHANNEL REGS - ON, GAIN 6, SHORTED ELECTRODE INPUT
  pr_word[0] = _WREG | CH1SET;
  pr_word[1] = 7;

  for (int i = 2; i < 10; i++) {
    pr_word[i] = InitParams[4] | InitParams[1];
  }

  spi_xfer_done = false;
  ads1298_write_spi(10, ads_id);
  while (spi_xfer_done == false)
    ;
  k_msleep(30);

  ads_initialized = true;
}

/*==============================================================================
 * Public Functions - Acquisition Control
 *============================================================================*/

/**
 * @brief Stop data acquisition on both ADS1298 devices
 *
 * Sends SDATAC (Stop Data Continuous) command to both devices.
 * This halts conversion and allows register access.
 *
 * Side effects:
 * - Sets skip_reads flag to discard first 500 samples on restart
 * - Required to ensure ADC has settled before streaming valid data
 *
 * @note Does not power down the device, just stops conversions
 * @note Both devices must be stopped for synchronized operation
 */
void ADS_Stop() {
  skip_reads = true; // Flag to skip the fist samples as to make sure the signal is stable.

  pr_word[0] = _SDATAC;
  spi_xfer_done = false;
  LOG_DBG("Stopping ADS1298 A");
  ads1298_write_spi(1, ADS1298_A);
  LOG_DBG("Waiting for ADS1298 A to stop");
  while (spi_xfer_done == false)
    ;
  LOG_DBG("ADS1298 A stopped");
  LOG_DBG("waited 30ms after stopping ADS1298 A");

  pr_word[0] = _SDATAC;
  spi_xfer_done = false;
  ads1298_write_spi(1, ADS1298_B);
  while (spi_xfer_done == false)
    ;
}

/**
 * @brief Clear the skip_reads flag and reset skipped sample counter
 *
 * Call this after manually waiting for ADC settling (e.g., during
 * synchronized streaming) to prevent additional sample skipping
 * in process_ads_data().
 */
void ADS_clear_skip_reads() {
  skip_reads = false;
  skiped_samples = 0;
}

/**
 * @brief Start synchronized data acquisition on both ADS1298 devices
 *
 * Sends START and RDATAC commands to both devices in sequence.
 * The devices begin converting and streaming data immediately.
 *
 * Command sequence per device:
 * 1. START (0x08): Begin conversions
 * 2. RDATAC (0x10): Enable continuous data read mode
 *
 * In RDATAC mode, data is automatically clocked out on each DRDY pulse
 * without needing to send read commands.
 *
 * @note Devices are synchronized via shared START pin (if used)
 * @note DRDY interrupts will begin firing once conversion starts
 * @note First 500 samples are skipped if skip_reads flag is set
 */
void ADS_Start() {
  pr_word[0] = _START;
  pr_word[1] = _RDATAC;
  spi_xfer_done = false;
  ads1298_write_spi(2, ADS1298_A);
  while (spi_xfer_done == false)
    ;

  pr_word[0] = _START;
  pr_word[1] = _RDATAC;
  spi_xfer_done = false;
  ads1298_write_spi(2, ADS1298_B);
  while (spi_xfer_done == false)
    ;
}

/*==============================================================================
 * Public Functions - Data Processing
 *============================================================================*/

/**
 * @brief Process ADS1298 data when DRDY interrupt occurs
 *
 * Main data acquisition handler called from application main loop.
 * Manages the complete data flow from ADS devices to BLE transmission:
 *
 * Data Flow:
 * 1. Check if DRDY interrupt occurred (ads_data_ready flag)
 * 2. Skip first 500 samples for ADC settling (if skip_reads is set)
 * 3. Read 27 bytes from ADS1298_A (3 status + 24 data bytes)
 * 4. Read 27 bytes from ADS1298_B
 * 5. Data is processed in SPI interrupt handler (spim_handler):
 *    - Extract 24 data bytes (8 channels × 3 bytes)
 *    - Combine with PPG data (if active)
 *    - Append IMU accelerometer data
 *    - Build 234-byte BLE packet (7 samples)
 *    - Queue for BLE transmission
 *
 * Error Handling:
 * - If previous DRDY not serviced: Stop acquisition (data overrun)
 * - Prevents data corruption and buffer overflow
 *
 * Timing Requirements:
 * - At 1 kSPS, this function must complete in <1ms
 * - SPI transfers take ~200μs each
 * - Total processing: ~500μs leaves margin for other tasks
 *
 * @note Must be called frequently from main loop (polled architecture)
 * @note Actual data packaging happens in spim_handler interrupt
 * @note Sets drdy_served=false to detect missed samples
 */
void process_ads_data(void) {
  // LOG_INF("Processing ADS data...");
  if (ads_data_ready) {
    // LOG_INF("ADS DATA READY interrupt received");
    // Clear flag first to avoid missing next interrupt
    ads_data_ready = false;

    // Process received data as needed
    if (Get_ADS_Function() == READ) {

      if (skip_reads) {
        if (skiped_samples++ == 500) {
          skip_reads = false;
          skiped_samples = 0;
        }
      }

      if (!skip_reads) {

        if (!drdy_served) {
          Set_ADS_Function(STOP);
        } else {
          drdy_served = false;

          // Read data from ADS using SPI
          // uint8_t rx_data[27]; // Adjust size based on your data format
          spi_xfer_done = false;
          ads_to_read = ADS1298_A;
          ads1298_read_samples(ads_rx_buf, 27, ADS1298_A);
          while (spi_xfer_done == false)
            ;
          spi_xfer_done = false;
          ads_to_read = ADS1298_B;
          ads1298_read_samples(ads_rx_buf, 27, ADS1298_B);
          while (spi_xfer_done == false)
            ;
        }
      }
    }
  }
  k_sleep(K_USEC(1));
}
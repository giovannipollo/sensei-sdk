/*
 * ----------------------------------------------------------------------
 *
 * File: main.c
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

/**
 * @file main.c
 * @brief Simple Hello World example demonstrating UART communication between
 *        nRF5340 and GAP9.
 *
 * This example shows bidirectional UART communication:
 * - nRF receives messages from GAP9 and logs them via RTT
 * - nRF can optionally send responses back to GAP9
 */

#include <stdio.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/kernel.h>

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>

#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>

#include "bsp/pwr_bsp.h"
#include "pwr/pwr.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

/* UART configuration */
#define UART_GAP_NODE DT_NODELABEL(uart_gap)
#define UART_BUF_SIZE 128
#define UART_WAIT_FOR_RX 50000  /* Timeout in microseconds */
#define UART_WAIT_FOR_BUF_DELAY K_MSEC(50)

/* Device handles */
static const struct device *const uart_gap = DEVICE_DT_GET(UART_GAP_NODE);

/* UART data structure */
struct uart_data_t {
  void *fifo_reserved;
  uint8_t data[UART_BUF_SIZE];
  uint16_t len;
};

/* Work item for UART re-enabling */
static struct k_work_delayable uart_gap_work;

/* FIFO for received data */
static K_FIFO_DEFINE(fifo_uart_gap_rx_data);

/**
 * @brief UART async callback handler for GAP9 communication
 */
static void uart_gap_cb(const struct device *dev, struct uart_event *evt, void *user_data) {
  ARG_UNUSED(dev);
  ARG_UNUSED(user_data);

  static size_t aborted_len;
  struct uart_data_t *buf;
  static uint8_t *aborted_buf;
  static bool disable_req;

  switch (evt->type) {
  case UART_TX_DONE:
    LOG_DBG("[GAP UART] TX_DONE");
    if ((evt->data.tx.len == 0) || (!evt->data.tx.buf)) {
      return;
    }

    if (aborted_buf) {
      buf = CONTAINER_OF(aborted_buf, struct uart_data_t, data[0]);
      aborted_buf = NULL;
      aborted_len = 0;
    } else {
      buf = CONTAINER_OF(evt->data.tx.buf, struct uart_data_t, data[0]);
    }

    k_free(buf);
    break;

  case UART_RX_RDY:
    LOG_DBG("[GAP UART] RX_RDY: %d bytes", evt->data.rx.len);
    buf = CONTAINER_OF(evt->data.rx.buf, struct uart_data_t, data[0]);
    buf->len += evt->data.rx.len;

    if (disable_req) {
      return;
    }

    /* Check for end of line */
    if ((evt->data.rx.buf[buf->len - 1] == '\n') || (evt->data.rx.buf[buf->len - 1] == '\r')) {
      disable_req = true;
      uart_rx_disable(uart_gap);
    }
    break;

  case UART_RX_DISABLED:
    LOG_DBG("[GAP UART] RX_DISABLED");
    disable_req = false;

    buf = k_malloc(sizeof(*buf));
    if (buf) {
      buf->len = 0;
    } else {
      LOG_WRN("Failed to allocate UART RX buffer");
      k_work_reschedule(&uart_gap_work, UART_WAIT_FOR_BUF_DELAY);
      return;
    }

    uart_rx_enable(uart_gap, buf->data, sizeof(buf->data), UART_WAIT_FOR_RX);
    break;

  case UART_RX_BUF_REQUEST:
    LOG_DBG("[GAP UART] RX_BUF_REQUEST");
    buf = k_malloc(sizeof(*buf));
    if (buf) {
      buf->len = 0;
      uart_rx_buf_rsp(uart_gap, buf->data, sizeof(buf->data));
    } else {
      LOG_WRN("Failed to allocate UART RX buffer for request");
    }
    break;

  case UART_RX_BUF_RELEASED:
    LOG_DBG("[GAP UART] RX_BUF_RELEASED");
    buf = CONTAINER_OF(evt->data.rx_buf.buf, struct uart_data_t, data[0]);

    if (buf->len > 0) {
      k_fifo_put(&fifo_uart_gap_rx_data, buf);
    } else {
      k_free(buf);
    }
    break;

  case UART_TX_ABORTED:
    LOG_DBG("[GAP UART] TX_ABORTED");
    if (!aborted_buf) {
      aborted_buf = (uint8_t *)evt->data.tx.buf;
    }
    aborted_len += evt->data.tx.len;
    buf = CONTAINER_OF((void *)aborted_buf, struct uart_data_t, data);
    uart_tx(uart_gap, &buf->data[aborted_len], buf->len - aborted_len, SYS_FOREVER_MS);
    break;

  default:
    break;
  }
}

/**
 * @brief Work handler to re-enable UART RX after allocation failure
 */
static void uart_gap_work_handler(struct k_work *item) {
  struct uart_data_t *buf;

  buf = k_malloc(sizeof(*buf));
  if (buf) {
    buf->len = 0;
    uart_rx_enable(uart_gap, buf->data, sizeof(buf->data), UART_WAIT_FOR_RX);
  } else {
    LOG_WRN("Failed to allocate UART RX buffer in work handler");
    k_work_reschedule(&uart_gap_work, UART_WAIT_FOR_BUF_DELAY);
  }
}

/**
 * @brief Send a message to GAP9 over UART
 */
static int send_to_gap9(const char *msg) {
  struct uart_data_t *buf = k_malloc(sizeof(*buf));
  if (!buf) {
    LOG_ERR("Failed to allocate TX buffer");
    return -ENOMEM;
  }

  size_t len = strlen(msg);
  if (len > UART_BUF_SIZE) {
    len = UART_BUF_SIZE;
  }

  memcpy(buf->data, msg, len);
  buf->len = len;

  int ret = uart_tx(uart_gap, buf->data, buf->len, SYS_FOREVER_MS);
  if (ret) {
    LOG_ERR("Failed to send UART TX: %d", ret);
    k_free(buf);
    return ret;
  }

  return 0;
}

int main(void) {
  LOG_INIT();

  LOG_INF("===========================================");
  LOG_INF("  nRF <-> GAP9 Hello World Example");
  LOG_INF("===========================================");
  LOG_INF("Board: %s", CONFIG_BOARD);

  /**************************************************************************/
  /*    Power Management                                                    */
  /**************************************************************************/
  if (pwr_init()) {
    LOG_ERR("PWR Init failed!");
    return -1;
  }

  /**************************************************************************/
  /*    Device Initialization                                               */
  /**************************************************************************/
  if (!device_is_ready(uart_gap)) {
    LOG_ERR("GAP UART device not ready");
    return -1;
  }
  LOG_INF("GAP UART device ready");

  /**************************************************************************/
  /*    UART Configuration                                                  */
  /**************************************************************************/
  k_work_init_delayable(&uart_gap_work, uart_gap_work_handler);

  if (uart_callback_set(uart_gap, uart_gap_cb, NULL)) {
    LOG_ERR("Failed to set GAP UART callback");
    return -1;
  }
  LOG_INF("GAP UART callback configured");

  /* Allocate initial RX buffer and enable reception */
  struct uart_data_t *rx_buf = k_malloc(sizeof(*rx_buf));
  if (!rx_buf) {
    LOG_ERR("Failed to allocate initial RX buffer");
    return -1;
  }
  rx_buf->len = 0;

  if (uart_rx_enable(uart_gap, rx_buf->data, sizeof(rx_buf->data), UART_WAIT_FOR_RX)) {
    LOG_ERR("Failed to enable UART RX");
    k_free(rx_buf);
    return -1;
  }
  LOG_INF("UART RX enabled");

  /**************************************************************************/
  /*    Start GAP9                                                          */
  /**************************************************************************/
  pwr_start();
  LOG_INF("Power started - GAP9 should be booting...");

  k_msleep(500);  /* Give GAP9 time to boot */

  /**************************************************************************/
  /*    Main Loop - Process received messages                               */
  /**************************************************************************/
  LOG_INF("Waiting for messages from GAP9...");

  uint32_t msg_count = 0;

  while (1) {
    /* Wait for data from GAP9 */
    struct uart_data_t *buf = k_fifo_get(&fifo_uart_gap_rx_data, K_MSEC(1000));

    if (buf) {
      msg_count++;

      /* Null-terminate the received data for printing */
      if (buf->len < UART_BUF_SIZE) {
        buf->data[buf->len] = '\0';
      } else {
        buf->data[UART_BUF_SIZE - 1] = '\0';
      }

      /* Remove trailing newline/carriage return for cleaner logging */
      for (int i = buf->len - 1; i >= 0; i--) {
        if (buf->data[i] == '\n' || buf->data[i] == '\r') {
          buf->data[i] = '\0';
        } else {
          break;
        }
      }

      LOG_INF("[RX #%u] %s", msg_count, buf->data);

      /* Optional: Send acknowledgment back to GAP9 */
      char ack_msg[64];
      snprintf(ack_msg, sizeof(ack_msg), "nRF ACK #%u\n", msg_count);
      send_to_gap9(ack_msg);

      k_free(buf);
    }
  }

  return 0;
}

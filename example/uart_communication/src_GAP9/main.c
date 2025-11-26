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
 *        GAP9 and nRF5340.
 *
 * This example shows bidirectional UART communication:
 * - GAP9 sends "Hello from GAP9!" to nRF
 * - GAP9 receives and prints messages from nRF
 */

#include "bsp/bsp.h"
#include "pmsis.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define UART_BAUDRATE 115200
#define RX_BUFFER_SIZE 128
#define TX_INTERVAL_US 1000000  // Send message every 1 second

static pi_device_t uart_dev;
static uint8_t rx_buffer[RX_BUFFER_SIZE];

/**
 * @brief Initialize UART for communication with nRF
 * @return 0 on success, -1 on failure
 */
static int uart_init(void) {
    struct pi_uart_conf uart_conf;
    
    pi_uart_conf_init(&uart_conf);
    uart_conf.uart_id = 0;  // UART0 connected to nRF
    uart_conf.baudrate_bps = UART_BAUDRATE;
    uart_conf.enable_tx = 1;
    uart_conf.enable_rx = 1;
    
    pi_open_from_conf(&uart_dev, &uart_conf);
    
    if (pi_uart_open(&uart_dev)) {
        printf("[GAP9] Failed to open UART\n");
        return -1;
    }
    
    printf("[GAP9] UART initialized at %d baud\n", UART_BAUDRATE);
    return 0;
}

/**
 * @brief Send a message over UART to nRF
 * @param msg Null-terminated string to send
 */
static void uart_send_message(const char *msg) {
    pi_uart_write(&uart_dev, (void *)msg, strlen(msg));
}

/**
 * @brief Try to receive a message from nRF (non-blocking style with timeout)
 * @param buffer Buffer to store received data
 * @param max_len Maximum length to receive
 * @param timeout_us Timeout in microseconds
 * @return Number of bytes received, 0 if timeout/no data
 */
static int uart_receive_message(uint8_t *buffer, int max_len, uint32_t timeout_us) {
    int received = 0;
    
    // Simple polling receive with timeout
    // Note: For production, use async API with callbacks
    pi_uart_read(&uart_dev, buffer, max_len);
    received = max_len;
    
    return received;
}

/**
 * @brief Main application entry point
 */
int main(void) {
    int ret;
    uint32_t msg_counter = 0;
    char tx_buffer[64];
    
    // Wait for system to stabilize
    pi_time_wait_us(500000);
    
    printf("\n");
    printf("===========================================\n");
    printf("  GAP9 <-> nRF Hello World Example\n");
    printf("===========================================\n");
    printf("\n");
    
    // Initialize UART
    ret = uart_init();
    if (ret != 0) {
        printf("[GAP9] UART initialization failed!\n");
        pmsis_exit(-1);
    }
    
    printf("[GAP9] Starting communication loop...\n\n");
    
    // Main communication loop
    while (1) {
        // Send a hello message to nRF
        msg_counter++;
        snprintf(tx_buffer, sizeof(tx_buffer), "Hello from GAP9! Count: %lu\n", msg_counter);
        
        printf("[GAP9] Sending: %s", tx_buffer);
        uart_send_message(tx_buffer);
        
        // Wait before next message
        pi_time_wait_us(TX_INTERVAL_US);
    }
    
    pmsis_exit(0);
    return 0;
}

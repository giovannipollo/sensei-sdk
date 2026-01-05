/*
 * ----------------------------------------------------------------------
 *
 * File: system_status.h
 *
 * Last edited: 05.01.2026
 *
 * Copyright (C) 2026, ETH Zurich and University of Bologna.
 *
 * ----------------------------------------------------------------------
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SYSTEM_STATUS_H_
#define SYSTEM_STATUS_H_

#include <stdint.h>
#include <stddef.h>

/**
 * @brief Initialize the system status aggregator.
 * 
 * @return 0 on success, negative on error.
 */
int system_status_init(void);

/**
 * @brief Build a BLE response packet for the system status.
 * 
 * Aggregates battery and sensor information into a format suitable for BLE transmission.
 * 
 * @param buffer Pointer to the buffer where the packet will be built.
 * @param buf_size Size of the buffer.
 * @param out_len Pointer to store the resulting packet length.
 * @return 0 on success, negative on error (e.g., buffer too small).
 */
int system_status_build_ble_packet(uint8_t *buffer, size_t buf_size, size_t *out_len);

#endif /* SYSTEM_STATUS_H_ */

/*
 * ----------------------------------------------------------------------
 *
 * File: i2c_helpers.h
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

#ifndef I2C_HELPERS_H
#define I2C_HELPERS_H

#include <stdint.h>

#include <zephyr/device.h>

// With addresses on 7 bits, we can have 128 peripherals maximum, per interface.
// See I2C documentation for more details.
#define MAX_PERIPHERALS 128

void resole_address_to_name(uint8_t address, char *name);

void i2c_scan(const struct device *dev);

#endif /* I2C_HELPERS_H */
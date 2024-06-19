/*
 * ----------------------------------------------------------------------
 *
 * File: bsp_sensei_basev1.h
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

#pragma once

#include <stdint.h>

#include "pmsis.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __GAP_NRFSPI__

/// @brief  Open necessary features that belongs to the board to be able to use
///         an NRFSPI device correctly.
/// @param  device      Pointer to NRFSPI device instance.
/// @return int32_t     Error management.
///                     @arg PI_OK : The operation succeed.
///                     @arg PI_FAIL : The operation failed.
int32_t bsp_nrfspi_open(pi_device_t *device);

/// @brief  Close and reset involved features by the NRFSPI device.
/// @param  device      Pointer to NRFSPI device instance.
/// @return int32_t     Error management.
///                     @arg PI_OK : The operation succeed.
///                     @arg PI_FAIL : The operation failed.
int32_t bsp_nrfspi_close(pi_device_t *device);

#endif

#ifdef CONFIG_MX25U51245G
int bsp_mx25u51245g_open(pi_device_t *device);
int bsp_mx25u51245g_close(pi_device_t *device);
#endif

#ifdef CONFIG_APS256XXN
int bsp_aps256xxn_open(pi_device_t *device);
int bsp_aps256xxn_close(pi_device_t *device);
#endif

#ifdef PRINTF_UART
int32_t bsp_printf_uart_open(pi_device_t *device);
#endif // PRINTF_UART

#ifndef pi_default_flash_conf
#ifdef __GAP_DRIVER_MRAM_DEFAULT__
#define pi_default_flash_conf pi_mram_conf
#else
#ifdef __GAP_DRIVER_MX25U51245G_DEFAULT__
#define pi_default_flash_conf pi_mx25u51245g_conf
#endif
#endif
#endif

#ifndef pi_default_flash_conf_init
#ifdef __GAP_DRIVER_MRAM_DEFAULT__
#define pi_default_flash_conf_init pi_mram_conf_init
#else
#ifdef __GAP_DRIVER_MX25U51245G_DEFAULT__
#define pi_default_flash_conf_init pi_mx25u51245g_conf_init
#endif
#endif
#endif

#ifndef pi_default_ram_conf
#ifdef __GAP_DRIVER_APS256XXN_DEFAULT__
#define pi_default_ram_conf pi_aps256xxn_conf
#endif
#endif

#ifndef pi_default_ram_conf_init
#ifdef __GAP_DRIVER_APS256XXN_DEFAULT__
#define pi_default_ram_conf_init pi_aps256xxn_conf_init
#endif
#endif

#ifdef __cplusplus
}
#endif
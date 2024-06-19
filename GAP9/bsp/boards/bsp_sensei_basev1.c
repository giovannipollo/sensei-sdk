/*
 * ----------------------------------------------------------------------
 *
 * File: bsp_sensei_basev1.c
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

#include "bsp_sensei_basev1.h"
#include "bsp/bsp.h"

// NRFSPI **********************************************************************
#if defined CONFIG_NRFSPI
void bsp_nrfspi_conf_init(struct pi_nrfspi_conf *conf) { conf->gpio_irq = PAD_NRF_IRQ; }

int32_t bsp_nrfspi_open(pi_device_t *device) {
  NRFSPI_LOG_DBG("Opening BSP\n");

  pi_assert(device);
  pi_assert(device->config);
  pi_nrfspi_conf_t *nrfspi_conf = (pi_nrfspi_conf_t *)device->config;

  bsp_nrfspi_conf_init(nrfspi_conf);

  // Set the GPIOs
  pi_pad_function_set(nrfspi_conf->gpio_irq, PI_PAD_FUNC1); // GPIO
  pi_gpio_pin_configure(nrfspi_conf->gpio_irq, PI_GPIO_OUTPUT | PI_GPIO_PULL_DISABLE | PI_GPIO_DRIVE_STRENGTH_LOW);

  return 0;
}

int32_t bsp_nrfspi_close(pi_device_t *device) { return 0; }

#endif

// FLASH ***********************************************************************
#if defined CONFIG_MX25U51245G

void bsp_mx25u51245g_conf_init(struct pi_mx25u51245g_conf *conf) {
  conf->spi_itf = DT_MX25U51245G_SPI_ITF;
  conf->spi_cs = DT_MX25U51245G_SPI_CS;
  conf->baudrate = DT_MX25U51245G_BAUDRATE;
  conf->sector_size = DT_MX25U51245G_SECTOR_SIZE;

  // workaround for hw limitation on gap9
  // conf->mba = DT_MX25U51245G_SPI_CS ? (DT_MX25U51245G_SIZE > DT_APS256XXN_SIZE ? DT_MX25U51245G_SIZE :
  // DT_APS256XXN_SIZE) : 0UL;
  conf->mba = 0UL;
}

int bsp_mx25u51245g_open(pi_device_t *device) {
  pi_err_t err = PI_OK;
  pi_device_t *mem1v8 = NULL;

  // Enabling 3V3CAM power supply
  err = pi_open(PI_POWERSUPPLY_MEM1V8, &mem1v8);

  if (err) {
    return err;
  }

  // Wait for a short moment  to ensure power is stable
  pi_time_wait_us(100000);

  struct pi_flash_conf *conf = (struct pi_flash_conf *)device->config;
  pi_flash_api_t *api = (pi_flash_api_t *)conf->api;
  device->api = (struct pi_device_api *)api;
  return api->open(device);
}

int bsp_mx25u51245g_close(pi_device_t *device) {
  pi_err_t err = PI_OK;
  pi_device_t *mem1v8 = NULL;

  // Enabling 3V3CAM power supply
  err = pi_close(PI_POWERSUPPLY_MEM1V8);

  if (err) {
    return err;
  }

  struct pi_flash_conf *conf = (struct pi_flash_conf *)device->config;
  pi_flash_api_t *api = (pi_flash_api_t *)conf->api;
  device->api = (struct pi_device_api *)api;
  return api->close(device);
}

#endif

// RAM *************************************************************************
#if defined CONFIG_APS256XXN

void bsp_aps256xxn_conf_init(struct pi_aps256xxn_conf *conf) {
  conf->ram_start = DT_APS256XXN_RAM_START;
  conf->ram_size = DT_APS256XXN_RAM_SIZE;
  conf->spi_itf = DT_APS256XXN_SPI_ITF;
  conf->spi_cs = DT_APS256XXN_SPI_CS;
  conf->baudrate = DT_APS256XXN_BAUDRATE;
  // workaround for hw limitation on gap9
  // conf->mba = DT_APS256XXN_SPI_CS ? (DT_MX25U51245G_SIZE > DT_APS256XXN_SIZE ? DT_MX25U51245G_SIZE :
  // DT_APS256XXN_SIZE) : 0UL;
  conf->mba = DT_APS256XXN_RAM_SIZE;
}

int bsp_aps256xxn_open(pi_device_t *device) {
  pi_err_t err = PI_OK;
  pi_device_t *mem1v8 = NULL;

  // Enabling 3V3CAM power supply
  err = pi_open(PI_POWERSUPPLY_MEM1V8, &mem1v8);

  if (err) {
    return err;
  }

  // Wait for a short moment  to ensure power is stable
  pi_time_wait_us(100000);

  struct pi_ram_conf *conf = (struct pi_ram_conf *)device->config;
  pi_ram_api_t *api = (pi_ram_api_t *)conf->api;
  device->api = (struct pi_device_api *)api;
  return api->open(device);
}

int bsp_aps256xxn_close(pi_device_t *device) {
  pi_err_t err = PI_OK;
  pi_device_t *mem1v8 = NULL;

  // Enabling 3V3CAM power supply
  err = pi_close(PI_POWERSUPPLY_MEM1V8);

  struct pi_ram_conf *conf = (struct pi_ram_conf *)device->config;
  pi_ram_api_t *api = (pi_ram_api_t *)conf->api;
  device->api = (struct pi_device_api *)api;
  return api->open(device);
}
#endif

// IO **************************************************************************

#ifdef PRINTF_UART
int32_t bsp_printf_uart_open(pi_device_t *device) {
  pi_pad_mux_group_set(PAD_PRINTF_UART_TX, DT_PRINTF_UART_MUXGROUP_TX);
  pi_pad_mux_group_set(PAD_PRINTF_UART_RX, DT_PRINTF_UART_MUXGROUP_RX);

#if defined(CONFIG_IO_UART_FLOW_CONTROL)
  pi_pad_function_set(DT_PRINTF_UART_PAD_CTS, PI_PAD_FUNC0);
  pi_pad_function_set(DT_PRINTF_UART_PAD_RTS, PI_PAD_FUNC0);
  pi_pad_mux_group_set(DT_PRINTF_UART_PAD_CTS, DT_PRINTF_UART_MUXGROUP_CTS);
  pi_pad_mux_group_set(DT_PRINTF_UART_PAD_RTS, DT_PRINTF_UART_MUXGROUP_RTS);
#endif

  return 0;
}

#endif // PRINTF_UART

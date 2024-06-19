/*
 * ----------------------------------------------------------------------
 *
 * File: bsp_sensei_sensorv1.c
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

#include "bsp_sensei_sensorv1.h"
#include "bsp/bsp.h"

// Cameras *********************************************************************

#if defined CONFIG_HM0360
void bsp_hm0360_conf_init(struct pi_hm0360_conf *conf) {
  conf->gpio_xsleep = PAD_HM0360_XLEEP;
  conf->gpio_xshdown = PAD_HM0360_XSHDOWN;
  conf->i2c_itf = DT_HM0360_I2C_ITF;
  conf->cpi_itf = DT_HM0360_CPI_ITF;
  conf->csi2_itf = DT_HM0360_CSI2_ITF;
  conf->i3c_itf = DT_HM0360_I3C_ITF;
  conf->bsp_open = DT_HM0360_BSP_OPEN;
  conf->bsp_close = DT_HM0360_BSP_CLOSE;
}

int32_t bsp_hm0360_open(pi_device_t *device) {
  HM0360_LOG_TRC("Opening BSP\n");

  pi_assert(device);
  pi_assert(device->config);
  pi_hm0360_conf_t *hm0360_conf = (pi_hm0360_conf_t *)device->config;

  bsp_hm0360_conf_init(hm0360_conf);

  // Set the GPIOs
  pi_pad_function_set(PAD_HM0360_CSI_PWR, PI_PAD_FUNC1); // GPIO
  pi_gpio_pin_configure(PAD_HM0360_CSI_PWR, PI_GPIO_OUTPUT | PI_GPIO_PULL_DISABLE | PI_GPIO_DRIVE_STRENGTH_LOW);

  pi_pad_function_set(hm0360_conf->gpio_xsleep, PI_PAD_FUNC1); // GPIO
  pi_gpio_pin_configure(hm0360_conf->gpio_xsleep, PI_GPIO_OUTPUT | PI_GPIO_PULL_DISABLE | PI_GPIO_DRIVE_STRENGTH_LOW);
  pi_pad_function_set(hm0360_conf->gpio_xshdown, PI_PAD_FUNC1); // GPIO
  pi_gpio_pin_configure(hm0360_conf->gpio_xshdown, PI_GPIO_OUTPUT | PI_GPIO_PULL_DISABLE | PI_GPIO_DRIVE_STRENGTH_LOW);

  pi_pad_function_set(PI_PAD_042, PI_PAD_FUNC0); // I3C SDA
  pi_pad_function_set(PI_PAD_043, PI_PAD_FUNC0); // I3C SCL

  // Enable the CSI power domain
  pi_gpio_pin_write(PAD_HM0360_CSI_PWR, 1);
  pi_time_wait_us(10000);
  pi_gpio_pin_write(hm0360_conf->gpio_xsleep, 1);
  pi_time_wait_us(10000);
  pi_gpio_pin_write(hm0360_conf->gpio_xshdown, 1);

  return 0;
}

int32_t bsp_hm0360_close(pi_device_t *device) {
  HM0360_LOG_TRC("Closing BSP\n");

  pi_assert(device);
  pi_assert(device->config);
  pi_hm0360_conf_t *hm0360_conf = (pi_hm0360_conf_t *)device->config;

  // Disable the camera
  pi_gpio_pin_write(hm0360_conf->gpio_xshdown, 0);
  pi_time_wait_us(10000);
  pi_gpio_pin_write(hm0360_conf->gpio_xsleep, 0);
  pi_time_wait_us(10000);
  pi_gpio_pin_write(PAD_HM0360_CSI_PWR, 0);

  // Reset i2c pad to default GPIO
  pi_pad_function_set(PI_PAD_042, PI_PAD_FUNC1); // I2C SDA
  pi_pad_function_set(PI_PAD_043, PI_PAD_FUNC1); // I2C SCL

  return 0;
}
#endif
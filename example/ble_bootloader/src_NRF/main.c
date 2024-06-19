/*
 * ----------------------------------------------------------------------
 *
 * File: main.c
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

#include <stdio.h>
#include <string.h>

#include <zephyr/dfu/mcuboot.h>

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/ring_buffer.h>

#include <zephyr/stats/stats.h>

#include <zephyr/drivers/uart.h>

#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>

#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/usbd.h>

#include "pwr/pwr.h"
#include "pwr/pwr_common.h"
#include "pwr/thread_pwr.h"
#include <flash_map_pm.h>

#include "common.h"

LOG_MODULE_REGISTER(main);

#define SLOT1_LABEL mcuboot_secondary_app

/* Define an example stats group; approximates seconds since boot. */
STATS_SECT_START(smp_svr_stats)
STATS_SECT_ENTRY(ticks)
STATS_SECT_END;

/* Assign a name to the `ticks` stat. */
STATS_NAME_START(smp_svr_stats)
STATS_NAME(smp_svr_stats, ticks)
STATS_NAME_END(smp_svr_stats);

/* Define an instance of the stats group. */
STATS_SECT_DECL(smp_svr_stats) smp_svr_stats;

int main(void) {

  LOG_INIT();

  int rc = STATS_INIT_AND_REG(smp_svr_stats, STATS_SIZE_32, "smp_svr_stats");
  if (IS_ENABLED(CONFIG_USB_DEVICE_STACK)) {
    rc = usb_enable(NULL);

    /* Ignore EALREADY error as USB CDC is likely already initialised */
    if (rc != 0 && rc != -EALREADY) {
      LOG_ERR("Failed to enable USB");
      return 0;
    }
  }
  if (rc < 0) {
    LOG_ERR("Error initializing stats system [%d]", rc);
  }

  LOG_INF("BLE with SMP Server on %s V1 ", CONFIG_BOARD);

  bool image_ok = boot_is_img_confirmed();
  LOG_INF("Image is%s confirmed OK", image_ok ? "" : " not");

  // Potentially auto-confirm image and erase second slot
  // if (!image_ok) {
  // 	int ret = boot_write_img_confirmed();
  // 	if (ret < 0) {
  // 		LOG_ERR("Couldn't confirm this image: %d", ret);
  // 		return ret;
  // 	}

  // 	LOG_INF("Marked image as OK");
  // 	ret = boot_erase_img_bank(FIXED_PARTITION_ID(slot1_partition));
  // 	if (ret) {
  // 		LOG_ERR("Failed to erase second slot: %d", ret);
  // 		return ret;
  // 	}
  // }

  // Initialize and start power management
  pwr_init();
  pwr_start();

#ifdef CONFIG_MCUMGR_TRANSPORT_BT
  start_smp_bluetooth_adverts();
#endif

  /* using __TIME__ ensure that a new binary will be built on every
   * compile which is convenient when testing firmware upgrade.
   */
  LOG_INF("build time: " __DATE__ " " __TIME__);

  k_msleep(100);

  while (1) {
    k_sleep(K_MSEC(1000));
    STATS_INC(smp_svr_stats, ticks);
    // LOG_INF("Ticks: %d", smp_svr_stats.ticks);
  }
  return 0;
}

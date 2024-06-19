/*
 * ----------------------------------------------------------------------
 *
 * File: as7331_reg.c
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

#include "as7331_reg.h"

int32_t as7331_write_reg(const as7331_ctx_t *ctx, uint8_t reg, uint8_t *data, uint16_t len) {
  if (!ctx)
    return -1;
  return ctx->write_reg(ctx->handle, reg, data, len);
}

int32_t as7331_read_reg(const as7331_ctx_t *ctx, uint8_t reg, uint8_t *data, uint16_t len) {
  if (!ctx)
    return -1;
  return ctx->read_reg(ctx->handle, reg, data, len);
}

int as7331_init(as7331_t *sensor, uint8_t mmode, uint8_t cclk, uint8_t sb, uint8_t break_time, uint8_t gain, uint8_t time) {
  //   set measurement mode (bits 6,7), standby on/off (bit 4)
  //   and internal clk (bits 0,1); bit 3 determines ready interrupt configuration, 0 means push pull
  //   1 means open drain

  uint8_t val;

  val = (gain << 4) | time;
  if (as7331_write_reg(&sensor->ctx, AS7331_CREG1, &val, 1)) return -1;

  val = (mmode << 6) | (sb << 4) | cclk;
  if (as7331_write_reg(&sensor->ctx, AS7331_CREG3, &val, 1)) return -1;

  val = break_time;
  return as7331_write_reg(&sensor->ctx, AS7331_BREAK, &val, 1);
}

int as7331_power_down(as7331_t *sensor) {
  uint8_t temp;
  if (as7331_read_reg(&sensor->ctx, AS7331_OSR, &temp, 1)) return -1;
  temp |= AS7331_POWER_DOWN_MASK;
  return as7331_write_reg(&sensor->ctx, AS7331_OSR, &temp, 1);
}

int as7331_power_up(as7331_t *sensor) {
  uint8_t temp;
  if (as7331_read_reg(&sensor->ctx, AS7331_OSR, &temp, 1)) return -1;
  temp &= AS7331_POWER_UP_MASK;
  return as7331_write_reg(&sensor->ctx, AS7331_OSR, &temp, 1);
}

int as7331_reset(as7331_t *sensor) {
  uint8_t temp;
  if (as7331_read_reg(&sensor->ctx, AS7331_OSR, &temp, 1)) return -1;
  temp |= AS7331_RESET_MASK;
  return as7331_write_reg(&sensor->ctx, AS7331_OSR, &temp, 1);
}

int as7331_set_configuration_mode(as7331_t *sensor) {
  uint8_t temp;
  if (as7331_read_reg(&sensor->ctx, AS7331_OSR, &temp, 1)) return -1;
  temp |= AS7331_CFG_MODE_MASK;
  return as7331_write_reg(&sensor->ctx, AS7331_OSR, &temp, 1);
}
int as7331_set_measurement_mode(as7331_t *sensor) {
  uint8_t temp;
  if (as7331_read_reg(&sensor->ctx, AS7331_OSR, &temp, 1)) return -1;
  temp |= AS7331_MEAS_MODE_MASK;
  return as7331_write_reg(&sensor->ctx, AS7331_OSR, &temp, 1);
}

int as7331_start_measurement(as7331_t *sensor) {
  uint8_t temp;
  if (as7331_read_reg(&sensor->ctx, AS7331_OSR, &temp, 1)) return -1;
  temp |= AS7331_START_MASK;
  return as7331_write_reg(&sensor->ctx, AS7331_OSR, &temp, 1);
}

int as7331_get_chip_id(as7331_t *sensor, uint8_t *id) { return as7331_read_reg(&sensor->ctx, AS7331_AGEN, id, 1); }

int as7331_get_status(as7331_t *sensor, as7331_reg_osrstat_t *status) {
  uint8_t buf[2];
  int ret = as7331_read_reg(&sensor->ctx, AS7331_STATUS, buf, 2);
  if (ret)
    return ret;
  status->word = ((uint16_t)buf[1] << 8) | buf[0];
  return 0;
}

int as7331_read_temp(as7331_t *sensor, uint16_t *temp) {
  uint8_t buf[2];
  int ret = as7331_read_reg(&sensor->ctx, AS7331_TEMP, buf, 2);
  if (ret)
    return ret;
  *temp = ((uint16_t)buf[1] << 8) | buf[0];
  return 0;
}

int as7331_read_uva(as7331_t *sensor, uint16_t *uva) {
  uint8_t buf[2];
  int ret = as7331_read_reg(&sensor->ctx, AS7331_MRES1, buf, 2);
  if (ret)
    return ret;
  *uva = ((uint16_t)buf[1] << 8) | buf[0];
  return 0;
}

int as7331_read_uvb(as7331_t *sensor, uint16_t *uvb) {
  uint8_t buf[2];
  int ret = as7331_read_reg(&sensor->ctx, AS7331_MRES2, buf, 2);
  if (ret)
    return ret;
  *uvb = ((uint16_t)buf[1] << 8) | buf[0];
  return 0;
}

int as7331_read_uvc(as7331_t *sensor, uint16_t *uvc) {
  uint8_t buf[2];
  int ret = as7331_read_reg(&sensor->ctx, AS7331_MRES3, buf, 2);
  if (ret)
    return ret;
  *uvc = ((uint16_t)buf[1] << 8) | buf[0];
  return 0;
}

int as7331_read_all(as7331_t *sensor, uint16_t *dest) {
  uint8_t buf[8];
  int ret = as7331_read_reg(&sensor->ctx, AS7331_TEMP, buf, 8);
  if (ret)
    return ret;
  dest[0] = ((uint16_t)buf[1] << 8) | buf[0]; // Temp
  dest[1] = ((uint16_t)buf[3] << 8) | buf[2]; // UVA
  dest[2] = ((uint16_t)buf[5] << 8) | buf[4]; // UVB
  dest[3] = ((uint16_t)buf[7] << 8) | buf[6]; // UVC
  return 0;
}

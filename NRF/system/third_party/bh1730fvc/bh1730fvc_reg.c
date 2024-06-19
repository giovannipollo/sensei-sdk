/*
 * ----------------------------------------------------------------------
 *
 * File: bh1730fvc_reg.c
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


#include "bh1730fvc_reg.h"

int32_t bh1730_write_reg(const bh1730_ctx_t *ctx, uint8_t reg, uint8_t *data, uint16_t len) {
    if (ctx == NULL) return -1;
    return ctx->write_reg(ctx->handle, BH1730_COMMAND | reg, data, len);
}

int32_t bh1730_read_reg(const bh1730_ctx_t *ctx, uint8_t reg, uint8_t *data, uint16_t len) {
    if (ctx == NULL) return -1;
    return ctx->read_reg(ctx->handle, BH1730_COMMAND | reg, data, len);
}

int bh1730_init(bh1730_t *sensor, uint8_t gain, uint8_t integration_time) {
    if (bh1730_write_reg(&sensor->ctx, BH1730_REG_TIMING, &integration_time, 1)) return -1;
    sensor->integration_time_us = ((uint32_t) 256 - integration_time) * 2700;

    if (bh1730_write_reg(&sensor->ctx, BH1730_REG_GAIN, &gain, 1)) return -1;

    switch (gain) {
        case BH1730_GAIN_X1:
            sensor->gain = 1;
            break;
        case BH1730_GAIN_X2:
            sensor->gain = 2;
            break;
        case BH1730_GAIN_X64:
            sensor->gain = 64;
            break;
        case BH1730_GAIN_X128:
            sensor->gain = 128;
            break;
        default:
            return -1;
    }

    uint8_t val = BIT_ADC_EN | BIT_POWER;
    return bh1730_write_reg(&sensor->ctx, BH1730_REG_CONTROL, &val, 1);
}

int bh1730_valid(bh1730_t *sensor, uint8_t *valid) {
    uint8_t val;
    if (bh1730_read_reg(&sensor->ctx, BH1730_REG_CONTROL, &val, 1)) return -1;
    if ((val & BIT_ADC_VALID) != 0) {
        *valid = 1;
    } else {
        *valid = 0;
    }
    return 0;
}

int bh1730_power_on(bh1730_t *sensor) {
    uint8_t val = BIT_POWER;
    return bh1730_write_reg(&sensor->ctx, BH1730_REG_CONTROL, &val, 1);
}

int bh1730_power_down(bh1730_t *sensor) {
    uint8_t val = 0x00;
    return bh1730_write_reg(&sensor->ctx, BH1730_REG_CONTROL, &val, 1);
}

int bh1730_read_visible(bh1730_t *sensor, uint16_t *visible) {
    uint8_t buf[2];
    if (bh1730_read_reg(&sensor->ctx, BH1730_REG_DATA0LOW, buf, 2)) return -1;

    *visible = (buf[1] << 8) | buf[0];
    return 0;
}

int bh1730_read_ir(bh1730_t *sensor, uint16_t *ir) {
    uint8_t buf[2];
    if (bh1730_read_reg(&sensor->ctx, BH1730_REG_DATA1LOW, buf, 2)) return -1;

    *ir = (buf[1] << 8) | buf[0];
    return 0;
}

int bh1730_read_lux(bh1730_t *sensor, uint32_t *lux) {
    uint8_t buf[4];
    if (bh1730_read_reg(&sensor->ctx, BH1730_REG_DATA0LOW, buf, 4)) return -1;

    uint32_t ch0 = 0;
    ch0 = (buf[1] << 8) | buf[0];
    uint32_t ch1 = 0;
    ch1 = (buf[3] << 8) | buf[2];

    if (ch0 == 0) {
        *lux = 0;
        return 0;
    }

    uint32_t ratio = (ch1 * 1000) / ch0;
    uint32_t calc_lux;

    if (ratio < 260)
        calc_lux = (1290 * ch0 - 2733 * ch1);
    else if (ratio < 550)
        calc_lux = (795 * ch0 - 859 * ch1);
    else if (ratio < 1090)
        calc_lux = (510 * ch0 - 345 * ch1);
    else if (ratio < 2130)
        calc_lux = (276 * ch0 - 130 * ch1);
    else
        calc_lux = 0;

    *lux = calc_lux * 100 / (sensor->gain * sensor->integration_time_us);

    return 0;
}

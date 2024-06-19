/*
 * ----------------------------------------------------------------------
 *
 * File: bh1730fvc_reg.h
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

/* Minimal example usage:

#include "bh1730.h"

int user_i2c_write(uint8_t device_addr, uint8_t reg_addr, uint8_t data);
int user_i2c_read(uint8_t device_addr, uint8_t reg_addr, uint8_t *data, size_t len);

int main(void) {
    bh1730_t sensor = {
        .i2c = {
            .i2c_write = user_i2c_write,
            .i2c_read = user_i2c_read,
            .device_address = 0x29
        }
    };

    uint32_t lux;

    if (bh1730_init(&sensor) != 0) {
        // Handle init error
    }

    if (bh1730_read_lux(&sensor, &lux) == 0) {
        // Use lux value
    }

    bh1730_power_down(&sensor);

    return 0;
}
*/

#ifndef BH1730FVC_REGS_H
#define BH1730FVC_REGS_H


#include <stdint.h>
#include <stddef.h>

#define BH1730_I2C_ADD           0x29

/* HW registers */
#define BH1730_COMMAND (0x80)

#define BH1730_REG_CONTROL (0x00)
#define BH1730_REG_TIMING (0x01)
#define BH1730_REG_INTERRUPT (0x02)
#define BH1730_REG_THLLOW (0x03)
#define BH1730_REG_THLHIGH (0x04)
#define BH1730_REG_THHLOW (0x05)
#define BH1730_REG_THHHIGH (0x06)
#define BH1730_REG_GAIN (0x07)
#define BH1730_REG_ID (0x12)
#define BH1730_REG_DATA0LOW (0x14)
#define BH1730_REG_DATA0HIGH (0x15)
#define BH1730_REG_DATA1LOW (0x16)
#define BH1730_REG_DATA1HIGH (0x17)

#define BIT_ADC_INTR 0x20
#define BIT_ADC_VALID 0x10
#define BIT_ONE_TIME 0x08
#define BIT_DATA_SEL 0x04
#define BIT_ADC_EN 0x02
#define BIT_POWER 0x01

#define BH1730_GAIN_X1           0x00
#define BH1730_GAIN_X2           0x01
#define BH1730_GAIN_X64          0x02
#define BH1730_GAIN_X128         0x03

#define BH1730_INT_50MS          0xED
#define BH1730_INT_100MS         0xDA
#define BH1730_INT_120MS         0xD3
#define BH1730_INT_200MS         0xB6

typedef int32_t (*write_ptr)(void *, uint8_t, const uint8_t *, uint16_t);
typedef int32_t (*read_ptr)(void *, uint8_t, uint8_t *, uint16_t);

typedef struct {
    write_ptr write_reg;
    read_ptr read_reg;
    void *handle;
} bh1730_ctx_t;

typedef struct {
    bh1730_ctx_t ctx;
    uint32_t integration_time_us;
    uint32_t gain;
} bh1730_t;

int32_t bh1730_write_reg(const bh1730_ctx_t *ctx, uint8_t reg, uint8_t *data, uint16_t len);
int32_t bh1730_read_reg(const bh1730_ctx_t *ctx, uint8_t reg, uint8_t *data, uint16_t len);

int bh1730_init(bh1730_t *sensor, uint8_t gain, uint8_t integration_time);
int bh1730_valid(bh1730_t *sensor, uint8_t *valid);
int bh1730_power_on(bh1730_t *sensor);
int bh1730_power_down(bh1730_t *sensor);
int bh1730_read_visible(bh1730_t *sensor, uint16_t *visible);
int bh1730_read_ir(bh1730_t *sensor, uint16_t *ir);
int bh1730_read_lux(bh1730_t *sensor, uint32_t *lux);

#endif /* BH1730FVC_REGS_H */
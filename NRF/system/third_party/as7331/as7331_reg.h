/*
 * ----------------------------------------------------------------------
 *
 * File: as7331_reg.h
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

#ifndef AS7331_REG_H
#define AS7331_REG_H

#include <stddef.h>
#include <stdint.h>

#define AS7331_I2C_ADD 0x74

// Configuration registers
#define AS7331_OSR 0x00
#define AS7331_AGEN 0x02
#define AS7331_CREG1 0x06
#define AS7331_CREG2 0x07
#define AS7331_CREG3 0x08
#define AS7331_BREAK 0x09

// Measurement registers
#define AS7331_STATUS 0x00
#define AS7331_TEMP 0x01
#define AS7331_MRES1 0x02
#define AS7331_MRES2 0x03
#define AS7331_MRES3 0x04

#define AS7331_POWER_DOWN_MASK (0x40)
#define AS7331_POWER_UP_MASK (~0x40)
#define AS7331_RESET_MASK (0x08)
#define AS7331_CFG_MODE_MASK (0x02)
#define AS7331_MEAS_MODE_MASK (0x03)
#define AS7331_START_MASK (0x80)

typedef int32_t (*write_ptr)(void *, uint8_t, const uint8_t *, uint16_t);
typedef int32_t (*read_ptr)(void *, uint8_t, uint8_t *, uint16_t);

typedef struct {
  write_ptr write_reg;
  read_ptr read_reg;
  void *handle;
} as7331_ctx_t;

typedef struct {
  as7331_ctx_t ctx;
} as7331_t;

typedef enum {
  AS7331_CONT_MODE = 0x00, // continuous mode
  AS7331_CMD_MODE = 0x01,  // force mode, one-time measurement
  AS7331_SYNS_MODE = 0x02,
  AS7331_SYND_MODE = 0x03
} MMODE;

typedef enum {
  AS7331_1024 = 0x00, // internal clock frequency, 1.024 MHz, etc
  AS7331_2048 = 0x01,
  AS7331_4096 = 0x02,
  AS7331_8192 = 0x03
} CCLK;


typedef union {
  struct {
    uint8_t dos : 3;       // Device Operating State   - OSR[0:2]
    uint8_t sw_res : 1;    // Software Reset           - OSR[3]
    uint8_t reserved : 2;  // Reserved, don't write.   - OSR[4:5]
    uint8_t pd : 1;        // Power Down Enabled       - OSR[6]
    uint8_t ss : 1;        // Start State              - OSR[7]
  };
  uint8_t byte;
} as7331_reg_osr_t;

typedef union {
  struct {
    as7331_reg_osr_t osr;     // See OSR configuration register above.        - OSRSTAT[0:7]
    uint8_t powerstate : 1;   // Power down state.                            - OSRSTAT[8]
    uint8_t standbystate : 1; // Standby mode state.                          - OSRSTAT[9]
    uint8_t notready : 1;     // Inverted value of the ready pin.             - OSRSTAT[10]
    uint8_t ndata : 1;        // Indicates new data available.                - OSRSTAT[11]
    uint8_t ldata : 1;        // Indicates data overwrite prior to retrieval. - OSRSTAT[12]
    uint8_t adcof : 1;        // OVF of at least one ADC channel.             - OSRSTAT[13]
    uint8_t mresof : 1;       // OVF of at least one of the MRES registers.   - OSRSTAT[14]
    uint8_t outconvof : 1;    // OVF of the internal 24-bit time reference.   - OSRSTAT[15]
  };
  uint16_t word;
} as7331_reg_osrstat_t;

int32_t as7331_write_reg(const as7331_ctx_t *ctx, uint8_t reg, uint8_t *data, uint16_t len);
int32_t as7331_read_reg(const as7331_ctx_t *ctx, uint8_t reg, uint8_t *data, uint16_t len);

int as7331_init(as7331_t *sensor, uint8_t mmode, uint8_t cclk, uint8_t sb, uint8_t break_time, uint8_t gain,
                uint8_t time);
int as7331_power_down(as7331_t *sensor);
int as7331_power_up(as7331_t *sensor);
int as7331_reset(as7331_t *sensor);
int as7331_set_configuration_mode(as7331_t *sensor);
int as7331_set_measurement_mode(as7331_t *sensor);
int as7331_start_measurement(as7331_t *sensor);

int as7331_get_chip_id(as7331_t *sensor, uint8_t *id);
int as7331_get_status(as7331_t *sensor, as7331_reg_osrstat_t *status);
int as7331_read_temp(as7331_t *sensor, uint16_t *temp);
int as7331_read_uva(as7331_t *sensor, uint16_t *uva);
int as7331_read_uvb(as7331_t *sensor, uint16_t *uvb);
int as7331_read_uvc(as7331_t *sensor, uint16_t *uvc);
int as7331_read_all(as7331_t *sensor, uint16_t *dest);

#endif // AS7331_REG_H
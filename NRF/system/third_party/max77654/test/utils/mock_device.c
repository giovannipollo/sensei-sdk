// Copyright (c) 2022 Philip Schilk.
// SPDX-License-Identifier: Apache-2.0

#include "mock_device.h"
#include "max77654.h"
#include <string.h>

#include "max77654_reg.h"
#include "max77654_utils.h"

#include "unity.h"
#include "unity_internals.h"

static uint8_t registers[0x100] = {0};
static uint32_t adc_max_reading = 0xFFF;
static uint32_t adc_fullscale_mV = 1250;
static uint32_t adc_readings[0xff] = {0};

max77654_err_t com_func_ret = E_MAX77654_SUCCESS;

#define UNUSED(_u_) (void)(_u_)

void reset_mock_sensor() {
  memset(registers, 0, 0x100);

  uint8_t cid_reg = 0;
  cid_reg = MAX77654_REG_FIELD_SET(CID, CID_LSB, cid_reg, (MOCK_CID & 0x0F));
  cid_reg = MAX77654_REG_FIELD_SET(CID, CID_MSB, cid_reg, (MOCK_CID & 0x10) >> 4);

  registers[MAX77654_REG(CID)] = cid_reg;

  com_func_ret = E_MAX77654_SUCCESS;
}

max77654_err_t mock_read_regs(uint8_t reg_adr, uint32_t n, uint8_t *buf) {

  for (unsigned int i = 0; i < n; i++) {
    buf[i] = registers[reg_adr];
    reg_adr++;
  }

  return com_func_ret;
}

max77654_err_t mock_write_regs(uint8_t reg_adr, uint32_t n, const uint8_t *buf) {

  for (unsigned int i = 0; i < n; i++) {
    registers[reg_adr + i] = buf[i];
  }

  return com_func_ret;
}

max77654_err_t mock_adc_read(struct max77654_adc_reading *reading) {
  reading->adc_max_reading = adc_max_reading;
  reading->adc_fullscale_mV = adc_fullscale_mV;

  // get current amux position:
  uint8_t cnfg_chg_i = registers[MAX77654_REG(CNFG_CHG_I)];
  uint8_t amux = MAX77654_REG_FIELD_GET(CNFG_CHG_I, MUX_SEL, cnfg_chg_i);

  reading->reading = adc_readings[amux];

  return E_MAX77654_SUCCESS;
}

void mock_set_adc_params(uint32_t max_reading, uint32_t fullscale_mV) {
  adc_max_reading = max_reading;
  adc_fullscale_mV = fullscale_mV;
}

void mock_set_adc_reading_code(uint8_t amux_pos, uint32_t reading) { adc_readings[amux_pos] = reading; }

void mock_set_adc_reading_voltage(uint8_t amux_pos, uint32_t input_mV) {

  uint32_t reading = (input_mV * adc_max_reading) / adc_fullscale_mV;

  adc_readings[amux_pos] = reading;
}

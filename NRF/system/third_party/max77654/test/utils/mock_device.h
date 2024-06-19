// Copyright (c) 2022 Philip Schilk.
// SPDX-License-Identifier: Apache-2.0

#ifndef _MOCK_DEVICE_H_
#define _MOCK_DEVICE_H_

#include "max77654.h"

#define MOCK_CID 0x1E

void reset_mock_sensor();
max77654_err_t mock_read_regs(uint8_t reg_adr, uint32_t n, uint8_t *buf);
max77654_err_t mock_write_regs(uint8_t reg_adr, uint32_t n, const uint8_t *buf);
max77654_err_t mock_adc_read(struct max77654_adc_reading *reading);
void mock_set_adc_params(uint32_t max_reading, uint32_t fullscale_mV);
void mock_set_adc_reading_code(uint8_t amux_pos, uint32_t reading);
void mock_set_adc_reading_voltage(uint8_t amux_pos, uint32_t input_mV);

#endif /* _MOCK_DEVICE_H_ */

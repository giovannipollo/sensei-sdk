// Copyright (c) 2022 Philip Schilk.
// SPDX-License-Identifier: Apache-2.0

#include "max77654.h"
#include "mock_device.h"

#include "stdlib.h"
#include "string.h"
#include "unity.h"
#include "unity_internals.h"

#include "max77654.c"

struct max77654_h h;

#define IS_WITHIN_1PERC(_want_, _is_) (((_is_) * 100) >= ((_want_) * 99) && ((_is_) * 100) <= ((_want_) * 101))
#define TEST_ASSERT_WITHIN_1PERC(_want_, _is_)                                                                         \
  TEST_ASSERT_TRUE_MESSAGE(IS_WITHIN_1PERC(_want_, _is_), "Not within margin.")
#define TEST_ASSERT_MUX_OFF()                                                                                          \
  do {                                                                                                                 \
    uint8_t cnfg_chg_i;                                                                                                \
    mock_read_regs(REG(CNFG_CHG_I), 1, &cnfg_chg_i);                                                                   \
    TEST_ASSERT_EQUAL_HEX(REG_CONST(CNFG_CHG_I, MUX_SEL, DISABLED), REG_FIELD_GET(CNFG_CHG_I, MUX_SEL, cnfg_chg_i));   \
  } while (0)

// ======== Tests ==================================================================================

void test_measure_chgin_v() {
  uint32_t result;

  mock_set_adc_params(0xFFF, 1250); // 12bit, 1.25V FS ADC

  // CHGIN_V maps 0V-7.5V to 0V-1.25V

  // 0V:
  mock_set_adc_reading_voltage(REG_CONST(CNFG_CHG_I, MUX_SEL, CHGIN_V), 0);
  max77654_measure(&h, MAX77654_CHGIN_V, &result);
  TEST_ASSERT_EQUAL(0, result);

  // Make sure the mux was returned to the 'off' position:
  TEST_ASSERT_MUX_OFF();

  // 7.5V (7.485 due rounding):
  mock_set_adc_reading_voltage(REG_CONST(CNFG_CHG_I, MUX_SEL, CHGIN_V), 1250);
  max77654_measure(&h, MAX77654_CHGIN_V, &result);
  TEST_ASSERT_WITHIN_1PERC(7500, result);

  // Make sure the mux was returned to the 'off' position:
  TEST_ASSERT_MUX_OFF();
}

void test_measure_chgin_i() {
  uint32_t result;

  mock_set_adc_params(0xFFFF, 3300); // 16bit, 3.3V FS ADC

  // CHGIN_I maps 0A-0.475A to 0V-1.25V

  // 0A:
  mock_set_adc_reading_voltage(REG_CONST(CNFG_CHG_I, MUX_SEL, CHGIN_I), 0);
  max77654_measure(&h, MAX77654_CHGIN_I, &result);
  TEST_ASSERT_EQUAL(0, result);

  // Make sure the mux was returned to the 'off' position:
  TEST_ASSERT_MUX_OFF();

  // 0.475A
  mock_set_adc_reading_voltage(REG_CONST(CNFG_CHG_I, MUX_SEL, CHGIN_I), 1250);
  max77654_measure(&h, MAX77654_CHGIN_I, &result);
  TEST_ASSERT_WITHIN_1PERC(475, result);

  // Make sure the mux was returned to the 'off' position:
  TEST_ASSERT_MUX_OFF();
}

void test_measure_batt_v() {
  uint32_t result;

  mock_set_adc_params(0xFFFF, 3000); // 16bit, 3V FS ADC

  // BATT_V maps 0V-4.6A to 0V-1.25V

  // 0A:
  mock_set_adc_reading_voltage(REG_CONST(CNFG_CHG_I, MUX_SEL, BATT_V), 0);
  max77654_measure(&h, MAX77654_BATT_V, &result);
  TEST_ASSERT_EQUAL(0, result);

  // Make sure the mux was returned to the 'off' position:
  TEST_ASSERT_MUX_OFF();

  // 4.6V
  mock_set_adc_reading_voltage(REG_CONST(CNFG_CHG_I, MUX_SEL, BATT_V), 1250);
  max77654_measure(&h, MAX77654_BATT_V, &result);
  TEST_ASSERT_WITHIN_1PERC(4600, result);

  // Make sure the mux was returned to the 'off' position:
  TEST_ASSERT_MUX_OFF();
}
void test_measure_batt_i_chg() {
  uint32_t result;

  mock_set_adc_params(0xFFFF, 1800); // 16bit, 1.8V FS ADC

  // BATT_CHG_I maps 0%-100% to 0V - 1.25V

  // 0%:
  mock_set_adc_reading_voltage(REG_CONST(CNFG_CHG_I, MUX_SEL, BATT_CHG_I), 0);
  max77654_measure(&h, MAX77654_BATT_I_CHG, &result);
  TEST_ASSERT_EQUAL(0, result);

  // Make sure the mux was returned to the 'off' position:
  TEST_ASSERT_MUX_OFF();

  // 100%:
  mock_set_adc_reading_voltage(REG_CONST(CNFG_CHG_I, MUX_SEL, BATT_CHG_I), 1250);
  max77654_measure(&h, MAX77654_BATT_I_CHG, &result);
  TEST_ASSERT_WITHIN_1PERC(100, result);

  // Make sure the mux was returned to the 'off' position:
  TEST_ASSERT_MUX_OFF();
}

void test_measure_vsys() {
  uint32_t result;

  mock_set_adc_params(0xFFFF, 1800); // 16bit, 1.8V FS ADC

  // VSYS maps 0V - 4.8V to 0V - 1.25V

  // 0V:
  mock_set_adc_reading_voltage(REG_CONST(CNFG_CHG_I, MUX_SEL, VSYS), 0);
  max77654_measure(&h, MAX77654_VSYS, &result);
  TEST_ASSERT_EQUAL(0, result);

  // Make sure the mux was returned to the 'off' position:
  TEST_ASSERT_MUX_OFF();

  // 4.8V:
  mock_set_adc_reading_voltage(REG_CONST(CNFG_CHG_I, MUX_SEL, VSYS), 1250);
  max77654_measure(&h, MAX77654_VSYS, &result);
  TEST_ASSERT_WITHIN_1PERC(4800, result);

  // Make sure the mux was returned to the 'off' position:
  TEST_ASSERT_MUX_OFF();
}

void test_measure_batt_i() {
  uint32_t result;

  mock_set_adc_params(0xFFFF, 1800); // 16bit, 1.8V FS ADC

  max77654_measure_t batt_i_measurement_ranges[11] = {
      MAX77654_BATT_I_8MA2,   MAX77654_BATT_I_40MA5,  MAX77654_BATT_I_72MA3,  MAX77654_BATT_I_103MA4,
      MAX77654_BATT_I_134MA1, MAX77654_BATT_I_164MA1, MAX77654_BATT_I_193MA7, MAX77654_BATT_I_222MA7,
      MAX77654_BATT_I_251MA2, MAX77654_BATT_I_279MA3, MAX77654_BATT_I_300MA,
  };

  // First, test full scale with Vnull = 0V
  mock_set_adc_reading_voltage(REG_CONST(CNFG_CHG_I, MUX_SEL, BATT_DISCHG_I_NULL), 0);

  // With Vnull=0, BTT_I maps 0% - 100% to 0V - 1.25V

  // 0%
  mock_set_adc_reading_voltage(REG_CONST(CNFG_CHG_I, MUX_SEL, BATT_DISCHG_I), 0);
  for (unsigned int i = 0; i < 11; i++) {
    max77654_measure(&h, batt_i_measurement_ranges[i], &result);
    TEST_ASSERT_EQUAL(0, result);

    // Make sure the mux was returned to the 'off' position:
    TEST_ASSERT_MUX_OFF();
  }

  // 100%:
  mock_set_adc_reading_voltage(REG_CONST(CNFG_CHG_I, MUX_SEL, BATT_DISCHG_I), 1250);
  for (unsigned int i = 0; i < 11; i++) {
    max77654_measure(&h, batt_i_measurement_ranges[i], &result);
    TEST_ASSERT_WITHIN_1PERC(100, result);

    // Make sure the mux was returned to the 'off' position:
    TEST_ASSERT_MUX_OFF();
  }

  // now, test with Vnull = 100mV
  mock_set_adc_reading_voltage(REG_CONST(CNFG_CHG_I, MUX_SEL, BATT_DISCHG_I_NULL), 100);

  // batt_i should now be 0% at 100mV:
  mock_set_adc_reading_voltage(REG_CONST(CNFG_CHG_I, MUX_SEL, BATT_DISCHG_I), 100);
  for (unsigned int i = 0; i < 11; i++) {
    max77654_measure(&h, batt_i_measurement_ranges[i], &result);
    TEST_ASSERT_EQUAL(0, result);

    // Make sure the mux was returned to the 'off' position:
    TEST_ASSERT_MUX_OFF();
  }

  // and still be 100% at 1.25V:
  mock_set_adc_reading_voltage(REG_CONST(CNFG_CHG_I, MUX_SEL, BATT_DISCHG_I), 1250);
  for (unsigned int i = 0; i < 11; i++) {
    max77654_measure(&h, batt_i_measurement_ranges[i], &result);
    TEST_ASSERT_WITHIN_1PERC(100, result);

    // Make sure the mux was returned to the 'off' position:
    TEST_ASSERT_MUX_OFF();
  }

  // Make sure batt_i < batt_i_null does not cause a problem:
  // Set BATT_I_NULL to 100mV, but BATT_I to 0mV:
  mock_set_adc_reading_voltage(REG_CONST(CNFG_CHG_I, MUX_SEL, BATT_DISCHG_I_NULL), 100);
  mock_set_adc_reading_voltage(REG_CONST(CNFG_CHG_I, MUX_SEL, BATT_DISCHG_I), 0);
  for (unsigned int i = 0; i < 11; i++) {
    max77654_measure(&h, batt_i_measurement_ranges[i], &result);
    TEST_ASSERT_EQUAL(0, result);

    // Make sure the mux was returned to the 'off' position:
    TEST_ASSERT_MUX_OFF();
  }
}

// All the measurements that are 1:1
void test_measure_identity() {
  uint32_t result;
  mock_set_adc_params(0xFFFF, 1800); // 16bit, 1.8V FS ADC

  // 0:
  mock_set_adc_reading_voltage(REG_CONST(CNFG_CHG_I, MUX_SEL, BATT_DISCHG_I_NULL), 0);
  max77654_measure(&h, MAX77654_BATT_I_NULL, &result);
  TEST_ASSERT_EQUAL(0, result);
  // Make sure the mux was returned to the 'off' position:
  TEST_ASSERT_MUX_OFF();

  mock_set_adc_reading_voltage(REG_CONST(CNFG_CHG_I, MUX_SEL, THM), 0);
  max77654_measure(&h, MAX77654_THM, &result);
  TEST_ASSERT_EQUAL(0, result);
  // Make sure the mux was returned to the 'off' position:
  TEST_ASSERT_MUX_OFF();

  mock_set_adc_reading_voltage(REG_CONST(CNFG_CHG_I, MUX_SEL, THM_BIAS), 0);
  max77654_measure(&h, MAX77654_TBIAS, &result);
  TEST_ASSERT_EQUAL(0, result);
  // Make sure the mux was returned to the 'off' position:
  TEST_ASSERT_MUX_OFF();

  mock_set_adc_reading_voltage(REG_CONST(CNFG_CHG_I, MUX_SEL, AGND), 0);
  max77654_measure(&h, MAX77654_AGND, &result);
  TEST_ASSERT_EQUAL(0, result);
  // Make sure the mux was returned to the 'off' position:
  TEST_ASSERT_MUX_OFF();

  // 1.25V:
  mock_set_adc_reading_voltage(REG_CONST(CNFG_CHG_I, MUX_SEL, BATT_DISCHG_I_NULL), 1250);
  max77654_measure(&h, MAX77654_BATT_I_NULL, &result);
  TEST_ASSERT_WITHIN_1PERC(1250, result);
  // Make sure the mux was returned to the 'off' position:
  TEST_ASSERT_MUX_OFF();

  mock_set_adc_reading_voltage(REG_CONST(CNFG_CHG_I, MUX_SEL, THM), 1250);
  max77654_measure(&h, MAX77654_THM, &result);
  TEST_ASSERT_WITHIN_1PERC(1250, result);
  // Make sure the mux was returned to the 'off' position:
  TEST_ASSERT_MUX_OFF();

  mock_set_adc_reading_voltage(REG_CONST(CNFG_CHG_I, MUX_SEL, THM_BIAS), 1250);
  max77654_measure(&h, MAX77654_TBIAS, &result);
  TEST_ASSERT_WITHIN_1PERC(1250, result);
  // Make sure the mux was returned to the 'off' position:
  TEST_ASSERT_MUX_OFF();

  mock_set_adc_reading_voltage(REG_CONST(CNFG_CHG_I, MUX_SEL, AGND), 1250); // if AGND ever measures 1.25V, panik.
  max77654_measure(&h, MAX77654_AGND, &result);
  TEST_ASSERT_WITHIN_1PERC(1250, result);

  // Make sure the mux was returned to the 'off' position:
  TEST_ASSERT_MUX_OFF();
}

// // ======== Main ===================================================================================

void setUp(void) {}
void tearDown(void) {}

int main(void) {
  h.read_regs = &mock_read_regs;
  h.write_regs = &mock_write_regs;
  h.adc_read = &mock_adc_read;

  UNITY_BEGIN();

  RUN_TEST(test_measure_chgin_v);
  RUN_TEST(test_measure_chgin_i);
  RUN_TEST(test_measure_batt_v);
  RUN_TEST(test_measure_batt_i_chg);
  RUN_TEST(test_measure_batt_i);
  RUN_TEST(test_measure_identity);
  RUN_TEST(test_measure_vsys);

  return UNITY_END();
}

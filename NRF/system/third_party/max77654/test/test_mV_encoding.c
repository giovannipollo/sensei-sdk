// Copyright (c) 2022 Philip Schilk.
// SPDX-License-Identifier: Apache-2.0

#include "unity.h"
#include "unity_internals.h"

#include "max77654.c"

#define UNUSED(x) (void)(x)

struct max77654_h h;

void test_tv_sbb_valid_accepted() {
  for (uint32_t mV = 800; mV <= 5500; mV += 50) {
    uint8_t code;
    TEST_ASSERT(encode_mV(&h, mV, false, &code) == E_MAX77654_SUCCESS);
  }
}

void test_tv_sbb_invalid_rejected() {
  uint8_t code;

  TEST_ASSERT(encode_mV(&h, 799, false, &code) == E_MAX77654_CONFIG_ERR);
  TEST_ASSERT(encode_mV(&h, 5501, false, &code) == E_MAX77654_CONFIG_ERR);
  TEST_ASSERT(encode_mV(&h, (900 + 1), false, &code) == E_MAX77654_CONFIG_ERR);
}

void test_tv_sbb_encoding() {

  uint8_t code;
  TEST_ASSERT(encode_mV(&h, 800, false, &code) == E_MAX77654_SUCCESS);
  TEST_ASSERT_EQUAL_UINT8(0, code);

  for (uint32_t mV = 850; mV <= 5500; mV += 50) {
    uint8_t new_code;
    TEST_ASSERT(encode_mV(&h, mV, false, &new_code) == E_MAX77654_SUCCESS);

    TEST_ASSERT_EQUAL_UINT8((code + 1), new_code);
    code = new_code;
  }

  TEST_ASSERT(encode_mV(&h, 5500, false, &code) == E_MAX77654_SUCCESS);
  TEST_ASSERT_EQUAL_UINT8(0x5E, code);
}

void test_tv_ldo_valid_accepted() {
  for (uint32_t mV = 800; mV <= 3975; mV += 25) {
    uint8_t code;
    TEST_ASSERT(encode_mV(&h, mV, true, &code) == E_MAX77654_SUCCESS);
  }
}

void test_tv_ldo_invalid_rejected() {
  uint8_t code;

  TEST_ASSERT(encode_mV(&h, 799, false, &code) == E_MAX77654_CONFIG_ERR);
  TEST_ASSERT(encode_mV(&h, 3976, false, &code) == E_MAX77654_CONFIG_ERR);
  TEST_ASSERT(encode_mV(&h, (900 + 1), false, &code) == E_MAX77654_CONFIG_ERR);
}

void test_tv_ldo_encoding() {

  uint8_t code;
  TEST_ASSERT(encode_mV(&h, 800, true, &code) == E_MAX77654_SUCCESS);
  TEST_ASSERT_EQUAL_UINT8(0, code);

  for (uint32_t mV = 825; mV <= 3975; mV += 25) {
    uint8_t new_code;
    TEST_ASSERT(encode_mV(&h, mV, true, &new_code) == E_MAX77654_SUCCESS);

    TEST_ASSERT_EQUAL_UINT8((code + 1), new_code);
    code = new_code;
  }

  TEST_ASSERT(encode_mV(&h, 3975, true, &code) == E_MAX77654_SUCCESS);
  TEST_ASSERT_EQUAL_UINT8(0x7F, code);
}

void setUp(void) {}
void tearDown(void) {}

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_tv_sbb_valid_accepted);
  RUN_TEST(test_tv_sbb_invalid_rejected);
  RUN_TEST(test_tv_sbb_encoding);
  RUN_TEST(test_tv_ldo_valid_accepted);
  RUN_TEST(test_tv_ldo_invalid_rejected);
  RUN_TEST(test_tv_ldo_encoding);

  return UNITY_END();
}

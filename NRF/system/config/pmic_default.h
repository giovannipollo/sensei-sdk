/*
 * ----------------------------------------------------------------------
 *
 * File: pmic_default.h
 *
 * Last edited: 30.10.2025
 *
 * Copyright (c) 2024 ETH Zurich and University of Bologna
 *
 * Authors:
 * - Philipp Schilk (schilkp@ethz.ch), ETH Zurich
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

#ifndef PMIC_DEFAULT_H_
#define PMIC_DEFAULT_H_

#include "max77654.h"

#define PMIC_CONFIG_DEFAULT                                                                                            \
  {                                                                                                                    \
      .cid = 0x2, /* Device ID. 0x2 = 'B' variant  */                                                                  \
                                                                                                                       \
      /* === Global configuration === */                                                                               \
      .main_bias_force_enable = true,    /* Main bias off. */                                                          \
      .main_bias_low_power_mode = false, /* Put main bias into low-power mode. */                                      \
                                                                                                                       \
      .vsys_regulation = MAX77654_VSYS_4V5, /* Maximum/regulation voltage for VSYS. */                                 \
                                                                                                                       \
      .sbb_drive_speed = MAX77654_SBB_DRIVE_FASTEST, /* SBB Drive speed. */                                            \
                                                                                                                       \
      .nEN_mode = MAX77654_nEN_PUSH_BUTTON,             /* nEN pin mode (momentary/slide switch). */                   \
      .nEN_pu_strength = MAX77654_nEN_PU_200k,          /* nEN internal pull-up strength. */                           \
      .nEN_debounce_time = MAX77654_nEN_DEBOUNCE_500us, /* nEN debounce time */                                        \
      .manual_reset_period = MAX77654_MANUAL_RESET_8s,  /* Period nEN has to be active for a manual reset. */          \
                                                                                                                       \
      /* === Watchdog timer configuration === */                                                                       \
      .wdt_enable = false,                            /* Watchdog timer disabled */                                    \
      .wdt_period = MAX77654_WDT_128s,                /* Watchdog timer period */                                      \
      .wdt_action_on_ovf = MAX77654_WDT_DO_PWR_RESET, /* Action performed when Watchdog timer overflows. */            \
                                                                                                                       \
      /* === Charger configuration === */                                                                              \
      .charger_enabled = true, /* Do not enable the charger. */                                                        \
      .USB_suspend = false,    /* Do not place the charger into USB suspend mode. */                                   \
                                                                                                                       \
      .chgin_v_min = MAX77654_VCHGIN_MIN_4V,     /* Minimum CHGIN regulation voltage. */                               \
      .chgin_i_lim = MAX77654_ICHGIN_LIM_0A285,  /* CHGIN IN current limit. */                                         \
      .termnation_i = MAX77654_I_TERM_15PERCENT, /* Termination current. */                                            \
      .topoff_t = MAX77654_T_TOPOFF_0MIN,        /* Topoff timer period. */                                            \
                                                                                                                       \
      .fast_chg_cc = MAX77654_CHG_CC_15MA,       /* Fast-charge constant current setting. */                           \
      .fast_chg_cc_jeita = MAX77654_CHG_CC_15MA, /* JEITA fast-charge constant current setting. */                     \
      .fast_chg_cv = MAX77654_CHG_CV_3V6,        /* Fast-charge battery voltage regulation. */                         \
      .fast_chg_cv_jeita = MAX77654_CHG_CV_4V,   /* JEITA fast-charge battery voltage regulation. */                   \
      .t_fast_chg = MAX77654_T_FAST_CHG_3H,      /* Fast-charge safety timer. */                                       \
                                                                                                                       \
      .preq_i = MAX77654_I_PQ_10PERCENT, /* Prequalifaction charge current, as a percent of cc current*/               \
      .preq_v_th = MAX77654_CHG_PQ_3V0,  /* Prequalifaction voltage threshold */                                       \
                                                                                                                       \
      .j_temp_reg = MAX77654_TJ_REG_60C,  /* Junction regulation termperature */                                       \
      .thm_en = false,                    /* Disable the thermistor */                                                 \
      .th_hot = MAX77654_THM_HOT_0V367,   /* Thermister threshold for 'hot'. */                                        \
      .th_warm = MAX77654_THM_WARM_0V459, /* Thermister threshold for 'warm'. */                                       \
      .th_cool = MAX77654_THM_COOL_0V747, /* Thermister threshold for 'cool'. */                                       \
      .th_cold = MAX77654_THM_COLD_0V867, /* Thermister threshold for 'cold'. */                                       \
                                                                                                                       \
      /* === SBB configuration === */                                                                                  \
      .sbb_conf = /* Configuration for regulators SBB0-2 */                                                            \
      {                                                                                                                \
          {                                                                                                            \
              .mode = MAX77654_SBB_MODE_BUCKBOOST,                                                                     \
              .active_discharge = false,                                                                               \
              .peak_current = MAX77654_SBB_PEAK_CURRENT_0A33,                                                          \
              .en = MAX77654_REG_OFF,                                                                                  \
              .output_voltage_mV = 1800,                                                                               \
          },                                                                                                           \
          {                                                                                                            \
              .mode = MAX77654_SBB_MODE_BUCKBOOST,                                                                     \
              .active_discharge = false,                                                                               \
              .peak_current = MAX77654_SBB_PEAK_CURRENT_0A33,                                                          \
              .en = MAX77654_REG_OFF,                                                                                  \
              .output_voltage_mV = 1800,                                                                               \
          },                                                                                                           \
          {                                                                                                            \
              .mode = MAX77654_SBB_MODE_BUCKBOOST,                                                                     \
              .active_discharge = false,                                                                               \
              .peak_current = MAX77654_SBB_PEAK_CURRENT_0A33,                                                          \
              .en = MAX77654_REG_OFF,                                                                                  \
              .output_voltage_mV = 1800,                                                                               \
          },                                                                                                           \
      },                                                                                                               \
                                                                                                                       \
      /* === LDO configuration === */                                                                                  \
      .ldo_conf = /* Configuration for regulators LDO0-1 */                                                            \
      {                                                                                                                \
          {                                                                                                            \
              .mode = MAX77654_LDO_MODE_LDO,                                                                           \
              .active_discharge = false,                                                                               \
              .en = MAX77654_REG_OFF,                                                                                  \
              .output_voltage_mV = 1800,                                                                               \
          },                                                                                                           \
          {                                                                                                            \
              .mode = MAX77654_LDO_MODE_LDO,                                                                           \
              .active_discharge = false,                                                                               \
              .en = MAX77654_REG_OFF,                                                                                  \
              .output_voltage_mV = 1800,                                                                               \
          },                                                                                                           \
      },                                                                                                               \
                                                                                                                       \
      /* === Interrupt configuration === */                                                                            \
      .int_en =                                                                                                        \
          {                                                                                                            \
              .chgin_ctrl = true, /* Interrupt on charge-input uvlo/over current */                                    \
              .chgin = true,      /* Interrupt on charge-input-status change */                                        \
              .nen_f = true,      /* Interrupt on nEN falling edge */                                                  \
          },                                                                                                           \
                                                                                                                       \
      /* === GPIO configuration === */                                                                                 \
      .gpio_conf = /* Configuration for GPIO pins 0-2 */                                                               \
      {                                                                                                                \
          {                                                                                                            \
              .alternate_mode = false,                                                                                 \
              .direction = MAX77654_GPIO_INPUT,                                                                        \
              .output_drive_type = MAX77654_GPIO_OD,                                                                   \
              .output_value = false,                                                                                   \
          },                                                                                                           \
          {                                                                                                            \
              .alternate_mode = false,                                                                                 \
              .direction = MAX77654_GPIO_INPUT,                                                                        \
              .output_drive_type = MAX77654_GPIO_OD,                                                                   \
              .output_value = false,                                                                                   \
          },                                                                                                           \
          {                                                                                                            \
              .alternate_mode = false,                                                                                 \
              .direction = MAX77654_GPIO_INPUT,                                                                        \
              .output_drive_type = MAX77654_GPIO_OD,                                                                   \
              .output_value = false,                                                                                   \
          },                                                                                                           \
      },                                                                                                               \
  }

#endif /* PMIC_DEFAULT_H_ */

/*
 * Copyright 2020 Makani Technologies LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "avionics/motor/firmware/svpwm.h"

#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "avionics/common/fast_math/fast_math.h"
#include "avionics/common/strings.h"
#include "avionics/firmware/cpu/peripherals.h"
#include "avionics/firmware/cpu/registers.h"
#include "avionics/motor/firmware/config_params.h"
#include "avionics/motor/firmware/nhet.h"

#define N2HET1_LRPFC 5  // Loop Resolution Prescaler Factor Code
#define N2HET1_LR (1 << N2HET1_LRPFC)  // N2HET Loop Resolution prescaler.
#define MIN_GATE_HOLD_TIME 1.0e-6f  // Min time between gate transitions [s].

// Period at which the ISR is called.
float g_svpwm_isr_period;

// Maximum value of a = v_ref / v_bus that can be achieved given a particular
// minimum high / low gate driver time. 1 / sqrt(3) is derived from gamma =
// sector angle = pi/6 where t1 + t2 is at a maximum.
float g_svpwm_vref_vbus_limit = 0.577350269f;  // 1 / sqrt(3)

// This initialization assumes HR = 1 and uses a LR = N2HET1_LR.  The pin
// numbers provided here should all be even as the NHET uses the XOR function
// for symmetrical PWM.
void SvpwmInit(float svpwm_freq, int32_t pwm_per_isr) {
  // Start N2HET1 clock.
  PeripheralEnable(kPeripheralN2Het1);

  // Copy blank instructions from svpwm_nhet.c to N2HET1 RAM.
  vmemcpy(&HET1RAM, kHetInitSvpwm, sizeof(kHetInitSvpwm));

  float f_vclk2 = (float)PeripheralGetClockFreq(kPeripheralN2Het1);

  // Set SVPWM frequency and store the achieved period.
  HET1RAM.REAL_CNT.MAX_COUNT
      = nearbyint(f_vclk2 / (N2HET1_LR * svpwm_freq)) - 1;
  float pwm_period
      = (float)(N2HET1_LR * (HET1RAM.REAL_CNT.MAX_COUNT + 1)) / f_vclk2;

  // Set ISR frequency and store the achieved period.
  HET1RAM.START.MAX_COUNT = pwm_per_isr * (HET1RAM.REAL_CNT.MAX_COUNT + 1) - 1;
  g_svpwm_isr_period
      = (float)(N2HET1_LR * (HET1RAM.START.MAX_COUNT + 1)) / f_vclk2;

  assert(fabsf(g_svpwm_isr_period - pwm_per_isr * pwm_period)
         < 10.0f * FLT_EPSILON * g_svpwm_isr_period);

  // Set the max duty cycle.
  g_svpwm_vref_vbus_limit
      = (1.0f - 2.0f * MIN_GATE_HOLD_TIME / pwm_period) / sqrtf(3.0f);
  assert(g_svpwm_vref_vbus_limit >= 0.0f
         && g_svpwm_vref_vbus_limit <= 1.0f / sqrtf(3.0f) + 5.0f * FLT_EPSILON);

  // Set the loop resolution prescaler.
  N2HET(1).HETPFR.LRPFC = N2HET1_LRPFC;

  // N2HET1 Pin  ECMP Instruction
  // ----------  ----------------
  // N2HET1[04]  YAN1  ZAN1
  // N2HET1[05]  YAN2  ZAN2
  // N2HET1[14]  YB1   ZB1
  // N2HET1[15]  YB2   ZB2
  // N2HET1[16]  YBN1  ZBN1
  // N2HET1[17]  YBN2  ZBN2
  // N2HET1[18]  YC1   ZC1
  // N2HET1[19]  YC2   ZC2
  // N2HET1[20]  YCN1  ZCN1
  // N2HET1[21]  YCN2  ZCN2
  // N2HET1[22]  YA1   ZA1
  // N2HET1[23]  YA2   ZA2

  // Set pins to output.
  N2HET(1).HETDIR.HETDIR4  = 1;
  N2HET(1).HETDIR.HETDIR14 = 1;
  N2HET(1).HETDIR.HETDIR16 = 1;
  N2HET(1).HETDIR.HETDIR18 = 1;
  N2HET(1).HETDIR.HETDIR20 = 1;
  N2HET(1).HETDIR.HETDIR22 = 1;

  // Set XOR pairs.
  N2HET(1).HETXOR.XORSHARE_5_4   = 1;
  N2HET(1).HETXOR.XORSHARE_15_14 = 1;
  N2HET(1).HETXOR.XORSHARE_17_16 = 1;
  N2HET(1).HETXOR.XORSHARE_19_18 = 1;
  N2HET(1).HETXOR.XORSHARE_21_20 = 1;
  N2HET(1).HETXOR.XORSHARE_23_22 = 1;

  // Enable DMA request.
  N2HET(1).REQENS.REQENA4 = 1;
  N2HET(1).REQDS.TDS4 = 1;

  // Ignore suspend.
  N2HET(1).HETGCR.IS = 1;

  // Set N2HET1 to master.
  N2HET(1).HETGCR.CMS = 1;
}

bool SvpwmCheckHetPinEna(void) {
  bool het_pin_ena = N2HET(1).HETGCR.HETPINENA;
  if (!het_pin_ena)
    N2HET(1).HETGCR.HETPINENA = 1;
  return het_pin_ena;
}

void SvpwmStart(void) {
  // Start N2HET1.
  N2HET(1).HETGCR.TO = 1;
}

// Set SVPWM times.  Sets the individual phase on and off times
// depending on which sector of the hexagon (see header file) we are
// in.  Representative timing diagrams for the first two sectors are
// shown below.  Note that the order of t1 and t2 swap, when starting
// from (000), in the even sectors.  The other sectors follow a
// similar pattern.
//
// Sector I:
//
//      t0/2 :  t1  :  t2  :      t0      :  t2  :  t1  : t0/2
//           :__________________________________________:
//   A  _____|      :      :              :      :      |______
//                  :____________________________:
//   B  ____________|      :              :      |_____________
//                         :______________:
//   C  ___________________|              |____________________
//
//      (000)  (100)  (110)      (111)      (110)  (100)  (000)
//
//
// Sector II:
//
//      t0/2 :  t1  :  t2  :      t0      :  t2  :  t1  : t0/2
//      ____________:      :              :      :_____________
//   A       :      |____________________________|      :
//      ___________________:              :____________________
//   B       :      :      |______________|      :      :
//      _____:      :      :              :      :      :______
//   C       |__________________________________________|
//
//      (111)  (110)  (010)      (000)      (010)  (110)  (000)
//
// The times t1 and t2 are consistent with Broeck (1988) with the
// exception that they have already been scaled by the switching
// period, i.e. they range from 0 to 1.
static void SvpwmSetTimes(int32_t sector, float t1, float t2) {
  assert(1 <= sector && sector <= 6);
  assert(0.0f <= t1 && t1 <= 1.0f);
  assert(0.0f <= t2 && t2 <= 1.0f - t1);

  // Calculate time periods.  Total period is MAX_COUNT multiplied by N2HET1_LR.
  int32_t t_total_counts = (HET1RAM.REAL_CNT.MAX_COUNT + 1) * N2HET1_LR;
  int32_t tz_counts = t_total_counts / 2;
  int32_t t1_counts = (int32_t)(t1 * (float)tz_counts);
  int32_t t2_counts = (int32_t)(t2 * (float)tz_counts);

  // Coerce values into valid range.
  if (t1_counts < 0) {
    t1_counts = 0;
  } else if (t1_counts > tz_counts) {
    t1_counts = tz_counts;
  }

  if (t2_counts < 0) {
    t2_counts = 0;
  } else if (t2_counts > tz_counts) {
    t2_counts = tz_counts;
  }

  int32_t t0_counts = tz_counts - t1_counts - t2_counts;

  if (t0_counts < 0) {
    t1_counts += t0_counts / 2;
    t2_counts = tz_counts - t1_counts;
    t0_counts = 0;
  }

  // Determine switching order from sector.  Because of symmetrical
  // PWM t1 and t2 switch for even sectors.
  int32_t ta_counts, tb_counts, tc_counts;
  switch (sector) {
    case 1:
      ta_counts = t0_counts / 2;
      tb_counts = ta_counts + t1_counts;
      tc_counts = tb_counts + t2_counts;
      break;

    case 2:
      tb_counts = t0_counts / 2;
      ta_counts = tb_counts + t2_counts;
      tc_counts = ta_counts + t1_counts;
      break;

    case 3:
      tb_counts = t0_counts / 2;
      tc_counts = tb_counts + t1_counts;
      ta_counts = tc_counts + t2_counts;
      break;

    case 4:
      tc_counts = t0_counts / 2;
      tb_counts = tc_counts + t2_counts;
      ta_counts = tb_counts + t1_counts;
      break;

    case 5:
      tc_counts = t0_counts / 2;
      ta_counts = tc_counts + t1_counts;
      tb_counts = ta_counts + t2_counts;
      break;

    case 6:
      ta_counts = t0_counts / 2;
      tc_counts = ta_counts + t2_counts;
      tb_counts = tc_counts + t1_counts;
      break;

    default:
      assert(false);
      return;
  }

  // Select Y or Z buffer.  LAST_CHK data is 1 when Z is in use and 0
  // when Y is in use.
  volatile EcmpInstruction *phase_a_cmp;
  volatile EcmpInstruction *phase_b_cmp;
  volatile EcmpInstruction *phase_c_cmp;
  if (HET1RAM.LAST_CHK.DATA == 1) {
    // Y buffer.
    phase_a_cmp = &HET1RAM.YA1;
    phase_b_cmp = &HET1RAM.YB1;
    phase_c_cmp = &HET1RAM.YC1;
  } else {
    // Z buffer.
    phase_a_cmp = &HET1RAM.ZA1;
    phase_b_cmp = &HET1RAM.ZB1;
    phase_c_cmp = &HET1RAM.ZC1;
  }

  // If phase A and C are physically swapped, swap here.
  if (kMotorConfigParams->phase_swap == kMotorPhaseSwapAc) {
    volatile EcmpInstruction *temp_cmp = phase_a_cmp;
    phase_a_cmp = phase_c_cmp;
    phase_c_cmp = temp_cmp;
  }

  // Calculate absolute switching times.
  // LR = 32, so shift the HR data compare value by 2 [TRM Table 23-6].
  // Instruction order: XX1, XX2, XXN1, XXN2.
  (phase_a_cmp + 0)->raw.data = ta_counts << 2;
  (phase_a_cmp + 2)->raw.data = ta_counts << 2;
  (phase_a_cmp + 1)->raw.data = (t_total_counts - ta_counts) << 2;
  (phase_a_cmp + 3)->raw.data = (t_total_counts - ta_counts) << 2;

  (phase_b_cmp + 0)->raw.data = tb_counts << 2;
  (phase_b_cmp + 2)->raw.data = tb_counts << 2;
  (phase_b_cmp + 1)->raw.data = (t_total_counts - tb_counts) << 2;
  (phase_b_cmp + 3)->raw.data = (t_total_counts - tb_counts) << 2;

  (phase_c_cmp + 0)->raw.data = tc_counts << 2;
  (phase_c_cmp + 2)->raw.data = tc_counts << 2;
  (phase_c_cmp + 1)->raw.data = (t_total_counts - tc_counts) << 2;
  (phase_c_cmp + 3)->raw.data = (t_total_counts - tc_counts) << 2;
}

// Latch opposite PWM buffer for next NHET cycle.
static void SvpwmLatchTimes(void) {
  if (HET1RAM.LAST_CHK.DATA == 0) {
    HET1RAM.LAST_CHK.DATA = 1;  // Y buffer.
  } else {
    HET1RAM.LAST_CHK.DATA = 0;  // Z buffer.
  }
}

// Determines sector, sector angle, and the t1 and t2 PWM times.
void SvpwmSetReference(float angle, float v_ref, float v_bus) {
  assert(v_ref >= 0.0f);
  assert(v_bus >= 0.0f);

  int32_t sector;
  float sector_angle;
  if (angle >= 0.0f) {
    sector = (int32_t)(angle * (3.0f / PI_F));
    sector_angle = angle - sector * (PI_F / 3.0f);
    sector = 1 + sector % 6;
  } else {
    sector = (int32_t)(-angle * (3.0f / PI_F));
    sector_angle = angle + (sector + 1) * (PI_F / 3.0f);
    sector = 6 - sector % 6;
  }

  // Find normalized vector length.  Limit solution to circle
  // inscribed in space vector hexagon.
  float a = g_svpwm_vref_vbus_limit;
  if (v_bus > 0.0f) {
    a = Saturatef(v_ref / v_bus, 0.0f, g_svpwm_vref_vbus_limit);
  }

  float sin_pi3_angle = SinLookup(PI_F / 3.0f - sector_angle);
  float sin_angle = SinLookup(sector_angle);

  // t1 and t2 are normalized by tz (the PWM half-period).  Note that
  // without the saturation, it is possible for these to go very
  // slightly negative due to numerical errors in the sector_angle
  // calculation.
  float t1 = Maxf(a * sin_pi3_angle * ((3.0f / 2.0f) / sinf(PI_F / 3.0f)),
                  0.0f);
  float t2 = Maxf(a * sin_angle * ((3.0f / 2.0f) / sinf(PI_F / 3.0f)),
                  0.0f);

  SvpwmSetTimes(sector, t1, t2);
  SvpwmLatchTimes();
}

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

#include "avionics/motor/firmware/adc.h"

#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "avionics/common/fast_math/fast_math.h"
#include "avionics/firmware/cpu/peripherals.h"
#include "avionics/firmware/cpu/registers.h"
#include "avionics/firmware/cpu/vim.h"
#include "avionics/firmware/serial/motor_serial_params.h"
#include "avionics/motor/firmware/config_params.h"
#include "avionics/motor/firmware/flags.h"
#include "avionics/motor/firmware/isr.h"
#include "common/macros.h"

// ADC channels.
#define ADC_PORT0_SD_CHANNEL       8
#define ADC_PORT1_SD_CHANNEL       9
#define ADC_V_IN_MONITOR_CHANNEL   11
#define ADC_V_AUX_MONITOR_CHANNEL  12
#define ADC_V_BUS_OZONE_CHANNEL    14
#define ADC_V_CHASSIS_CHANNEL      15
#define ADC_V_CM_CHANNEL           16
#define ADC_V_BUS_GIN_CHANNEL      16
#define ADC_I_PHASE_C_CHANNEL      17
#define ADC_I_PHASE_A_CHANNEL      18
#define ADC_I_PHASE_B_CHANNEL      19
#define ADC_I_BUS_CHANNEL          20

// ADC magnitude interrupt numbers.
// These must match the ordering in MotorAdcMagnitutedInterruptInit().
#define ADC1_MAG_INT_IAP_NUM 1
#define ADC1_MAG_INT_IAN_NUM 2
#define ADC1_MAG_INT_IBP_NUM 3
#define ADC2_MAG_INT_IBN_NUM 1
#define ADC2_MAG_INT_ICP_NUM 2
#define ADC2_MAG_INT_ICN_NUM 3

// ADC magnitude interrupt masks corresponding to the interrupt offset.
#define ADC1_MAG_INT1_ENABLE_MASK (1 << (ADC1_MAG_INT_IAP_NUM - 1))
#define ADC1_MAG_INT2_ENABLE_MASK (1 << (ADC1_MAG_INT_IAN_NUM - 1))
#define ADC1_MAG_INT3_ENABLE_MASK (1 << (ADC1_MAG_INT_IBP_NUM - 1))
#define ADC2_MAG_INT1_ENABLE_MASK (1 << (ADC2_MAG_INT_IBN_NUM - 1))
#define ADC2_MAG_INT2_ENABLE_MASK (1 << (ADC2_MAG_INT_ICP_NUM - 1))
#define ADC2_MAG_INT3_ENABLE_MASK (1 << (ADC2_MAG_INT_ICN_NUM - 1))

#define ADC1_MAG_INT_ENABLE_MASK (ADC1_MAG_INT1_ENABLE_MASK |   \
                                  ADC1_MAG_INT2_ENABLE_MASK |   \
                                  ADC1_MAG_INT3_ENABLE_MASK)
#define ADC2_MAG_INT_ENABLE_MASK (ADC2_MAG_INT1_ENABLE_MASK |   \
                                  ADC2_MAG_INT2_ENABLE_MASK |   \
                                  ADC2_MAG_INT3_ENABLE_MASK)

// ADC calibration from bench tests.
typedef struct {
  float slope;
  float offset;
} AdcCal;

typedef struct {
  float i_phase_slope;
  float i_bus_slope;
  AdcCal v_bus;
  AdcCal v_chassis;
  AdcCal v_cm;
  AdcCal v_in_mon;
  AdcCal v_aux_mon;
} AdcCalibration;

#define NUM_SAMP_F ((float)NUM_ADC_SAMPLES)
static const AdcCalibration kAdcCalibration[] = {
  [kMotorHardwareGinA1] = {
    .i_phase_slope = -0.1978239367f / NUM_SAMP_F,
    .i_bus_slope   = 0.1978239367f / NUM_SAMP_F,
    .v_bus         = {.slope = 0.2690967654f / NUM_SAMP_F, .offset = -5.92013f},
    .v_in_mon      = {.slope = 0.0058887945f / NUM_SAMP_F, .offset = 0.06321f},
    .v_aux_mon     = {.slope = 0.0058887945f / NUM_SAMP_F, .offset = 0.06321f},
  },
  [kMotorHardwareGinA2] = {
    .i_phase_slope = -0.1568588191f / NUM_SAMP_F,
    .i_bus_slope   = 0.1568588191f / NUM_SAMP_F,
    .v_bus         = {.slope = 0.3524540324f / NUM_SAMP_F, .offset = -19.9091f},
    .v_in_mon      = {.slope = 0.0058887945f / NUM_SAMP_F, .offset = 0.06321f},
    .v_aux_mon     = {.slope = 0.0058887945f / NUM_SAMP_F, .offset = 0.06321f},
  },
  [kMotorHardwareGinA3] = {
    .i_phase_slope = -0.1568588191f / NUM_SAMP_F,
    .i_bus_slope   = 0.1568588191f / NUM_SAMP_F,
    .v_bus         = {.slope = 0.3524540324f / NUM_SAMP_F, .offset = -19.9091f},
    .v_in_mon      = {.slope = 0.0058887945f / NUM_SAMP_F, .offset = 0.06321f},
    .v_aux_mon     = {.slope = 0.0058887945f / NUM_SAMP_F, .offset = 0.06321f},
  },
  [kMotorHardwareGinA4Clk16] = {
    .i_phase_slope = -0.1568588191f / NUM_SAMP_F,
    .i_bus_slope   = 0.1568588191f / NUM_SAMP_F,
    .v_bus         = {.slope = 0.3524540324f / NUM_SAMP_F, .offset = -19.9091f},
    .v_in_mon      = {.slope = 0.0058887945f / NUM_SAMP_F, .offset = 0.06321f},
    .v_aux_mon     = {.slope = 0.0058887945f / NUM_SAMP_F, .offset = 0.06321f},
  },
  [kMotorHardwareGinA4Clk8] = {
    .i_phase_slope = -0.1568588191f / NUM_SAMP_F,
    .i_bus_slope   = 0.1568588191f / NUM_SAMP_F,
    .v_bus         = {.slope = 0.3524540324f / NUM_SAMP_F, .offset = -19.9091f},
    .v_in_mon      = {.slope = 0.0058887945f / NUM_SAMP_F, .offset = 0.06321f},
    .v_aux_mon     = {.slope = 0.0058887945f / NUM_SAMP_F, .offset = 0.06321f},
  },
  [kMotorHardwareOzoneA1] = {
    .i_phase_slope = 0.3670427637f / NUM_SAMP_F,
    .i_bus_slope   = -0.3670427637f / NUM_SAMP_F,
    .v_bus         = {.slope = 0.5450254841f / NUM_SAMP_F, .offset = -1.24337f},
    .v_chassis     = {.slope = -2.015340266f / NUM_SAMP_F, .offset = -4282.21f},
    .v_cm          = {.slope = 0.5382049148f / NUM_SAMP_F, .offset = -0.66737f},
    .v_in_mon      = {.slope = 0.0058956829f / NUM_SAMP_F, .offset = 0.059697f},
    .v_aux_mon     = {.slope = 0.0058956829f / NUM_SAMP_F, .offset = 0.059697f},
  }
};

volatile Adc1PackedData g_adc_1_buffer;
volatile Adc2PackedData g_adc_2_buffer;

static AdcCal g_i_phase_a_cal = {.slope = 1.0f, .offset = 0.0f};
static AdcCal g_i_phase_b_cal = {.slope = 1.0f, .offset = 0.0f};
static AdcCal g_i_phase_c_cal = {.slope = 1.0f, .offset = 0.0f};
static AdcCal g_i_bus_cal = {.slope = 1.0f, .offset = 0.0f};
static uint32_t g_adc1_mag_int_enable_mask = 0U;
static uint32_t g_adc2_mag_int_enable_mask = 0U;

static MotorHardware g_motor_controller_type = -1;

void MotorAdcInit(MotorHardware motor_controller_type) {
  assert(0 <= motor_controller_type
         && motor_controller_type < ARRAYSIZE(kAdcCalibration));
  g_motor_controller_type = motor_controller_type;

  g_i_phase_a_cal.slope = kAdcCalibration[motor_controller_type].i_phase_slope;
  g_i_phase_b_cal.slope = kAdcCalibration[motor_controller_type].i_phase_slope;
  g_i_phase_c_cal.slope = kAdcCalibration[motor_controller_type].i_phase_slope;
  g_i_bus_cal.slope = kAdcCalibration[motor_controller_type].i_bus_slope;

  // Enable peripheral clocks.
  PeripheralEnable(kPeripheralMibAdc1);
  PeripheralEnable(kPeripheralMibAdc2);

  // Reset ADC modules.
  ADC(1).RSTCR.RESET = 1;
  ADC(1).RSTCR.RESET = 0;

  ADC(2).RSTCR.RESET = 1;
  ADC(2).RSTCR.RESET = 0;

  // Enable ADCs state machine in 12-bit mode.
  ADC(1).OPMODECR._10_12_BIT = 1;
  ADC(1).OPMODECR.ADC_EN = 1;

  ADC(2).OPMODECR._10_12_BIT = 1;
  ADC(2).OPMODECR.ADC_EN = 1;

  // Set ADC clock.
  // fADCLK = fVCLK / (PS + 1) [TRM Table 22-9].
  // Maximum fADCLK is 30 MHz [DS Table 5-21].
  // When fVCLK = 80 MHz and PS = 2, fADCLK = 26.6 MHz.
  double f_adclk = 30e6;
  double f_vclk = PeripheralGetClockFreq(kPeripheralMibAdc1);
  int32_t prescale = ceil(f_vclk / f_adclk - 1.0);
  ADC(1).CLOCKCR.PS = prescale;
  ADC(2).CLOCKCR.PS = prescale;
  f_adclk = f_vclk / (double)(prescale + 1);

  // Set sampling window.
  // SW = (G1_ACQ + 2) * tADCLK [TRM Table 22-32].
  // Minimum SW is 0.2 us. [DS Table 5-21].
  // When fADCLK = 26.6 MHz and G1_ACQ = 4, SW = 0.225 us.
  ADC(1).G1SAMP.G1_ACQ = ceil(0.3e-6 * f_adclk - 2.0);
  ADC(2).G1SAMP.G1_ACQ = ceil(0.3e-6 * f_adclk - 2.0);

  // Set result memory size.
  // TMS570 supports up to 64 result buffers per ADC [TRM Table 22-30].
  ADC(1).BNDEND.BNDEND = 2;
  ADC(2).BNDEND.BNDEND = 2;

  // Set result memory segments.
  // Event group result memory segment is [0, 2*BNDA) [TRM Table 22-29].
  // When BNDA = 0 and BNDB = 32, 64 result buffers are allocated to Group 1.
  int32_t num_channels = motor_controller_type == kMotorHardwareOzoneA1 ? 5 : 4;
  ADC(1).BNDCR.BNDA = 0;
  ADC(1).BNDCR.BNDB = num_channels * NUM_ADC_SAMPLES / 2;

  ADC(2).BNDCR.BNDA = 0;
  ADC(2).BNDCR.BNDB = num_channels * NUM_ADC_SAMPLES / 2;

  // Channel ID, conversion mode, and ram overwrite.
  ADC(1).G1MODECR.G1_CHID = 1;  // Enable channel ID for group 1 ADC results.
  ADC(1).G1MODECR.G1_MODE = 1;  // Convert channels continuously.
  ADC(1).G1MODECR.OVR_G1_RAM_IGN = 1;  // Overwrite RAM with new ADC results.

  ADC(2).G1MODECR.G1_CHID = 1;
  ADC(2).G1MODECR.G1_MODE = 1;
  ADC(2).G1MODECR.OVR_G1_RAM_IGN = 1;

  // Select channels to sample.
  // There are a number of restrictions to the channel mapping:
  // - Each ADC needs to sample at least two phase current channels due to
  //   limitations with the magnitude interrupt hardware.
  // - Gin controllers should not sample v_chassis and v_common_mode to
  //   eliminate a potential noise path.
  // - The bus voltage switches from ADC1-CH16 / ADC2-CH0 on the Gin PCB to
  //   ADC1-CH14 / ADC2-CH14 on the Ozone PCB.
  // - Channels are sampled in ascending order and this order needs to match the
  //   packed data structures.
  // It can be shown that there is no ordering which satisfies these
  // requirements for both Gin and Ozone which is why each controller is treated
  // separately.
  if (motor_controller_type == kMotorHardwareOzoneA1) {
    ADC(1).G1SEL.raw
        = 1 << ADC_V_IN_MONITOR_CHANNEL
        | 1 << ADC_V_CHASSIS_CHANNEL
        | 1 << ADC_I_PHASE_A_CHANNEL
        | 1 << ADC_I_PHASE_B_CHANNEL
        | 1 << ADC_I_BUS_CHANNEL;

    // ADC1 CH16-23 happen to map to ADC2 CH0-7.
    ADC(2).G1SEL.raw
        = 1 << (ADC_V_CM_CHANNEL % 16)
        | 1 << (ADC_I_PHASE_C_CHANNEL % 16)
        | 1 << (ADC_I_PHASE_B_CHANNEL % 16)
        | 1 << (ADC_V_AUX_MONITOR_CHANNEL % 16)
        | 1 << (ADC_V_BUS_OZONE_CHANNEL % 16);
  } else {
    ADC(1).G1SEL.raw
        = 1 << ADC_V_IN_MONITOR_CHANNEL
        | 1 << ADC_I_PHASE_A_CHANNEL
        | 1 << ADC_I_PHASE_B_CHANNEL
        | 1 << ADC_I_BUS_CHANNEL;

    // ADC1 CH16-23 happen to map to ADC2 CH0-7.
    ADC(2).G1SEL.raw
        = 1 << (ADC_V_BUS_GIN_CHANNEL % 16)
        | 1 << (ADC_I_PHASE_C_CHANNEL % 16)
        | 1 << (ADC_I_PHASE_B_CHANNEL % 16)
        | 1 << (ADC_V_AUX_MONITOR_CHANNEL % 16);
  }
}

static float AdcCalcOffset(float slope, int32_t adc_id, int32_t channel_id) {
  union ADC_G1BUFFER data;
  int32_t sum = 0;

  const int32_t kNumSamples = 128;
  for (int32_t i = 0; i < kNumSamples; ++i) {
    do {
      data.raw = ADC(adc_id).G1BUFFER[0].raw;
    } while (data.G1_EMPTY || data.G1_CHID != channel_id);
    sum += data.G1_DR;
  }
  // The calibration slope also includes a factor of 1 / NUM_ADC_SAMPLES which
  // needs to be canceled when converting the ADC count average to a dimensional
  // quantity.
  return (float)NUM_ADC_SAMPLES * slope * (float)sum / (float)kNumSamples;
}

void MotorAdcCalibrateOffsets(void) {
  // g_motor_controller_type >= 0 implies that MotorAdcInit has been called such
  // that the calibration slopes have been initialized.
  assert(g_motor_controller_type >= 0);
  g_i_phase_a_cal.offset = AdcCalcOffset(g_i_phase_a_cal.slope, 1,
                                         ADC_I_PHASE_A_CHANNEL);
  g_i_phase_b_cal.offset = AdcCalcOffset(g_i_phase_b_cal.slope, 1,
                                         ADC_I_PHASE_B_CHANNEL);
  g_i_phase_c_cal.offset = AdcCalcOffset(g_i_phase_c_cal.slope, 2,
                                         ADC_I_PHASE_C_CHANNEL % 16);
  g_i_bus_cal.offset = AdcCalcOffset(g_i_bus_cal.slope, 1,
                                     ADC_I_BUS_CHANNEL);
}

static inline uint32_t MagIntCalcThresh(int32_t mag_int_thresh,
                                        uint32_t select_mask,
                                        uint32_t *mag_int_enable_mask) {
  if (mag_int_thresh > 0x0FFF) {
    *mag_int_enable_mask &= ~select_mask;
    return 0x0FFF;
  } else if (mag_int_thresh < 0x00) {
    *mag_int_enable_mask &= ~select_mask;
    return 0x00;
  } else {
    *mag_int_enable_mask |= select_mask;
    return mag_int_thresh;
  }
}

// Calculate and set maximum counts for the ADC interrupt.
static inline uint32_t MagIntSetThresholds(float phase_limit) {
  // Saturate phase_limit so that conversion to integer doesn't overflow with
  // undefined behavior.
  assert(g_motor_controller_type >= 0);
  AdcCalibration const *adc_cal = &kAdcCalibration[g_motor_controller_type];
  float amps_to_adc_counts = 1.0f / (adc_cal->i_phase_slope * NUM_SAMP_F);

  float adc_limit = Saturatef(amps_to_adc_counts * phase_limit,
                              (float)(-0x3000), (float)(0x3000));

  // Phase A (upper and lower limit, respectively).
  float adc_offset = amps_to_adc_counts * g_i_phase_a_cal.offset;
  ADC(1).MAGINT1CR.MAG_THR = MagIntCalcThresh((int32_t)(adc_offset + adc_limit),
                                              ADC1_MAG_INT1_ENABLE_MASK,
                                              &g_adc1_mag_int_enable_mask);
  ADC(1).MAGINT2CR.MAG_THR = MagIntCalcThresh((int32_t)(adc_offset - adc_limit),
                                              ADC1_MAG_INT2_ENABLE_MASK,
                                              &g_adc1_mag_int_enable_mask);

  // Phase B (upper and lower limit, respectively).
  adc_offset = amps_to_adc_counts * g_i_phase_b_cal.offset;
  ADC(1).MAGINT3CR.MAG_THR = MagIntCalcThresh((int32_t)(adc_offset + adc_limit),
                                              ADC1_MAG_INT3_ENABLE_MASK,
                                              &g_adc1_mag_int_enable_mask);
  ADC(2).MAGINT1CR.MAG_THR = MagIntCalcThresh((int32_t)(adc_offset - adc_limit),
                                              ADC2_MAG_INT1_ENABLE_MASK,
                                              &g_adc2_mag_int_enable_mask);

  // Phase C (upper and lower limit, respectively).
  adc_offset = amps_to_adc_counts * g_i_phase_c_cal.offset;
  ADC(2).MAGINT2CR.MAG_THR = MagIntCalcThresh((int32_t)(adc_offset + adc_limit),
                                              ADC2_MAG_INT2_ENABLE_MASK,
                                              &g_adc2_mag_int_enable_mask);
  ADC(2).MAGINT3CR.MAG_THR = MagIntCalcThresh((int32_t)(adc_offset - adc_limit),
                                              ADC2_MAG_INT3_ENABLE_MASK,
                                              &g_adc2_mag_int_enable_mask);

  // Throw warnings if comparison interrupts have been disabled.
  uint32_t warnings = kMotorWarningNone;
  if (g_adc1_mag_int_enable_mask != ADC1_MAG_INT_ENABLE_MASK ||
      g_adc2_mag_int_enable_mask != ADC2_MAG_INT_ENABLE_MASK) {
    warnings |= kMotorWarningAdcFaultDisabledSome;
  }

  if (g_adc1_mag_int_enable_mask == 0u && g_adc2_mag_int_enable_mask == 0u) {
    warnings |= kMotorWarningAdcFaultDisabledAll;
  }

  return warnings;
}

static inline void MagIntClearIntFlags(void) {
  ADC(1).MAGINTFLG.raw = 0x07;
  ADC(2).MAGINTFLG.raw = 0x07;
}

// Reset the ADC magnitude interrupt to use phase_limit.
uint32_t MotorAdcMagnitudeInterruptReset(float phase_limit) {
  static float phase_limit_last = -1.0f;
  uint32_t warnings = kMotorWarningNone;

  if (phase_limit != phase_limit_last) {
    VimDisableInterrupt(kVimChannelAdc1MagnitudeCompare);
    VimDisableInterrupt(kVimChannelAdc2MagnitudeCompare);

    warnings = MagIntSetThresholds(phase_limit);
    MagIntClearIntFlags();
    MotorAdcMagnitudeInterruptEnable();

    VimEnableInterrupt(kVimChannelAdc1MagnitudeCompare);
    VimEnableInterrupt(kVimChannelAdc2MagnitudeCompare);

    phase_limit_last = phase_limit;
  }
  return warnings;
}

uint32_t MotorAdcMagnitudeInterruptInit(float phase_limit) {
  assert(g_motor_controller_type >= 0);
  AdcCalibration const *adc_cal = &kAdcCalibration[g_motor_controller_type];

  // Enable magnitude compare interrupt.
  ADC(1).MAGINT1CR.MAG_CHID = ADC_I_PHASE_A_CHANNEL;
  ADC(1).MAGINT2CR.MAG_CHID = ADC_I_PHASE_A_CHANNEL;
  ADC(1).MAGINT3CR.MAG_CHID = ADC_I_PHASE_B_CHANNEL;

  // Mapping CH16-23 -> CH0-7 for ADC2 because of unfortunate TI ADC
  // mapping.
  ADC(2).MAGINT1CR.MAG_CHID = ADC_I_PHASE_B_CHANNEL % 16;
  ADC(2).MAGINT2CR.MAG_CHID = ADC_I_PHASE_C_CHANNEL % 16;
  ADC(2).MAGINT3CR.MAG_CHID = ADC_I_PHASE_C_CHANNEL % 16;

  // Configure the interrupt comparison.
  // Phase A.
  ADC(1).MAGINT1CR.CMP_GE_LT = adc_cal->i_phase_slope > 0.0f ? 1 : 0;
  ADC(1).MAGINT2CR.CMP_GE_LT = adc_cal->i_phase_slope > 0.0f ? 0 : 1;

  // Phase B.
  ADC(1).MAGINT3CR.CMP_GE_LT = adc_cal->i_phase_slope > 0.0f ? 1 : 0;
  ADC(2).MAGINT1CR.CMP_GE_LT = adc_cal->i_phase_slope > 0.0f ? 0 : 1;

  // Phase C.
  ADC(2).MAGINT2CR.CMP_GE_LT = adc_cal->i_phase_slope > 0.0f ? 1 : 0;
  ADC(2).MAGINT3CR.CMP_GE_LT = adc_cal->i_phase_slope > 0.0f ? 0 : 1;

  uint32_t warnings = MagIntSetThresholds(fabsf(phase_limit));

  // Clear the pending interrupt flags and enable the interrupts.
  MagIntClearIntFlags();
  MotorAdcMagnitudeInterruptEnable();

  // Setup ISR for ADC1 and ADC2 magnitude interrupts.
  VimRegisterFiq(kVimChannelAdc1MagnitudeCompare,
                 MotorAdcMagnitudeInterruptHandler);
  VimEnableInterrupt(kVimChannelAdc1MagnitudeCompare);
  VimRegisterFiq(kVimChannelAdc2MagnitudeCompare,
                 MotorAdcMagnitudeInterruptHandler);
  VimEnableInterrupt(kVimChannelAdc2MagnitudeCompare);

  return warnings;
}

// Enable all ADC magnitude interrupts. This function should reflect the
// ordering in MotorAdcMagnitudeInterruptInit().
void MotorAdcMagnitudeInterruptEnable(void) {
  // Enable or disable all magnitude interrupts according to their enable masks.
  ADC(1).MAGINTENACLR.raw = ~g_adc1_mag_int_enable_mask;
  ADC(1).MAGINTENASET.raw = g_adc1_mag_int_enable_mask;

  ADC(2).MAGINTENACLR.raw = ~g_adc2_mag_int_enable_mask;
  ADC(2).MAGINTENASET.raw = g_adc2_mag_int_enable_mask;
}

// Map ADC threshold interrupts to specific motor phase errors. In addition,
// this function clears the pending interrupt flag, and disables the interrupt.
// This function should reflect the ordering in
// MotorAdcMagnitudeInterruptInit().
uint32_t MotorAdcGetMagnitudeErrors(void) {
  uint32_t int_src1 = ADC(1).MAGINTOFF.raw;
  uint32_t int_src2 = ADC(2).MAGINTOFF.raw;

  uint32_t errors = 0;

  switch (int_src1) {
    case ADC1_MAG_INT_IAP_NUM:
      ADC(1).MAGINTENACLR.raw = (1 << (ADC1_MAG_INT_IAP_NUM - 1));
      errors |= kMotorErrorFaultCurrentIaP;
      break;
    case ADC1_MAG_INT_IAN_NUM:
      ADC(1).MAGINTENACLR.raw = (1 << (ADC1_MAG_INT_IAN_NUM - 1));
      errors |= kMotorErrorFaultCurrentIaN;
      break;
    case ADC1_MAG_INT_IBP_NUM:
      ADC(1).MAGINTENACLR.raw = (1 << (ADC1_MAG_INT_IBP_NUM - 1));
      errors |= kMotorErrorFaultCurrentIbP;
      break;
    default:
      break;
  }

  switch (int_src2) {
    case ADC2_MAG_INT_IBN_NUM:
      ADC(2).MAGINTENACLR.raw = (1 << (ADC2_MAG_INT_IBN_NUM - 1));
      errors |= kMotorErrorFaultCurrentIbN;
      break;
    case ADC2_MAG_INT_ICP_NUM:
      ADC(2).MAGINTENACLR.raw = (1 << (ADC2_MAG_INT_ICP_NUM - 1));
      errors |= kMotorErrorFaultCurrentIcP;
      break;
    case ADC2_MAG_INT_ICN_NUM:
      ADC(2).MAGINTENACLR.raw = (1 << (ADC2_MAG_INT_ICN_NUM - 1));
      errors |= kMotorErrorFaultCurrentIcN;
      break;
    default:
      break;
  }

  return errors;
}

__attribute__((optimize("unroll-loops")))
    static inline float AdcValue(volatile union ADCRAM_RAM buf[NUM_ADC_SAMPLES],
                                 const AdcCal *adc_cal, int32_t channel_id) {
  int32_t data_sum = 0;
  for (int32_t i = 0; i < NUM_ADC_SAMPLES; ++i) {
    data_sum += buf[i].raw;
  }
  data_sum -= NUM_ADC_SAMPLES * (channel_id << 12);
  return adc_cal->slope * (float)data_sum - adc_cal->offset;
}

// Return the measured phase currents; bus current; bus voltage; and, if
// configured for ozone, chassis voltage and commmon mode voltage.
//
// This function contains a large number of loops. When just compiled with -O2,
// loop overhead can account for as much as 65% of the total execution time.
// When originally applied and profiled, the unroll-loops attribute reduced
// TMS570 execution time by 3.0 us down to about 1.6 us.
void MotorAdcGetValues(float *ia, float *ib, float *ic,
                       float *i_bus, float *v_bus, float *v_chassis,
                       float *v_cm, float *v_in_mon, float *v_aux_mon) {
  assert(g_motor_controller_type >= 0);
  assert(ia != NULL && ib != NULL && ic != NULL);
  assert(i_bus != NULL && v_bus != NULL && v_chassis != NULL && v_cm != NULL);

  AdcCalibration const *adc_cal = &kAdcCalibration[g_motor_controller_type];
  if (g_motor_controller_type == kMotorHardwareOzoneA1) {
    *ia = AdcValue(g_adc_1_buffer.ozone_i_phase_a, &g_i_phase_a_cal,
                   ADC_I_PHASE_A_CHANNEL);
    *ib = AdcValue(g_adc_1_buffer.ozone_i_phase_b, &g_i_phase_b_cal,
                   ADC_I_PHASE_B_CHANNEL);
    *ic = AdcValue(g_adc_2_buffer.ozone_i_phase_c, &g_i_phase_c_cal,
                   ADC_I_PHASE_C_CHANNEL % 16);
    *i_bus = AdcValue(g_adc_1_buffer.ozone_i_bus, &g_i_bus_cal,
                      ADC_I_BUS_CHANNEL);
    *v_bus = AdcValue(g_adc_2_buffer.ozone_v_bus, &adc_cal->v_bus,
                      ADC_V_BUS_OZONE_CHANNEL % 16);
    *v_cm = AdcValue(g_adc_2_buffer.ozone_v_cm, &adc_cal->v_cm,
                     ADC_V_CM_CHANNEL % 16);
    *v_chassis = AdcValue(g_adc_1_buffer.ozone_v_chassis, &adc_cal->v_chassis,
                          ADC_V_CHASSIS_CHANNEL);
    *v_in_mon = AdcValue(g_adc_1_buffer.ozone_v_in_monitor,
                         &adc_cal->v_in_mon, ADC_V_IN_MONITOR_CHANNEL);
    *v_aux_mon = AdcValue(g_adc_2_buffer.ozone_v_aux_monitor,
                          &adc_cal->v_aux_mon, ADC_V_AUX_MONITOR_CHANNEL);
  } else {
    *ia = AdcValue(g_adc_1_buffer.gin_i_phase_a, &g_i_phase_a_cal,
                   ADC_I_PHASE_A_CHANNEL);
    *ib = AdcValue(g_adc_1_buffer.gin_i_phase_b, &g_i_phase_b_cal,
                   ADC_I_PHASE_B_CHANNEL);
    *ic = AdcValue(g_adc_2_buffer.gin_i_phase_c, &g_i_phase_c_cal,
                   ADC_I_PHASE_C_CHANNEL % 16);
    *i_bus = AdcValue(g_adc_1_buffer.gin_i_bus, &g_i_bus_cal,
                      ADC_I_BUS_CHANNEL);
    *v_bus = AdcValue(g_adc_2_buffer.gin_v_bus, &adc_cal->v_bus,
                      ADC_V_BUS_GIN_CHANNEL % 16);
    *v_cm = 0.0f;
    *v_chassis = 0.0f;
    *v_in_mon = AdcValue(g_adc_1_buffer.gin_v_in_monitor,
                         &adc_cal->v_in_mon, ADC_V_IN_MONITOR_CHANNEL);
    *v_aux_mon = AdcValue(g_adc_2_buffer.gin_v_aux_monitor,
                          &adc_cal->v_aux_mon, ADC_V_AUX_MONITOR_CHANNEL);
  }

  // If phase A and C are swapped.
  if (kMotorConfigParams->phase_swap == kMotorPhaseSwapAc) {
    Swapf(ia, ic);
  }
}

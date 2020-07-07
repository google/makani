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

#include "avionics/firmware/cpu/adc.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "avionics/firmware/cpu/clock.h"
#include "avionics/firmware/cpu/peripherals.h"
#include "avionics/firmware/cpu/registers.h"

#define ADC_CAL_SAMPLES 128
#define ADC_CAL_TIMEOUT_CYCLES CLOCK32_USEC_TO_CYCLES(10)

typedef enum {
  kAdcCalReference1v25,
  kAdcCalReference1v75,
  kAdcCalReferenceHigh,
  kAdcCalReferenceLow,
  kAdcCalReferenceVin
} AdcCalReference;

// Number of channels to convert.
static int32_t g_channel_count1 = 0x0;
static int32_t g_channel_count2 = 0x0;

static bool g_adc_initialized[2] = {false, false};

// The ADC converts all channels specified in the Gx_SEL bitmask and places
// the result in ADCRAM. The least significant bit occurs first followed by
// the next set bit. This function counts the total number of results to
// expect in ADCRAM.
static int32_t CountChannels(uint32_t channel_mask) {
  int32_t channels = 0;
  for (uint32_t m = channel_mask; m != 0U; m &= m - 1U) {
    ++channels;
  }
  return channels;
}

static void ReadResults(int32_t adc, int32_t start, int32_t channels,
                        int32_t result_length, int16_t *result_data) {
  assert(adc == 1 || adc == 2);
  assert(0 <= start);
  assert(0 <= channels && channels <= ADC_CHANNELS);
  assert(0 <= result_length);
  assert(result_data != NULL);
  memset(result_data, 0, result_length * sizeof(*result_data));

  int32_t end = start + channels;
  for (int32_t i = start; i < end; ++i) {
    union ADCRAM_RAM data = ADCRAM(adc).RAM[i];
    int32_t ch = data.CHID;
    if (0 <= ch && ch < result_length) {
      result_data[ch] = data.DR;
    }
  }
}

static int16_t Calibrate(int32_t adc, AdcCalReference ref) {
  assert(adc == 1 || adc == 2);
  // Configure voltage divider calibration reference bridge.
  union ADC_ALCR alcr = ADC(adc).ALCR;
  int16_t expected_value;
  switch (ref) {
    case kAdcCalReference1v25:
      alcr.CAL_EN = 1;
      alcr.BRIDGE_EN = 0;
      alcr.HILO = 1;
      expected_value = (int16_t)(ADC_MAX_VALUE * 1.25f / ADC_MAX_VOLTAGE);
      break;
    case kAdcCalReference1v75:
      alcr.CAL_EN = 1;
      alcr.BRIDGE_EN = 0;
      alcr.HILO = 0;
      expected_value = (int16_t)(ADC_MAX_VALUE * 1.75f / ADC_MAX_VOLTAGE);
      break;
    case kAdcCalReferenceHigh:
      alcr.CAL_EN = 1;
      alcr.BRIDGE_EN = 1;
      alcr.HILO = 1;
      expected_value = ADC_MAX_VALUE;
      break;
    case kAdcCalReferenceLow:
      alcr.CAL_EN = 1;
      alcr.BRIDGE_EN = 1;
      alcr.HILO = 0;
      expected_value = 0;
      break;
    case kAdcCalReferenceVin:
      // Fall-through intentional.
    default:
      alcr.CAL_EN = 0;
      alcr.BRIDGE_EN = 0;  // Don't care.
      alcr.HILO = 0;  // Don't care.
      expected_value = ADC_MAX_VALUE;
      break;
  }
  ADC(adc).ALCR = alcr;
  ADC(adc).ALCR.CAL_ST = 1;

  // Wait for conversion.
  uint32_t timeout = Clock32GetCycles() + ADC_CAL_TIMEOUT_CYCLES;
  while (ADC(adc).ALCR.CAL_ST != 0 && CLOCK32_LT(Clock32GetCycles(), timeout)) {
  }

  // Result.
  return ADC(adc).ALCR.CAL_ST == 0 ? expected_value : 0;
}

void AdcInit(int32_t adc) {
  assert(adc == 1 || adc == 2);

  if (g_adc_initialized[adc - 1]) {
    return;
  }
  // Configure hardware pins.

  // Enable peripheral clocks.
  if (adc == 1) {
    PeripheralEnable(kPeripheralMibAdc1);
  } else if (adc == 2) {
    PeripheralEnable(kPeripheralMibAdc2);
  }

  // Reset ADC modules.
  ADC(adc).RSTCR.RESET = 1;
  ADC(adc).RSTCR.RESET = 0;

  // Enable ADCs state machine in 12-bit mode.
  ADC(adc).OPMODECR._10_12_BIT = 1;
  ADC(adc).OPMODECR.ADC_EN = 1;

  // Set clock prescale and sampling window.
  ADC(adc).CLOCKCR.PS = 2;
  ADC(adc).G1SAMP.G1_ACQ = 4;
  ADC(adc).G2SAMP.G2_ACQ = 4;

  // Set conversion memory boundaries.
  ADC(adc).BNDCR.BNDA = 0;
  ADC(adc).BNDCR.BNDB = ADC_CHANNELS;
  ADC(adc).BNDEND.BNDEND = 2;  // 2 = 64 words.

  g_adc_initialized[adc - 1] = true;
}

float AdcCalibrate(int32_t adc) {
  assert(adc == 1 || adc == 2);
  // See calibration procedure in TMS570 TRM 22.8.1.2 "Calibration and Offset
  // Error Correction Sequences".
  ADC(adc).ALCR.CAL_EN = 1;  // Enter calibration mode.

  // Accumulate ADC_CAL_SAMPLES to minimize the affect of noise and numerical
  // precision errors.
  int32_t ref_low = 0;
  for (int32_t i = 0; i < ADC_CAL_SAMPLES; ++i) {
    ref_low += Calibrate(adc, kAdcCalReferenceLow);
  }
  int32_t ref_high = 0;
  for (int32_t i = 0; i < ADC_CAL_SAMPLES; ++i) {
    ref_high += Calibrate(adc, kAdcCalReferenceHigh);
  }
  int32_t ref_mid = (ref_high - ref_low) / 2;

  ADC(adc).ALCR.CAL_EN = 0;  // Leave calibration mode.

  // The TMS570 will automatically add this value to each sample.
  ADC(adc).ALR.ADCALR = ref_mid / ADC_CAL_SAMPLES - ADC_MAX_VALUE / 2;

  // Return the calibration scale factor.
  return (ref_high > ref_low) ?
      ADC_MAX_VALUE * ADC_CAL_SAMPLES / (float)(ref_high - ref_low) : 0.0f;
}

void AdcTriggerGroup1(int32_t adc, uint32_t channel_mask) {
  assert(adc == 1 || adc == 2);
  // Trigger start of conversion.
  ADC(adc).G1SEL.G1_SEL = channel_mask;
  g_channel_count1 = CountChannels(channel_mask);
}

void AdcTriggerGroup2(int32_t adc, uint32_t channel_mask) {
  assert(adc == 1 || adc == 2);
  // Trigger start of conversion.
  ADC(adc).G2SEL.G2_SEL = channel_mask;
  g_channel_count2 = CountChannels(channel_mask);
}

bool AdcReadGroup1(int32_t adc, int32_t length, int16_t *result) {
  assert(adc == 1 || adc == 2);
  assert(0 <= length && length <= ADC_CHANNELS);
  assert(result != NULL);

  // Wait for end of conversion.
  if (!ADC(adc).G1SR.G1_END) {
    return false;
  }

  // Read results.
  ReadResults(adc, ADC(adc).BNDCR.BNDA, g_channel_count1, length, result);

  return true;
}

bool AdcReadGroup2(int32_t adc, int32_t length, int16_t *result) {
  assert(adc == 1 || adc == 2);
  assert(0 <= length && length <= ADC_CHANNELS);
  assert(result != NULL);

  // Wait for end of conversion.
  if (!ADC(adc).G2SR.G2_END) {
    return false;
  }

  // Read results.
  ReadResults(adc, ADC(adc).BNDCR.BNDB, g_channel_count2, length, result);

  return true;
}

bool AdcPollGroup1(int32_t adc, uint32_t channel_mask, int32_t length,
                   int16_t *result) {
  assert(adc == 1 || adc == 2);
  union ADC_G1SR sr = ADC(adc).G1SR;
  if (sr.G1_END) {
    assert(0 <= length && length <= ADC_CHANNELS);
    ReadResults(adc, ADC(adc).BNDCR.BNDA, g_channel_count1, length, result);
    ADC(adc).G1SR.raw = ADC_G1SR_G1_END;
    return true;
  } else if (!sr.G1_BUSY && channel_mask != 0x0) {
    AdcTriggerGroup1(adc, channel_mask);
  }
  return false;
}

bool AdcPollGroup2(int32_t adc, uint32_t channel_mask, int32_t length,
                   int16_t *result) {
  assert(adc == 1 || adc == 2);
  union ADC_G2SR sr = ADC(adc).G2SR;
  if (sr.G2_END) {
    assert(0 <= length && length <= ADC_CHANNELS);
    ReadResults(adc, ADC(adc).BNDCR.BNDB, g_channel_count2, length, result);
    ADC(adc).G2SR.raw = ADC_G2SR_G2_END;
    return true;
  } else if (!sr.G2_BUSY && channel_mask != 0x0) {
    AdcTriggerGroup2(adc, channel_mask);
  }
  return false;
}

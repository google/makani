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

#include "avionics/firmware/monitors/analog.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

#include "avionics/firmware/cpu/adc.h"
#include "common/macros.h"

#define ADC_MODULE 1

static float ConvertToVoltage(const AnalogMonitor *mon, int16_t in) {
  return (float)in * mon->volts_per_count - mon->offset;
}

static uint32_t ComputeVoltageFlags(const AnalogMonitor *mon, float in) {
  if (in >= mon->max) {
    return kAnalogFlagOverVoltage;
  } else if (in <= mon->min) {
    return kAnalogFlagUnderVoltage;
  } else {
    return 0x0;
  }
}

static uint32_t ComputeLogicLowFlags(int16_t in) {
  return (in < ADC_LOGIC_LOW) ? kAnalogFlagAsserted : 0x0;
}

static uint32_t ComputeLogicHighFlags(int16_t in) {
  return (in > ADC_LOGIC_HIGH) ? kAnalogFlagAsserted : 0x0;
}

static uint32_t ComputePortDetectFlags(int16_t in) {
  // ADC measures ~2.3v when detected, ~1.6v when not detected.
  int16_t high = (int16_t)(1.95f * ADC_MAX_VALUE / ADC_MAX_VOLTAGE);
  return (in > high) ? kAnalogFlagAsserted : 0x0;
}

static uint32_t ComputeFlags(const AnalogMonitor *mon, int16_t raw,
                             float value) {
  switch (mon->type) {
    case kAnalogTypeVoltage:
      return ComputeVoltageFlags(mon, value);
    case kAnalogTypeLogicLow:
      return ComputeLogicLowFlags(raw);
    case kAnalogTypeLogicHigh:
      return ComputeLogicHighFlags(raw);
    case kAnalogTypePortDetect:
      return ComputePortDetectFlags(raw);
    default:
      return 0x0;
  }
}

void AnalogMonitorInit(void) {
  AdcInit(ADC_MODULE);
}

bool AnalogMonitorPoll(const AnalogMonitors *config,
                       AnalogOutputFunction output_function) {
  // If no valid configuration exists, return true to advance to next monitor.
  if (config == NULL || config->num_devices <= 0) {
    return true;
  }

  int16_t adc[ADC_CHANNELS];
  assert(output_function != NULL);
  if (AdcPollGroup1(ADC_MODULE, config->channel_mask, ARRAYSIZE(adc), adc)) {
    for (int32_t i = 0; i < config->num_devices; ++i) {
      const AnalogMonitor *mon = &config->device[i];
      int16_t raw = adc[mon->channel];
      float value = ConvertToVoltage(mon, raw);
      output_function(mon, value, ComputeFlags(mon, raw, value));
    }
    return true;
  }
  return false;
}

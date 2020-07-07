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

#include "avionics/firmware/identity/carrier_hardware.h"

#include <stdint.h>

#include "avionics/firmware/cpu/adc.h"
#include "avionics/firmware/identity/identity_types.h"
#include "common/macros.h"

#define CARRIER_ADC_MODULE 1

#define CARRIER_ADC_CHANNEL_0 4
#define CARRIER_ADC_CHANNEL_1 5
#define CARRIER_ADC_CHANNEL_2 6
#define CARRIER_ADC_CHANNEL_3 7

CarrierHardwareType g_carrier_hardware_type = kCarrierHardwareTypeUnknown;

static CarrierHardwareType ReadCarrierHardwareType(void) {
  int16_t adc_result[ADC_CHANNELS];
  AdcInit(CARRIER_ADC_MODULE);
  AdcTriggerGroup1(CARRIER_ADC_MODULE,
                   1 << CARRIER_ADC_CHANNEL_0
                   | 1 << CARRIER_ADC_CHANNEL_1
                   | 1 << CARRIER_ADC_CHANNEL_2
                   | 1 << CARRIER_ADC_CHANNEL_3);
  while (!AdcReadGroup1(CARRIER_ADC_MODULE,
                        ARRAYSIZE(adc_result), adc_result)) {}

  CarrierHardwareType type =
      (adc_result[CARRIER_ADC_CHANNEL_0] > ADC_LOGIC_LOW ? 1 << 0 : 0)
      | (adc_result[CARRIER_ADC_CHANNEL_1] > ADC_LOGIC_LOW ? 1 << 1 : 0)
      | (adc_result[CARRIER_ADC_CHANNEL_2] > ADC_LOGIC_LOW ? 1 << 2 : 0)
      | (adc_result[CARRIER_ADC_CHANNEL_3] > ADC_LOGIC_LOW ? 1 << 3 : 0);

  return type;
}

CarrierHardwareType GetCarrierHardwareType(void) {
  if (g_carrier_hardware_type == kCarrierHardwareTypeUnknown) {
    g_carrier_hardware_type = ReadCarrierHardwareType();
  }
  return g_carrier_hardware_type;
}

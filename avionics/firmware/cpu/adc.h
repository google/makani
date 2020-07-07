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

#ifndef AVIONICS_FIRMWARE_CPU_ADC_H_
#define AVIONICS_FIRMWARE_CPU_ADC_H_

#include <stdbool.h>
#include <stdint.h>

#define ADC_CHANNELS    24
#define ADC_RESOLUTION  12
#define ADC_MAX_VOLTAGE 3.0f
#define ADC_MAX_VALUE   ((1 << ADC_RESOLUTION) - 1)
#define ADC_LOGIC_LOW   (ADC_MAX_VALUE / 3)
#define ADC_LOGIC_HIGH  (ADC_MAX_VALUE * 2 / 3)

// Initialize the ADC module.
void AdcInit(int32_t adc);

// Calibrate ADC module. Returns scale factor.
float AdcCalibrate(int32_t adc);

// Trigger an ADC conversion for each channel in channel_mask.  Channels
// are converted in ascending order.
//
// Assumes that ADC is idle.
void AdcTriggerGroup1(int32_t adc, uint32_t channel_mask);
void AdcTriggerGroup2(int32_t adc, uint32_t channel_mask);

// If an ADC conversion is finished,  reads the results.  Each channel's
// data is written into result[channel_index].  If channel_index beyond the
// end of result (as determined by length), the result is thrown away.
//
// Returns true if conversion data was read, false otherwise.
bool AdcReadGroup1(int32_t adc, int32_t length, int16_t *result);
bool AdcReadGroup2(int32_t adc, int32_t length, int16_t *result);

// If ADC is idle, triggers a conversion (see AdcTriggerGroupN().)  If an ADC
// conversion is finished, the results are read into result (see
// AdcReadGroupN().)
//
// Returns true if conversion data was read, false otherwise.
bool AdcPollGroup1(int32_t adc, uint32_t channel_mask, int32_t length,
                   int16_t *result);
bool AdcPollGroup2(int32_t adc, uint32_t channel_mask, int32_t length,
                   int16_t *result);

#endif  // AVIONICS_FIRMWARE_CPU_ADC_H_

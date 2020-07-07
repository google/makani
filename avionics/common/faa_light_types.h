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

#ifndef AVIONICS_COMMON_FAA_LIGHT_TYPES_H_
#define AVIONICS_COMMON_FAA_LIGHT_TYPES_H_

#include <stdbool.h>

#include "avionics/network/aio_labels.h"

typedef enum {
  kMotorLightTypeForceSigned = -1,
  kLightTypeVisible,
  kLightTypeInfrared,
  kNumLightTypes
} LightType;

typedef struct {
  float flashes_per_minute;
  float flash_pulse_width_us;
  float pwm_duty_cycle;
} LightInputParams;

typedef struct {
  int64_t flashes_per_minute_period_us;
  int64_t pwm_period_us;
  int64_t pwm_on_time_us;
  uint16_t bit_pattern;
} LightState;

typedef struct {
  int64_t time_us;  // Computed flash time [us].
  int64_t gps_time_of_week_us;  // [us].
  int64_t gps_update_timestamp_us;  // [us].
  AioNode source;
  bool source_valid;
} LightTiming;

#endif  // AVIONICS_COMMON_FAA_LIGHT_TYPES_H_

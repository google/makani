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

#include "avionics/firmware/monitors/thermal_types.h"

#include <assert.h>
#include <stddef.h>
#include <stdint.h>

// Converts ADC code to a temperature using a lookup table.  This
// requires the lookup table codes to be monotonically decreasing.
float ThermalCodeToTemp(int32_t code, const ThermalSensor *sensor) {
  assert(sensor != NULL);
  assert(sensor->len > 0);
  const ThermalCalData *point = &sensor->cal[sensor->len - 1];
  for (int32_t i = 0; i < sensor->len; ++i) {
    if (code > sensor->cal[i].code) {
      point = &sensor->cal[i];
      break;
    }
  }
  return point->slope * (code - point->code) + point->temperature;
}

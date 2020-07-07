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

#include "avionics/firmware/drivers/si7021_types.h"

#include <assert.h>
#include <stdint.h>

#include "avionics/common/fast_math/fast_math.h"


uint8_t Si7021BuildUserReg1(Si7021Resolution res) {
  return (uint8_t)((res & 0x02) << 6 | (res & 0x01) << 0);
}

// See Si7021-A20: 5.1.1 "Measuring Relative Humidity".
float Si7021RelHumidityRawToPercent(uint16_t raw) {
  float percent = (125.0f * raw) / 65536.0f - 6.0f;
  return Saturatef(percent, 0.0f, 100.0f);
}

float Si7021TempRawToC(uint16_t raw) {
  return (175.72f * (float)raw) / 65536.0f - 46.85f;
}

void Si7021Convert(const Si7021OutputRaw *raw, Si7021OutputData *data) {
  data->rel_humidity = Si7021RelHumidityRawToPercent(raw->rel_humidity);
  data->temperature = Si7021TempRawToC(raw->temperature);
}

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

#include "avionics/firmware/drivers/ina219_types.h"

#include <assert.h>
#include <stdint.h>

uint16_t Ina219BuildConfig(Ina219BusVoltage bus_voltage, Ina219Range range,
                           Ina219Adc bus_adc, Ina219Adc shunt_adc,
                           Ina219Mode mode) {
  assert((bus_voltage & 0x01) == bus_voltage);
  assert((range & 0x03) == range);
  assert((bus_adc & 0x0F) == bus_adc);
  assert((shunt_adc & 0x0F) == shunt_adc);
  assert((mode & 0x07) == mode);

  return (uint16_t)((bus_voltage & 0x01) << 13
                    | (range & 0x03) << 11
                    | (bus_adc & 0x0F) << 7
                    | (shunt_adc & 0x0F) << 3
                    | (mode & 0x07));
}

int32_t Ina219BusRawToMillivolts(uint16_t bus_raw) {
  return (int32_t)(bus_raw >> 3) * 4;
}

float Ina219ShuntRawToAmps(int16_t shunt_raw, float shunt_resistor) {
  return (float)shunt_raw * 10e-6f / shunt_resistor;
}

void Ina219Convert(const Ina219Config *config, const Ina219OutputRaw *raw,
                   Ina219OutputData *data) {
  data->voltage = 0.001f * (float)Ina219BusRawToMillivolts(raw->bus_raw);
  data->current = Ina219ShuntRawToAmps(raw->shunt_raw, config->shunt_resistor);
}

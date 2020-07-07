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

#include "avionics/firmware/drivers/ltc4151_types.h"

#include <stdint.h>

int32_t Ltc4151BusRawToMillivolts(uint16_t bus_raw) {
  return (int32_t)(bus_raw >> 4) * 25;  // See Ltc4151 datasheet p12, table 4.
}

float Ltc4151ShuntRawToAmps(int16_t shunt_raw, float shunt_resistor) {
  // See Ltc4151 datasheet page 12, table 3.
  return (float)(shunt_raw >> 4) * 20e-6f / shunt_resistor;
}

void Ltc4151Convert(const Ltc4151Config *config, const Ltc4151OutputRaw *raw,
                    Ltc4151OutputData *data) {
  data->voltage = 0.001f * (float)Ltc4151BusRawToMillivolts(raw->bus_raw);
  data->current = Ltc4151ShuntRawToAmps(raw->shunt_raw, config->shunt_resistor);
}

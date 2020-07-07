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

#ifndef AVIONICS_FIRMWARE_DRIVERS_LTC4151_TYPES_H_
#define AVIONICS_FIRMWARE_DRIVERS_LTC4151_TYPES_H_

#include <stdint.h>

typedef struct {
  uint8_t addr;
  uint8_t binary_config;
  float shunt_resistor;
} Ltc4151Config;

typedef struct {
  uint16_t bus_raw;
  int16_t shunt_raw;
} Ltc4151OutputRaw;

typedef struct {
  float voltage;
  float current;
} Ltc4151OutputData;

int32_t Ltc4151BusRawToMillivolts(uint16_t bus_raw);
float Ltc4151ShuntRawToAmps(int16_t shunt_raw, float shunt_resistor);
void Ltc4151Convert(const Ltc4151Config *config, const Ltc4151OutputRaw *raw,
                    Ltc4151OutputData *data);

#endif  // AVIONICS_FIRMWARE_DRIVERS_LTC4151_TYPES_H_

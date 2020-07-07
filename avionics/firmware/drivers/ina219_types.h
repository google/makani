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

#ifndef AVIONICS_FIRMWARE_DRIVERS_INA219_TYPES_H_
#define AVIONICS_FIRMWARE_DRIVERS_INA219_TYPES_H_

#include <stdint.h>

typedef enum {
  kIna219BusVoltage16V,
  kIna219BusVoltage32V  // Hardware default.
} Ina219BusVoltage;

typedef enum {
  kIna219Range40mv,
  kIna219Range80mv,
  kIna219Range160mv,
  kIna219Range320mv  // Hardware default.
} Ina219Range;

typedef enum {
  kIna219Adc9Bit       = 0x00,
  kIna219Adc10Bit      = 0x01,
  kIna219Adc11Bit      = 0x02,
  kIna219Adc12Bit      = 0x03,  // Hardware default.
  kIna219Adc2Samples   = 0x08,
  kIna219Adc4Samples   = 0x09,
  kIna219Adc8Samples   = 0x0A,
  kIna219Adc16Samples  = 0x0B,
  kIna219Adc32Samples  = 0x0C,
  kIna219Adc64Samples  = 0x0D,
  kIna219Adc128Samples = 0x0F
} Ina219Adc;

typedef enum {
  kIna219ModePowerDown,
  kIna219ModeShuntTriggered,
  kIna219ModeBusTriggered,
  kIna219ModeShuntAndBusTriggered,
  kIna219ModeAdcDisabled,
  kIna219ModeShuntContinuous,
  kIna219ModeBusContinuous,
  kIna219ModeShuntAndBusContinuous  // Hardware default.
} Ina219Mode;

typedef struct {
  uint8_t addr;
  uint16_t config;
  float shunt_resistor;
  Ina219BusVoltage bus_voltage;
  Ina219Range range;
  Ina219Adc bus_adc;
  Ina219Adc shunt_adc;
  Ina219Mode mode;
} Ina219Config;

typedef struct {
  uint16_t bus_raw;
  int16_t shunt_raw;
} Ina219OutputRaw;

typedef struct {
  float voltage;
  float current;
} Ina219OutputData;

uint16_t Ina219BuildConfig(Ina219BusVoltage bus_voltage, Ina219Range range,
                           Ina219Adc bus_adc, Ina219Adc shunt_adc,
                           Ina219Mode mode);
int32_t Ina219BusRawToMillivolts(uint16_t bus_raw);
float Ina219ShuntRawToAmps(int16_t shunt_raw, float shunt_resistor);
void Ina219Convert(const Ina219Config *config, const Ina219OutputRaw *raw,
                   Ina219OutputData *data);

#endif  // AVIONICS_FIRMWARE_DRIVERS_INA219_TYPES_H_

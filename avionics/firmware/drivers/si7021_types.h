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

#ifndef AVIONICS_FIRMWARE_DRIVERS_SI7021_TYPES_H_
#define AVIONICS_FIRMWARE_DRIVERS_SI7021_TYPES_H_

#include <stdint.h>

typedef enum {
  kSi7021ResolutionRh12BitTemp14Bit = 0x00,
  kSi7021ResolutionRh8BitTemp12Bit  = 0x01,
  kSi7021ResolutionRh10BitTemp13Bit = 0x02,
  kSi7021ResolutionRh11BitTemp11Bit = 0x03
} Si7021Resolution;

typedef enum {
  kSi7021CommandMeasureRelHumidityHold   = 0xE5,
  kSi7021CommandMeasureRelHumidityNoHold = 0xF5,
  kSi7021CommandMeasureTemperatureHold   = 0xE3,
  kSi7021CommandMeasureTemperatureNoHold = 0xF3,
  kSi7021CommandReadTemperature          = 0xE0,
  kSi7021CommandReset                    = 0xFE,
  kSi7021CommandWriteUserReg1            = 0xE6,
  kSi7021CommandReadUserReg1             = 0xE7,
  kSi7021CommandWriteHeaterControlReg    = 0x51,
  kSi7021CommandReadHeaterControlReg     = 0x11,
  kSi7021CommandReadElectronicIdByte1    = 0xFA0F,
  kSi7021CommandReadElectronicIdByte2    = 0xFCC9,
  kSi7021CommandReadFirmwareRevision     = 0x84B8
} Si7021Command;

typedef struct {
  uint16_t rel_humidity;
  uint16_t temperature;
} Si7021OutputRaw;

typedef struct {
  float rel_humidity;
  float temperature;
} Si7021OutputData;

typedef struct {
  uint8_t addr;
  uint8_t user_reg1;
} Si7021Config;

uint8_t Si7021BuildUserReg1(Si7021Resolution res);
float Si7021RelHumidityRawToPercent(uint16_t raw);
float Si7021TempRawToC(uint16_t raw);
void Si7021Convert(const Si7021OutputRaw *raw, Si7021OutputData *data);

#endif  // AVIONICS_FIRMWARE_DRIVERS_SI7021_TYPES_H_

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

#ifndef AVIONICS_FIRMWARE_DRIVERS_LTC2309_TYPES_H_
#define AVIONICS_FIRMWARE_DRIVERS_LTC2309_TYPES_H_

#include <stdint.h>

// Select channel to convert.
typedef enum {
  kLtc2309SelectDiffCh0Ch1 = 0x00,  // Channel 0 = +in, channel 1 = -in.
  kLtc2309SelectDiffCh2Ch3 = 0x10,  // Channel 2 = +in, channel 3 = -in.
  kLtc2309SelectDiffCh4Ch5 = 0x20,  // Channel 4 = +in, channel 5 = -in.
  kLtc2309SelectDiffCh6Ch7 = 0x30,  // Channel 6 = +in, channel 7 = -in.
  kLtc2309SelectDiffCh1Ch0 = 0x40,  // Channel 0 = -in, channel 1 = +in.
  kLtc2309SelectDiffCh3Ch2 = 0x50,  // Channel 2 = -in, channel 3 = +in.
  kLtc2309SelectDiffCh5Ch4 = 0x60,  // Channel 4 = -in, channel 5 = +in.
  kLtc2309SelectDiffCh7Ch6 = 0x70,  // Channel 6 = -in, channel 7 = +in.
  kLtc2309SelectSingleCh0  = 0x80,
  kLtc2309SelectSingleCh2  = 0x90,
  kLtc2309SelectSingleCh4  = 0xA0,
  kLtc2309SelectSingleCh6  = 0xB0,
  kLtc2309SelectSingleCh1  = 0xC0,
  kLtc2309SelectSingleCh3  = 0xD0,
  kLtc2309SelectSingleCh5  = 0xE0,
  kLtc2309SelectSingleCh7  = 0xF0,
} Ltc2309Select;

// Select converter operation mode.
typedef enum {
  kLtc2309Bipolar  = 0x00,  // UNI=0.
  kLtc2309Unipolar = 0x08,  // UNI=1.
} Ltc2309ConversionMode;

// Select power saving mode between conversions.
typedef enum {
  kLtc2309NapMode   = 0x00,  // SLP=0.
  kLtc2309SleepMode = 0x04,  // SLP=1.
} Ltc2309PowerSavingMode;

typedef struct {
  uint8_t addr;
  uint8_t command;
} Ltc2309Config;

uint8_t Ltc2309BuildCommand(Ltc2309Select select, Ltc2309ConversionMode convert,
                            Ltc2309PowerSavingMode sleep);

#endif  // AVIONICS_FIRMWARE_DRIVERS_LTC2309_TYPES_H_

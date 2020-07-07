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

#ifndef AVIONICS_FIRMWARE_DRIVERS_ADS7828_TYPES_H_
#define AVIONICS_FIRMWARE_DRIVERS_ADS7828_TYPES_H_

#include <stdint.h>

// Select channel to convert.
typedef enum {
  kAds7828SelectDiffCh0Ch1 = 0x00,  // Channel 0 = +in, channel 1 = -in.
  kAds7828SelectDiffCh2Ch3 = 0x10,  // Channel 2 = +in, channel 3 = -in.
  kAds7828SelectDiffCh4Ch5 = 0x20,  // Channel 4 = +in, channel 5 = -in.
  kAds7828SelectDiffCh6Ch7 = 0x30,  // Channel 6 = +in, channel 7 = -in.
  kAds7828SelectDiffCh1Ch0 = 0x40,  // Channel 0 = -in, channel 1 = +in.
  kAds7828SelectDiffCh3Ch2 = 0x50,  // Channel 2 = -in, channel 3 = +in.
  kAds7828SelectDiffCh5Ch4 = 0x60,  // Channel 4 = -in, channel 5 = +in.
  kAds7828SelectDiffCh7Ch6 = 0x70,  // Channel 6 = -in, channel 7 = +in.
  kAds7828SelectSingleCh0  = 0x80,
  kAds7828SelectSingleCh2  = 0x90,
  kAds7828SelectSingleCh4  = 0xA0,
  kAds7828SelectSingleCh6  = 0xB0,
  kAds7828SelectSingleCh1  = 0xC0,
  kAds7828SelectSingleCh3  = 0xD0,
  kAds7828SelectSingleCh5  = 0xE0,
  kAds7828SelectSingleCh7  = 0xF0,
} Ads7828Select;

// Select converter power behavior between conversions.
typedef enum {
  kAds7828PowerConverterOff = 0x00,  // PD0=0.
  kAds7828PowerConverterOn  = 0x04,  // PD0=1.
} Ads7828PowerConverter;

// Select reference power behavior between conversions.
typedef enum {
  kAds7828PowerReferenceOff = 0x00,  // PD1=0.
  kAds7828PowerReferenceOn  = 0x08,  // PD1=1.
} Ads7828PowerReference;

typedef struct {
  uint8_t addr;
  uint8_t command;
} Ads7828Config;

uint8_t Ads7828BuildCommand(Ads7828Select select, Ads7828PowerConverter convert,
                            Ads7828PowerReference ref);

#endif  // AVIONICS_FIRMWARE_DRIVERS_ADS7828_TYPES_H_

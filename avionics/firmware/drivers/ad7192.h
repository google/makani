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

#ifndef AVIONICS_FIRMWARE_DRIVERS_AD7192_H_
#define AVIONICS_FIRMWARE_DRIVERS_AD7192_H_

#include <stdbool.h>
#include <stdint.h>

#include "avionics/firmware/util/state_machine.h"

typedef enum {
  kAd7192InputForceSigned = -1,
  kAd7192InputDiff12 = 0,
  kAd7192InputDiff34,
  kAd7192InputTemp,
  kAd7192InputDiff22,
  kAd7192Input1,
  kAd7192Input2,
  kAd7192Input3,
  kAd7192Input4,
  kNumAd7192Inputs
} Ad7192Input;

typedef enum {
  kAd7192ClockForceSigned = -1,
  kAd7192ClockExternalCrystal = 0,
  kAd7192ClockExternalClock,
  kAd7192ClockInternal,
  kAd7192ClockInternalOutput,
} Ad7192Clock;

typedef enum {
  kAd7192RefForceSigned = -1,
  kAd7192RefIn1 = 0,
  kAd7192RefIn2 = 1,
} Ad7192Ref;

typedef enum {
  kAd7192GainForceSigned = -1,
  kAd7192Gain1 = 0,
  kAd7192Gain8 = 3,
  kAd7192Gain16 = 4,
  kAd7192Gain32 = 5,
  kAd7192Gain64 = 6,
  kAd7192Gain128 = 7,
} Ad7192Gain;

typedef struct {
  Ad7192Clock clock;
  bool sinc3;
  bool clock_divide;
  bool single_cycle;
  bool reject_60hz;
  uint16_t filter_rate;
  bool chop;
  Ad7192Ref ref;
  uint8_t input_select;
  bool burn;
  bool ref_detect;
  bool buffer;
  bool unipolar;
  Ad7192Gain gain;
  int8_t cs_pin;
  bool configure_sync;
} Ad7192Config;

typedef struct {
  Ad7192Input input;
  uint32_t value_raw;
  bool valid_parity;
  bool error;
} Ad7192Data;

typedef struct {
  FsmState state;
  const Ad7192Config *config;
  int32_t spi_device;
  bool run_calibration;
  Ad7192Data sample;
  bool new_sample;
  bool write_reg;
} Ad7192;

void Ad7192Init(Ad7192 *device);
void Ad7192Poll(const Ad7192Config *config, Ad7192 *device);
bool Ad7192Ready(const Ad7192 *device);
bool Ad7192Read(Ad7192 *device, const Ad7192Data **data);
bool Ad7192Sync(Ad7192 *device);
void Ad7192RunCalibration(Ad7192 *device);

#endif  // AVIONICS_FIRMWARE_DRIVERS_AD7192_H_

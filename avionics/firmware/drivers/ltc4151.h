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

#ifndef AVIONICS_FIRMWARE_DRIVERS_LTC4151_H_
#define AVIONICS_FIRMWARE_DRIVERS_LTC4151_H_

#include <stdbool.h>
#include <stdint.h>

#include "avionics/firmware/drivers/ltc4151_types.h"

typedef enum {
  kLtc4151StateInit,
  kLtc4151StateSetConfig,
  kLtc4151StateReadConfig,
  kLtc4151StateReadBus,
  kLtc4151StateReadShunt,
  kLtc4151StateIdle
} Ltc4151State;

typedef struct {
  Ltc4151State state;
  bool error;
  bool first_entry;
  bool flag_set_config;  // TODO: Nix, add setconfig to state loop.
  Ltc4151OutputRaw output;
} Ltc4151;

void Ltc4151Init(Ltc4151 *device);
void Ltc4151SetConfig(Ltc4151 *device);
bool Ltc4151Poll(const Ltc4151Config *config, Ltc4151 *device);

#endif  // AVIONICS_FIRMWARE_DRIVERS_LTC4151_H_

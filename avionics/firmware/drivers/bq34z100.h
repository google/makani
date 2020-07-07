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

#ifndef AVIONICS_FIRMWARE_DRIVERS_BQ34Z100_H_
#define AVIONICS_FIRMWARE_DRIVERS_BQ34Z100_H_

#include <stdbool.h>
#include <stdint.h>

#include "avionics/firmware/drivers/bq34z100_types.h"

typedef enum {
  kBq34z100StateInit,
  kBq34z100StateReadVoltage,
  kBq34z100StateReadCurrent,
  kBq34z100StateReadCapacity,
  kBq34z100StateReadFullCapacity,
  kBq34z100StateReadSoc,
  kBq34z100StateReadTemp,
  kBq34z100StateIdle
} Bq34z100State;

typedef struct {
  Bq34z100State state;
  bool error;
  bool first_entry;
  Bq34z100OutputRaw output;
} Bq34z100;

void Bq34z100Init(Bq34z100 *device);
bool Bq34z100Poll(const Bq34z100Config *config, Bq34z100 *device);

#endif  // AVIONICS_FIRMWARE_DRIVERS_BQ34Z100_H_

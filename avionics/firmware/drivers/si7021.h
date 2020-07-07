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

#ifndef AVIONICS_FIRMWARE_DRIVERS_SI7021_H_
#define AVIONICS_FIRMWARE_DRIVERS_SI7021_H_

#include <stdbool.h>
#include <stdint.h>

#include "avionics/firmware/drivers/si7021_types.h"

typedef enum {
  kSi7021StateInit,
  kSi7021StateFlush,
  kSi7021StateReset,
  kSi7021StateReadUserReg1,
  kSi7021StateWriteUserReg1,
  kSi7021StateReadHeaterControl,
  kSi7021StateWriteHeaterControl,
  kSi7021StateMeasureRelHumidity,
  kSi7021StateReadRelHumidity,
  kSi7021StateReadTemperature,
  kSi7021StateIdle,
} Si7021State;

typedef struct {
  uint8_t user_reg1;
  uint8_t heater_control;
} Si7021Registers;

typedef struct {
  Si7021State state;
  bool first_entry;
  uint32_t timeout;
  bool error;
  Si7021OutputRaw output;
  Si7021Registers config;
  Si7021Registers status;
} Si7021;

void Si7021Init(Si7021 *device);
bool Si7021Poll(const Si7021Config *config, Si7021 *device);

#endif  // AVIONICS_FIRMWARE_DRIVERS_SI7021_H_

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

#ifndef AVIONICS_GROUND_POWER_Q7_LOADBANK_H_
#define AVIONICS_GROUND_POWER_Q7_LOADBANK_H_

#include <modbus.h>
#include <stdbool.h>

#include "avionics/common/avionics_messages.h"
#include "avionics/ground_power/q7/loadbank_types.h"

typedef struct {
  uint8_t id;
  modbus_t *mb;
  bool mb_status;
} Loadbank;

typedef struct {
  int32_t size;
  Loadbank loadbank_array[kNumLoadbanks];
} LoadbankSet;

typedef struct {
  modbus_t *kite_load_mb;
  modbus_t *loadbank_load_mb;
  bool kite_mb_status;
  bool loadbank_mb_status;
  LoadbankStatusMessage status_message;
} SharkMeters;

void LoadbankSetInit(LoadbankSet *loadbanks);
void SharkInit(SharkMeters *shark_meters);
void *LoadbankCommandThread(void *loadbank_arg);
void *LoadbankRelayCalcThread(void *shark_meters_arg);
void *LoadbankDataThread(void *shark_meters_arg);
#endif  // AVIONICS_GROUND_POWER_Q7_LOADBANK_H_

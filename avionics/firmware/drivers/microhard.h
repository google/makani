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

#ifndef AVIONICS_FIRMWARE_DRIVERS_MICROHARD_H_
#define AVIONICS_FIRMWARE_DRIVERS_MICROHARD_H_

#include <stdbool.h>
#include <stdint.h>
#include <sys/types.h>

#include "avionics/firmware/cpu/sci.h"
#include "avionics/firmware/drivers/microhard_params.h"
#include "avionics/firmware/drivers/microhard_types.h"
#include "avionics/firmware/util/timer.h"

typedef enum {
  kMicrohardStateInit,
  kMicrohardStateActivate,
  kMicrohardStateLogin,
  kMicrohardStatePassword,
  kMicrohardStateReady,
  kMicrohardStateConfig,
  kMicrohardStatePollRssi,
} MicrohardState;

typedef struct {
  const MicrohardParams *params;
  MicrohardState next_state;
  bool first_entry;
  Timer timer;
  uint32_t next_poll;

  bool configured;
  ssize_t config_index;

  // TODO: This is sized to fit the login message (95 bytes).
  // If this were converted to a ring buffer and MicrohardResponseEndsWith()
  // converted to not use strcmp, it's size could be reduced.
  char response[128];
  ssize_t response_len;
} MicrohardData;

typedef struct {
  MicrohardData *data;
  const SciDevice *sci;
  int32_t baud_rate;

  void (* const set_status)(MicrohardStatus status);
} Microhard;

void MicrohardInit(const Microhard *microhard, const MicrohardParams *params);
void MicrohardPoll(const Microhard *microhard);

#endif  // AVIONICS_FIRMWARE_DRIVERS_MICROHARD_H_

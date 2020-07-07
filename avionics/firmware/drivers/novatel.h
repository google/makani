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

#ifndef AVIONICS_FIRMWARE_DRIVERS_NOVATEL_H_
#define AVIONICS_FIRMWARE_DRIVERS_NOVATEL_H_

#include <stdbool.h>
#include <stdint.h>

#include "avionics/common/novatel_serial.h"
#include "avionics/firmware/cpu/sci.h"

typedef struct {
  const SciDevice *this_port;
  const SciDevice *other_port;
  const char **init_string;
  int32_t num_init_strings;
} NovAtelDevice;

void NovAtelInit(void);
bool NovAtelPoll(const NovAtelDevice *dev, NovAtelProto *proto);
void NovAtelInsertRtcm(const NovAtelDevice *dev, int32_t length,
                       const uint8_t *data);
bool NovAtelIsReady(void);

#endif  // AVIONICS_FIRMWARE_DRIVERS_NOVATEL_H_

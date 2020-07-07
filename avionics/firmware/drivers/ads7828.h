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

#ifndef AVIONICS_FIRMWARE_DRIVERS_ADS7828_H_
#define AVIONICS_FIRMWARE_DRIVERS_ADS7828_H_

#include <stdbool.h>
#include <stdint.h>

#include "avionics/firmware/drivers/ads7828_types.h"

typedef struct {
  bool waiting_for_reference;
  bool waiting_for_conversion;
  uint8_t last_command;
  uint16_t output;
  uint32_t timeout;
} Ads7828;

void Ads7828Init(Ads7828 *device);
bool Ads7828Poll(const Ads7828Config *config, Ads7828 *device);

#endif  // AVIONICS_FIRMWARE_DRIVERS_ADS7828_H_

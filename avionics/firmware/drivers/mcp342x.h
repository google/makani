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

#ifndef AVIONICS_FIRMWARE_DRIVERS_MCP342X_H_
#define AVIONICS_FIRMWARE_DRIVERS_MCP342X_H_

#include <stdbool.h>
#include <stdint.h>

#include "avionics/firmware/drivers/mcp342x_types.h"

typedef enum {
  kMcp342xStateInit,
  kMcp342xStateGeneralCallLatch,
  kMcp342xStateSetConfig,
  kMcp342xStateGetConfig,
  kMcp342xStateNotReady,
  kMcp342xStateGetResult,
  kMcp342xStateIdle
} Mcp342xState;

typedef struct {
  uint8_t addr;
  Mcp342xState state;
  bool first_entry;
  bool error;
  uint8_t config;
  int32_t config_tries;
  uint32_t config_index;
  uint8_t status;
  int32_t result;
  uint32_t timeout;
} Mcp342x;

void Mcp342xInit(Mcp342x *device);
void Mcp342xSetConfig(const Mcp342xConfig *config, Mcp342x *device);
bool Mcp342xGetResult(const Mcp342x *device, int32_t *result);
bool Mcp342xPoll(uint8_t addr, const Mcp342xConfig *config, Mcp342x *device);

#endif  // AVIONICS_FIRMWARE_DRIVERS_MCP342X_H_

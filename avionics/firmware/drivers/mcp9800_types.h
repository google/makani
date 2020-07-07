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

#ifndef AVIONICS_FIRMWARE_DRIVERS_MCP9800_TYPES_H_
#define AVIONICS_FIRMWARE_DRIVERS_MCP9800_TYPES_H_

#include <stdint.h>

typedef enum {
  kMcp9800Resolution0C5,    // 33 samples/sec (typical).
  kMcp9800Resolution0C25,   // 17 samples/sec (typical).
  kMcp9800Resolution0C125,  // 8 samples/sec (typical).
  kMcp9800Resolution0C0625  // 4 samples/sec (typical).
} Mcp9800Resolution;

typedef struct {
  uint8_t addr;
  uint8_t binary_config;
} Mcp9800Config;

uint8_t Mcp9800BuildConfig(Mcp9800Resolution res);
float Mcp9800TempRawToC(uint16_t ta_raw);

#endif  // AVIONICS_FIRMWARE_DRIVERS_MCP9800_TYPES_H_

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

#ifndef AVIONICS_FIRMWARE_DRIVERS_MCP342X_TYPES_H_
#define AVIONICS_FIRMWARE_DRIVERS_MCP342X_TYPES_H_

#include <stdint.h>

#define MCP3426_CHANNELS 2
#define MCP3427_CHANNELS 2
#define MCP3428_CHANNELS 4

typedef enum {
  kMcp342xChannel1,
  kMcp342xChannel2,
  kMcp342xChannel3,  // MCP3428 only.
  kMcp342xChannel4   // MCP3428 only.
} Mcp342xChannel;

typedef enum {
  kMcp342xPolarityPositive,
  kMcp342xPolarityNegative
} Mcp342xPolarity;

typedef enum {
  kMcp342xModeSingle,
  kMcp342xModeContinuous
} Mcp342xMode;

typedef enum {
  kMcp342xSps240,
  kMcp342xSps60,
  kMcp342xSps15
} Mcp342xSps;

typedef enum {
  kMcp342xGain1X,
  kMcp342xGain2X,
  kMcp342xGain4X,
  kMcp342xGain8X
} Mcp342xGain;

typedef struct {
  Mcp342xChannel channel;
  Mcp342xPolarity polarity;
  Mcp342xMode mode;
  Mcp342xSps sps;
  Mcp342xGain gain;
} Mcp342xConfig;

uint8_t Mcp342xBuildConfig(const Mcp342xConfig *config);

#endif  // AVIONICS_FIRMWARE_DRIVERS_MCP342X_TYPES_H_


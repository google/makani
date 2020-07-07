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

#ifndef AVIONICS_FIRMWARE_MONITORS_MCP342X_H_
#define AVIONICS_FIRMWARE_MONITORS_MCP342X_H_

#include <stdbool.h>
#include <stdint.h>

#include "avionics/firmware/drivers/mcp342x.h"
#include "avionics/firmware/drivers/mcp342x_types.h"
#include "avionics/firmware/monitors/mcp342x_types.h"

// Mcp342xMonitorPoll() calls a function with this prototype for each MCP342x
// monitor update.
typedef void (* const Mcp342xOutputFunction)(int32_t monitor, int32_t value,
                                             bool valid);

// Mcp342xMonitorInit() initializes an array of MCP342x devices.
void Mcp342xMonitorInit(int32_t num_devices, Mcp342x *devices);

// Mcp342xMonitorPoll() polls all MCP342x devices in the given configuration,
// and calls 'output_function' for each update. The 'device_index' and
// config_indices are input/output arguments used by Mcp342xMonitorPoll() to
// track the internal state. The caller should initialize 'device_index' and
// all elements in 'config_indices' to zero before the first call to
// Mcp342xMonitorPoll() then not set them again. Their value should not be used
// outside of calls to Mcp342xMonitorPoll(). This function returns true when
// the I2C bus returns to idle and it is safe to call another monitoring
// function.
bool Mcp342xMonitorPoll(const Mcp342xMonitors *config,
                        Mcp342xOutputFunction output_function,
                        uint32_t *device_index, uint32_t *config_indices,
                        Mcp342x *devices);

#endif  // AVIONICS_FIRMWARE_MONITORS_MCP342X_H_

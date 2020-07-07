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

#ifndef AVIONICS_FIRMWARE_MONITORS_MCP9800_H_
#define AVIONICS_FIRMWARE_MONITORS_MCP9800_H_

#include <stdbool.h>
#include <stdint.h>

#include "avionics/firmware/drivers/mcp9800.h"
#include "avionics/firmware/monitors/mcp9800_types.h"

// Mcp9800MonitorPoll() calls a function with this prototype for each MCP9800
// device update. The 'device' argument corresponds to the Mcp9800Monitor.device
// member field, not the 'device_index' passed to Mcp9800MonitorPoll().
typedef void (* const Mcp9800OutputFunction)(int32_t device, float raw);

// Mcp9800MonitorInit() initializes an array of MCP9800 devices.
void Mcp9800MonitorInit(int32_t num_devices, Mcp9800 *devices);

// Mcp9800MonitorPoll() polls all MCP9800 devices in the given configuration,
// and calls 'output_function' for each update. The 'device_index' input/output
// argument is used by Mcp9800MonitorPoll() to track the internal state. The
// caller should initialize 'device_index' to zero before the first call to
// Mcp9800MonitorPoll() then not set it again. It's value should not be used
// outside of calls to Mcp9800MonitorPoll(). This function returns true when
// the I2C bus returns to idle and it is safe to call another monitoring
// function.
bool Mcp9800MonitorPoll(const Mcp9800Monitors *config,
                        Mcp9800OutputFunction output_function,
                        uint32_t *device_index, Mcp9800 *devices);

#endif  // AVIONICS_FIRMWARE_MONITORS_MCP9800_H_

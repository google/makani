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

#ifndef AVIONICS_FIRMWARE_MONITORS_ADS7828_H_
#define AVIONICS_FIRMWARE_MONITORS_ADS7828_H_

#include <stdbool.h>
#include <stdint.h>

#include "avionics/firmware/drivers/ads7828.h"
#include "avionics/firmware/drivers/ads7828_types.h"
#include "avionics/firmware/monitors/ads7828_types.h"

// Ads7828MonitorPoll() calls a function with this prototype for each ADS7828
// device input update. The 'input' argument corresponds to the
// Ads7828Monitor.input member field, not the 'device_index' or 'config_index'
// argument passed to Ads7828MonitorPoll(). See the Ads7828MonitorFlag
// enumeration for possible flag values.
typedef void (* const Ads7828OutputFunction)(int32_t input, float data,
                                             uint32_t flags);

// Ads7828MonitorInit() initializes an array of ADS7828 devices.
void Ads7828MonitorInit(int32_t num_devices, Ads7828 *devices);

// Ads7828MonitorPoll() polls all ADS7828 devices in the given configuration,
// and calls 'output_function' for each update. The 'device_index' input/output
// argument is used by Ads7828MonitorPoll() to track the internal state. The
// caller should initialize 'device_index' to zero before the first call to
// Ads7828MonitorPoll() then not set it again. It's value should not be used
// outside of calls to Ads7828MonitorPoll(). This function returns true when
// the I2C bus returns to idle and it is safe to call another monitoring
// function.
bool Ads7828MonitorPoll(const Ads7828Monitors *monitors,
                        Ads7828OutputFunction output_function,
                        uint32_t *device_index, uint32_t *config_indices,
                        Ads7828 *devices);

#endif  // AVIONICS_FIRMWARE_MONITORS_ADS7828_H_

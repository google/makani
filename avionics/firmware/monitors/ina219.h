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

#ifndef AVIONICS_FIRMWARE_MONITORS_INA219_H_
#define AVIONICS_FIRMWARE_MONITORS_INA219_H_

#include <stdbool.h>
#include <stdint.h>

#include "avionics/firmware/drivers/ina219.h"
#include "avionics/firmware/drivers/ina219_types.h"
#include "avionics/firmware/monitors/ina219_types.h"

// Ina219MonitorPoll() calls a function with this prototype for each INA219
// device update. The 'device' argument corresponds to the Ina219Monitor.device
// member field, not the 'device_index' passed to Ina219MonitorPoll(). See
// the Ina219MonitorFlag enumeration for possible flag values.
typedef void (* const Ina219OutputFunction)(int32_t device,
                                            const Ina219OutputData *data,
                                            uint32_t flags);

// Ina219MonitorInit() initializes an array of INA219 devices.
void Ina219MonitorInit(int32_t num_devices, Ina219 *devices);

// Ina219MonitorPoll() polls all INA219 devices in the given configuration,
// and calls 'output_function' for each update. The 'device_index' input/output
// argument is used by Ina219MonitorPoll() to track the internal state. The
// caller should initialize 'device_index' to zero before the first call to
// Ina219MonitorPoll() then not set it again. It's value should not be used
// outside of calls to Ina219MonitorPoll(). This function returns true when
// the I2C bus returns to idle and it is safe to call another monitoring
// function.
bool Ina219MonitorPoll(const Ina219Monitors *config,
                       Ina219OutputFunction output_function,
                       uint32_t *device_index, Ina219 *devices);

#endif  // AVIONICS_FIRMWARE_MONITORS_INA219_H_

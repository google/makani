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

#ifndef AVIONICS_FIRMWARE_MONITORS_LTC2309_H_
#define AVIONICS_FIRMWARE_MONITORS_LTC2309_H_

#include <stdbool.h>
#include <stdint.h>

#include "avionics/firmware/drivers/ltc2309.h"
#include "avionics/firmware/drivers/ltc2309_types.h"
#include "avionics/firmware/monitors/ltc2309_types.h"

// Ltc2309MonitorPoll() calls a function with this prototype for each LTC2309
// device input update. The 'input' argument corresponds to the
// Ltc2309Monitor.input member field, not the 'device_index' or 'config_index'
// argument passed to Ltc2309MonitorPoll(). See the Ltc2309MonitorFlag
// enumeration for possible flag values.
typedef void (* const Ltc2309OutputFunction)(int32_t input, float data,
                                             uint32_t flags);

// Ltc2309MonitorInit() initializes an array of LTC2309 devices.
void Ltc2309MonitorInit(int32_t num_devices, Ltc2309 *devices);

// Ltc2309MonitorPoll() polls all LTC2309 devices in the given configuration,
// and calls 'output_function' for each update. The 'device_index' input/output
// argument is used by Ltc2309MonitorPoll() to track the internal state. The
// caller should initialize 'device_index' to zero before the first call to
// Ltc2309MonitorPoll() then not set it again. It's value should not be used
// outside of calls to Ltc2309MonitorPoll(). This function returns true when
// the I2C bus returns to idle and it is safe to call another monitoring
// function.
bool Ltc2309MonitorPoll(const Ltc2309Monitors *monitors,
                        Ltc2309OutputFunction output_function,
                        uint32_t *device_index, uint32_t *config_indices,
                        Ltc2309 *devices);

#endif  // AVIONICS_FIRMWARE_MONITORS_LTC2309_H_

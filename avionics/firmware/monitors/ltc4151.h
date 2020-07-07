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

#ifndef AVIONICS_FIRMWARE_MONITORS_LTC4151_H_
#define AVIONICS_FIRMWARE_MONITORS_LTC4151_H_

#include <stdbool.h>
#include <stdint.h>

#include "avionics/firmware/drivers/ltc4151.h"
#include "avionics/firmware/drivers/ltc4151_types.h"
#include "avionics/firmware/monitors/ltc4151_types.h"

// LTC4151 ADC/voltage and current monitor.

typedef void (* const Ltc4151OutputFunction)(int32_t device,
                                             const Ltc4151OutputData *data,
                                             uint32_t flags);

void Ltc4151MonitorInit(int32_t num_devices, Ltc4151 *devices);

bool Ltc4151MonitorPoll(const Ltc4151Monitors *config,
                        Ltc4151OutputFunction output_function,
                        uint32_t *device_index, Ltc4151 *devices);

#endif  // AVIONICS_FIRMWARE_MONITORS_LTC4151_H_

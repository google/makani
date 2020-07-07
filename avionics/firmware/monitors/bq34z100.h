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

#ifndef AVIONICS_FIRMWARE_MONITORS_BQ34Z100_H_
#define AVIONICS_FIRMWARE_MONITORS_BQ34Z100_H_

#include <stdbool.h>
#include <stdint.h>

#include "avionics/firmware/drivers/bq34z100.h"
#include "avionics/firmware/drivers/bq34z100_types.h"
#include "avionics/firmware/monitors/bq34z100_types.h"

// BQ34Z100 pack capacity, voltage and current monitor.

typedef void (* const Bq34z100OutputFunction)(int32_t device,
                                              const Bq34z100OutputData *data,
                                              uint32_t flags);

void Bq34z100MonitorInit(int32_t num_devices, Bq34z100 *devices);

bool Bq34z100MonitorPoll(const Bq34z100Monitors *config,
                         Bq34z100OutputFunction output_function,
                         uint32_t *device_index, Bq34z100 *devices);

#endif  // AVIONICS_FIRMWARE_MONITORS_BQ34Z100_H_

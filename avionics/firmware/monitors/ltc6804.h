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

#ifndef AVIONICS_FIRMWARE_MONITORS_LTC6804_H_
#define AVIONICS_FIRMWARE_MONITORS_LTC6804_H_

#include "avionics/firmware/drivers/ltc6804.h"
#include "avionics/firmware/drivers/ltc6804_types.h"
#include "avionics/firmware/monitors/ltc6804_types.h"

typedef void (* const Ltc6804OutputFunction)(const Ltc6804OutputData *data);
void Ltc6804MonitorInit(const Ltc6804Monitors *config, int32_t index_length,
                        Ltc6804CellIndex *cell_index, Ltc6804 *device);
bool Ltc6804MonitorPoll(const Ltc6804Monitors *config,
                        bool reduced_ltc6804_read_rate,
                        Ltc6804CellIndex *cell_index,
                        Ltc6804OutputFunction output_function,
                        Ltc6804 *device);

#endif  // AVIONICS_FIRMWARE_MONITORS_LTC6804_H_

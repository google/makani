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

#ifndef AVIONICS_FIRMWARE_MONITORS_ANALOG_H_
#define AVIONICS_FIRMWARE_MONITORS_ANALOG_H_

#include <stdbool.h>
#include <stdint.h>

#include "avionics/firmware/monitors/analog_types.h"

// AnalogMonitorPoll() calls a function with this prototype for each analog
// input update. The 'value' argument represents the analog value in counts
// for voltage measurements (multiply by config->volts_per_count to scale to
// a voltage), or the logic level (0 or 1) for digital measurements.
typedef void (* const AnalogOutputFunction)(const AnalogMonitor *config,
                                            float value, uint32_t flags);

// AnalogMonitorInit() initializes the ADC converter.
void AnalogMonitorInit(void);

// AnalogMonitorPoll() polls the analog channels in config->channel_mask, and
// then calls 'output_function' for each analog input update.
bool AnalogMonitorPoll(const AnalogMonitors *config,
                       AnalogOutputFunction output_function);

#endif  // AVIONICS_FIRMWARE_MONITORS_ANALOG_H_

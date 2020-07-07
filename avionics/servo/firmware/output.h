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

#ifndef AVIONICS_SERVO_FIRMWARE_OUTPUT_H_
#define AVIONICS_SERVO_FIRMWARE_OUTPUT_H_

#include <stdint.h>

#include "avionics/firmware/monitors/aio_types.h"
#include "avionics/firmware/monitors/servo_types.h"
#include "avionics/servo/firmware/control.h"
#include "avionics/servo/firmware/input.h"

void ServoOutputInit(void);

// Monitors.
AioModuleMonitorData *ServoOutputGetAioModuleMonitors(void);
ServoMonitorData *ServoOutputGetServoMonitors(void);

// Non-control state.
void ServoOutputErrorLog(int32_t data_length, const uint32_t *data);

// Control state.
void ServoOutputSendStatusMessage(ServoState *state);

#endif  // AVIONICS_SERVO_FIRMWARE_OUTPUT_H_

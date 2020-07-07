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

#ifndef AVIONICS_JOYSTICK_FIRMWARE_OUTPUT_H_
#define AVIONICS_JOYSTICK_FIRMWARE_OUTPUT_H_

#include <stdbool.h>
#include <stdint.h>

#include "avionics/firmware/drivers/microhard_types.h"
#include "avionics/firmware/monitors/aio.h"
#include "avionics/firmware/monitors/ground_io.h"
#include "avionics/firmware/monitors/joystick.h"
#include "system/labels.h"

void JoystickOutputInit(void);
AioModuleMonitorData *JoystickOutputGetAioModuleMonitors(void);
JoystickMonitorData *JoystickOutputGetJoystickMonitors(void);
GroundIoMonitorData *JoystickOutputGetGroundIoMonitors(void);
void JoystickOutputPresence(bool present);
void JoystickOutputRoll(float value);
void JoystickOutputPitch(float value);
void JoystickOutputYaw(float value);
void JoystickOutputThrottle(float value);
void JoystickOutputTriSwitch(JoystickSwitchPositionLabel value);
void JoystickOutputMomentarySwitch(JoystickSwitchPositionLabel value);
void JoystickOutputSetTetherReleaseInterlockCode(uint32_t code);
void JoystickOutputSetScuttleCode(uint32_t code);
void JoystickOutputArmedSwitch(bool armed);
void JoystickOutputMicrohardStatus(MicrohardStatus status);
void JoystickOutputSendStatusMessage(void);
void JoystickOutputSendMonitorStatusMessage(void);

#endif  // AVIONICS_JOYSTICK_FIRMWARE_OUTPUT_H_

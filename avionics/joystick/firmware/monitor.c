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

#include "avionics/joystick/firmware/monitor.h"

#include <assert.h>
#include <stdbool.h>

#include "avionics/firmware/identity/identity.h"
#include "avionics/firmware/monitors/aio.h"
#include "avionics/firmware/monitors/ground_io.h"
#include "avionics/firmware/monitors/joystick.h"
#include "avionics/firmware/serial/board_serial.h"
#include "avionics/firmware/serial/carrier_serial.h"
#include "avionics/joystick/firmware/output.h"
#include "avionics/firmware/serial/joystick_serial_params.h"

// This file defines the joystick hardware monitoring configuration.

static bool PollCarrier(void) {
  switch (GetCarrierHardwareType()) {
    case kCarrierHardwareTypeGroundIo:
      return GroundIoMonitorPoll(GetCarrierHardwareRevision(),
                                 JoystickOutputGetGroundIoMonitors());
    case kCarrierHardwareTypeJoystick:
      return JoystickMonitorPoll(GetCarrierHardwareRevision(),
                                 JoystickOutputGetJoystickMonitors());
    default:
      assert(false);
      return false;
  }
}

void JoystickMonInit(void) {
  AioMonitorInit();
  switch (GetCarrierHardwareType()) {
    case kCarrierHardwareTypeGroundIo:
      GroundIoMonitorInit();
      break;
    case kCarrierHardwareTypeJoystick:
      JoystickMonitorInit();
      break;
    default:
      assert(false);
      break;
  }
}

void JoystickMonPoll(void) {
  assert(BootConfigGetHardwareType() == kHardwareTypeAio);
  AioMonitorPollStack(GetBoardHardwareRevision(),
                      JoystickOutputGetAioModuleMonitors(),
                      PollCarrier);
}

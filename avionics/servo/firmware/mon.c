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

#include "avionics/servo/firmware/mon.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

#include "avionics/firmware/identity/identity.h"
#include "avionics/firmware/monitors/aio.h"
#include "avionics/firmware/monitors/servo.h"
#include "avionics/firmware/serial/board_serial.h"
#include "avionics/servo/firmware/output.h"
#include "avionics/firmware/serial/aio_serial_params.h"
#include "avionics/firmware/serial/carrier_serial.h"
#include "avionics/firmware/serial/servo_serial_params.h"
#include "common/macros.h"

static bool PollServo(void) {
  return ServoMonitorPoll(GetCarrierHardwareRevision(),
                          ServoOutputGetServoMonitors());
}

void ServoMonInit(void) {
  AioMonitorInit();
  ServoMonitorInit();
}

void ServoMonPoll(void) {
  assert(BootConfigGetHardwareType() == kHardwareTypeAio);
  AioMonitorPollStack(GetBoardHardwareRevision(),
                      ServoOutputGetAioModuleMonitors(),
                      PollServo);
}

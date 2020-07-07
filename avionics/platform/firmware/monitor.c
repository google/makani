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

#include "avionics/platform/firmware/monitor.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

#include "avionics/firmware/identity/identity.h"
#include "avionics/firmware/monitors/aio.h"
#include "avionics/firmware/monitors/ground_io.h"
#include "avionics/firmware/serial/aio_serial_params.h"
#include "avionics/firmware/serial/board_serial.h"
#include "avionics/firmware/serial/ground_io_serial_params.h"
#include "avionics/platform/firmware/output.h"
#include "common/macros.h"

static bool PollGroundIo(void) {
  return GroundIoMonitorPoll(kGroundIoHardwareRevAa,
                             PlatformOutputGetGroundIoMonitors());
}

void PlatformMonInit(void) {
  AioMonitorInit();
  GroundIoMonitorInit();
}

void PlatformMonPoll(void) {
  if (BootConfigGetHardwareType() == kHardwareTypeAio) {
    AioMonitorPollStack(GetBoardHardwareRevision(),
                        PlatformOutputGetAioModuleMonitors(),
                        PollGroundIo);
  }
}

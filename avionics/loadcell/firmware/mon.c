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

#include "avionics/loadcell/firmware/mon.h"

#include <assert.h>
#include <stdbool.h>

#include "avionics/firmware/identity/identity.h"
#include "avionics/firmware/monitors/aio.h"
#include "avionics/firmware/monitors/loadcell.h"
#include "avionics/firmware/serial/aio_serial_params.h"
#include "avionics/firmware/serial/board_serial.h"
#include "avionics/firmware/serial/carrier_serial.h"
#include "avionics/firmware/serial/loadcell_serial_params.h"
#include "avionics/loadcell/firmware/output.h"

static bool PollLoadcell(void) {
  return LoadcellMonitorPoll(GetCarrierHardwareRevision(),
                             LoadcellOutputGetLoadcellMonitors());
}

void LoadcellMonInit(void) {
  AioMonitorInit();
  LoadcellMonitorInit();
}

void LoadcellMonPoll(void) {
  assert(BootConfigGetHardwareType() == kHardwareTypeAio);
  AioMonitorPollStack(GetBoardHardwareRevision(),
                      LoadcellOutputGetAioModuleMonitors(),
                      PollLoadcell);
}

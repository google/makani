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

#include "avionics/batt/firmware/mon.h"

#include <assert.h>
#include <stdbool.h>

#include "avionics/batt/firmware/output.h"
#include "avionics/common/faults.h"
#include "avionics/firmware/identity/identity.h"
#include "avionics/firmware/monitors/aio.h"
#include "avionics/firmware/monitors/batt.h"
#include "avionics/firmware/monitors/batt_types.h"
#include "avionics/firmware/serial/board_serial.h"
#include "avionics/firmware/serial/carrier_serial.h"

static bool PollBatt(void) {
  BattMonitorData *mon = BattOutputGetBattMonitors();
  bool reduced_ltc6804_read_rate = false;
  /* If batteries are disconnected from bus, read cell voltages
     at slower rate to reduce quiescent power draw on the pack. */
  if (!CheckStatus(&mon->flags, kBattMonitorStatusConnected)) {
    reduced_ltc6804_read_rate = true;
  }
  return BattMonitorPoll(GetCarrierHardwareRevision(),
                         reduced_ltc6804_read_rate,
                         mon);
}

void BattMonInit(void) {
  AioMonitorInit();
  BattMonitorInit(GetCarrierHardwareRevision());
}

void BattMonPoll(void) {
  assert(BootConfigGetHardwareType() == kHardwareTypeAio);
  AioMonitorPollStack(GetBoardHardwareRevision(),
                      BattOutputGetAioModuleMonitors(),
                      PollBatt);
}

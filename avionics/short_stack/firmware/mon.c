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

#include "avionics/short_stack/firmware/mon.h"

#include <assert.h>
#include <stdbool.h>

#include "avionics/short_stack/firmware/output.h"
#include "avionics/firmware/identity/identity.h"
#include "avionics/firmware/monitors/aio.h"
#include "avionics/firmware/monitors/short_stack.h"
#include "avionics/firmware/serial/board_serial.h"
#include "avionics/firmware/serial/carrier_serial.h"

static bool PollShortStack(void) {
  return ShortStackMonitorPoll(GetCarrierHardwareRevision(),
                               ShortStackOutputGetShortStackMonitors());
}

void ShortStackMonInit(void) {
  AioMonitorInit();
  ShortStackMonitorInit();
}

void ShortStackMonPoll(void) {
  assert(BootConfigGetHardwareType() == kHardwareTypeAio);
  AioMonitorPollStack(GetBoardHardwareRevision(),
                      ShortStackOutputGetAioModuleMonitors(),
                      PollShortStack);
}

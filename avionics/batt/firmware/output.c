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

#include "avionics/batt/firmware/output.h"

#include <string.h>

#include "avionics/common/avionics_messages.h"
#include "avionics/firmware/network/net_send.h"

static BatteryStatusMessage g_status;

void BattOutputInit(void) {
  memset(&g_status, 0, sizeof(g_status));
}

AioModuleMonitorData *BattOutputGetAioModuleMonitors(void) {
  return &g_status.aio_mon;
}

BattMonitorData *BattOutputGetBattMonitors(void) {
  return &g_status.batt_mon;
}

void BattOutputSendStatusMessage(void) {
  NetSendAioBatteryStatusMessage(&g_status);
}

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

#include "avionics/cs/firmware/output.h"

#include <assert.h>
#include <stdint.h>
#include <string.h>

#include "avionics/common/avionics_messages.h"
#include "avionics/firmware/monitors/cs_types.h"
#include "avionics/firmware/network/net_send.h"
#include "common/macros.h"

static CoreSwitchStatusMessage g_status;

void CsOutputInit(void) {
  memset(&g_status, 0, sizeof(g_status));
}

CsMonitorData *CsOutputGetCsMonitors(void) {
  return &g_status.cs_mon;
}

void CsOutputSendStatusMessage(void) {
  NetSendAioCoreSwitchStatusMessage(&g_status);
}

void CsOutputDisabledPortMask(uint32_t disabled_port_mask) {
  g_status.disabled_port_mask = disabled_port_mask;
}

void CsOutputMicrohardStatus(MicrohardStatus status) {
  g_status.microhard_status = status;
}

const MicrohardStatus *CsOutputGetMicrohardStatus(void) {
  return &g_status.microhard_status;
}

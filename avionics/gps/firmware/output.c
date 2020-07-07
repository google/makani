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

#include "avionics/gps/firmware/output.h"

#include <string.h>

#include "avionics/firmware/monitors/aio_types.h"
#include "avionics/firmware/monitors/fc_types.h"
#include "avionics/firmware/network/net_mon.h"
#include "avionics/firmware/network/net_send.h"

static GpsStatusMessage g_status;

void GpsOutputInit(void) {
  memset(&g_status, 0, sizeof(g_status));
}

AioModuleMonitorData *GpsOutputGetAioModuleMonitors(void) {
  return &g_status.aio_mon;
}

FcMonitorData *GpsOutputGetFcMonitors(void) {
  return &g_status.fc_mon;
}

void GpsOutputSendStatus(void) {
  NetSendAioGpsStatusMessage(&g_status);
}

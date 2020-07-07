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

#include "avionics/recorder/firmware/output.h"

#include <string.h>

#include "avionics/common/avionics_messages.h"
#include "avionics/firmware/network/net_send.h"

// Status message.
static RecorderStatusMessage g_sensor;

void RecorderOutputInit(void) {
  // Status message.
  memset(&g_sensor, 0, sizeof(g_sensor));
}

AioModuleMonitorData *RecorderOutputGetAioModuleMonitors(void) {
  return &g_sensor.aio_mon;
}

RecorderMonitorData *RecorderOutputGetRecorderMonitors(void) {
  return &g_sensor.recorder_mon;
}

void RecorderOutputSendStatusMessage(void) {
  NetSendAioRecorderStatusMessage(&g_sensor);
}

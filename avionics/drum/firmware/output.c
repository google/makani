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

#include "avionics/drum/firmware/output.h"

#include <stdbool.h>
#include <string.h>

#include "avionics/common/avionics_messages.h"
#include "avionics/drum/firmware/encoders.h"
#include "avionics/firmware/identity/identity.h"
#include "avionics/firmware/monitors/aio.h"
#include "avionics/firmware/monitors/ground_io.h"
#include "avionics/firmware/network/net_send.h"
#include "avionics/firmware/serial/aio_serial_params.h"
#include "avionics/firmware/serial/ground_io_serial_params.h"
#include "avionics/firmware/serial/board_serial.h"

static DrumSensorsMessage g_status;
static DrumSensorsMonitorMessage g_mon_status;

static bool PollGroundIo(void) {
  return GroundIoMonitorPoll(kGroundIoHardwareRevAa,
                             &g_mon_status.ground_io_mon);
}

void DrumOutputInit(void) {
  memset(&g_status, 0, sizeof(g_status));
  EncodersInit();
  AioMonitorInit();
  GroundIoMonitorInit();
}

void DrumOutputPoll(void) {
  if (BootConfigGetHardwareType() == kHardwareTypeAio) {
    AioMonitorPollStack(GetBoardHardwareRevision(), &g_mon_status.aio_mon,
                        PollGroundIo);
  }
}

void DrumOutputSend(void) {
  EncodersRead(&g_status.encoders);
  NetSendAioDrumSensorsMessage(&g_status);
}

void DrumMonitorOutputSend(void) {
  NetSendAioDrumSensorsMonitorMessage(&g_mon_status);
}

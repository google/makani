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

#include "avionics/short_stack/firmware/output.h"

#include <stdint.h>
#include <string.h>

#include "avionics/common/avionics_messages.h"
#include "avionics/firmware/monitors/short_stack_analog_types.h"
#include "avionics/firmware/network/net_send.h"
#include "avionics/network/aio_labels.h"

static ShortStackStatusMessage g_status;

void ShortStackOutputInit(void) {
  memset(&g_status, 0, sizeof(g_status));
}

AioModuleMonitorData *ShortStackOutputGetAioModuleMonitors(void) {
  return &g_status.aio_mon;
}

ShortStackMonitorData *ShortStackOutputGetShortStackMonitors(void) {
  return &g_status.short_stack_mon;
}

// Short stack sends status and voltages to the motors for fault operation.
void ShortStackOutputSendStackingMessage(void) {
  ShortStackStackingMessage message;
  message.firing_status = g_status.short_stack_mon.flags.status;
  // If number of motors changes, short_stack_analog.py needs to be modified
  // along with the AIO ADC measurement circuit.
  assert(kNumMotors / 2 == 4);
  for (int32_t i = 0; i < kNumMotors / 2; ++i) {
    message.motor_voltage[i] = g_status.short_stack_mon.analog_data[
        kShortStackAnalogVoltageBlock0 + i];
  }
  NetSendAioShortStackStackingMessage(&message);
}

void ShortStackOutputSendStatusMessage(void) {
  NetSendAioShortStackStatusMessage(&g_status);
}

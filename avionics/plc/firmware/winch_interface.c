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

#include "avionics/plc/firmware/winch_interface.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "avionics/common/cvt_avionics_messages.h"
#include "avionics/common/network_addresses.h"
#include "avionics/common/network_config.h"
#include "avionics/common/pack_avionics_messages.h"
#include "avionics/common/pack_winch_messages.h"
#include "avionics/common/winch_messages.h"
#include "avionics/firmware/identity/identity.h"
#include "avionics/firmware/network/net.h"
#include "avionics/network/winch_message_type.h"
#include "common/macros.h"

// TODO: Listen to TetherDown, not ControllerCommand messages.

// Hard code on PLC.
COMPILE_ASSERT(kWinchMessageTypePlcWinchCommand == 1,
               kWinchMessageTypeWinchCommandPlc_hard_coded_on_PLC);
COMPILE_ASSERT(kWinchMessageTypePlcWinchStatus == 2,
               kWinchMessageTypeWinchStatusPlc_hard_coded_on_PLC);
COMPILE_ASSERT(kWinchMessageTypePlcWinchSetState == 3,
               kWinchMessageTypeWinchSetStatePlc_hard_coded_on_PLC);

static bool IsPlcWinchStatusMessage(uint16_t dest_port, int32_t length) {
  return dest_port == UDP_PORT_WINCH
      && length == PACK_PLCWINCHSTATUSMESSAGE_SIZE;
}

static void ForwardSetStateMessage(const GroundStationWinchSetStateMessage *in,
                                   uint16_t seq) {
  // Input from AIO network (CVT).
  // Output to winch PLC.
  const PlcWinchSetStateMessage out = {
    .sequence = seq,
    .state_command = in->state_command,
    .arming_signal = in->arming_signal
  };

  uint8_t data[PACK_PLCWINCHSETSTATEMESSAGE_SIZE];
  PackPlcWinchSetStateMessage(&out, 1, data);
  NetSendUdp(
      WinchMessageTypeToIpAddress(kWinchMessageTypePlcWinchSetState),
      WinchMessageTypeToEthernetAddress(kWinchMessageTypePlcWinchSetState),
      UDP_PORT_WINCH, UDP_PORT_WINCH, data, ARRAYSIZE(data), NULL);
}

static void ForwardCommandMessage(uint8_t source,
                                  const ControllerCommandMessage *in,
                                  uint16_t seq) {
  // Input from AIO network (CVT).
  // Output to winch PLC.
  PlcWinchCommandMessage out = {
    .source = source,
    .sequence = seq,
    .velocity = in->winch_velocity
  };
  uint8_t data[PACK_PLCWINCHCOMMANDMESSAGE_SIZE];
  PackPlcWinchCommandMessage(&out, 1, data);
  NetSendUdp(
      WinchMessageTypeToIpAddress(kWinchMessageTypePlcWinchCommand),
      WinchMessageTypeToEthernetAddress(kWinchMessageTypePlcWinchCommand),
      UDP_PORT_WINCH, UDP_PORT_WINCH, data, ARRAYSIZE(data), NULL);
}

static void ForwardStatusMessage(const PlcWinchStatusMessage *in) {
  // Output to AIO network.
  GroundStationWinchStatusMessage out = {
    .plc = *in
  };
  NetSendSpoofedAio(
      kAioNodePlcTophat, kMessageTypeGroundStationWinchStatus, in->sequence,
      (PackAioMessageFunction)PackGroundStationWinchStatusMessage, &out);
}

static void QueryController(AioNode controller, uint8_t source) {
  ControllerCommandMessage message;
  uint16_t sequence;
  if (CvtGetControllerCommandMessage(controller, &message, &sequence, NULL)) {
    ForwardCommandMessage(source, &message, sequence);
  }
}

static void QueryOperator(AioNode operator) {
  uint16_t sequence;
  GroundStationWinchSetStateMessage message;
  if (CvtGetGroundStationWinchSetStateMessage(operator, &message, &sequence,
                                              NULL)) {
    ForwardSetStateMessage(&message, sequence);
  }
}

void WinchPollCvt(void) {
  QueryController(kAioNodeControllerA, 0);
  QueryController(kAioNodeControllerB, 1);
  QueryController(kAioNodeControllerC, 2);
  QueryOperator(kAioNodeOperator);
}

bool HandleWinchMessage(uint16_t dest_port, int32_t length,
                        const uint8_t *data) {
  if (IsPlcWinchStatusMessage(dest_port, length)) {
    static PlcWinchStatusMessage in;
    UnpackPlcWinchStatusMessage(data, 1, &in);
    ForwardStatusMessage(&in);
    return true;
  }
  return false;
}

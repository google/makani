// Copyright 2020 Makani Technologies LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "gs/state_command/winch_state.h"

#include <stdint.h>
#include <string.h>

#include "avionics/common/cvt_avionics_messages.h"
#include "avionics/common/pack_avionics_messages.h"
#include "avionics/common/safety_codes.h"
#include "avionics/linux/aio.h"
#include "avionics/network/aio_labels.h"
#include "gs/state_command/state_command.h"

StateCommandBase::LabelsMap WinchStateCommand::GetLabels(void) const {
  LabelsMap labels;
  for (int32_t i = 0; i < kNumPlcTophats; ++i) {
    labels[i] = PlcTophatLabelToString(static_cast<PlcTophatLabel>(i));
  }
  return labels;
}

bool WinchStateCommand::GetActuatorState(int32_t index, ActuatorState *state) {
  PlcTophatLabel label = static_cast<PlcTophatLabel>(index);
  AioNode node = PlcTophatLabelToPlcTophatAioNode(label);
  GroundStationWinchStatusMessage status;
  if (CvtGetGroundStationWinchStatusMessage(node, &status, nullptr, nullptr)) {
    *state = static_cast<ActuatorState>(status.plc.state);
    return true;
  }
  return false;
}

void WinchStateCommand::SendActuatorStateCommand(ActuatorStateCommand command) {
  GroundStationWinchSetStateMessage m;
  memset(&m, 0, sizeof(m));
  m.state_command = command;
  if (command == kActuatorStateCommandArm) {
    m.arming_signal = WINCH_ARMING_SIGNAL;
  } else if (command == kActuatorStateCommandDisarm) {
    m.arming_signal = WINCH_DISARM_SIGNAL;
  }
  AIO_SEND_PACKED(kMessageTypeGroundStationWinchSetState,
                  PackGroundStationWinchSetStateMessage,
                  PACK_GROUNDSTATIONWINCHSETSTATEMESSAGE_SIZE, &m);
}

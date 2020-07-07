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

#include "gs/state_command/servo_state.h"

#include <stdint.h>
#include <string.h>

#include "avionics/common/cvt_avionics_messages.h"
#include "avionics/common/pack_avionics_messages.h"
#include "avionics/common/safety_codes.h"
#include "avionics/common/tether_message.h"
#include "avionics/linux/aio.h"
#include "avionics/network/aio_labels.h"
#include "avionics/network/aio_node.h"
#include "gs/state_command/state_command.h"

ServoStateCommand::ServoStateCommand(void)
    : StateCommandBase("Servo"), tether_down_() {
  TetherDownMergeStateInit(&tether_down_);
}

StateCommandBase::LabelsMap ServoStateCommand::GetLabels(void) const {
  LabelsMap labels;
  for (int32_t i = 0; i < kNumServos; ++i) {
    labels[i] = ServoLabelToString(static_cast<ServoLabel>(i));
  }
  return labels;
}

bool ServoStateCommand::GetActuatorStateFromServoStatus(AioNode node,
                                                        ActuatorState *state) {
  ServoStatusMessage status;
  if (CvtGetServoStatusMessage(node, &status, nullptr, nullptr)) {
    *state = static_cast<ActuatorState>(status.state);
    return true;
  }
  return false;
}

bool ServoStateCommand::GetActuatorStateFromTetherDown(ServoLabel label,
                                                       ActuatorState *state) {
  const TetherDownMessage *m = TetherDownMergeCvtGet(&tether_down_);
  const TetherServoStatus *status = &m->servo_statuses[label];
  if (TetherIsNoUpdateCountValid(status->no_update_count)) {
    *state = static_cast<ActuatorState>(status->state);
    return true;
  }
  return false;
}

bool ServoStateCommand::GetActuatorState(int32_t index, ActuatorState *state) {
  ServoLabel label = static_cast<ServoLabel>(index);
  AioNode node = ServoLabelToServoAioNode(label);

  return GetActuatorStateFromServoStatus(node, state)
      || GetActuatorStateFromTetherDown(label, state);
}

void ServoStateCommand::SendActuatorStateCommand(ActuatorStateCommand command) {
  ServoSetStateMessage m;
  memset(&m, 0, sizeof(m));
  m.state_command = command;
  m.selected_servos = static_cast<uint16_t>(GetPendingTargets());
  if (command == kActuatorStateCommandArm) {
    m.servo_arming_signal = SERVO_ARMING_SIGNAL;
  } else if (command == kActuatorStateCommandDisarm) {
    m.servo_arming_signal = SERVO_DISARM_SIGNAL;
  }
  AIO_SEND_PACKED(kMessageTypeServoSetState, PackServoSetStateMessage,
                  PACK_SERVOSETSTATEMESSAGE_SIZE, &m);
}

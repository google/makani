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

#include "gs/state_command/motor_state.h"

#include <stdint.h>
#include <string.h>

#include "avionics/common/cvt_avionics_messages.h"
#include "avionics/common/pack_avionics_messages.h"
#include "avionics/common/safety_codes.h"
#include "avionics/common/tether_message.h"
#include "avionics/linux/aio.h"
#include "avionics/network/aio_labels.h"
#include "gs/state_command/state_command.h"

MotorStateCommand::MotorStateCommand(void)
    : StateCommandBase("Motor"), tether_down_() {
  TetherDownMergeStateInit(&tether_down_);
}

StateCommandBase::LabelsMap MotorStateCommand::GetLabels(void) const {
  LabelsMap labels;
  for (int32_t i = 0; i < kNumMotors; ++i) {
    labels[i] = MotorLabelToString(static_cast<MotorLabel>(i));
  }
  return labels;
}

bool MotorStateCommand::GetActuatorStateFromMotorStatus(AioNode node,
                                                        ActuatorState *state) {
  MotorStatusMessage status;
  if (CvtGetMotorStatusMessage(node, &status, nullptr, nullptr)) {
    *state = static_cast<ActuatorState>(status.state);
    return true;
  }
  return false;
}

bool MotorStateCommand::GetActuatorStateFromTetherDown(MotorLabel label,
                                                       ActuatorState *state) {
  const TetherDownMessage *m = TetherDownMergeCvtGet(&tether_down_);
  const TetherMotorStatus *status = &m->motor_statuses[label];
  if (TetherIsNoUpdateCountValid(status->no_update_count)) {
    *state = static_cast<ActuatorState>(status->state);
    return true;
  }
  return false;
}

bool MotorStateCommand::GetActuatorState(int32_t index, ActuatorState *state) {
  MotorLabel label = static_cast<MotorLabel>(index);
  AioNode node = MotorLabelToMotorAioNode(label);

  return GetActuatorStateFromMotorStatus(node, state)
      || GetActuatorStateFromTetherDown(label, state);
}

void MotorStateCommand::SendActuatorStateCommand(ActuatorStateCommand command) {
  MotorSetStateMessage m;
  memset(&m, 0, sizeof(m));
  m.command = command;
  m.selected_motors = static_cast<uint8_t>(GetPendingTargets());
  if (command == kActuatorStateCommandArm) {
    m.command_data = MOTOR_ARMING_SIGNAL;
  } else if (command == kActuatorStateCommandDisarm) {
    m.command_data = MOTOR_DISARMING_SIGNAL;
  }
  AIO_SEND_PACKED(kMessageTypeMotorSetState, PackMotorSetStateMessage,
                  PACK_MOTORSETSTATEMESSAGE_SIZE, &m);
}

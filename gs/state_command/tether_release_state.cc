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

#include "gs/state_command/tether_release_state.h"

#include <stdint.h>
#include <string.h>

#include "avionics/common/cvt_avionics_messages.h"
#include "avionics/common/pack_avionics_messages.h"
#include "avionics/common/safety_codes.h"
#include "avionics/common/tether_message.h"
#include "avionics/linux/aio.h"
#include "avionics/network/aio_labels.h"
#include "gs/state_command/state_command.h"

TetherReleaseStateCommand::TetherReleaseStateCommand(void)
    : StateCommandBase("Tether release"), tether_down_() {
  TetherDownMergeStateInit(&tether_down_);
}

StateCommandBase::LabelsMap TetherReleaseStateCommand::GetLabels(void) const {
  LabelsMap labels;
  for (int32_t i = 0; i < kNumLoadcellNodes; ++i) {
    labels[i] = LoadcellNodeLabelToString(static_cast<LoadcellNodeLabel>(i));
  }
  return labels;
}

bool TetherReleaseStateCommand::GetActuatorStateFromLoadcellStatus(
    AioNode node, ActuatorState *state) {
  LoadcellMessage status;
  if (CvtGetLoadcellMessage(node, &status, nullptr, nullptr)) {
    *state = static_cast<ActuatorState>(status.tether_release_state);
    return true;
  }
  return false;
}

bool TetherReleaseStateCommand::GetActuatorStateFromTetherDown(
    LoadcellNodeLabel label, ActuatorState *state) {
  const TetherDownMessage *m = TetherDownMergeCvtGet(&tether_down_);
  const TetherReleaseStatus *status = &m->release_statuses[label];
  if (TetherIsNoUpdateCountValid(status->no_update_count)) {
    *state = static_cast<ActuatorState>(status->state);
    return true;
  }
  return false;
}

bool TetherReleaseStateCommand::GetActuatorState(int32_t index,
                                                 ActuatorState *state) {
  LoadcellNodeLabel label = static_cast<LoadcellNodeLabel>(index);
  AioNode node = LoadcellNodeLabelToLoadcellNodeAioNode(label);

  return GetActuatorStateFromLoadcellStatus(node, state)
      || GetActuatorStateFromTetherDown(label, state);
}

void TetherReleaseStateCommand::SendActuatorStateCommand(
    ActuatorStateCommand command) {
  TetherReleaseSetStateMessage m;
  memset(&m, 0, sizeof(m));
  m.state_command = command;
  m.selected_loadcells = static_cast<uint8_t>(GetPendingTargets());
  if (command == kActuatorStateCommandArm) {
    m.arming_signal = TETHER_RELEASE_ARMING_SIGNAL;
  } else if (command == kActuatorStateCommandDisarm) {
    m.arming_signal = TETHER_RELEASE_DISARM_SIGNAL;
  }
  AIO_SEND_PACKED(kMessageTypeTetherReleaseSetState,
                  PackTetherReleaseSetStateMessage,
                  PACK_TETHERRELEASESETSTATEMESSAGE_SIZE, &m);
}

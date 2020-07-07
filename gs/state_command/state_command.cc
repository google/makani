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

#include "gs/state_command/state_command.h"

#include <assert.h>
#include <stdint.h>
#include <string.h>

#include <algorithm>
#include <string>

#include "avionics/common/actuator_types.h"
#include "avionics/linux/clock.h"

StateCommandBase::StateCommandBase(const std::string& name)
    : name_(name), state_command_(kActuatorStateCommandNone),
      targets_selected_(), targets_set_(0), targets_unset_(0),
      num_attempts_(0), send_time_(0) {
}

void StateCommandBase::SetCommand(ActuatorStateCommand command) {
  state_command_ = command;
  targets_set_ = 0U;
  targets_unset_ = 0U;
  num_attempts_ = 0;
}

void StateCommandBase::SetTargets(uint32_t mask) {
  targets_selected_ = mask;
  targets_set_ = 0U;
  targets_unset_ = 0U;
  num_attempts_ = 0;
}

uint32_t StateCommandBase::GetAllTargets(void) const {
  uint32_t targets = 0U;
  for (auto iter : GetLabels()) {
    targets |= 1U << iter.first;
  }
  return targets;
}

bool StateCommandBase::UpdateProgress(void) {
  for (int32_t i = 0; i < 32; ++i) {
    uint32_t mask = 1U << i;
    ActuatorState state;
    if ((GetPendingTargets() & mask) != 0U && GetActuatorState(i, &state)) {
      if (CheckActuatorState(state)) {
        targets_set_ |= mask;
        targets_unset_ &= ~mask;
      } else {
        targets_set_ &= ~mask;
        targets_unset_ |= mask;
      }
    }
  }
  return GetPendingTargets() == 0U;
}

void StateCommandBase::SendCommand(void) {
  send_time_ = ClockGetUs();
  ++num_attempts_;
  SendActuatorStateCommand(state_command_);
}

bool StateCommandBase::CheckActuatorState(ActuatorState state) const {
  switch (state_command_) {
    case kActuatorStateCommandDisarm:
      return (state != kActuatorStateArmed) && (state != kActuatorStateRunning);
    case kActuatorStateCommandArm:
      return (state == kActuatorStateArmed) || (state == kActuatorStateRunning);
    case kActuatorStateCommandClearErrors:
      return state != kActuatorStateError;
    case kActuatorStateCommandTest:
      return state == kActuatorStateTest;
    case kActuatorStateCommandNone:
    default:
      assert(false);
      return false;
  }
}

uint32_t StateCommandBase::GetPendingTargets(void) const {
  return targets_selected_ & ~targets_set_;
}

uint32_t StateCommandBase::GetUnresponsiveTargets(void) const {
  return targets_selected_ & ~(targets_set_ | targets_unset_);
}

uint32_t StateCommandBase::GetSkipTargets(void) const {
  return GetAllTargets() & ~targets_selected_;
}

std::string StateCommandBase::GetTargetsString(uint32_t targets) const {
  std::string str;
  for (auto iter : GetLabels()) {
    if (targets & (1U << iter.first)) {
      str += " ";
      str += iter.second;
    }
  }
  transform(str.begin(), str.end(), str.begin(), ::toupper);
  return str;
}

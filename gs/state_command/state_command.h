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

#ifndef GS_STATE_COMMAND_STATE_COMMAND_H_
#define GS_STATE_COMMAND_STATE_COMMAND_H_

#include <stdint.h>

#include <map>
#include <string>

#include "avionics/common/actuator_types.h"

class StateCommandBase {
 public:
  explicit StateCommandBase(const std::string& name);
  virtual ~StateCommandBase(void) {}

  // Maps the target's label index to the target's name.
  typedef std::map<int32_t, std::string> LabelsMap;

  // These functions are target-type specific.
  virtual LabelsMap GetLabels(void) const = 0;
  virtual bool GetActuatorState(int32_t, ActuatorState *state) = 0;
  virtual void SendActuatorStateCommand(ActuatorStateCommand command) = 0;

  // Configure the operation.
  void SetCommand(ActuatorStateCommand command);
  void SetTargets(uint32_t mask);

  // Check progress. Returns true when complete.
  bool UpdateProgress(void);

  // Send next target command.
  void SendCommand(void);

  const std::string& GetName(void) const { return name_; }
  std::string GetTargetsString(uint32_t targets) const;
  ActuatorStateCommand GetCommand(void) const { return state_command_; }

  uint32_t GetAllTargets(void) const;
  uint32_t GetSetTargets(void) const { return targets_set_; }
  uint32_t GetUnsetTargets(void) const { return targets_unset_; }
  uint32_t GetSelectedTargets(void) const { return targets_selected_; }
  uint32_t GetUnresponsiveTargets(void) const;
  uint32_t GetSkipTargets(void) const;
  uint32_t GetPendingTargets(void) const;
  int32_t GetNumAttempts(void) const { return num_attempts_; }
  int64_t GetSendTime(void) const { return send_time_; }

 private:
  bool CheckActuatorState(ActuatorState state) const;

  std::string name_;
  ActuatorStateCommand state_command_;
  uint32_t targets_selected_;
  uint32_t targets_set_;
  uint32_t targets_unset_;
  int32_t num_attempts_;
  int64_t send_time_;
};

#endif  // GS_STATE_COMMAND_STATE_COMMAND_H_

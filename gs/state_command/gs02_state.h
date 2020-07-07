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

#ifndef GS_STATE_COMMAND_GS02_STATE_H_
#define GS_STATE_COMMAND_GS02_STATE_H_

#include <stdint.h>

#include "avionics/common/actuator_types.h"
#include "gs/state_command/state_command.h"

class Gs02StateCommand : public StateCommandBase {
 public:
  Gs02StateCommand(void) : StateCommandBase("Gs02") {}
  ~Gs02StateCommand(void) {}

  LabelsMap GetLabels(void) const override;
  bool GetActuatorState(int32_t index, ActuatorState *state) override;
  void SendActuatorStateCommand(ActuatorStateCommand command) override;
};

#endif  // GS_STATE_COMMAND_GS02_STATE_H_

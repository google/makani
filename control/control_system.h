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

#ifndef CONTROL_CONTROL_SYSTEM_H_
#define CONTROL_CONTROL_SYSTEM_H_

#include <stdbool.h>

#include "avionics/common/avionics_messages.h"
#include "control/control_types.h"
#include "control/system_types.h"
#include "system/labels.h"

#ifdef __cplusplus
extern "C" {
#endif

// Initializes the control system state.
void ControlSystemInit(const ControlParams *params, ControlState *state);

// Runs an iteration of the controller and computes command and synchronization
// messages.
//
// Args:
//   controller_label: This controller's label.
//   system_params: System parameters.
//   control_params: Controller parameters.
//   state: Controller state.
//   command_message: Command message to be sent if true is returned.
//   sync_message: Synchronization message to be sent if true is returned.
//
// Returns:
//   True if the messages have been populated and should be sent.
bool ControlSystemStep(ControllerLabel controller_label,
                       const SystemParams *system_params,
                       const ControlParams *control_params, ControlState *state,
                       ControllerCommandMessage *command_message,
                       ControllerSyncMessage *sync_message);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // CONTROL_CONTROL_SYSTEM_H_

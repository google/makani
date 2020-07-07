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

// Controller command arbitration interface.
//
// Users should maintain a ControllerArbitrationState, initialized by
// calling ControllerArbitrationInit once.
//
// Calling ControllerArbitrationGetCommand computes the arbitrated
// command from the current state. This function will return NULL if
// the state indicates no valid command is available.

#ifndef AVIONICS_COMMON_CONTROLLER_ARBITRATION_H_
#define AVIONICS_COMMON_CONTROLLER_ARBITRATION_H_

#include <stdint.h>

#include "avionics/common/avionics_messages.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  ControllerCommandMessage commands[kNumControllers];
  uint16_t sequence_numbers[kNumControllers];
  int64_t timestamps[kNumControllers];  // As returned by ClockGetUs().
} ControllerArbitrationState;

// Function used to validate ControllerCommandMessages already in the CVT.  A
// message that passes validation therefore won't be blocked by a newer invalid
// one from a different source, although an invalid message will overwrite an
// older valid message from the same source.
typedef bool (*MessageValidator)(const ControllerCommandMessage *);

// Initialize the state for controller arbitration.
//
// Args:
//   now: time as returned by ClockGetUs().
//   state: State to initialize.
void ControllerArbitrationInit(int64_t now, ControllerArbitrationState *state);

// Get the arbitrated controller command.
//
// Args:
//   now: Time as returned by ClockGetUs().
//   state: Current arbitration state.
//   source: Returns the label of the controller that sent the command, if any.
//
// Returns:
//   A pointer to the arbitrated command or NULL if no valid, non-stale message
//   from any controller exists in the CVT.
const ControllerCommandMessage *ControllerArbitrationGetCommand(
    int64_t now, const MessageValidator func,
    const ControllerArbitrationState *state, ControllerLabel *source);

// Poll the CVT to update the arbitration state.  This function has
// the side-effect of reading the ControllerCommandMessage entries for
// each controller.
// TODO: Consider just rolling this into ControllerArbitrationGetCommand.
void ControllerArbitrationUpdateFromCvt(ControllerArbitrationState *state);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // AVIONICS_COMMON_CONTROLLER_ARBITRATION_H_

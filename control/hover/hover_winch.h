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

#ifndef CONTROL_HOVER_HOVER_WINCH_H_
#define CONTROL_HOVER_HOVER_WINCH_H_

#include <stdbool.h>

#include "control/control_types.h"
#include "control/estimator/estimator_types.h"
#include "control/hover/hover_types.h"

#ifdef __cplusplus
extern "C" {
#endif

// Initializes the state of the winch loop.
void HoverWinchInit(const HoverWinchParams *params, HoverWinchState *state);

// Returns true if the winch parameters are valid.
bool HoverWinchValidateParams(const HoverWinchParams *params);

// Schedules the winch velocity during pay-out and reel-in, based on
// the current flight mode, winch position, and joystick command.
// Returns the winch velocity command.
double HoverWinchStep(FlightMode flight_mode, const StateEstimate *state_est,
                      const HoverWinchParams *params, HoverWinchState *state);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // CONTROL_HOVER_HOVER_WINCH_H_

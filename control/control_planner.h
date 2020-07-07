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

#ifndef CONTROL_CONTROL_PLANNER_H_
#define CONTROL_CONTROL_PLANNER_H_

#include <stdbool.h>

#include "control/control_types.h"
#include "control/estimator/estimator_types.h"
#include "control/fault_detection/fault_detection_types.h"
#include "system/labels.h"

#ifdef __cplusplus
extern "C" {
#endif

// Initializes the state of the planner.
void PlannerInit(PlannerState *state);

// Returns the current flight status, which includes the current
// flight mode and flight mode timer.
const FlightStatus *PlannerGetFlightStatus(const PlannerState *state);

// Updates the state of the planner.  Sets the current flight mode and
// outputs the controller synchronization data used to synchronize
// flight modes with the other controllers.
void PlannerStep(ControllerLabel controller_label,
                 const ControlSyncData sync_data_in[],
                 const StateEstimate *state_est, const FaultMask faults[],
                 const bool ready_states[], bool force_hover_accel,
                 const PlannerParams *planner_params,
                 const JoystickControlParams *joystick_params,
                 const ControlOutput *output, PlannerState *state,
                 ControlSyncData *sync_data_out);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // CONTROL_CONTROL_PLANNER_H_

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

#ifndef CONTROL_MANUAL_MANUAL_H_
#define CONTROL_MANUAL_MANUAL_H_

#include "control/control_types.h"
#include "control/estimator/estimator_types.h"
#include "control/manual/manual_types.h"

#ifdef __cplusplus
extern "C" {
#endif

// Manual Mode allows control of the kite after tether release.
// Because the tether is no longer connected, motor control is no
// longer available; all remaining control is via the flight control
// surfaces (ailerons, flaps, elevator, and rudder).
//
// Manual Mode is activated by the tether release switch on the
// joystick, which also commands the tether release mechanism.  This
// mode transition is latched and cannot be reversed.
//
// Primary flight control is available via the pitch, roll, and yaw
// axes on the joystick, which actuate the elevator, ailerons, and
// rudder, respectively.
//
// A stability enhancement function called "auto-glide" is activated
// after some delay.  In auto-glide mode, feedback is provided to the
// ailerons and elevator to attempt to hold the wings level and
// maintain a pre-specified pitch attitude with respect to the ground.

// Initializes the state of the manual mode controller.
void ManualInit(const StateEstimate *state_est, const ManualParams *params,
                ManualState *state);

// Returns true if the manual mode parameters are valid.
bool ManualValidateParams(const ManualParams *params);

// Updates the state of the manual controller by a single time step.
// Returns the output commands to the actuators.
void ManualStep(const FlightStatus *flight_status,
                const StateEstimate *state_est, const ManualParams *params,
                ManualState *state, ControlOutput *control_output);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // CONTROL_MANUAL_MANUAL_H_

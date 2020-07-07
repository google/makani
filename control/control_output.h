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

#ifndef CONTROL_CONTROL_OUTPUT_H_
#define CONTROL_CONTROL_OUTPUT_H_

#include "control/control_types.h"
#include "control/estimator/estimator_types.h"

#ifdef __cplusplus
extern "C" {
#endif

void ControlOutputInit(const ControlOutputParams *params,
                       ControlOutputState *state);

const double *ControlOutputGetLastFlapCommand(const ControlOutputState *state);

void ControlOutputStep(const ControlOutput *raw_control_output,
                       const JoystickEstimate *joystick,
                       const PerchAziEstimate *perch_azi,
                       const ControlOutputParams *params,
                       ControlOutputState *state,
                       ControlOutput *control_output);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // CONTROL_CONTROL_OUTPUT_H_

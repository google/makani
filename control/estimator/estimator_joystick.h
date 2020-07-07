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

#ifndef CONTROL_ESTIMATOR_ESTIMATOR_JOYSTICK_H_
#define CONTROL_ESTIMATOR_ESTIMATOR_JOYSTICK_H_

#include "control/estimator/estimator_types.h"
#include "control/fault_detection/fault_detection_types.h"
#include "control/sensor_types.h"

#ifdef __cplusplus
extern "C" {
#endif

void EstimatorJoystickInit(EstimatorJoystickState *state);

void EstimatorJoystickStep(const JoystickData *joystick_data,
                           const FaultMask *joystick_fault,
                           const EstimatorJoystickParams *params,
                           EstimatorJoystickState *state,
                           JoystickEstimate *joystick);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // CONTROL_ESTIMATOR_ESTIMATOR_JOYSTICK_H_

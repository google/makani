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

#ifndef CONTROL_CONTROL_PARAMS_H_
#define CONTROL_CONTROL_PARAMS_H_

#include "control/control_types.h"

typedef struct {
  const FlightPlan *flight_plan;
  const ControlOption *control_opt;
  const SimpleAeroModelParams *simple_aero_model;
  const JoystickControlParams *joystick_control;
  const RotorControlParams *rotor_control;
} GlobalControlParams;

extern GlobalControlParams g_cont;

#ifdef __cplusplus
extern "C" {
#endif

const ControlParams *GetControlParams(void);
ControlParams *GetControlParamsUnsafe(void);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // CONTROL_CONTROL_PARAMS_H_

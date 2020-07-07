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

#ifndef CONTROL_SIMPLE_AERO_TYPES_H_
#define CONTROL_SIMPLE_AERO_TYPES_H_

#include "system/labels.h"

#define NUM_SIMPLE_ROTOR_MODEL_COEFFS 3

typedef struct {
  double thrust_coeffs[NUM_SIMPLE_ROTOR_MODEL_COEFFS];
  double J_neutral;
  double J_max;
  double D;
  double D4;
} SimpleRotorModelParams;

typedef struct {
  double dCL_dalpha;
  double dCD_dalpha;
  double CL_0;
  double CD_0;
  double base_flaps[kNumFlaps];  // Flap deflections at which CL_0 is calculated
  double dCL_dflap[kNumFlaps];
  double dCY_dbeta;
  double CY_0;
} SimpleAeroModelParams;

#endif  // CONTROL_SIMPLE_AERO_TYPES_H_

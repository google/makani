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

#ifndef CONTROL_SYSTEM_PARAMS_H_
#define CONTROL_SYSTEM_PARAMS_H_

#include "control/system_types.h"

typedef struct {
  const double *ts;
  const PhysParams *phys;
  const TetherParams *tether;
  const WingParams *wing;
  const PerchParams *perch;
  const RotorParams *rotors[kNumMotors];
} GlobalSystemParams;

extern const GlobalSystemParams g_sys;

#ifdef __cplusplus
extern "C" {
#endif

const SystemParams *GetSystemParams(void);
SystemParams *GetSystemParamsUnsafe(void);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // CONTROL_SYSTEM_PARAMS_H_

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

#ifndef SIM_SIM_PARAMS_H_
#define SIM_SIM_PARAMS_H_

#include "sim/sim_types.h"

typedef struct {
  const SimOption *sim_opt;
  const PhysSimParams *phys_sim;
} GlobalSimParams;

extern const GlobalSimParams g_sim;

#ifdef __cplusplus
extern "C" {
#endif

const SimParams *GetSimParams(void);
SimParams *GetSimParamsUnsafe(void);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // SIM_SIM_PARAMS_H_

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

#ifndef AVIONICS_COMMON_CONING_SCULLING_H_
#define AVIONICS_COMMON_CONING_SCULLING_H_

#include <stdbool.h>
#include <stdint.h>

#include "common/c_math/vec3f.h"

#ifdef __cplusplus
extern "C" {
#endif

// Initialize coning & sculling internal state.
void ConingScullingInit(void);

// Integrate l-step one cycle using inertial increments.
bool ConingScullingUpdateInc(int32_t dec, float dt_l, const Vec3f *dalpha_l,
                             const Vec3f *dnu_l);

// Integrate l-step one cycle using trapezoidal integration of raw measurements.
bool ConingScullingUpdateRaw(int32_t dec, float dt_l, const Vec3f *omega_ib_b,
                             const Vec3f *a_sf_b);

// Sample integrated l-steps at m-step cycle.
void ConingScullingSample(float *dt_m, Vec3f *phi_m, Vec3f *dvsf_m,
                          Vec3f *alpha_m, Vec3f *nu_m);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // AVIONICS_COMMON_CONING_SCULLING_H_

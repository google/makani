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

#ifndef CONTROL_VESSEL_FRAME_H_
#define CONTROL_VESSEL_FRAME_H_

#include "common/c_math/mat3.h"
#include "common/c_math/vec3.h"
#include "control/system_types.h"

#ifdef __cplusplus
extern "C" {
#endif

double VecVToAzimuth(const Vec3 *pos_v);
double VecVToElevation(const Vec3 *pos_v);
double AziGToV(double azi_g, const Mat3 *dcm_g2v);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // CONTROL_VESSEL_FRAME_H_

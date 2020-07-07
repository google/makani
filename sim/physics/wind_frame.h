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

#ifndef SIM_PHYSICS_WIND_FRAME_H_
#define SIM_PHYSICS_WIND_FRAME_H_

#include "common/c_math/mat3.h"
#include "common/c_math/vec3.h"

#ifdef __cplusplus
extern "C" {
#endif

// Generates a DCM from the mean wind frame to the ground frame.
//
// Args:
//   mean_wind_direction: Angle [rad] that the mean wind direction makes
//       with the -x_g-direction, measured about the z_g-axis.
//   dcm_mw2g: Generated DCM.
void CalcDcmMwToG(double mean_wind_direction, Mat3 *dcm_mw2g);

// Transforms the coordinates of a point from the ground frame to the
// mean wind frame.
//
// Args:
//   point_g: Coordinates of the point in the ground frame.
//   dcm_mw2g: DCM that transforms from the mw-frame to the g-frame.
//   ground_z: z-coordinate of the ground in the ground-frame.
//   point_mw: Output coordinates of the point in the mean wind frame.
void TransformGToMw(const Vec3 *point_g, const Mat3 *dcm_mw2g, double ground_z,
                    Vec3 *point_mw);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // SIM_PHYSICS_WIND_FRAME_H_

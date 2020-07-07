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

#include "sim/physics/wind_frame.h"

#include <math.h>

#include "common/c_math/geometry.h"
#include "common/c_math/util.h"

void CalcDcmMwToG(double mean_wind_direction, Mat3 *dcm_mw2g) {
  // See control/coordinate_systems.md for the direction of the mean wind frame.
  // This transformation first rotates about the x-axis to make z point "down",
  // then rotates about the new z-axis to align the x-axis with +x_g.
  AngleToDcm(PI, 0.0, -(PI + mean_wind_direction), kRotationOrderXyz, dcm_mw2g);
}

void TransformGToMw(const Vec3 *point_g, const Mat3 *dcm_mw2g, double ground_z,
                    Vec3 *point_mw) {
  *point_mw = *point_g;
  point_mw->z -= ground_z;
  Mat3TransVec3Mult(dcm_mw2g, point_mw, point_mw);
}

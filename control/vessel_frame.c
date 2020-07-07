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

#include "control/vessel_frame.h"

#include <math.h>

#include "control/system_params.h"

double VecVToAzimuth(const Vec3 *pos_v) {
  return Wrap(atan2(pos_v->y, pos_v->x) +
                  GetSystemParams()->ground_station.azi_ref_offset,
              -PI, PI);
}

double VecVToElevation(const Vec3 *pos_v) {
  return atan2(-pos_v->z, Vec3XyNorm(pos_v));
}

// Compute azimuth in G frame to azimuth in V frame.
//
// Note that this problem is under-constrained; the azimuth of a point in V
// frame is determined jointly by the azimuth AND elevation in G frame, should
// there be nonzero pitch and roll angles between the V frame and G frame.
// Here the assumption is that the pitch and roll angles are small and therefore
// the choice of the elevation angle in G frame minimally affects the value of
// the azimuth in V frame.
double AziGToV(double azi_g, const Mat3 *dcm_g2v) {
  Vec3 hat_g, hat_v;
  SphToCart(azi_g, 0.0, 1.0, &hat_g);
  Mat3Vec3Mult(dcm_g2v, &hat_g, &hat_v);
  return VecVToAzimuth(&hat_v);
}

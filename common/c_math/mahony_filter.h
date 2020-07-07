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

#ifndef COMMON_C_MATH_MAHONY_FILTER_H_
#define COMMON_C_MATH_MAHONY_FILTER_H_

#include <stdint.h>

#include "common/c_math/quaternion.h"
#include "common/c_math/vec3.h"
#include "common/macros.h"

#ifdef __cplusplus
extern "C" {
#endif

// General form of Mahony's nonlinear complementary filter.
//
// Args:
//   pqr: Angular rate measurement.
//   Kp: Proportional gain for vector errors (sets complementary filter
//     frequency).
//   Ki: Integral gain on vector error for bias estimation.
//   vi: Array of num_vec vectors in the inertial frame.
//   vb: Array of num_vec vector measurements in the body frame.
//   k: Array of num_vec coefficients indicating how much you trust a
//     given vector measurement.
//   fc: Array of num_vec cutoff frequencies for the first order LPF on each
//     error vector.
//   num_vec: The number of vector measurements to be processed.  Must be
//     non-negative.
//   ts: Sample period.
//   bias_bound_low: Lower bound on the components of the bias.
//   bias_bound_high: Upper bound on the components of the bias.
//   q: In/out parameter storing the previous attitude estimate quaternion and
//     updated to the new value.
//   bias: In/out parameter storing the previous angular rate bias estimate and
//     updated to the new value.
//   ef: In/out parameter storing an array of num_vec previous filtered error
//     vectors, and updated to hold the new values.
void MahonyFilter(const Vec3 *pqr, double Kp, double Ki, const Vec3 *vi,
                  const Vec3 *vb, const double *k, const double *fc,
                  int32_t num_vec, double ts, double bias_bound_low,
                  double bias_bound_high, Quat *q, Vec3 *bias, Vec3 *ef);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // COMMON_C_MATH_MAHONY_FILTER_H_

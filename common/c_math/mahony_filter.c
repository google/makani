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

#include "common/c_math/mahony_filter.h"

#include <assert.h>
#include <stdint.h>

#include "common/c_math/filter.h"
#include "common/c_math/quaternion.h"
#include "common/c_math/util.h"
#include "common/c_math/vec3.h"

void MahonyFilter(const Vec3 *pqr, double Kp, double Ki, const Vec3 *vi,
                  const Vec3 *vb, const double *k, const double *fc,
                  int32_t num_vec, double ts, double bias_bound_low,
                  double bias_bound_high, Quat *q, Vec3 *bias, Vec3 *ef) {
  assert(num_vec >= 0);
  assert(k != NULL && vi != NULL && vb != NULL && fc != NULL);
  assert(q != NULL && bias != NULL && ef != NULL && pqr != NULL);
  Vec3 w_est, w_err = kVec3Zero;

  for (int32_t i = 0; i < num_vec; ++i) {
    Vec3 vi_norm, vb_norm, vb_est, e;
    // Normalize vectors (safely).
    Vec3Scale(&vi[i], 1.0 / Vec3NormBound(&vi[i], 1.0e-6), &vi_norm);
    Vec3Scale(&vb[i], 1.0 / Vec3NormBound(&vb[i], 1.0e-6), &vb_norm);

    // Estimate inertial vectors in body coordinates.
    QuatRotate(q, &vi_norm, &vb_est);

    // Compute angular rate error from vector measurements.
    Vec3Cross(&vb_norm, &vb_est, &e);

    // Filter the vector errors (note: fc == 0 implies no filtering)
    assert(fc[i] >= 0.0);
    if (fc[i] > 0.0) {
      LpfVec3(&e, fc[i], ts, &ef[i]);
    } else {
      ef[i] = e;
    }

    // Compute angular rate error based on vectors alone.
    Vec3Axpy(k[i], &ef[i], &w_err);
  }

  // Integrate the gyro bias.
  Vec3LinComb(1.0, bias, -Ki * ts, &w_err, bias);
  SaturateVec3ByScalar(bias, bias_bound_low, bias_bound_high, bias);

  // Estimated the angular rate (omega_ref).
  Vec3LinComb3(1.0, pqr, -1.0, bias, Kp, &w_err, &w_est);

  // Integrate the quaternion.
  Quat dq, q_omega = {0.0, 0.5 * w_est.x, 0.5 * w_est.y, 0.5 * w_est.z};
  QuatMultiply(q, &q_omega, &dq);
  QuatLinComb(1.0, q, ts, &dq, q);
  QuatNormalize(q, q);
}

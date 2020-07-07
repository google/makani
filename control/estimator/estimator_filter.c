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

#include "control/estimator/estimator_filter.h"

#include <assert.h>
#include <stdint.h>

#include "common/c_math/kalman.h"
#include "common/c_math/linalg.h"
#include "common/c_math/vec3.h"

void EstimatorFilterCorrectScalarSparse3(const Vec3 *h_meas, int32_t meas_index,
                                         double sigma_meas,
                                         double total_state_meas_err,
                                         Vec *x_hat, Vec *ud, Vec *h, Vec *k,
                                         double *dz, double *pzz,
                                         Vec *dx_plus) {
  assert(h_meas != NULL && x_hat != NULL && ud != NULL && h != NULL &&
         k != NULL && dz != NULL && pzz != NULL && dx_plus != NULL);
  assert(sigma_meas > 0.0);
  assert(0 <= meas_index && meas_index <= x_hat->length - 3);
  assert(x_hat->length == h->length && x_hat->length == k->length &&
         ud->length == UdKalmanArrayLength(x_hat->length));
  assert(x_hat->length == dx_plus->length);

  *dz = total_state_meas_err - h_meas->x * VecGet(x_hat, meas_index) -
        h_meas->y * VecGet(x_hat, meas_index + 1) -
        h_meas->z * VecGet(x_hat, meas_index + 2);

  // TODO: Exploit sparsity of H matrix.
  VecZero(h);
  *VecPtr(h, meas_index) = h_meas->x;
  *VecPtr(h, meas_index + 1) = h_meas->y;
  *VecPtr(h, meas_index + 2) = h_meas->z;

  // TODO: Validate measurements before applying corrections.
  UdKalmanCalcWeightVectors(ud, h, k);
  // Calculate innovation covariance: Pzz = H * P * H^T + R.
  *pzz = VecDot(h, k) + sigma_meas * sigma_meas;
  UdKalmanCalcGain(sigma_meas * sigma_meas, h, ud, k);
  VecScale(k, *dz, dx_plus);
  VecAdd(dx_plus, x_hat, x_hat);
}

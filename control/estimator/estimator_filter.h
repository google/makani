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

#ifndef CONTROL_ESTIMATOR_ESTIMATOR_FILTER_H_
#define CONTROL_ESTIMATOR_ESTIMATOR_FILTER_H_

#include <stdint.h>

#include "common/c_math/linalg.h"
#include "common/c_math/vec3.h"
#include "common/macros.h"

#ifdef __cplusplus
extern "C" {
#endif

// Apply a scalar measurement update for H with 3 contiguous non-zero entries.
//
// Args:
//   h_meas: Gives the non-zero entries of the H matrix.
//   meas_index: gives the index of the first non-zero entry of the H matrix.
//   sigma_meas: Standard deviation for the measurement.
//   total_state_meas_err: Measurement error not including the contribution
//       from H * x_hat.
//   x_hat: n-by-1 error state vector.
//   ud: Vector storing the UD-factorization of the estimate covariance.
//   h: n-by-1 working space vector.
//   k: n-by-1 output vector storing the Kalman gain.
//   dz: Returns the innovation.
//   pzz: Returns the innovation covariance.
//   dx_plus: Returns the correction to the error state.
void EstimatorFilterCorrectScalarSparse3(const Vec3 *h_meas, int32_t meas_index,
                                         double sigma_meas,
                                         double total_state_meas_err,
                                         Vec *x_hat, Vec *ud, Vec *h, Vec *k,
                                         double *dz, double *pzz, Vec *dx_plus);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // CONTROL_ESTIMATOR_ESTIMATOR_FILTER_H_

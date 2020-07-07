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

#ifndef CONTROL_ESTIMATOR_ESTIMATOR_ATTITUDE_H_
#define CONTROL_ESTIMATOR_ESTIMATOR_ATTITUDE_H_

#include <stdbool.h>

#include "common/c_math/mat3.h"
#include "common/c_math/quaternion.h"
#include "common/c_math/vec3.h"
#include "control/control_types.h"
#include "control/estimator/estimator_types.h"
#include "control/fault_detection/fault_detection_types.h"
#include "control/sensor_types.h"
#include "control/system_types.h"

#ifdef __cplusplus
extern "C" {
#endif

// Notation:
//
//  [v x] :=[   0, -v.z,  v.y
//            v.z,    0, -v.x
//           -v.y,  v.x,    0].
//
// The attitude estimator makes use of a six state Kalman filter based
// on corrections using vector measurements.  The total state is given
// by a quaternion rotating g-coordinates to b-coordinates and a
// vector of gyro biases.  The error states are defined by:
//
//   R_g2b \approx \hat R_g2b (I + [psi_g2b^g x]),
//   bias \approx \hat bias + delta_bias.
//
// Propagation Model:
// ------------------
//
// The estimated and true states evolve according to:
//
//   \hat R_g2b(+) = exp(-dt * \hat omega^b) * \hat R_g2b,
//   \hat bias(+) = \hat bias,
//
//   R_g2b(+) = exp(-dt * (\hat omega^b - delta_bias - v_omega)) R_g2b,
//            ~ exp(-dt * \hat omega^b) (I + dt*[delta_bias + v_omega x]) R_g2b,
//   bias(+) = bias + w_bias_random_walk.
//
// where \hat omega^b = gyro - \hat bias.  Neglecting second order errors:
//
//   psi_g2b^g(+)  ~ psi_g2b^g + dt * \hat R_g2b^T (\delta_bias + v_omega)
//   delta_bias(+) = delta_bias + w_bias_random_walk
//
// Measurement Model:
// ------------------
//
// When applying vector corrections to this model we use the prediction:
//
//   \hat y^b = \hat R_g2b u^g,
//
// where u^g is the reference vector in the g coordinate frame. The
// measurement model is:
//
//   y^b = R_g2b u^g + v_noise = \hat R_g2b (I + [psi_g2b^g x]) u^g + v_noise
//
// yielding the measurement error:
//
//   y^b - \hat y^b = \hat R_g2b [psi_g2b^g x] u^g + v_noise,
//                  = -[u^b x] \hat R_g2b psi_g2b^g + v_noise.

// Initialize the attitude estimator.
void EstimatorAttitudeInit(const EstimatorAttitudeParams *params,
                           EstimatorAttitudeState *state);

// Calculate the filter's expected state covariances.
void EstimatorAttitudeGetCovariances(const EstimatorAttitudeState *state,
                                     Vec3 *cov_attitude_err,
                                     Vec3 *cov_gyro_bias);

void ApplyAttitudeErrorState(const Vec *x_hat,
                             const EstimatorAttitudeParams *params,
                             EstimatorAttitudeFilterState *state);

// Coarsely initialize the kite attitude.
void RunCoarseInitialization(const Vec3 *gyro, const Vec3 *acc,
                             const Vec3 *f_sf_g, const Vec3 *mag,
                             const Vec3 *mag_g,
                             const EstimatorAttitudeParams *params,
                             EstimatorAttitudeFilterState *state);

// Apply an attitude filter correction based on a known vector in the g
// coordinate frame, measured in the body frame.
//
// Args:
//   vec_g: Known vector in the g-coordinate frame.
//   meas_b: Measured vector in the body frame.
//   sigma_meas: Measurement standard deviation (taken to be uncorrelated and
//       equal for each component of the measurement).
//   state: State of the filter to update.
//   correct: The filter correction.
void AttitudeFilterCorrectVector(const Vec3 *vec_g, const Vec3 *meas_b,
                                 double sigma_meas, Vec *x_hat,
                                 EstimatorAttitudeFilterState *state,
                                 EstimatorAttitudeCorrection3 *correct);

// Update the attitude estimate based on gyro measurements.
//
// Args:
//   gyro: Gyroscope measurements [rad/s].
//   params: Parameters.
//   state: State.
void AttitudeFilterPropagate(const Vec3 *gyro,
                             const EstimatorAttitudeParams *params,
                             EstimatorAttitudeFilterState *state);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // CONTROL_ESTIMATOR_ESTIMATOR_ATTITUDE_H_

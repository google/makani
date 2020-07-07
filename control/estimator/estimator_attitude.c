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

#include "control/estimator/estimator_attitude.h"

#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "common/c_math/filter.h"
#include "common/c_math/mahony_filter.h"
#include "common/c_math/mat3.h"
#include "common/c_math/quaternion.h"
#include "common/c_math/vec2.h"
#include "common/c_math/vec3.h"
#include "common/c_math/voting.h"
#include "common/macros.h"
#include "control/common.h"
#include "control/control_telemetry.h"
#include "control/control_types.h"
#include "control/estimator/estimator_filter.h"
#include "control/fault_detection/fault_detection_types.h"
#include "control/sensor_types.h"
#include "control/sensor_util.h"
#include "control/system_params.h"
#include "system/labels.h"

void EstimatorAttitudeInit(const EstimatorAttitudeParams *params,
                           EstimatorAttitudeState *state) {
  assert(params != NULL && state != NULL);
  memset(state, 0, sizeof(*state));

  state->acc_f_z1 = kVec3Zero;
  state->filter.q_g2b = params->q_g2b_0;
  state->filter.gyro_bias = kVec3Zero;

  // Initialize covariance.
  VEC(kNumAttitudeStates, diag);
  *VecPtr(&diag, kAttitudeStateAttX) =
      params->sigma_attitude_0 * params->sigma_attitude_0;
  *VecPtr(&diag, kAttitudeStateAttY) =
      params->sigma_attitude_0 * params->sigma_attitude_0;
  *VecPtr(&diag, kAttitudeStateAttZ) =
      params->sigma_attitude_0 * params->sigma_attitude_0;

  *VecPtr(&diag, kAttitudeStateBiasGX) =
      params->sigma_gyro_bias_0 * params->sigma_gyro_bias_0;
  *VecPtr(&diag, kAttitudeStateBiasGY) =
      params->sigma_gyro_bias_0 * params->sigma_gyro_bias_0;
  *VecPtr(&diag, kAttitudeStateBiasGZ) =
      params->sigma_gyro_bias_0 * params->sigma_gyro_bias_0;

  Vec ud;
  VEC_WRAP(state->filter.ud, ud);
  UdKalmanStoreUpperTri(NULL, &diag, &ud);
}

void EstimatorAttitudeGetCovariances(const EstimatorAttitudeState *state,
                                     Vec3 *cov_attitude_err,
                                     Vec3 *cov_gyro_bias) {
  VEC(kNumAttitudeStates, cov);
  assert((kNumAttitudeStates + 1) * kNumAttitudeStates / 2 ==
         ARRAYSIZE(state->filter.ud));
  UdKalmanExtractCovariances(state->filter.ud, &cov);
  cov_attitude_err->x = VecGet(&cov, kAttitudeStateAttX);
  cov_attitude_err->y = VecGet(&cov, kAttitudeStateAttY);
  cov_attitude_err->z = VecGet(&cov, kAttitudeStateAttZ);
  cov_gyro_bias->x = VecGet(&cov, kAttitudeStateBiasGX);
  cov_gyro_bias->y = VecGet(&cov, kAttitudeStateBiasGY);
  cov_gyro_bias->z = VecGet(&cov, kAttitudeStateBiasGZ);
}

void ApplyAttitudeErrorState(const Vec *x_hat,
                             const EstimatorAttitudeParams *params,
                             EstimatorAttitudeFilterState *state) {
  // Update the attitude quaternion and biases.
  Quat dq;
  Vec3 mrp = {-VecGet(x_hat, kAttitudeStateAttX) / 4.0,
              -VecGet(x_hat, kAttitudeStateAttY) / 4.0,
              -VecGet(x_hat, kAttitudeStateAttZ) / 4.0};
  MrpToQuat(&mrp, &dq);
  QuatMultiply(&dq, &state->q_g2b, &state->q_g2b);
  QuatNormalize(&state->q_g2b, &state->q_g2b);

  state->gyro_bias.x += VecGet(x_hat, kAttitudeStateBiasGX);
  state->gyro_bias.y += VecGet(x_hat, kAttitudeStateBiasGY);
  state->gyro_bias.z += VecGet(x_hat, kAttitudeStateBiasGZ);

  SaturateVec3ByScalar(&state->gyro_bias, -params->max_gyro_bias,
                       params->max_gyro_bias, &state->gyro_bias);
}

void RunCoarseInitialization(const Vec3 *gyro, const Vec3 *acc,
                             const Vec3 *f_sf_g, const Vec3 *mag,
                             const Vec3 *mag_g,
                             const EstimatorAttitudeParams *params,
                             EstimatorAttitudeFilterState *state) {
  // Here we use the Mahony filter to initialize the attitude estimate
  // based on plumb-bob gravity and magnetometer measurements.
  const Vec3 vi[2] = {*f_sf_g, *mag_g};
  const Vec3 vb[2] = {*acc, *mag};
  const double kp[] = {params->coarse_init_kp_acc, params->coarse_init_kp_mag};
  const double fc[] = {0.0, 0.0};
  Vec3 ef[2] = {kVec3Zero, kVec3Zero};
  MahonyFilter(gyro, params->coarse_init_kp, 0.0, vi, vb, kp, fc, ARRAYSIZE(vi),
               *g_sys.ts, 0.0, 0.0, &state->q_g2b, &state->gyro_bias, ef);
}

void AttitudeFilterCorrectVector(const Vec3 *vec_g, const Vec3 *meas_b,
                                 double sigma_meas, Vec *x_hat,
                                 EstimatorAttitudeFilterState *state,
                                 EstimatorAttitudeCorrection3 *correct) {
  // Form measurement error.
  Vec3 vec_b;
  QuatRotate(&state->q_g2b, vec_g, &vec_b);
  Vec3 meas_err;
  Vec3Sub(meas_b, &vec_b, &meas_err);

  Quat q_b2g;
  QuatConj(&state->q_g2b, &q_b2g);

  Vec ud;
  VEC_WRAP(state->ud, ud);
  VEC(kNumAttitudeStates, h);
  VEC(kNumAttitudeStates, k);
  // The 3-by-6 measurement matrix is given by:
  //
  // H = [-[vec_b x] \hat R_g2b  0_3x3].
  //
  // We assume uncorrelated measurement noise and apply each row as a
  // scalar update.  To compute the non-zero elements of the rows of H
  // we treat the rows of -[vec_b x] as columns and rotate them by
  // \hat R_g2b^T.
  Vec3 h_meas;
  h_meas.x = 0.0;
  h_meas.y = vec_b.z;
  h_meas.z = -vec_b.y;
  QuatRotate(&q_b2g, &h_meas, &h_meas);
  Vec dx_plus;
  VEC_WRAP(correct->x.dx_plus, dx_plus);
  EstimatorFilterCorrectScalarSparse3(
      &h_meas, kAttitudeStateAttX, sigma_meas, meas_err.x, x_hat, &ud, &h, &k,
      &correct->x.dz, &correct->x.pzz, &dx_plus);

  h_meas.x = -vec_b.z;
  h_meas.y = 0.0;
  h_meas.z = vec_b.x;
  QuatRotate(&q_b2g, &h_meas, &h_meas);
  VEC_WRAP(correct->y.dx_plus, dx_plus);
  EstimatorFilterCorrectScalarSparse3(
      &h_meas, kAttitudeStateAttX, sigma_meas, meas_err.y, x_hat, &ud, &h, &k,
      &correct->y.dz, &correct->y.pzz, &dx_plus);

  h_meas.x = vec_b.y;
  h_meas.y = -vec_b.x;
  h_meas.z = 0.0;
  QuatRotate(&q_b2g, &h_meas, &h_meas);
  VEC_WRAP(correct->z.dx_plus, dx_plus);
  EstimatorFilterCorrectScalarSparse3(
      &h_meas, kAttitudeStateAttX, sigma_meas, meas_err.z, x_hat, &ud, &h, &k,
      &correct->z.dz, &correct->z.pzz, &dx_plus);
}

void AttitudeFilterPropagate(const Vec3 *gyro,
                             const EstimatorAttitudeParams *params,
                             EstimatorAttitudeFilterState *state) {
  // Update attitude estimate.
  Vec3 dpqr;
  Vec3LinComb(*g_sys.ts, gyro, -*g_sys.ts, &state->gyro_bias, &dpqr);
  Quat dq;
  AxisToQuat(&dpqr, &dq);
  QuatMultiply(&state->q_g2b, &dq, &state->q_g2b);
  QuatNormalize(&state->q_g2b, &state->q_g2b);

  // Update the uncertainty.
  // TODO: Sparse matrix multiplication should be used here to
  // improve runtime.
  MAT_INIT(kNumAttitudeStates, kNumAttitudeStates, F, {{0}});
  // Sensitivity to attitude error.
  *MatPtr(&F, kAttitudeStateAttX, kAttitudeStateAttX) = 1.0;
  *MatPtr(&F, kAttitudeStateAttY, kAttitudeStateAttY) = 1.0;
  *MatPtr(&F, kAttitudeStateAttZ, kAttitudeStateAttZ) = 1.0;

  // Sensitivity to gyro bias.
  Mat3 dcm_g2b;
  QuatToDcm(&state->q_g2b, &dcm_g2b);
  for (int32_t i = 0; i < 3; ++i) {
    for (int32_t j = 0; j < 3; ++j) {
      *MatPtr(&F, kAttitudeStateAttX + i, kAttitudeStateBiasGX + j) =
          *g_sys.ts * dcm_g2b.d[j][i];
    }
  }
  const double f_gyro_bias = 1.0 - *g_sys.ts / params->gyro_bias_time_constant;
  *MatPtr(&F, kAttitudeStateBiasGX, kAttitudeStateBiasGX) = f_gyro_bias;
  *MatPtr(&F, kAttitudeStateBiasGY, kAttitudeStateBiasGY) = f_gyro_bias;
  *MatPtr(&F, kAttitudeStateBiasGZ, kAttitudeStateBiasGZ) = f_gyro_bias;
  Vec3Scale(&state->gyro_bias, f_gyro_bias, &state->gyro_bias);

  VEC(kNumAttitudeStates + kNumAttitudeNoises, d);
  MAT(kNumAttitudeStates, kNumAttitudeStates + kNumAttitudeNoises, W);
  Vec ud;
  VEC_WRAP(state->ud, ud);
  UdKalmanTransitionMatrixMultiply(&F, &ud, &W, &d);

  // Populate remaining elements of W.
  for (int32_t i = 0; i < W.nr; ++i) {
    for (int32_t j = F.nc; j < W.nc; ++j) {
      *MatPtr(&W, i, j) = 0.0;
    }
  }
  // As we use the same magnitude for all gyroscope noises, we neglect
  // the \hat R_g2b dependence here.
  *MatPtr(&W, kAttitudeStateAttX, kNumAttitudeStates + kAttitudeNoiseGyroX) =
      1.0;
  *MatPtr(&W, kAttitudeStateAttY, kNumAttitudeStates + kAttitudeNoiseGyroY) =
      1.0;
  *MatPtr(&W, kAttitudeStateAttZ, kNumAttitudeStates + kAttitudeNoiseGyroZ) =
      1.0;
  *MatPtr(&W, kAttitudeStateBiasGX,
          kNumAttitudeStates + kAttitudeNoiseBiasGRwX) = 1.0;
  *MatPtr(&W, kAttitudeStateBiasGY,
          kNumAttitudeStates + kAttitudeNoiseBiasGRwY) = 1.0;
  *MatPtr(&W, kAttitudeStateBiasGZ,
          kNumAttitudeStates + kAttitudeNoiseBiasGRwZ) = 1.0;

  // Populate noise covariances in remaining elements of d.
  const double dt = *g_sys.ts;
  const double attitude_sqrt_Q = params->sigma_gyro_noise;
  const double gyro_bias_sqrt_Q = params->sigma_gyro_bias_instability;
  *VecPtr(&d, kNumAttitudeStates + kAttitudeNoiseGyroX) =
      attitude_sqrt_Q * attitude_sqrt_Q * dt;
  *VecPtr(&d, kNumAttitudeStates + kAttitudeNoiseGyroY) =
      attitude_sqrt_Q * attitude_sqrt_Q * dt;
  *VecPtr(&d, kNumAttitudeStates + kAttitudeNoiseGyroZ) =
      attitude_sqrt_Q * attitude_sqrt_Q * dt;
  *VecPtr(&d, kNumAttitudeStates + kAttitudeNoiseBiasGRwX) =
      gyro_bias_sqrt_Q * gyro_bias_sqrt_Q * dt;
  *VecPtr(&d, kNumAttitudeStates + kAttitudeNoiseBiasGRwY) =
      gyro_bias_sqrt_Q * gyro_bias_sqrt_Q * dt;
  *VecPtr(&d, kNumAttitudeStates + kAttitudeNoiseBiasGRwZ) =
      gyro_bias_sqrt_Q * gyro_bias_sqrt_Q * dt;

  // Update covariance.
  UdKalmanTimeUpdate(&d, &W, &ud);
}

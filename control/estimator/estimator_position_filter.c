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

#include "control/estimator/estimator_position_filter.h"

#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "common/c_math/kalman.h"
#include "common/c_math/linalg.h"
#include "common/macros.h"
#include "control/common.h"
#include "control/control_telemetry.h"
#include "control/estimator/estimator_filter.h"
#include "control/estimator/estimator_types.h"
#include "control/system_params.h"

COMPILE_ASSERT(ARRAYSIZE(((EstimatorPositionFilterState *)NULL)->ud) ==
                   (kNumPositionStates + 1) * kNumPositionStates / 2,
               ud_vector_must_be_correct_size);

// The position estimator makes use of a six state Kalman filter based
// on position and velocity corrections.
//
// The error states are defined by:
//
//   delta_pos_g = pos_g - \hat pos_g,
//   delta_vel_g = vel_g - \hat vel_g.
//
// Propagation Model:
// ------------------
//
// The true states evolve according to:
//   d/dt vel_g = accel_g,  d/dt pos_g = vel_g,
//
// which we approximate as:
//   vel_g(+) ~ vel_g + dt * accel_g,
//   pos_g(+) ~ pos_g + dt * vel_g + 0.5 * dt^2 * accel_g.
//
// The estimated states evolve according to:
//
//   \hat vel_g(+) = \hat vel_g + dt * \hat accel_g
//   \hat pos_g(+) = \hat pos_g + dt * \hat vel_g
//                   + 0.5 * dt^2 * \hat accel_g
//
// where \hat accel_g = \hat R_g2b^T * \hat acc + g_g.  Here \hat
// R_g2b and \hat acc are outputs of the attitude estimator.  The
// process noise for the model derives from assumed errors in \hat
// accel_g.

bool EstimatorPositionFilterValidateParams(
    const EstimatorPositionFilterParams *params) {
  assert(params != NULL);

  if (params->sigma_vel_g_0 <= 0.0) {
    assert(!(bool)"sigma_vel_g_0 must be positive.");
    return false;
  }

  if (params->gps_sigma_multiplier <= 1.0) {
    assert(!(bool)"gps_sigma_multiplier must be at least 1.0.");
    return false;
  }

  if (params->min_gps_sigma_vel_g <= 0.0) {
    assert(!(bool)"min_gps_sigma_vel_g must be positive.");
    return false;
  }

  if (params->min_gps_sigma_pos_g <= 0.0) {
    assert(!(bool)"min_gps_sigma_pos_g must be positive.");
    return false;
  }

  if (params->sigma_wing_accel_hover < 0.05 * g_sys.phys->g) {
    assert(!(bool)"sigma_wing_accel_hover is out of range.");
    return false;
  }

  if (params->sigma_wing_accel_dynamic < params->sigma_wing_accel_hover) {
    assert(!(bool)"sigma_wing_accel_dynamic is out of range.");
    return false;
  }

  return true;
}

void EstimatorPositionFilterInit(const EstimatorPositionFilterParams *params,
                                 EstimatorPositionFilterState *state) {
  assert(params != NULL && state != NULL);
  memset(state, 0, sizeof(*state));
  state->pos_g = kVec3Zero;
  state->vel_g = kVec3Zero;

  // Initialize covariance.
  VEC(kNumPositionStates, diag);
  *VecPtr(&diag, kPositionStateVelX) =
      params->sigma_vel_g_0 * params->sigma_vel_g_0;
  *VecPtr(&diag, kPositionStateVelY) =
      params->sigma_vel_g_0 * params->sigma_vel_g_0;
  *VecPtr(&diag, kPositionStateVelZ) =
      params->sigma_vel_g_0 * params->sigma_vel_g_0;
  *VecPtr(&diag, kPositionStatePosX) =
      g_sys.tether->length * g_sys.tether->length;
  *VecPtr(&diag, kPositionStatePosY) =
      g_sys.tether->length * g_sys.tether->length;
  *VecPtr(&diag, kPositionStatePosZ) =
      g_sys.tether->length * g_sys.tether->length;

  Vec ud;
  VEC_WRAP(state->ud, ud);
  UdKalmanStoreUpperTri(NULL, &diag, &ud);
}

void ApplyPositionErrorState(const Vec *x_hat,
                             EstimatorPositionFilterState *state) {
  state->vel_g.x += VecGet(x_hat, kPositionStateVelX);
  state->vel_g.y += VecGet(x_hat, kPositionStateVelY);
  state->vel_g.z += VecGet(x_hat, kPositionStateVelZ);
  state->pos_g.x += VecGet(x_hat, kPositionStatePosX);
  state->pos_g.y += VecGet(x_hat, kPositionStatePosY);
  state->pos_g.z += VecGet(x_hat, kPositionStatePosZ);
}

static void CorrectVelocityVector(const Vec3 *vel_meas_g,
                                  const Vec3 *sigma_meas_g, Vec *x_hat,
                                  EstimatorPositionFilterState *state,
                                  EstimatorPositionCorrection3 *correct) {
  Vec ud;
  VEC_WRAP(state->ud, ud);
  VEC(kNumPositionStates, h);
  VEC(kNumPositionStates, k);

  Vec3 h_meas = kVec3X;
  Vec dx_plus;
  VEC_WRAP(correct->x.dx_plus, dx_plus);
  EstimatorFilterCorrectScalarSparse3(
      &h_meas, kPositionStateVelX, sigma_meas_g->x,
      vel_meas_g->x - state->vel_g.x, x_hat, &ud, &h, &k, &correct->x.dz,
      &correct->x.pzz, &dx_plus);

  h_meas = kVec3Y;
  VEC_WRAP(correct->y.dx_plus, dx_plus);
  EstimatorFilterCorrectScalarSparse3(
      &h_meas, kPositionStateVelX, sigma_meas_g->y,
      vel_meas_g->y - state->vel_g.y, x_hat, &ud, &h, &k, &correct->y.dz,
      &correct->y.pzz, &dx_plus);

  h_meas = kVec3Z;
  VEC_WRAP(correct->z.dx_plus, dx_plus);
  EstimatorFilterCorrectScalarSparse3(
      &h_meas, kPositionStateVelX, sigma_meas_g->z,
      vel_meas_g->z - state->vel_g.z, x_hat, &ud, &h, &k, &correct->z.dz,
      &correct->z.pzz, &dx_plus);
}

void CorrectPositionVector(const Vec3 *pos_meas_g, const Vec3 *sigma_meas_g,
                           Vec *x_hat, EstimatorPositionFilterState *state,
                           EstimatorPositionCorrection3 *correct) {
  Vec ud;
  VEC_WRAP(state->ud, ud);
  VEC(kNumPositionStates, h);
  VEC(kNumPositionStates, k);

  Vec3 h_meas = kVec3X;
  Vec dx_plus;
  VEC_WRAP(correct->x.dx_plus, dx_plus);
  EstimatorFilterCorrectScalarSparse3(
      &h_meas, kPositionStatePosX, sigma_meas_g->x,
      pos_meas_g->x - state->pos_g.x, x_hat, &ud, &h, &k, &correct->x.dz,
      &correct->x.pzz, &dx_plus);

  h_meas = kVec3Y;
  VEC_WRAP(correct->y.dx_plus, dx_plus);
  EstimatorFilterCorrectScalarSparse3(
      &h_meas, kPositionStatePosX, sigma_meas_g->y,
      pos_meas_g->y - state->pos_g.y, x_hat, &ud, &h, &k, &correct->y.dz,
      &correct->y.pzz, &dx_plus);

  h_meas = kVec3Z;
  VEC_WRAP(correct->z.dx_plus, dx_plus);
  EstimatorFilterCorrectScalarSparse3(
      &h_meas, kPositionStatePosX, sigma_meas_g->z,
      pos_meas_g->z - state->pos_g.z, x_hat, &ud, &h, &k, &correct->z.dz,
      &correct->z.pzz, &dx_plus);
}

bool ApplyGpsVelocityCorrection(const EstimatorPositionGpsEstimate *gps,
                                const EstimatorPositionFilterParams *params,
                                Vec *x_hat, EstimatorPositionFilterState *state,
                                EstimatorPositionCorrection3 *correct) {
  if (gps->new_data && gps->wing_vel_valid) {
    Vec3 sigma = {
        fmax(params->min_gps_sigma_vel_g, gps->sigma_Vg.x),
        fmax(params->min_gps_sigma_vel_g, gps->sigma_Vg.y),
        fmax(params->min_gps_sigma_vel_g, gps->sigma_Vg.z),
    };
    Vec3Scale(&sigma, params->gps_sigma_multiplier, &sigma);
    CorrectVelocityVector(&gps->Vg, &sigma, x_hat, state, correct);
  }

  // Returns true while avionics faults no-update timer is less than the
  // maximum no-update count. This practice ensures that we do not switch to
  // GLAS between GPS updates (20 Hz).
  return gps->wing_vel_valid;
}

bool ApplyGpsPositionCorrection(const EstimatorPositionGpsEstimate *gps,
                                const EstimatorPositionFilterParams *params,
                                Vec *x_hat, EstimatorPositionFilterState *state,
                                EstimatorPositionCorrection3 *correct) {
  if (gps->new_data && gps->wing_pos_valid) {
    Vec3 sigma = {
        fmax(params->min_gps_sigma_pos_g, gps->sigma_Xg.x),
        fmax(params->min_gps_sigma_pos_g, gps->sigma_Xg.y),
        fmax(params->min_gps_sigma_pos_g, gps->sigma_Xg.z),
    };
    Vec3Scale(&sigma, params->gps_sigma_multiplier, &sigma);
    CorrectPositionVector(&gps->Xg, &sigma, x_hat, state, correct);
  }

  // Returns true while avionics faults no-update timer is less than the
  // maximum no-update count. This practice ensures that we do not switch to
  // GLAS between GPS updates (20 Hz).
  return gps->wing_pos_valid;
}

void EstimatorPositionFilterPropagate(
    const Mat3 *dcm_g2b, const Vec3 *acc, bool hover_flight_mode,
    const EstimatorPositionFilterParams *params,
    EstimatorPositionFilterState *state) {
  // Propagation step.
  Vec3 Ag;
  Mat3TransVec3Mult(dcm_g2b, acc, &Ag);
  Vec3Add(&Ag, &g_sys.phys->g_g, &Ag);

  double ts = *g_sys.ts;
  Vec3Axpy(0.5 * ts, &state->vel_g, &state->pos_g);
  Vec3Axpy(ts, &Ag, &state->vel_g);
  Vec3Axpy(0.5 * ts, &state->vel_g, &state->pos_g);

  // Update covariance.
  MAT_INIT(6, 6, F, {{0}});
  *MatPtr(&F, kPositionStateVelX, kPositionStateVelX) = 1.0;
  *MatPtr(&F, kPositionStateVelY, kPositionStateVelY) = 1.0;
  *MatPtr(&F, kPositionStateVelZ, kPositionStateVelZ) = 1.0;
  *MatPtr(&F, kPositionStatePosX, kPositionStateVelX) = ts;
  *MatPtr(&F, kPositionStatePosY, kPositionStateVelY) = ts;
  *MatPtr(&F, kPositionStatePosZ, kPositionStateVelZ) = ts;
  *MatPtr(&F, kPositionStatePosX, kPositionStatePosX) = 1.0;
  *MatPtr(&F, kPositionStatePosY, kPositionStatePosY) = 1.0;
  *MatPtr(&F, kPositionStatePosZ, kPositionStatePosZ) = 1.0;

  VEC(kNumPositionStates + kNumPositionNoises, d);
  MAT(kNumPositionStates, kNumPositionStates + kNumPositionNoises, W);
  Vec ud;
  VEC_WRAP(state->ud, ud);
  UdKalmanTransitionMatrixMultiply(&F, &ud, &W, &d);

  // Populate remaining elements of W.
  for (int32_t i = 0; i < W.nr; ++i) {
    for (int32_t j = F.nc; j < W.nc; ++j) {
      *MatPtr(&W, i, j) = 0.0;
    }
  }

  double half_ts_squared = 0.5 * ts * ts;
  *MatPtr(&W, kPositionStateVelX, kNumPositionStates + kPositionNoiseAccelX) =
      ts;
  *MatPtr(&W, kPositionStateVelY, kNumPositionStates + kPositionNoiseAccelY) =
      ts;
  *MatPtr(&W, kPositionStateVelZ, kNumPositionStates + kPositionNoiseAccelZ) =
      ts;
  *MatPtr(&W, kPositionStatePosX, kNumPositionStates + kPositionNoiseAccelX) =
      half_ts_squared;
  *MatPtr(&W, kPositionStatePosY, kNumPositionStates + kPositionNoiseAccelY) =
      half_ts_squared;
  *MatPtr(&W, kPositionStatePosZ, kNumPositionStates + kPositionNoiseAccelZ) =
      half_ts_squared;

  // Schedule acceleration noise based on flight mode.
  double sigma_Ag = hover_flight_mode ? params->sigma_wing_accel_hover
                                      : params->sigma_wing_accel_dynamic;
  *VecPtr(&d, kNumPositionStates + kPositionNoiseAccelX) = sigma_Ag * sigma_Ag;
  *VecPtr(&d, kNumPositionStates + kPositionNoiseAccelY) = sigma_Ag * sigma_Ag;
  *VecPtr(&d, kNumPositionStates + kPositionNoiseAccelZ) = sigma_Ag * sigma_Ag;

  // Update covariance.
  UdKalmanTimeUpdate(&d, &W, &ud);
}

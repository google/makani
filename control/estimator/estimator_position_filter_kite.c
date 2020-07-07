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

#include "control/estimator/estimator_position_filter_kite.h"

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

// Measurement Model:
// ------------------
//
// The measurements are taken to be direct observations of position
// and velocity of the kite body origin.  As a result, the following
// processing happens outside of the filter:
//
//   - The barometric altitude is corrected for its lever-arm,
//   - The GPS antenna position and velocity are resolved in g-coordinates,
//     and corrected for their lever-arm,
//   - A position estimate is computed from the GLAS encoder chain and tension
//     measurements.
//
// Biasing of the barometric altitude and GLAS sensors also takes
// place outside this filter.

static EstimatorVelocitySolutionType EstimatorPositionFilterCorrect(
    const EstimatorPositionBaroEstimate *baro,
    const EstimatorPositionGlasEstimate *glas,
    const EstimatorPositionGpsEstimate *gps_center,
    const EstimatorPositionGpsEstimate *gps_port,
    const EstimatorPositionGpsEstimate *gps_star, FlightMode flight_mode,
    const EstimatorPositionFilterParams *params,
    EstimatorPositionFilterState *state,
    EstimatorPositionCorrections *correct) {
  VEC_INIT(kNumPositionStates, x_hat, {0});

  bool gps_velocity_good = ApplyGpsVelocityCorrection(
      gps_center, params, &x_hat, state, &correct->gps_center_velocity);
  bool gps_position_good = ApplyGpsPositionCorrection(
      gps_center, params, &x_hat, state, &correct->gps_center_position);

  if (AnyCrosswindFlightMode(flight_mode)) {
    gps_velocity_good |= ApplyGpsVelocityCorrection(
        gps_port, params, &x_hat, state, &correct->gps_port_velocity);
    gps_position_good |= ApplyGpsPositionCorrection(
        gps_port, params, &x_hat, state, &correct->gps_port_position);

    gps_velocity_good |= ApplyGpsVelocityCorrection(
        gps_star, params, &x_hat, state, &correct->gps_star_velocity);
    gps_position_good |= ApplyGpsPositionCorrection(
        gps_star, params, &x_hat, state, &correct->gps_star_position);
  }

  if (baro->valid && params->baro_enabled) {
    Vec ud;
    VEC_WRAP(state->ud, ud);
    VEC(kNumPositionStates, h);
    VEC(kNumPositionStates, k);
    Vec dx_plus;
    VEC_WRAP(correct->baro.dx_plus, dx_plus);

    EstimatorFilterCorrectScalarSparse3(
        &kVec3Z, kPositionStatePosX, baro->sigma_Xg_z,
        baro->Xg_z - state->pos_g.z, &x_hat, &ud, &h, &k, &correct->baro.dz,
        &correct->baro.pzz, &dx_plus);
  }

  if (gps_position_good) {
    state->gps_position_timer_cycles = 0;
  } else if (state->gps_position_timer_cycles < INT32_MAX) {
    ++state->gps_position_timer_cycles;
  }
  if (state->gps_position_timer_cycles >= params->gps_position_timeout_cycles &&
      AnyCrosswindFlightMode(flight_mode) && glas->wing_pos_valid &&
      params->glas_enabled) {
    CorrectPositionVector(&glas->Xg, &glas->sigma_Xg, &x_hat, state,
                          &correct->glas_position);
  }

  ApplyPositionErrorState(&x_hat, state);

  if (gps_position_good || gps_velocity_good) {
    return kEstimatorVelocitySolutionTypeGps;
  } else if (glas->wing_pos_valid && params->glas_enabled) {
    return kEstimatorVelocitySolutionTypeGlas;
  } else {
    return kEstimatorVelocitySolutionTypeDeadReckoned;
  }
}

void EstimatorPositionFilterKiteStep(
    const Mat3 *dcm_g2b, const Vec3 *acc,
    const EstimatorPositionBaroEstimate *baro,
    const EstimatorPositionGlasEstimate *glas,
    const EstimatorPositionGpsEstimate *gps_center,
    const EstimatorPositionGpsEstimate *gps_port,
    const EstimatorPositionGpsEstimate *gps_star, FlightMode flight_mode,
    const EstimatorPositionFilterParams *params,
    EstimatorPositionFilterState *state, EstimatorPositionCorrections *correct,
    Vec3 *Xg, Vec3 *Vg, EstimatorVelocitySolutionType *vel_type) {
  *vel_type =
      EstimatorPositionFilterCorrect(baro, glas, gps_center, gps_port, gps_star,
                                     flight_mode, params, state, correct);
  EstimatorPositionFilterPropagate(
      dcm_g2b, acc, AnyHoverFlightMode(flight_mode), params, state);
  *Xg = state->pos_g;
  *Vg = state->vel_g;

  // Update telemetry.
  EstimatorTelemetry *est = GetEstimatorTelemetry();
  VEC(kNumPositionStates, cov);
  UdKalmanExtractCovariances(state->ud, &cov);
  est->cov_vel_g.x = VecGet(&cov, kPositionStateVelX);
  est->cov_vel_g.y = VecGet(&cov, kPositionStateVelY);
  est->cov_vel_g.z = VecGet(&cov, kPositionStateVelZ);
  est->cov_pos_g.x = VecGet(&cov, kPositionStatePosX);
  est->cov_pos_g.y = VecGet(&cov, kPositionStatePosY);
  est->cov_pos_g.z = VecGet(&cov, kPositionStatePosZ);
}

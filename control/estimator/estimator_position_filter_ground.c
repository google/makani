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

#include "control/estimator/estimator_position_filter_ground.h"

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
#include "control/estimator/estimator_types.h"
#include "control/ground_telemetry.h"
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
//   - The GPS antenna position and velocity are resolved in g-coordinates,
//     and corrected for their lever-arm,
//

static EstimatorVelocitySolutionType EstimatorPositionFilterCorrect(
    const EstimatorPositionGpsEstimate *gps_estimate,
    const EstimatorPositionFilterParams *params,
    EstimatorPositionFilterState *state,
    EstimatorPositionCorrections *correct) {
  VEC_INIT(kNumPositionStates, x_hat, {0});

  bool gps_velocity_good = ApplyGpsVelocityCorrection(
      gps_estimate, params, &x_hat, state, &correct->gps_center_velocity);
  bool gps_position_good = ApplyGpsPositionCorrection(
      gps_estimate, params, &x_hat, state, &correct->gps_center_position);

  if (gps_position_good) {
    state->gps_position_timer_cycles = 0;
  } else if (state->gps_position_timer_cycles < INT32_MAX) {
    ++state->gps_position_timer_cycles;
  }

  ApplyPositionErrorState(&x_hat, state);

  if (gps_position_good || gps_velocity_good) {
    return kEstimatorVelocitySolutionTypeGps;
  } else {
    return kEstimatorVelocitySolutionTypeDeadReckoned;
  }
}

void EstimatorPositionFilterGroundStep(
    const Mat3 *dcm_g2b, const Vec3 *acc,
    const EstimatorPositionGpsEstimate *gps_estimate,
    const EstimatorPositionFilterParams *params,
    EstimatorPositionFilterState *state, EstimatorPositionCorrections *correct,
    Vec3 *Xg, Vec3 *Vg, EstimatorVelocitySolutionType *vel_type) {
  *vel_type =
      EstimatorPositionFilterCorrect(gps_estimate, params, state, correct);
  EstimatorPositionFilterPropagate(dcm_g2b, acc, true, params, state);
  *Xg = state->pos_g;
  *Vg = state->vel_g;

  // Update telemetry.
  GroundEstimatorTelemetry *est = &GetGroundTelemetryMessage()->estimator;
  VEC(kNumPositionStates, cov);
  UdKalmanExtractCovariances(state->ud, &cov);
  est->cov_vel_g.x = VecGet(&cov, kPositionStateVelX);
  est->cov_vel_g.y = VecGet(&cov, kPositionStateVelY);
  est->cov_vel_g.z = VecGet(&cov, kPositionStateVelZ);
  est->cov_pos_g.x = VecGet(&cov, kPositionStatePosX);
  est->cov_pos_g.y = VecGet(&cov, kPositionStatePosY);
  est->cov_pos_g.z = VecGet(&cov, kPositionStatePosZ);
}

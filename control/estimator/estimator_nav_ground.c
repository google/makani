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

#include "control/estimator/estimator_nav_ground.h"

#include <assert.h>
#include <stdbool.h>
#include <string.h>

#include "common/macros.h"
#include "control/estimator/estimator_attitude_ground.h"
#include "control/estimator/estimator_nav.h"
#include "control/estimator/estimator_position_ground.h"
#include "control/ground_telemetry.h"

void EstimatorNavGroundInit(const SystemParams *system_params,
                            const EstimatorNavParams *params,
                            EstimatorNavGroundState *state) {
  assert(params != NULL && state != NULL);
  memset(state, 0, sizeof(*state));

  EstimatorNavInit(params, system_params->ground_frame.heading,
                   &state->estimator_nav_state);
  EstimatorAttitudeInit(&params->attitude, &state->attitude);
  EstimatorPositionGroundInit(system_params, &params->position,
                              &state->position);
}

// Run the position estimator.
void EstimatorNavGroundStep(
    bool initializing, const ImuData *imu, const GpsData *gps_data,
    const GpsCompassData *gps_compass, const FaultMask faults[],
    const SystemParams *system_params, const EstimatorNavParams *params,
    EstimatorNavGroundState *state, GroundEstimateMessage *ground_estimate) {
  assert(imu != NULL && gps_data != NULL && gps_compass != NULL &&
         faults != NULL && system_params != NULL && params != NULL &&
         state != NULL && ground_estimate != NULL);

  Quat q_g2p;
  Vec3 acc_p;
  EstimatorAttitudeCorrections correct;
  EstimatorVelocitySolutionType vel_type;

  EstimatorAttitudeGroundStep(initializing, imu, gps_compass,
                              &system_params->gs_imus[0], &params->attitude,
                              &faults[kSubsysGsAcc], &system_params->gs_gps,
                              &faults[kSubsysGsCompassAngles], &state->attitude,
                              &ground_estimate->pqr, &q_g2p, &acc_p, &correct);

  QuatToDcm(&q_g2p, &ground_estimate->dcm_g2p);

  EstimatorPositionGroundStep(
      &acc_p, &ground_estimate->pqr, &ground_estimate->dcm_g2p, gps_data,
      faults, system_params, &params->position, &state->position,
      &ground_estimate->Xg, &ground_estimate->Vg, &vel_type);

  GroundTelemetry *gt = GetGroundTelemetryMessage();
  gt->estimator.initializing = initializing;
  gt->estimator.attitude_corrections = correct;
  gt->estimator.cov_gyro_bias = state->attitude.filter.gyro_bias;
  EstimatorAttitudeGetCovariances(&state->attitude,
                                  &gt->estimator.cov_attitude_err,
                                  &gt->estimator.cov_gyro_bias);

  // TODO(b/138584949): Below we extract critical data from the
  // telemetry. Instead, refactor to provide a function
  // EstimatorPositionGetCovariances to get this data from the original source
  // of truth.
  double position_sigma_norm =
      Sqrt(gt->estimator.cov_pos_g.x + gt->estimator.cov_pos_g.y +
           gt->estimator.cov_pos_g.z);

  // TODO: Check how this behaves on initialization.
  ground_estimate->position_valid =
      position_sigma_norm < params->max_valid_position_sigma_norm;

  // TODO: Determine whether attitude estimate is valid.
  ground_estimate->attitude_valid = true;
}

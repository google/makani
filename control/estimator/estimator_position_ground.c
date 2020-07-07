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

#include "control/estimator/estimator_position_ground.h"

#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "common/c_math/coord_trans.h"
#include "common/c_math/filter.h"
#include "common/c_math/kalman.h"
#include "common/c_math/mat3.h"
#include "common/c_math/util.h"
#include "common/c_math/vec3.h"
#include "common/macros.h"
#include "control/common.h"
#include "control/control_params.h"
#include "control/control_telemetry.h"
#include "control/control_types.h"
#include "control/estimator/estimator_position_filter_ground.h"
#include "control/estimator/estimator_position_gps.h"
#include "control/ground_frame.h"
#include "control/ground_telemetry.h"
#include "control/perch_frame.h"
#include "control/sensor_types.h"
#include "control/system_params.h"
#include "control/system_types.h"

bool EstimatorPositionGroundValidateParams(
    const EstimatorPositionParams *params) {
  assert(params != NULL);

  // TODO: Validate filter params.
  return EstimatorPositionFilterValidateParams(&params->filter) &&
         EstimatorPositionGpsValidateParams(&params->gps);
}

void EstimatorPositionGroundInitializeGroundFrame(
    const Vec3 *origin_ecef, GroundStationPoseEstimate *ground_frame) {
  ground_frame->pos_ecef = *origin_ecef;
  CalcDcmEcefToNed(&ground_frame->pos_ecef, &ground_frame->dcm_ecef2g);
  ground_frame->valid = true;
}

void EstimatorPositionGroundInit(const SystemParams *system_params,
                                 const EstimatorPositionParams *params,
                                 EstimatorPositionGroundState *state) {
  assert(params != NULL && state != NULL);
  assert(EstimatorPositionGroundValidateParams(params));
  memset(state, 0, sizeof(*state));

  EstimatorPositionGroundInitializeGroundFrame(
      &system_params->ground_frame.origin_ecef, &state->ground_frame);

  EstimatorPositionFilterInit(&params->filter, &state->filter);
  EstimatorPositionGpsInit(&state->gps);
}

void EstimatorPositionGroundStep(const Vec3 *acc, const Vec3 *pqr,
                                 const Mat3 *dcm_g2p, const GpsData *gps_data,
                                 const FaultMask faults[],
                                 const SystemParams *system_params,
                                 const EstimatorPositionParams *params,
                                 EstimatorPositionGroundState *state, Vec3 *Xg,
                                 Vec3 *Vg,
                                 EstimatorVelocitySolutionType *vel_type) {
  assert(acc != NULL && pqr != NULL && dcm_g2p != NULL && gps_data != NULL &&
         faults != NULL && system_params != NULL && params != NULL &&
         state != NULL && Xg != NULL && Vg != NULL && vel_type != NULL);

  EstimatorPositionGpsEstimate gps_estimate;

  EstimatorPositionGpsStep(gps_data, &state->ground_frame, dcm_g2p, pqr,
                           &faults[kSubsysGsGpsPos], &faults[kSubsysGsGpsVel],
                           &system_params->gs_gps.primary_antenna_p,
                           &params->gps, &state->gps, &gps_estimate);

  EstimatorPositionCorrections correct;
  memset(&correct, 0, sizeof(correct));

  EstimatorPositionFilterGroundStep(dcm_g2p, acc, &gps_estimate,
                                    &params->filter, &state->filter, &correct,
                                    Xg, Vg, vel_type);

  GroundTelemetry *gt = GetGroundTelemetryMessage();
  gt->estimator.position_corrections = correct;
  gt->estimator.gps = gps_estimate;
}

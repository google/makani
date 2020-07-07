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

#include "control/estimator/estimator_ground_station.h"

#include <assert.h>
#include <stdbool.h>
#include <string.h>

#include "common/c_math/filter.h"
#include "common/c_math/vec3.h"
#include "control/control_params.h"
#include "control/control_telemetry.h"
#include "control/fault_detection/fault_detection_types.h"
#include "control/ground_frame.h"
#include "control/platform_frame.h"
#include "control/sensor_util.h"
#include "control/system_params.h"

void EstimatorGroundStationInit(EstimatorGroundStationState *state) {
  assert(state != NULL);
  memset(state, 0, sizeof(*state));
  state->position_fixed = false;
  state->pos_ecef = kVec3Zero;
  state->dcm_ecef2g = kMat3Identity;

  for (int32_t i = 0; i < kNumGroundStationModes; ++i) {
    state->mode_counts[i] = 0;
  }
  state->last_confirmed_mode = kGroundStationModeManual;
  state->last_valid_transform_stage = 0U;
}

static GroundStationMode DebounceGroundStationMode(
    GroundStationMode candidate, bool candidate_valid, int32_t num_debounce,
    EstimatorGroundStationState *state) {
  assert(0 <= (int32_t)candidate &&
         (int32_t)candidate < kNumGroundStationModes);

  for (int32_t i = 0; i < kNumGroundStationModes; ++i) {
    int32_t increment =
        (GroundStationMode)i == candidate && candidate_valid ? 1 : -1;
    state->mode_counts[i] =
        SaturateInt32(state->mode_counts[i] + increment, 0, num_debounce);
  }

  for (int32_t i = 0; i < kNumGroundStationModes; ++i) {
    if (state->mode_counts[i] >= num_debounce) {
      state->last_confirmed_mode = (GroundStationMode)i;
      break;
    }
  }
  return state->last_confirmed_mode;
}

void EstimatorVesselInit(EstimatorVesselState *state) {
  VesselEstimate *vessel = &state->last_valid;

  // Platform estimate.
  vessel->position_valid = false;
  vessel->attitude_valid = false;
  vessel->pos_g = kVec3Zero;
  vessel->vel_g = kVec3Zero;
  vessel->dcm_g2p = kMat3Identity;
  vessel->pqr = kVec3Zero;

  // Vessel estimate.
  vessel->dcm_g2v_valid = false;
  vessel->dcm_g2v = kMat3Identity;
}

void EstimatorVesselStep(const GroundEstimateMessage *ground_estimate,
                         const PerchAziEstimate *platform_azi,
                         const FaultMask *ground_estimator_faults,
                         EstimatorVesselState *state, VesselEstimate *vessel) {
  assert(ground_estimate != NULL && platform_azi != NULL &&
         ground_estimator_faults != NULL && state != NULL && vessel != NULL);

  bool position_valid = false;
  bool attitude_valid = false;

  if (!HasAnyFault(&ground_estimator_faults
                       [kFaultDetectionGroundStationEstimatorSignalPosition])) {
    // Pass-thru values.
    state->last_valid.pos_g = ground_estimate->Xg;
    state->last_valid.vel_g = ground_estimate->Vg;
    position_valid = true;
  }

  if (!HasAnyFault(&ground_estimator_faults
                       [kFaultDetectionGroundStationEstimatorSignalAttitude])) {
    // Pass-thru values.
    state->last_valid.dcm_g2p = ground_estimate->dcm_g2p;
    state->last_valid.pqr = ground_estimate->pqr;
    attitude_valid = true;
  }

  *vessel = state->last_valid;
  vessel->position_valid = position_valid;
  vessel->attitude_valid = attitude_valid;

  // Compute the vessel attitude from the platform attitude.
  Mat3 dcm_v2p;
  CalcDcmVesselToPlatform(platform_azi->angle, &dcm_v2p);
  Mat3Mult(&dcm_v2p, kTrans, &vessel->dcm_g2p, kNoTrans, &vessel->dcm_g2v);
  vessel->dcm_g2v_valid = vessel->attitude_valid && platform_azi->valid;
}

void EstimatorGroundStationStep(
    const GsGpsData *gs_gps, const FaultMask *gs_gps_fault,
    GroundStationMode gs_mode, const FaultMask *ground_station_fault,
    double detwist_angle, const FaultMask *detwist_fault,
    uint8_t gs_transform_stage, const GroundFrameParams *ground_frame_params,
    const EstimatorGroundStationParams *est_params,
    EstimatorGroundStationState *state, GroundStationEstimate *ground_station) {
  assert(gs_gps != NULL && gs_gps_fault != NULL &&
         ground_frame_params != NULL && state != NULL &&
         ground_station != NULL);

  if (!state->position_fixed) {
    GsGpsPosEcefToGsPosEcef(&ground_frame_params->origin_ecef, &kVec3Zero,
                            ground_frame_params->heading, &state->pos_ecef,
                            &state->dcm_ecef2g);
    state->position_fixed = true;
  }

  ground_station->pose.valid = state->position_fixed;
  ground_station->pose.pos_ecef = state->pos_ecef;
  ground_station->pose.dcm_ecef2g = state->dcm_ecef2g;
  ground_station->detwist_angle = detwist_angle;
  ground_station->detwist_angle_valid = !HasAnyFault(detwist_fault);

  ground_station->mode =
      DebounceGroundStationMode(gs_mode, !HasAnyFault(ground_station_fault),
                                est_params->num_debounce, state);

  if (!HasAnyFault(ground_station_fault)) {
    ground_station->transform_stage = gs_transform_stage;
  } else {
    ground_station->transform_stage = state->last_valid_transform_stage;
  }
  state->last_valid_transform_stage = ground_station->transform_stage;
}

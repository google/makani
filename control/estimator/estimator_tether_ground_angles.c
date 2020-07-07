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

#include "control/estimator/estimator_tether_ground_angles.h"

#include <math.h>
#include "common/c_math/geometry.h"
#include "common/c_math/mat3.h"
#include "common/c_math/vec3.h"
#include "control/ground_frame.h"
#include "control/sensor_util.h"
#include "control/system_params.h"

void EstimatorTetherGroundAnglesInit(EstimatorTetherGroundAnglesState *state) {
  assert(state != NULL);
  memset(state, 0, sizeof(*state));
  state->last_valid_elevation_g = 0.0;
  state->last_valid_elevation_p = 0.0;
  state->last_valid_detwist_angle = 0.0;
  state->last_detwist_angle = 0.0;
  state->last_valid_accumulated_detwist_angle = 0.0;
}

void EstimatorTetherGroundAnglesStep(
    bool initializing, GroundStationModel gs_model,
    const GroundStationEstimate *ground_station, const WinchEstimate *winch,
    const VesselEstimate *vessel, const WinchParams *winch_params,
    const EstimatorTetherGroundAnglesParams *params,
    const EncodersEstimate *encoders, double detwist_ele,
    EstimatorTetherGroundAnglesState *state,
    TetherGroundAnglesEstimate *tether_angles) {
  enum TetherAngleSource {
    kTetherAngleSourceInvalid,
    kTetherAngleSourceLevelwind,
    kTetherAngleSourceGsg,
    kTetherAngleSourceGsgEle
  } source = kTetherAngleSourceInvalid;

  switch (gs_model) {
    case kGroundStationModelGSv2:

      switch (ground_station->mode) {
        case kGroundStationModeManual:
        case kGroundStationModeReel:
          source = kTetherAngleSourceLevelwind;
          break;

        case kGroundStationModeHighTension:
          source = kTetherAngleSourceGsg;
          break;

        case kGroundStationModeTransform:
          switch (ground_station->transform_stage) {
            case 0:  // Initialization stage.
              // Stage 0 can take place when the levelwind is engaged
              // (in TransformUp) and when it is not engaged (in TransformDown).
              // This also leads to several corner cases.
              // Setting it to invalid for now.
              source = kTetherAngleSourceInvalid;
              break;
            case 1:  // Drum rotates when the gsg faces the kite.
            case 2:  // Platform azimuth slew.
            case 3:  // Tether is disengaged from levelwind, and off racetrack.
              source = kTetherAngleSourceGsg;
              break;
            case 4:  // Drum rotates, tether is disengaged during the process.
              source = kTetherAngleSourceInvalid;
              break;
            default:
              assert(false);
              source = kTetherAngleSourceInvalid;
              break;
          }
          break;

        default:
        case kNumGroundStationModes:
          assert(false);
          break;
      }
      break;

    case kGroundStationModelTopHat:
      source = kTetherAngleSourceGsgEle;
      break;

    case kGroundStationModelGSv1:
      // Not currently supported.
      source = kTetherAngleSourceInvalid;
      break;

    case kNumGroundStationModels:
    case kGroundStationModelForceSigned:
    default:
      assert(false);
      source = kTetherAngleSourceInvalid;
      break;
  }

  if (initializing) {
    state->last_detwist_angle = ground_station->detwist_angle;
  }

  switch (source) {
    case kTetherAngleSourceInvalid:
      tether_angles->elevation_valid = false;
      break;
    case kTetherAngleSourceGsgEle:
      // This case is only exercised by the tophat, onshore.
      tether_angles->elevation_valid = encoders->gsg_ele_valid;
      if (tether_angles->elevation_valid) {
        state->last_valid_elevation_g = encoders->gsg.ele;
        state->last_valid_elevation_p = encoders->gsg.ele;
      }
      break;
    case kTetherAngleSourceLevelwind:
      tether_angles->elevation_valid = encoders->levelwind_ele_valid;
      if (tether_angles->elevation_valid) {
        CalcTetherAnglesFromLevelwind(&vessel->dcm_g2p, encoders->levelwind_ele,
                                      &state->last_valid_elevation_p,
                                      &state->last_valid_elevation_g);
      }
      break;
    case kTetherAngleSourceGsg:
      tether_angles->elevation_valid =
          (encoders->gsg_azi_valid && encoders->gsg_ele_valid &&
           encoders->perch_azi_valid && winch->valid &&
           ground_station->detwist_angle_valid);

      if (tether_angles->elevation_valid) {
        CalcTetherAnglesFromGsg(
            &vessel->dcm_g2p, winch->position / winch_params->r_drum,
            detwist_ele, ground_station->detwist_angle, encoders->gsg.azi,
            encoders->gsg.ele, params->hold_cone_half_angle,
            params->detwist_axis_offset, &state->last_valid_elevation_p,
            &state->last_valid_elevation_g, &state->last_valid_detwist_angle,
            &state->last_detwist_angle,
            &state->last_valid_accumulated_detwist_angle);
      }
      break;
    default:
      assert(false);
      break;
  }
  tether_angles->elevation_g = state->last_valid_elevation_g;
  tether_angles->elevation_p = state->last_valid_elevation_p;
  tether_angles->departure_detwist_angle = state->last_valid_detwist_angle;
  tether_angles->accumulated_detwist_angle =
      state->last_valid_accumulated_detwist_angle;
}

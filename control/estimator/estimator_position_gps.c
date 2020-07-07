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

#include "control/estimator/estimator_position_gps.h"

#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <string.h>

#include "common/c_math/coord_trans.h"
#include "common/c_math/filter.h"
#include "common/c_math/mat3.h"
#include "common/c_math/util.h"
#include "common/c_math/vec3.h"
#include "common/macros.h"
#include "control/common.h"
#include "control/control_telemetry.h"
#include "control/ground_frame.h"
#include "control/sensor_types.h"
#include "control/sensor_util.h"
#include "control/system_params.h"
#include "control/system_types.h"

bool EstimatorPositionGpsValidateParams(
    const EstimatorPositionGpsParams *params) {
  assert(params != NULL);

  if (params->outage_hold_num < 0) {
    assert(!(bool)"outage_hold_num must be non-negative.");
    return false;
  }

  if (params->min_disagreement_distance_hover <= 0.0) {
    assert(!(bool)"min_disagreement_distance_hover must be positive.");
    return false;
  }

  if (params->min_disagreement_distance <=
      params->min_disagreement_distance_hover) {
    assert(!(
        bool)"min_disagreement_distance must be greater than "
             "min_disagreement_distance_hover.");
    return false;
  }

  if (params->disagreement_hysteresis_ratio <= 0.0) {
    assert(!(bool)"disagreement_hysteresis_ratio must be positive.");
    return false;
  }

  if (params->sigma_hysteresis <= 0.0) {
    assert(!(bool)"sigma_hysteresis must be positive.");
    return false;
  }

  if (params->relative_sigma_threshold <= 0.0) {
    assert(!(bool)"relative_sigma_threshold must be positive.");
    return false;
  }

  return true;
}

void EstimatorPositionGpsInit(EstimatorPositionGpsState *state) {
  assert(state != NULL);
  memset(state, 0, sizeof(*state));
  state->outage_timer = 0;
}

static bool GpsEstimatesDisagree(
    double min_disagreement_distance,
    const EstimatorPositionGpsEstimate gps_estimates[]) {
  Vec3 tmp;
  Vec3Sub(&gps_estimates[kWingGpsReceiverCrosswind].Xg,
          &gps_estimates[kWingGpsReceiverHover].Xg, &tmp);
  FabsVec3(&tmp, &tmp);

  return (
      tmp.x > (min_disagreement_distance +
               2.0 * hypot(gps_estimates[kWingGpsReceiverCrosswind].sigma_Xg.x,
                           gps_estimates[kWingGpsReceiverHover].sigma_Xg.x)) ||
      tmp.y > (min_disagreement_distance +
               2.0 * hypot(gps_estimates[kWingGpsReceiverCrosswind].sigma_Xg.y,
                           gps_estimates[kWingGpsReceiverHover].sigma_Xg.y)) ||
      tmp.z > (min_disagreement_distance +
               2.0 * hypot(gps_estimates[kWingGpsReceiverCrosswind].sigma_Xg.z,
                           gps_estimates[kWingGpsReceiverHover].sigma_Xg.z)));
}

// Select one of the two center GPS estimates to fly with.
WingGpsReceiverLabel EstimatorPositionGpsSelectCenterEstimate(
    FlightMode flight_mode, WingGpsReceiverLabel last_valid,
    const EstimatorPositionGpsEstimate gps_estimates[], const Vec3 *Xg,
    const EstimatorPositionGpsParams *params) {
  // The estimator is hard-coded to use only these two GPS receivers.
  assert(last_valid == kWingGpsReceiverCrosswind ||
         last_valid == kWingGpsReceiverHover);
  assert(gps_estimates != NULL && Xg != NULL && params != NULL);

  double min_disagreement_distance =
      (flight_mode == kFlightModePerched || AnyHoverFlightMode(flight_mode))
          ? params->min_disagreement_distance_hover
          : params->min_disagreement_distance;

  WingGpsReceiverLabel selection = last_valid;

  if (gps_estimates[kWingGpsReceiverCrosswind].wing_pos_valid &&
      gps_estimates[kWingGpsReceiverHover].wing_pos_valid) {
    double crosswind_sigma_norm =
        Vec3Norm(&gps_estimates[kWingGpsReceiverCrosswind].sigma_Xg);
    double hover_sigma_norm =
        Vec3Norm(&gps_estimates[kWingGpsReceiverHover].sigma_Xg);

    if ((flight_mode == kFlightModePerched ||
         AnyHoverFlightMode(flight_mode)) &&
        hover_sigma_norm < crosswind_sigma_norm) {
      // Prefer the Hover receiver in hover flight modes. If it has a better
      // sigma than the crosswind GPS in hover flight modes, switch to it
      // without considering disagreement or sigma ratios.
      selection = kWingGpsReceiverHover;

    } else if (GpsEstimatesDisagree(min_disagreement_distance, gps_estimates)) {
      // If the receivers disagree by an amount that is significantly
      // larger than their reported uncertainties, pick the solution
      // closer to our current position.
      Vec3 tmp;
      Vec3Sub(&gps_estimates[kWingGpsReceiverCrosswind].Xg, Xg, &tmp);
      double crosswind_dist = Vec3Norm(&tmp);

      Vec3Sub(&gps_estimates[kWingGpsReceiverHover].Xg, Xg, &tmp);
      double hover_dist = Vec3Norm(&tmp);

      // Add hysteresis to avoid chatter if our estimate lies between
      // the two GPS reported values.
      if (last_valid == kWingGpsReceiverCrosswind) {
        hover_dist +=
            params->disagreement_hysteresis_ratio * min_disagreement_distance;
      } else {
        crosswind_dist +=
            params->disagreement_hysteresis_ratio * min_disagreement_distance;
      }
      selection = (crosswind_dist <= hover_dist) ? kWingGpsReceiverCrosswind
                                                 : kWingGpsReceiverHover;

    } else {
      // If one receiver has significantly worse reported uncertainties,
      // choose the other receiver.

      // Add hysteresis to avoid chatter if reported uncertainties
      // are very small, or large but nearly equal.
      if (last_valid == kWingGpsReceiverCrosswind) {
        hover_sigma_norm *= params->relative_sigma_threshold;
        hover_sigma_norm += params->sigma_hysteresis;
      } else {
        crosswind_sigma_norm *= params->relative_sigma_threshold;
        crosswind_sigma_norm += params->sigma_hysteresis;
      }
      selection = (crosswind_sigma_norm <= hover_sigma_norm)
                      ? kWingGpsReceiverCrosswind
                      : kWingGpsReceiverHover;
    }

  } else if (gps_estimates[kWingGpsReceiverCrosswind].wing_pos_valid) {
    // If only Crosswind is valid, choose Crosswind.
    selection = kWingGpsReceiverCrosswind;

  } else if (gps_estimates[kWingGpsReceiverHover].wing_pos_valid) {
    // If only Hover is valid, choose Hover.
    selection = kWingGpsReceiverHover;
  }

  return selection;
}

// Calculate GPS pos/vel measurement.
static void CalcXgVgGps(const GpsData *gps, const Vec3 *gs_pos_ecef,
                        const Mat3 *dcm_ecef2g, const Mat3 *dcm_g2parent,
                        const Vec3 *pqr, const Vec3 *gps_antenna_pos_parent,
                        Vec3 *Xg_gps, Vec3 *Vg_gps) {
  PoseTransform(dcm_ecef2g, gs_pos_ecef, &gps->pos, Xg_gps);
  Mat3Vec3Mult(dcm_ecef2g, &gps->vel, Vg_gps);

  // Correct Xg_gps for antenna offset.
  Vec3 tmp;
  Mat3TransVec3Mult(dcm_g2parent, Vec3Cross(pqr, gps_antenna_pos_parent, &tmp),
                    &tmp);
  Vec3Sub(Vg_gps, &tmp, Vg_gps);
  Vec3Sub(Xg_gps, Mat3TransVec3Mult(dcm_g2parent, gps_antenna_pos_parent, &tmp),
          Xg_gps);
}

// Mark GPS data as invalid for a window of time after no solution is computed.
// See https://wiki.makanipower.com/index.php/GPS_Analysis.
static void ExtendGpsOutage(int32_t outage_hold_num,
                            const FaultMask *gps_pos_fault,
                            const FaultMask *gps_vel_fault,
                            int32_t *outage_timer, bool *use_gps_pos,
                            bool *use_gps_vel) {
  *use_gps_pos = !HasAnyFault(gps_pos_fault) && *outage_timer == 0;
  // TODO: Replace the kFaultTypeOutOfRange test here with a
  // check on the position standard deviations.
  *use_gps_vel = (!HasAnyFault(gps_vel_fault) && *outage_timer == 0 &&
                  !HasFault(kFaultTypeOutOfRange, gps_pos_fault));

  // Update outage timer if a thrown error is reported.
  if (HasFault(kFaultTypeThrownError, gps_pos_fault) ||
      HasFault(kFaultTypeThrownError, gps_vel_fault)) {
    *outage_timer = outage_hold_num;
  } else {
    *outage_timer = MaxInt32(*outage_timer - 1, 0);
  }
}

// Find GPS position, velocity, and associated measurements.
void EstimatorPositionGpsStep(
    const GpsData *gps_data, const GroundStationPoseEstimate *ground_station,
    const Mat3 *dcm_g2parent, const Vec3 *pqr,
    const FaultMask *wing_gps_pos_fault, const FaultMask *wing_gps_vel_fault,
    const GpsParams *gps_params, const EstimatorPositionGpsParams *params,
    EstimatorPositionGpsState *state, EstimatorPositionGpsEstimate *gps) {
  assert(gps_data != NULL && ground_station != NULL && dcm_g2parent != NULL &&
         pqr != NULL && wing_gps_pos_fault != NULL &&
         wing_gps_vel_fault != NULL && gps_params != NULL && params != NULL &&
         state != NULL && gps != NULL);
  CalcXgVgGps(gps_data, &ground_station->pos_ecef, &ground_station->dcm_ecef2g,
              dcm_g2parent, pqr, &gps_params->pos, &gps->Xg, &gps->Vg);

  ConvSigmaEcefToLocal(&gps_data->pos_sigma, &gps_data->vel_sigma,
                       &ground_station->dcm_ecef2g, &gps->sigma_Xg,
                       &gps->sigma_Vg);

  ExtendGpsOutage(params->outage_hold_num, wing_gps_pos_fault,
                  wing_gps_vel_fault, &state->outage_timer,
                  &gps->wing_pos_valid, &gps->wing_vel_valid);

  if (!ground_station->valid) {
    gps->wing_pos_valid = false;
    gps->wing_vel_valid = false;
  }
  gps->new_data = gps_data->new_data;
}

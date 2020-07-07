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

#include "control/hover/hover_mode.h"

#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "common/c_math/mat3.h"
#include "common/c_math/util.h"
#include "common/c_math/vec3.h"
#include "control/actuator_util.h"
#include "control/common.h"
#include "control/control_params.h"
#include "control/control_telemetry.h"
#include "control/ground_frame.h"
#include "control/hover/hover_types.h"
#include "control/system_params.h"
#include "control/system_types.h"
#include "control/vessel_frame.h"

// Checks if the controller can switch to kFlightModeHoverAscend.  Only
// allow this flight mode if at least one of the final proximity
// sensors is active. Furthermore, do not launch if the perch is not
// reasonably well aligned with the downwind direction.
//
// TODO: The wind_g.dir_f should be biased in the direction
// of perch_azi in very low wind conditions (consider the case of zero
// wind).
//
// TODO: We might want to require that the wind and perch are
// well-aligned for a certain length of time before signaling
// readiness.
static bool IsReadyForHoverAscend(const StateEstimate *state_est,
                                  const HoverModeParams *params) {
  int32_t ascend_gates = 0;

  if (!(state_est->winch.proximity_valid)) {
    // We formerly required that the proximity sensor be triggered
    // as a gate on HoverAscend.  However, the winch typically
    // releases ~1 cm of tether when the winch is e-stopped, as is
    // typical on a test day, which is enough to deactivate the
    // proximity sensor and prevent ascent.  Instead, we now
    // require that the proximity sensor be simply valid, and we
    // additionally gate on tension to ensure that the tether is not
    // slack.  See b/113601392.
    ascend_gates |= (1 << kHoverAscendGateProximityValid);
  }
  if (!(state_est->tether_force_b.valid &&
        state_est->tether_force_b.tension_f > params->min_tension_for_ascend)) {
    ascend_gates |= (1 << kHoverAscendGateTension);
  }
  double wind_dir_p =
      AziGToV(state_est->wind_g.dir_f, &state_est->vessel.dcm_g2v);
  double perch_wind_misalignment = fabs(
      Wrap(state_est->perch_azi.angle +
               params->aligned_perch_azi_to_wind_angle_for_ascend - wind_dir_p,
           -PI, PI));
  if (!(state_est->perch_azi.valid && state_est->wind_g.valid &&
        perch_wind_misalignment <
            params->max_perch_wind_misalignment_for_ascend)) {
    ascend_gates |= (1 << kHoverAscendGatePerchWindMisalignment);
  }

  // Update telemetry.
  ControlTelemetry *ct = GetControlTelemetry();
  ct->flight_mode_gates[kFlightModeHoverAscend] = ascend_gates;

  return ascend_gates == 0;
}

// Checks if the controller can switch to kFlightModeHoverPayOut.
// Only allow this flight mode if the ascent is complete, the gain
// ramp is done, the wing is close to the the altitude and yaw
// set-points, and it has a low yaw rate.
static bool IsReadyForHoverPayOut(const StateEstimate *state_est,
                                  const Vec3 *angles_error,
                                  bool ascent_complete, bool gain_ramp_done,
                                  const HoverModeParams *params) {
  int32_t pay_out_gates = 0;

  if (!ascent_complete) {
    pay_out_gates |= (1 << kHoverPayOutGateAscentComplete);
  }

  if (!gain_ramp_done) {
    pay_out_gates |= (1 << kHoverPayOutGateGainRampDone);
  }

  Vec3 Xp;
  PoseTransform(&state_est->vessel.dcm_g2p, &state_est->vessel.pos_g,
                &state_est->Xg, &Xp);

  if (Xp.z >= params->max_z_for_pay_out) {
    pay_out_gates |= (1 << kHoverPayOutGateZPosition);
  }

  if (fabs(angles_error->z) >= params->max_yaw_angle_error_for_pay_out) {
    pay_out_gates |= (1 << kHoverPayOutGateYawError);
  }

  if (fabs(state_est->pqr_f.z) >= params->max_yaw_rate_for_pay_out) {
    pay_out_gates |= (1 << kHoverPayOutGateYawRate);
  }

  // Update telemetry.
  ControlTelemetry *ct = GetControlTelemetry();
  ct->flight_mode_gates[kFlightModeHoverPayOut] = pay_out_gates;

  return pay_out_gates == 0;
}

// Checks if the controller can switch to kFlightModeHoverFullLength.
static bool IsReadyForHoverFullLength(const StateEstimate *state_est,
                                      bool forcing_detwist_turn) {
  int32_t full_length_gates = 0;

  if (GetSystemParams()->gs_model == kGroundStationModelGSv2 &&
      state_est->gs_mode != kGroundStationModeHighTension) {
    full_length_gates |= (1 << kHoverFullLengthGateGroundStationMode);
  }

  if (forcing_detwist_turn) {
    full_length_gates |= (1 << kHoverFullLengthGateForceDetwistTurn);
  }

  // Update telemetry.
  ControlTelemetry *ct = GetControlTelemetry();
  ct->flight_mode_gates[kFlightModeHoverFullLength] = full_length_gates;

  return full_length_gates == 0;
}

// TODO: Although it doesn't matter after the absolute value, the
// azimuth here is negative about the z_g-axis. Make it positive.
static double CalcAbsAzimuthError(const Vec3 *accel_start_pos_g,
                                  const Vec3 *Xg) {
  return fabs(
      Wrap(VecGToAzimuth(accel_start_pos_g) - VecGToAzimuth(Xg), -PI, PI));
}

// Checks if the controller can switch to kFlightModeHoverPrepTransformGsUp.
static bool IsReadyForHoverPrepTransformGsUp(const StateEstimate *state_est,
                                             const HoverModeParams *params) {
  int32_t prep_transform_up_gates = 0;

  if (!(*g_cont.flight_plan == kFlightPlanStartDownwind ||
        *g_cont.flight_plan == kFlightPlanDisengageEngage ||
        *g_cont.flight_plan == kFlightPlanTurnKey ||
        *g_cont.flight_plan == kFlightPlanHighHover)) {
    prep_transform_up_gates |= (1 << kHoverPrepTransformGsUpGateFlightPlan);
  }

  if (GetSystemParams()->gs_model == kGroundStationModelGSv2 &&
      state_est->gs_mode != kGroundStationModeReel) {
    prep_transform_up_gates |=
        (1 << kHoverPrepTransformGsUpGateGroundStationMode);
  }

  if (!state_est->winch.valid ||
      state_est->winch.position < params->min_winch_pos_for_transform_gs_up) {
    prep_transform_up_gates |= (1 << kHoverPrepTransformGsUpGateWinchPosition);
  }

  // Update telemetry.
  ControlTelemetry *ct = GetControlTelemetry();
  ct->flight_mode_gates[kFlightModeHoverPrepTransformGsUp] =
      prep_transform_up_gates;

  return prep_transform_up_gates == 0;
}

static bool IsTetherElevationReadyForGsTransform(
    const FlightStatus *flight_status, double good_elevation_since,
    double tether_elevation_error, const HoverModeParams *params) {
  // Stay for a while to build up confidence.
  double readiness_duration =
      flight_status->flight_mode_time - good_elevation_since;

  // Start the transform if the tether elevation sustains.
  return (readiness_duration > params->min_gs_transform_staging_time) &&
         (fabs(tether_elevation_error) <
          params->max_tether_elevation_error_for_gs_transform_kickoff);
}

// Checks if the controller can switch to kFlightModeHoverPrepTransformGsUp.
static bool IsReadyForHoverTransformGsUp(double azi_error,
                                         const FlightStatus *flight_status,
                                         double good_elevation_since,
                                         double tether_elevation_error,
                                         const StateEstimate *state_est,
                                         const Vec3 *wing_pos_g_cmd,
                                         const HoverModeParams *params) {
  int32_t transform_up_gates = 0;

  if (GetSystemParams()->gs_model == kGroundStationModelGSv2) {
    if (!IsTetherElevationReadyForGsTransform(flight_status,
                                              good_elevation_since,
                                              tether_elevation_error, params)) {
      transform_up_gates |= (1 << kHoverTransformGsUpGateTetherElevation);
    }

    double z_err = wing_pos_g_cmd->z - state_est->Xg.z;
    if (fabs(z_err) > params->max_z_error_for_transform) {
      transform_up_gates |= (1 << kHoverTransformGsUpGateZError);
    }
  }

  if (fabs(azi_error) > params->max_azimuth_error_for_transform) {
    transform_up_gates |= (1 << kHoverTransformGsUpGateAzimuthError);
  }

  // Update telemetry.
  ControlTelemetry *ct = GetControlTelemetry();
  ct->flight_mode_gates[kFlightModeHoverTransformGsUp] = transform_up_gates;

  return transform_up_gates == 0;
}

// Checks if the controller can switch to kFlightModeHoverAccel.  Only
// allow accelerations under reasonable attitude, angular rate,
// position, velocity, and tension constraints.
static bool IsReadyForHoverAccel(const StateEstimate *state_est,
                                 const Vec3 *accel_start_pos_g,
                                 const Vec3 *angles_error,
                                 bool forcing_detwist_turn,
                                 const HoverModeParams *params) {
  double abs_azimuth_err =
      CalcAbsAzimuthError(accel_start_pos_g, &state_est->Xg);
  double z_err = accel_start_pos_g->z - state_est->Xg.z;

  int32_t accel_gates = 0;

  if (!(*g_cont.flight_plan == kFlightPlanStartDownwind ||
        *g_cont.flight_plan == kFlightPlanTurnKey)) {
    accel_gates |= (1 << kHoverAccelGateFlightPlan);
  }

  if (fabs(angles_error->x) >= params->max_roll_angle_error_for_accel) {
    accel_gates |= (1 << kHoverAccelGateRollError);
  }

  if (fabs(angles_error->z) >= params->max_yaw_angle_error_for_accel) {
    accel_gates |= (1 << kHoverAccelGateYawError);
  }

  if (fabs(state_est->pqr_f.z) >= params->max_yaw_rate_for_accel) {
    accel_gates |= (1 << kHoverAccelGateYawRate);
  }

  if (Vec3Norm(&state_est->pqr_f) >= params->max_angular_rate_for_accel) {
    accel_gates |= (1 << kHoverAccelGateAngularRate);
  }

  if (abs_azimuth_err >= params->max_azimuth_error_for_accel) {
    accel_gates |= (1 << kHoverAccelGateAzimuthError);
  }

  if (fabs(z_err) >= params->max_z_error_for_accel) {
    accel_gates |= (1 << kHoverAccelGateZError);
  }

  if (Vec3Norm(&state_est->Vg) >= params->max_speed_for_accel) {
    accel_gates |= (1 << kHoverAccelGateSpeed);
  }

  if (fabs(state_est->Vb.y) >= params->max_body_y_vel_for_accel) {
    accel_gates |= (1 << kHoverAccelGateYVelocity);
  }

  if (!state_est->tether_force_b.valid ||
      state_est->tether_force_b.tension_f <= params->min_tension_for_accel) {
    accel_gates |= (1 << kHoverAccelGateTension);
  }

  if (GetSystemParams()->gs_model == kGroundStationModelGSv2 &&
      state_est->gs_mode != kGroundStationModeHighTension) {
    accel_gates |= (1 << kHoverAccelGateGroundStationMode);
  }

  // Prevent mode switch during forced detwist turn.
  if (forcing_detwist_turn) {
    accel_gates |= (1 << kHoverAccelGateForceDetwistTurn);
  }

  // Update telemetry.
  ControlTelemetry *ct = GetControlTelemetry();
  ct->flight_mode_gates[kFlightModeHoverAccel] = accel_gates;

  return accel_gates == 0;
}

// Checks if the controller can switch to kFlightModeHoverTransformGsDown.
static bool IsReadyForHoverPrepTransformGsDown(
    const StateEstimate *state_est, const FlightStatus *flight_status,
    bool forcing_detwist_turn, const HoverModeParams *params) {
  int32_t prep_transform_down_gates = 0;

  // Disable reel-in for a short period following transition-out.
  if (flight_status->flight_mode == kFlightModeHoverTransOut &&
      flight_status->flight_mode_time < params->min_time_in_trans_out) {
    prep_transform_down_gates |=
        (1 << kHoverPrepTransformGsDownGateTimeInTransOut);
  }

  if (GetSystemParams()->gs_model == kGroundStationModelGSv2 &&
      state_est->gs_mode != kGroundStationModeHighTension) {
    prep_transform_down_gates |=
        (1 << kHoverPrepTransformGsDownGateGroundStationMode);
  }

  if (forcing_detwist_turn) {
    prep_transform_down_gates |=
        (1 << kHoverPrepTransformGsDownGateForceDetwistTurn);
  }

  // Update telemetry.
  ControlTelemetry *ct = GetControlTelemetry();
  ct->flight_mode_gates[kFlightModeHoverPrepTransformGsDown] =
      prep_transform_down_gates;

  return prep_transform_down_gates == 0;
}

// Checks if the controller can switch to kFlightModeHoverTransformGsDown.
static bool IsReadyForHoverTransformGsDown(
    double azi_error, const FlightStatus *flight_status,
    double good_elevation_since, double tether_elevation_error,
    bool forcing_detwist_turn, const StateEstimate *state_est,
    const Vec3 *wing_pos_g_cmd, const HoverModeParams *params) {
  int32_t transform_down_gates = 0;

  if (GetSystemParams()->gs_model == kGroundStationModelGSv2) {
    if (!IsTetherElevationReadyForGsTransform(flight_status,
                                              good_elevation_since,
                                              tether_elevation_error, params)) {
      transform_down_gates |= (1 << kHoverTransformGsDownGateTetherElevation);
    }

    double z_err = wing_pos_g_cmd->z - state_est->Xg.z;
    if (fabs(z_err) > params->max_z_error_for_transform) {
      transform_down_gates |= (1 << kHoverTransformGsDownGateZError);
    }
  }

  if (fabs(azi_error) > params->max_azimuth_error_for_transform) {
    transform_down_gates |= (1 << kHoverTransformGsDownGateAzimuthError);
  }

  // Prevent mode switch during forced detwist turn.
  if (forcing_detwist_turn) {
    transform_down_gates |= (1 << kHoverTransformGsDownGateForceDetwistTurn);
  }

  // Update telemetry.
  ControlTelemetry *ct = GetControlTelemetry();
  ct->flight_mode_gates[kFlightModeHoverTransformGsDown] = transform_down_gates;

  return transform_down_gates == 0;
}

// Checks if the controller can switch to kFlightModeHoverReelIn.
//
// TODO: For completely autonomous operation, we will need
// criteria to begin reel-in.
static bool IsReadyForHoverReelIn(const StateEstimate *state_est) {
  int32_t reel_in_gates = 0;

  // Disable reel-in in flight plans that do not use the winch.
  if (!(*g_cont.flight_plan == kFlightPlanDisengageEngage ||
        *g_cont.flight_plan == kFlightPlanHighHover ||
        *g_cont.flight_plan == kFlightPlanLaunchPerch ||
        *g_cont.flight_plan == kFlightPlanStartDownwind ||
        *g_cont.flight_plan == kFlightPlanTurnKey)) {
    reel_in_gates |= (1 << kHoverReelInGateFlightPlan);
  }

  if (GetSystemParams()->gs_model == kGroundStationModelGSv2 &&
      state_est->gs_mode != kGroundStationModeReel) {
    reel_in_gates |= (1 << kHoverReelInGateGroundStationMode);
  }

  // Update telemetry.
  ControlTelemetry *ct = GetControlTelemetry();
  ct->flight_mode_gates[kFlightModeHoverReelIn] = reel_in_gates;

  return reel_in_gates == 0;
}

// Checks if the controller can switch to kFlightModeHoverDescend.
static bool IsReadyForHoverDescend(const StateEstimate *state_est,
                                   const HoverModeParams *params) {
  int32_t descend_gates = 0;

  // TODO: Determine what to do if the proximity flag is
  // not valid.
  if (!state_est->winch.proximity) {
    descend_gates |= (1 << kHoverDescendGateProximity);
  }
  const Gs02Params *gs02_params = &GetSystemParams()->ground_station.gs02;
  bool target_valid;

  // TODO: Check vessel->valid.
  double desired_platform_azi =
      CalcHoverGsTargetAzimuthReel(&state_est->vessel.pos_g,
                                   &state_est->vessel.dcm_g2v, &state_est->Xg,
                                   gs02_params, &target_valid) -
      gs02_params->boom_azimuth_p;

  if (!state_est->perch_azi.valid || !target_valid ||
      fabs(Wrap(state_est->perch_azi.angle - desired_platform_azi, -PI, PI)) >
          params->max_platform_misalignment_for_descend) {
    descend_gates |= (1 << kHoverDescendGateAbovePerch);
  }

  // Update telemetry.
  ControlTelemetry *ct = GetControlTelemetry();
  ct->flight_mode_gates[kFlightModeHoverDescend] = descend_gates;

  return descend_gates == 0;
}

// Checks if the controller can switch to kFlightModePerched.  The
// only way to get into kFlightModePerched is through
// kFlightModePilotHover.
static bool IsReadyForPerched(void) {
  int32_t perched_gates = 0;

  // Do not allow perched mode from autonomous controller.
  perched_gates |= (1 << kHoverPerchedGateDisabled);

  // Update telemetry.
  ControlTelemetry *ct = GetControlTelemetry();
  ct->flight_mode_gates[kFlightModePerched] = perched_gates;

  return perched_gates == 0;
}

bool HoverModeIsReadyFor(
    FlightMode proposed_flight_mode, const FlightStatus *flight_status,
    const StateEstimate *state_est, bool ascent_complete, bool gain_ramp_done,
    bool forcing_detwist_turn, double good_tether_elevation_since,
    double tether_elevation_error, const Vec3 *angles_error,
    const Vec3 *accel_start_pos_g, double waypoint_azi_error,
    const Vec3 *wing_pos_g_cmd, const HoverModeParams *params) {
  if (proposed_flight_mode == kFlightModeHoverAscend) {
    return IsReadyForHoverAscend(state_est, params);

  } else if (proposed_flight_mode == kFlightModeHoverPayOut) {
    return IsReadyForHoverPayOut(state_est, angles_error, ascent_complete,
                                 gain_ramp_done, params);

  } else if (proposed_flight_mode == kFlightModeHoverFullLength) {
    return IsReadyForHoverFullLength(state_est, forcing_detwist_turn);

  } else if (proposed_flight_mode == kFlightModeHoverPrepTransformGsUp) {
    return IsReadyForHoverPrepTransformGsUp(state_est, params);

  } else if (proposed_flight_mode == kFlightModeHoverTransformGsUp) {
    return IsReadyForHoverTransformGsUp(
        waypoint_azi_error, flight_status, good_tether_elevation_since,
        tether_elevation_error, state_est, wing_pos_g_cmd, params);

  } else if (proposed_flight_mode == kFlightModeHoverAccel) {
    return IsReadyForHoverAccel(state_est, accel_start_pos_g, angles_error,
                                forcing_detwist_turn, params);

  } else if (proposed_flight_mode == kFlightModeHoverPrepTransformGsDown) {
    return IsReadyForHoverPrepTransformGsDown(state_est, flight_status,
                                              forcing_detwist_turn, params);

  } else if (proposed_flight_mode == kFlightModeHoverTransformGsDown) {
    return IsReadyForHoverTransformGsDown(
        waypoint_azi_error, flight_status, good_tether_elevation_since,
        tether_elevation_error, forcing_detwist_turn, state_est, wing_pos_g_cmd,
        params);

  } else if (proposed_flight_mode == kFlightModeHoverReelIn) {
    return IsReadyForHoverReelIn(state_est);

  } else if (proposed_flight_mode == kFlightModeHoverDescend) {
    return IsReadyForHoverDescend(state_est, params);

  } else if (proposed_flight_mode == kFlightModePerched) {
    return IsReadyForPerched();
  }

  return false;
}

void HoverModeCheckTetherElevationForGsTransform(
    const HoverPathParams *path_params, const HoverModeParams *params,
    const TetherGroundAnglesEstimate *tether_ground_angles,
    const FlightStatus *flight_status, HoverTetherElevationState *state) {
  if (flight_status->flight_mode_first_entry) {
    state->good_elevation_since = flight_status->flight_mode_time;
  }
  // Reset the timer whenever elevation is invalid or out of range.
  if (!tether_ground_angles->elevation_valid ||
      (fabs(tether_ground_angles->elevation_p -
            path_params->target_transform_tether_elevation) >
       params->max_tether_elevation_error_for_gs_transform_staging)) {
    state->good_elevation_since = flight_status->flight_mode_time;
  }
}

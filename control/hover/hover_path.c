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

#include "control/hover/hover_path.h"

#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <string.h>

#include "common/c_math/filter.h"
#include "common/c_math/geometry.h"
#include "common/c_math/util.h"
#include "common/macros.h"
#include "control/common.h"
#include "control/control_params.h"
#include "control/control_telemetry.h"
#include "control/crosswind/crosswind_playbook.h"
#include "control/ground_frame.h"
#include "control/sensor_util.h"
#include "control/system_params.h"
#include "control/system_types.h"
#include "control/tether_util.h"

void HoverPathInit(const Vec3 *wing_pos_g, const Vec3 *previous_perched_pos_g,
                   const HoverPathParams *params, HoverPathState *state) {
  assert(wing_pos_g != NULL && params != NULL && state != NULL);
  assert(HoverPathValidateParams(params));

  memset(state, 0, sizeof(*state));
  state->raw_wing_pos_g_cmd = *wing_pos_g;
  state->wing_pos_g_cmd_z1 = *wing_pos_g;
  state->wing_vel_g_cmd_z1 = kVec3Zero;
  for (int32_t i = 0; i < ARRAYSIZE(state->wing_vel_g_cmd_zs); ++i) {
    state->wing_vel_g_cmd_zs[i] = kVec3Zero;
  }
  state->fixed_pos_g = *wing_pos_g;
  if (previous_perched_pos_g != NULL) {
    state->perched_pos_g = *previous_perched_pos_g;
  } else {
    // Record the current position as the perched position.
    state->perched_pos_g = *wing_pos_g;
  }

  for (int32_t i = 0; i < ARRAYSIZE(state->tether_elevation_error_zs); ++i) {
    state->tether_elevation_error_zs[i] = 0.0;
  }

  state->transout_azi = 0.0;
  state->transout_Xg_z_min = -params->transout_min_altitude;
  state->int_kite_elevation = 0.0;
}

bool HoverPathValidateParams(const HoverPathParams *params) {
  if (!(params->max_acceleration_g.x > 0.0 &&
        params->max_acceleration_g.y > 0.0 &&
        params->max_acceleration_g.z > 0.0)) {
    assert(!(bool)"Each component of max_acceleration_g must be positive.");
    return false;
  }

  if (!(params->max_normal_radial_speed > 0.0)) {
    assert(!(bool)"max_normal_radial_speed must be positive.");
    return false;
  }

  if (!(params->max_normal_tangential_speed > 0.0)) {
    assert(!(bool)"max_normal_radial_speed must be positive.");
    return false;
  }

  if (!(0.0 < params->max_ascend_perch_z_speed &&
        params->max_ascend_perch_z_speed <
            params->max_ascend_near_perch_z_speed &&
        params->max_ascend_near_perch_z_speed <
            params->max_ascend_normal_z_speed &&
        params->max_ascend_normal_z_speed < params->max_accel_z_speed)) {
    assert(!(
        bool)"The ascend z speeds must obey: 0 < max_ascend_perch_z_speed"
             " < max_ascend_near_perch_z_speed < max_ascend_normal_z_speed"
             " < max_accel_z_speed.");
    return false;
  }

  if (!(0.0 < params->max_descend_perch_z_speed &&
        params->max_descend_perch_z_speed <
            params->max_descend_near_perch_z_speed &&
        params->max_descend_near_perch_z_speed <
            params->max_descend_normal_z_speed)) {
    assert(!(
        bool)"The descend z speeds must obey: 0 < max_descend_perch_z_speed"
             " < max_descend_near_perch_z_speed < max_descend_normal_z_speed");
    return false;
  }

  if (!(params->ascend_offset_g_z < 0.0)) {
    assert(!(bool)"ascend_offset_g_z must be negative.");
    return false;
  }

  if (!(params->descend_offset_g_z > 0.0)) {
    assert(!(bool)"descend_offset_g_z must be positive.");
    return false;
  }

  if (!(params->velocity_cutoff_freq > 0.0)) {
    assert(!(bool)"velocity_cutoff_freq must be positive.");
    return false;
  }

  if (!(params->velocity_damping_ratio > 0.0)) {
    assert(!(bool)"velocity_damping_ratio must be positive.");
    return false;
  }

  if (!(fabs(params->target_reel_tether_elevation) < 30.0 * PI / 180.0)) {
    assert(!(bool)"target_reel_tether_elevation must be reasonably small.");
    return false;
  }

  if (!(fabs(params->target_transform_tether_elevation) < 30.0 * PI / 180.0)) {
    assert(
        !(bool)"target_transform_tether_elevation must be reasonably small.");
    return false;
  }

  if (!(0.0 <= params->reel_short_tether_length &&
        params->reel_short_tether_length < params->reel_long_tether_length &&
        params->reel_long_tether_length <= g_sys.tether->length)) {
    assert(!(
        bool)"reel_short_tether_length must be positive and shorter than"
             " reel_long_tether_length, which must be shorter than the full"
             " tether length.");
    return false;
  }

  if (!(fabs(params->reel_azimuth_offset) < 45.0 * PI / 180.0)) {
    assert(!(bool)"reel_azimuth_offset must be reasonably small.");
    return false;
  }

  if (!(0.0 <= params->reel_elevation_min &&
        params->reel_elevation_min <= params->reel_elevation_max &&
        params->reel_elevation_max < PI / 2.0)) {
    assert(!(
        bool)"reel_elevation_min must be positive and smaller than reel"
             " elevation max.");
    return false;
  }

  if (!(0.0 <= params->accel_start_elevation &&
        params->accel_start_elevation < PI / 2.0)) {
    assert(!(
        bool)"accel_start_elevation must be non-negative and reasonably "
             "small.");
    return false;
  }

  if (!(params->transout_vg_cmd_crossfade_duration >= 0.0)) {
    assert(!(bool)"transout_vg_cmd_crossfade_length must be non-negative.");
  }

  return true;
}

bool HoverPathIsAscentComplete(FlightMode flight_mode,
                               const HoverPathParams *params,
                               const StateEstimate *state_est) {
  assert(0 <= flight_mode && flight_mode < kNumFlightModes);
  assert(params != NULL && state_est != NULL);

  return flight_mode == kFlightModeHoverAscend &&
         state_est->tether_ground_angles.elevation_valid &&
         state_est->tether_ground_angles.elevation_p >=
             params->target_above_perch_tether_elevation;
}

double HoverPathGetZOffsetFromPerchedPosition(const Vec3 *wing_pos_g,
                                              const HoverPathState *state) {
  return wing_pos_g->z - state->perched_pos_g.z;
}

void HoverPathGetWaypointError(const Vec3 *wing_pos_g,
                               const HoverPathState *state,
                               Vec3 *waypoint_g_error) {
  Vec3Sub(&state->raw_wing_pos_g_cmd, wing_pos_g, waypoint_g_error);
}

double HoverPathGetWaypointAzimuthError(const Vec3 *wing_pos_g,
                                        const HoverPathState *state) {
  return Wrap(
      VecGToAzimuth(&state->raw_wing_pos_g_cmd) - VecGToAzimuth(wing_pos_g),
      -PI, PI);
}

static void CalcAccelStartAndCrosswindLoopCenterPosition(
    const WindEstimate *wind_g, const Playbook *playbook,
    const Vec3 *perched_pos_g, const HoverPathParams *params,
    Vec3 *accel_start_pos_g, Vec3 *loop_center_pos_g) {
  assert(perched_pos_g != NULL && params != NULL && accel_start_pos_g != NULL);
  assert(loop_center_pos_g != NULL);

  PlaybookEntry playbook_entry;
  GetPlaybookEntry(playbook, NULL, wind_g->speed_f_playbook, 0.0,
                   &playbook_entry);

  double loop_center_azimuth =
      GetPlaybookEntryAzimuthWithLimits(wind_g->dir_f, &playbook_entry);

  double half_loop_angle =
      Asin(playbook_entry.path_radius_target / g_sys.tether->length);
  double path_azi_offset = AdjustHalfConeAziOffsetForElevation(
      half_loop_angle, playbook_entry.elevation);

  // Assumes loop direction is clockwise.
  // TODO: Allow for arbitrary direction.
  double accel_start_azimuth =
      Wrap(loop_center_azimuth - path_azi_offset, -PI, PI);

  SphToVecG(accel_start_azimuth, params->accel_start_elevation,
            g_sys.tether->length, accel_start_pos_g);
  SphToVecG(loop_center_azimuth, playbook_entry.elevation, g_sys.tether->length,
            loop_center_pos_g);
}

void HoverPathCalcElevationLimits(double norm_wing_pos_g,
                                  const HoverPathParams *params,
                                  double *elevation_min,
                                  double *elevation_max) {
  assert(norm_wing_pos_g >= 0.0);
  assert(params != NULL && elevation_min != NULL && elevation_min != NULL);

  *elevation_min =
      Crossfade(params->launch_perch_elevation_min, params->reel_elevation_min,
                norm_wing_pos_g, params->reel_short_tether_length,
                params->reel_long_tether_length);

  *elevation_max =
      Crossfade(params->launch_perch_elevation_max, params->reel_elevation_max,
                norm_wing_pos_g, params->reel_short_tether_length,
                params->reel_long_tether_length);
}

static void CalcAscendDescendPosition(const Vec3 *vessel_pos_g,
                                      double wind_dir_f, double offset_g_z,
                                      const HoverPathParams *params,
                                      Vec3 *pos_g) {
  assert(-PI <= wind_dir_f && wind_dir_f <= PI);

  CylToVecG(wind_dir_f + params->reel_azimuth_offset,
            Vec3XyNorm(&params->perched_wing_pos_p),
            params->perched_wing_pos_p.z, pos_g);

  Vec3Add(pos_g, vessel_pos_g, pos_g);
  pos_g->z += offset_g_z;
}

static void CalcAscendPosition(const Vec3 *vessel_pos_g, double wind_dir_f,
                               const HoverPathParams *params,
                               Vec3 *ascend_pos_g) {
  CalcAscendDescendPosition(vessel_pos_g, wind_dir_f, params->ascend_offset_g_z,
                            params, ascend_pos_g);
}

static void CalcPositionCommandFromTransOutState(const HoverPathState *state,
                                                 Vec3 *Xg_cmd) {
  // The intent here is to hold the trans-out position command at the
  // point defined by the given azimuth and altitude.  To get a radial
  // component, we place the command on the tether sphere, although in
  // reality due to the catenary the kite will remain strictly inside
  // of the sphere.  This is not a problem as we do not have closed-
  // loop control on the radial position.

  double r = Sqrt(Square(GetSystemParams()->tether.length) -
                  Square(state->transout_Xg_z_min));

  // Protect against unreasonable values of transout_Xg_z_max (see b/130840325).
  assert(r > 0.25 * GetSystemParams()->tether.length);
  r = fmax(r, 0.5 * GetSystemParams()->tether.length);

  // TODO: Consider enforcing minimum and maximum limits on
  // elevation in trans-out.
  CylToVecG(state->transout_azi, r, state->transout_Xg_z_min, Xg_cmd);
}

// Calculates the position command for the wing during and immediately after
// trans-out. For the first few seconds as the vehicle continues upward, the
// inertial position is used as the position command to zero the position error
// and prevent large angle commands. The altitude is latched at the highest
// altitude seen during this deceleration phase and the azimuth is latched to
// prevent the wing from sliding back downwind.
static void CalcTransOutPosition(double flight_mode_time, const Vec3 *Xg,
                                 Vec3 *Xg_cmd, HoverPathState *state) {
  // Track the maximum altitude of the vehicle (minimum Xg->z position).
  if (flight_mode_time < 2.0) {
    state->transout_Xg_z_min = fmin(state->transout_Xg_z_min, Xg->z);
  }

  // Track the most extreme azimuth of the vehicle.
  // TODO(b/37000252): This is only valid for one loop direction.
  double azi = VecGToAzimuth(Xg);
  if (flight_mode_time < 1.0) {
    state->transout_azi = azi;
  } else {
    state->transout_azi += fmin(Wrap(azi - state->transout_azi, -PI, PI), 0.0);
  }

  CalcPositionCommandFromTransOutState(state, Xg_cmd);
}

static bool IsPrepTransformMode(FlightMode flight_mode) {
  return (flight_mode == kFlightModeHoverPrepTransformGsUp ||
          flight_mode == kFlightModeHoverPrepTransformGsDown);
}

static bool IsTransformMode(FlightMode flight_mode) {
  return (flight_mode == kFlightModeHoverTransformGsUp ||
          flight_mode == kFlightModeHoverTransformGsDown);
}

static bool IsTetherElevationControlActive(FlightMode flight_mode) {
  return (flight_mode == kFlightModeHoverPayOut ||
          flight_mode == kFlightModeHoverReelIn ||
          IsPrepTransformMode(flight_mode) || IsTransformMode(flight_mode));
}

// Control the tether elevation angle in PayOut/ReelIn by feeding back
// to the (kite) elevation_cmd.
static double ControlTetherElevation(
    double tether_elevation_error_f, bool tether_elevation_valid,
    double elevation_ff, const Vec3 *tether_anchor_g, const Vec3 *wing_pos_g,
    const FlightStatus *flight_status, const HoverPathParams *params,
    HoverPathState *state) {
  IntegratorMode int_mode;

  if (!IsTetherElevationControlActive(flight_status->flight_mode)) {
    int_mode = kIntegratorModeReset;
  } else if (tether_elevation_valid) {
    int_mode = kIntegratorModeIntegrate;
  } else {
    int_mode = kIntegratorModeHold;
  }

  // The following feature delays turning on feedback in PrepTransformDown until
  // the feed-forward action has had a chance to work.  This is to avoid winding
  // up the integrator on tether elevation error.
  // TODO: Move the constants in the following two
  // expressions into the configuration system.
  const bool still_slewing = (Vec3Distance(&state->raw_wing_pos_g_cmd,
                                           &state->wing_pos_g_cmd_z1) > 10.0);
  const bool near_target = (tether_elevation_valid &&
                            (fabs(tether_elevation_error_f) < DegToRad(3.0)));

  // While we are far from the target, use feed-forward to get closer.
  if (flight_status->flight_mode == kFlightModeHoverPrepTransformGsDown &&
      !near_target &&
      (flight_status->flight_mode_first_entry || still_slewing)) {
    int_mode = kIntegratorModeHold;
  }

  // Initialize the elevation integrator to make sure the integrator
  // produces an offset to elevation_ff that meets the current elevation.
  // This smooths out the position command when switching from a flight
  // mode that does not use this integrator. In particular, it helps on
  // on transitions from PilotHover back to autonomous mode. We specifically
  // do not pre-populate the integrator on the switch from HoverTransOut, as
  // the kite is typically far from its target location and pure feed-forward
  // is effective in quickly closing this gap.
  if (flight_status->flight_mode_first_entry &&
      !IsTetherElevationControlActive(flight_status->last_flight_mode) &&
      flight_status->last_flight_mode != kFlightModeHoverDescend &&
      flight_status->last_flight_mode != kFlightModeHoverTransOut) {
    // NOTE: The following will initialize the integrator
    // such that the resulting raw_wing_pos_g_cmd matches the
    // _current_ wing altitude.  To avoid a step change in command
    // we'd rather match the value of raw_wing_pos_g_cmd. A
    // complication is that CalcPayOutReelInPosition uses the norm of
    // wing_pos_g in the call to SphToVecG. So we would have to use
    // both the norm of wing_pos_g, and the current value of
    // raw_wing_pos_g_cmd here. For now we avoid these complications and just
    // match the current wing position.

    // Correct for changes made to reel_pos_g in
    // CalcPayOutReelInPosition after the conversion from spherical
    // coordinates.
    Vec3 tether_anchor_to_wing_g;
    Vec3Sub(wing_pos_g, tether_anchor_g, &tether_anchor_to_wing_g);

    const double kite_elevation = VecGToElevation(&tether_anchor_to_wing_g);
    const double elevation_deficit =
        elevation_ff +
        params->reel_tether_elevation_pid.kp * tether_elevation_error_f -
        kite_elevation;
    state->int_kite_elevation = -elevation_deficit;
  }

  const double fb =
      Pid(tether_elevation_error_f, 0.0, *g_sys.ts, int_mode,
          &params->reel_tether_elevation_pid, &state->int_kite_elevation);

  GetHoverTelemetry()->int_kite_elevation = state->int_kite_elevation;
  return fb;
}

static void CalcFullLengthPosition(const Vec3 *fixed_pos_g,
                                   const Vec3 *hover_accel_pos_g,
                                   Vec3 *full_length_pos_g) {
  if (*g_cont.flight_plan == kFlightPlanHoverInPlace) {
    *full_length_pos_g = *fixed_pos_g;
  } else {
    *full_length_pos_g = *hover_accel_pos_g;
  }
}

static double ConvertPayoutToArclength(double payout) {
  // Approximate the initial tether anchor point in the perch frame
  // by assuming the winch position is reeled in a full tether length
  // and the platform azimuth is 0, so that the platform and ground
  // frames are coincident and aligned.
  Vec3 tether_anchor_p;
  CalcTetherAnchorPoint(-GetSystemParams()->tether.length, &kMat3Identity,
                        &kVec3Zero, &tether_anchor_p);
  // The measured payout differs from the catenary arclength by this
  // offset [m], approximately.
  Vec3Sub(&GetSystemParams()->ground_station.gs02.perched_wing_pos_p,
          &tether_anchor_p, &tether_anchor_p);
  const double payout_offset = Vec3Norm(&tether_anchor_p);
  assert(payout_offset > 0.0);
  return payout + payout_offset;
}

// Use catenary theory to calculate the relative position of a cable
// end-point given the arclength, horizontal tension, linear density,
// and slope at the start.  See b/118584540#comment31.
static void ConvertArclengthToPosition(double arclength,
                                       double horizontal_tension_cmd,
                                       double tether_elevation_cmd_g,
                                       double *radial_position,
                                       double *vertical_position,
                                       double *elevation) {
  // Sanitize inputs.
  arclength = fmax(0.0, arclength);
  horizontal_tension_cmd = fmax(1e3, horizontal_tension_cmd);
  tether_elevation_cmd_g =
      Saturate(tether_elevation_cmd_g, -PI / 2.0, PI / 2.0);

  assert(radial_position != NULL);
  assert(vertical_position != NULL);
  assert(elevation != NULL);

  // Catenary curvature is given by horizontal tension and linear
  // weight density.  See https://en.wikipedia.org/wiki/Catenary#Analysis.
  const double a =
      (horizontal_tension_cmd /
       (GetSystemParams()->tether.linear_density * -g_sys.phys->g));

  // Solve for r0 by requiring a particular slope at r=0.
  const double r0 = -a * asinh(tan(-tether_elevation_cmd_g));

  // Solve for z0 by requiring that the curve intercept the origin.
  // No additional z offset is needed here; the location of the tether
  // origin is accounted for by tether_anchor_g in
  // CalcPayOutReelInPosition.
  const double z0 = -a * cosh(-r0 / a);

  // Solve for the radial position at the end of the tether, using the
  // arclength.
  *radial_position = asinh(arclength / a + sinh(-r0 / a)) * a + r0;

  // Finally, evaluate the height at the end of the tether, using the
  // catenary equation; and then convert to an elevation angle.
  *vertical_position = a * cosh((*radial_position - r0) / a) + z0;

  *elevation = atan2(-*vertical_position, *radial_position);
}

static void ConvertPayoutToPosition(
    double payout, double horizontal_tension_cmd, double tether_elevation_cmd_g,
    double *radial_position, double *vertical_position, double *elevation) {
  const double arclength = ConvertPayoutToArclength(payout);
  ConvertArclengthToPosition(arclength, horizontal_tension_cmd,
                             tether_elevation_cmd_g, radial_position,
                             vertical_position, elevation);
}

double HoverPathCalcTetherElevationCommand(const FlightStatus *flight_status,
                                           double payout,
                                           const HoverPathParams *params) {
  double tether_elevation_cmd =
      ((flight_status->flight_mode == kFlightModeHoverPrepTransformGsUp ||
        flight_status->flight_mode == kFlightModeHoverPrepTransformGsDown ||
        flight_status->flight_mode == kFlightModeHoverTransformGsUp ||
        flight_status->flight_mode == kFlightModeHoverTransformGsDown)
           ? params->target_transform_tether_elevation
           : params->target_reel_tether_elevation);

  tether_elevation_cmd = Crossfade(params->target_above_perch_tether_elevation,
                                   tether_elevation_cmd, payout, 0.0,
                                   params->max_payout_for_perching_prep);

  return tether_elevation_cmd;
}

// Calculates the position command for the wing during pay-out and
// reel-in.  This position is set to be the same distance from the
// ground origin as the wing actually is.  The azimuth is set to be a
// specific, payout dependent, offset from downwind, and the elevation
// is set according to elevation_cmd with payout dependent
// saturations.
static void CalcPayOutReelInPosition(double elevation_cmd,
                                     double norm_reel_pos_g, double wind_dir_f,
                                     const Vec3 *tether_anchor_g,
                                     const Vec3 *fixed_pos_g,
                                     const FlightStatus *flight_status,
                                     const HoverPathParams *params,
                                     HoverPathState *state, Vec3 *reel_pos_g) {
  assert(-PI / 2.0 <= elevation_cmd && elevation_cmd < PI / 2.0);
  assert(-PI <= wind_dir_f && wind_dir_f <= PI);

  // Here we convert the elevation command to a height using the
  // elevation_cmd and a spherical radius (norm_reel_pos_g).
  // Check that the spherical radius is never less than what it would be
  // when perched (essentially the tether arc length when perched).
  const double norm_perched_pos_g = ConvertPayoutToArclength(0.0);
  norm_reel_pos_g = fmax(norm_reel_pos_g, norm_perched_pos_g);

  double azi_reel =
      wind_dir_f + Crossfade(params->reel_azimuth_offset,
                             params->transform_azimuth_offset, norm_reel_pos_g,
                             params->reel_short_tether_length,
                             params->reel_long_tether_length);

  // Freeze the azimuth on first entry to the PrepTransform modes.
  if (IsPrepTransformMode(flight_status->flight_mode) &&
      flight_status->flight_mode_first_entry) {
    state->transform_azi = wind_dir_f + params->transform_azimuth_offset;
  }

  if (IsPrepTransformMode(flight_status->flight_mode) ||
      IsTransformMode(flight_status->flight_mode)) {
    azi_reel = state->transform_azi;
  }

  double elevation_min, elevation_max;
  HoverPathCalcElevationLimits(norm_reel_pos_g, params, &elevation_min,
                               &elevation_max);

  double ele_reel = Saturate(elevation_cmd, elevation_min, elevation_max);

  // Convert azimuth and elevation to a position in ground
  // coordinates.
  SphToVecG(azi_reel, ele_reel, norm_reel_pos_g, reel_pos_g);
  Vec3Add(reel_pos_g, tether_anchor_g, reel_pos_g);

  // Never request an altitude below the perch panel.
  // TODO: Add a better altitude lower bound, which may change
  // over payout. Note that params->ascend_offset_g_z is the upperbound of
  // the ascent, not necessarily the intended ascent altitude.
  reel_pos_g->z = fmin(reel_pos_g->z, params->perched_wing_pos_p.z);

  // During kFlightPlanDisengageEngage, the hover azimuth position
  // should be held.
  if (*g_cont.flight_plan == kFlightPlanDisengageEngage) {
    reel_pos_g->x = fixed_pos_g->x;
    reel_pos_g->y = fixed_pos_g->y;
  }

  GetHoverTelemetry()->elevation_cmd_reel = ele_reel;
  GetHoverTelemetry()->elevation_min = elevation_min;
  GetHoverTelemetry()->elevation_max = elevation_max;
}

static void CalcAccelPosition(const Vec3 *accel_start_pos_g,
                              Vec3 *accel_pos_g) {
  *accel_pos_g = *accel_start_pos_g;
  accel_pos_g->z = -g_sys.tether->length;
  Vec3Scale(accel_pos_g,
            g_sys.tether->length / Vec3NormBound(accel_pos_g, 1e-6),
            accel_pos_g);
}

static void CalcDescendPosition(const Vec3 *vessel_pos_g, double wind_dir_f,
                                const HoverPathParams *params,
                                Vec3 *descend_pos_g) {
  CalcAscendDescendPosition(vessel_pos_g, wind_dir_f,
                            params->descend_offset_g_z, params, descend_pos_g);
}

static void SaturateToAzimuthLimits(Vec3 *wing_pos_g_cmd) {
  double unused_azi, azi_sat, r, z;
  VecGToCyl(wing_pos_g_cmd, &unused_azi, &r, &z);
  azi_sat = SaturateWrapped(
      unused_azi, GetSystemParams()->test_site_params.azi_allow_start,
      GetSystemParams()->test_site_params.azi_allow_end, -PI, PI);
  CylToVecG(azi_sat, r, z, wing_pos_g_cmd);
}

// Calculates the wing position command based on flight mode.
static void CalcRawPositionCommand(
    const Vec3 *tether_anchor_g, const Vec3 *vessel_pos_g,
    const FlightStatus *flight_status, double elevation_cmd,
    double norm_reel_pos_g, double wind_dir_f, const Vec3 *wing_pos_g,
    const Vec3 *fixed_pos_g, const Vec3 *perched_pos_g,
    const Vec3 *accel_start_pos_g, const HoverPathParams *params,
    HoverPathState *state, Vec3 *wing_pos_g_cmd) {
  assert(flight_status != NULL);
  assert(IsValidFlightMode(flight_status->flight_mode));
  assert(-PI / 2.0 <= elevation_cmd && elevation_cmd < PI / 2.0);
  assert(-PI <= wind_dir_f && wind_dir_f <= PI);

  switch (flight_status->flight_mode) {
    case kFlightModePilotHover:
    case kFlightModePerched:
      *wing_pos_g_cmd = *wing_pos_g;
      break;

    case kFlightModeHoverAscend:
      CalcAscendPosition(vessel_pos_g, wind_dir_f, params, wing_pos_g_cmd);
      break;

    case kFlightModeHoverPayOut:
    case kFlightModeHoverReelIn:
    case kFlightModeHoverPrepTransformGsUp:
    case kFlightModeHoverPrepTransformGsDown:
    case kFlightModeHoverTransformGsUp:
    case kFlightModeHoverTransformGsDown:
      CalcPayOutReelInPosition(elevation_cmd, norm_reel_pos_g, wind_dir_f,
                               tether_anchor_g, fixed_pos_g, flight_status,
                               params, state, wing_pos_g_cmd);
      SaturateToAzimuthLimits(wing_pos_g_cmd);
      break;

    case kFlightModeHoverFullLength:
      CalcFullLengthPosition(fixed_pos_g, accel_start_pos_g, wing_pos_g_cmd);
      SaturateToAzimuthLimits(wing_pos_g_cmd);
      break;

    case kFlightModeHoverAccel:
      CalcAccelPosition(accel_start_pos_g, wing_pos_g_cmd);
      SaturateToAzimuthLimits(wing_pos_g_cmd);
      break;

    case kFlightModeHoverTransOut:
      CalcTransOutPosition(flight_status->flight_mode_time, wing_pos_g,
                           wing_pos_g_cmd, state);
      break;

    case kFlightModeHoverDescend:
      CalcDescendPosition(vessel_pos_g, wind_dir_f, params, wing_pos_g_cmd);
      break;

    default:
    case kFlightModeForceSigned:
    case kFlightModeTransIn:
    case kFlightModeCrosswindNormal:
    case kFlightModeCrosswindPrepTransOut:
    case kFlightModeOffTether:
    case kNumFlightModes:
      assert(false);
      *wing_pos_g_cmd = *wing_pos_g;
      break;
  }

  // The maximum distance is the furthest the wing should ever be away
  // from the origin while on tether.  It accounts for tether stretch
  // and a non-zero tether attachment point and is intentionally large
  // to avoid false asserts.
  const double max_distance =
      g_sys.tether->length * 1.11 + Vec3Norm(&params->perched_wing_pos_p);

  assert(Vec3Distance(wing_pos_g, vessel_pos_g) <= max_distance);
  assert(Vec3Distance(fixed_pos_g, vessel_pos_g) <= max_distance);
  assert(Vec3Distance(perched_pos_g, vessel_pos_g) <= max_distance);
  assert(Vec3Distance(wing_pos_g_cmd, vessel_pos_g) <= max_distance);
}

// Rate limit a (cartesian) command on a cylindrical domain.
//
// Arguments:
//
// wing_pos_g_cmd      Input position [m] in ground coordinates.
// max_tangential_vel  [m/s]
// max_radial_vel      [m/s]
// min_z_vel           [m/s]
// max_z_vel           [m/s]
// ts                  Controller sample time [s]
// wing_pos_g_cmd_z1   Output position [m] in ground coordinates.
static void CylindricalRateLimit(const Vec3 *wing_pos_g_cmd,
                                 double max_tangential_vel,
                                 double max_radial_vel, double min_z_vel,
                                 double max_z_vel, double ts,
                                 Vec3 *wing_pos_g_cmd_z1) {
  assert(wing_pos_g_cmd != NULL);
  assert(max_tangential_vel >= 0.0);
  assert(max_radial_vel >= 0.0);
  assert(min_z_vel <= max_z_vel);
  assert(ts > 0.0);
  assert(wing_pos_g_cmd_z1 != NULL);

  double azi, z, radius, azi_z1, z_z1, radius_z1;
  VecGToCyl(wing_pos_g_cmd, &azi, &radius, &z);
  VecGToCyl(wing_pos_g_cmd_z1, &azi_z1, &radius_z1, &z_z1);

  const double max_azi_rate = max_tangential_vel / fmax(1.0, radius);

  // RateLimitCircular makes sure it goes shortest way around.
  RateLimitCircular(azi, -max_azi_rate, max_azi_rate, -PI, PI, ts, &azi_z1);
  RateLimit(radius, -max_radial_vel, max_radial_vel, ts, &radius_z1);
  RateLimit(z, min_z_vel, max_z_vel, ts, &z_z1);

  CylToVecG(azi_z1, radius_z1, z_z1, wing_pos_g_cmd_z1);
}

// Rate limits the position command between waypoints for autonomous
// flight modes.
static void SmoothRawPositionCommand(const Vec3 *raw_wing_pos_g_cmd,
                                     FlightMode flight_mode,
                                     const HoverPathParams *params,
                                     HoverPathState *state,
                                     Vec3 *wing_pos_g_cmd) {
  assert(IsValidFlightMode(flight_mode));

  // Set velocity limits.
  double max_radial_path_vel = params->max_normal_radial_speed;
  double max_tangential_path_vel = params->max_normal_tangential_speed;
  double min_z_path_vel = -params->max_descend_normal_z_speed;
  double max_z_path_vel = params->max_descend_normal_z_speed;

  // Several effects motivate us to descend from high altitude (such as may
  // occur following TransOut) before traversing in azimuth:
  //
  // 1. Apparent wind from the side may advect the rotor wake over onto one
  //    wing, potentially creating a large roll moment.
  //
  // 2. Roll stabilization is better at low altitude where the tether pitch is
  //    at a more favorable angle.
  //
  // 3. Translating sideways not only induces sidelip but also about creates yaw
  //    attitude transients as the motion starts/stops.
  //
  // TODO(b/143181116): Revisit the need for this feature once the roll moments
  // are better understood, or the bridle geometry is changed.
  if (IsPrepTransformMode(flight_mode) &&
      (-(state->wing_pos_g_cmd_z1.z - raw_wing_pos_g_cmd->z) >=
       params->max_altitude_error_for_translation)) {
    max_tangential_path_vel = 0.0;
  }

  // Modify velocity limits based on flight mode and proximity to
  // perch.
  if (flight_mode == kFlightModeHoverAscend ||
      flight_mode == kFlightModeHoverDescend) {
    min_z_path_vel = -params->max_ascend_perch_z_speed;
    max_z_path_vel = params->max_descend_perch_z_speed;
  } else if (Vec3Norm(raw_wing_pos_g_cmd) <
             3.0 * Vec3Norm(&params->perched_wing_pos_p)) {
    // Reducing speed limits near the perch is for safety while
    // testing under constraints.
    min_z_path_vel = -params->max_ascend_near_perch_z_speed;
    max_z_path_vel = params->max_descend_near_perch_z_speed;
  } else if (flight_mode == kFlightModeHoverAccel) {
    min_z_path_vel = -params->max_accel_z_speed;
    // Only open up z-velocity limit in the upward direction.
  }

  // We calculate spherical coordinates to check for snap through.
  // These conversions back and forth are awkward.
  // TODO: Make everything spherical coord for ease and consistency.
  double azi_raw = VecGToAzimuth(raw_wing_pos_g_cmd);
  double azi_z1 = VecGToAzimuth(&state->wing_pos_g_cmd_z1);

  // Rate-limit the path if we are in any of the autonomous hover
  // modes except hover trans-out.  This allows us to set waypoints
  // for the path without passing unrealistic positions to the
  // position controller. Hover trans-out is excluded because it
  // inherently starts with large velocities as the vehicle continues
  // to lose energy.
  if (AnyAutoHoverFlightMode(flight_mode) &&
      flight_mode != kFlightModeHoverTransOut) {
    bool azi_snap_through = false;
    // If there is a no-go zone, check if we've snapped through.
    if (GetSystemParams()->test_site_params.azi_no_go_size >=
        MIN_AZI_NO_GO_SIZE) {
      // TODO: Move snap tolerance to somewhere more logical.
      // Snap through detection still has issues, specified in b/116805392.
      // TODO: Make snap through detection more robust.
      double snap_tol = 0.05;
      double azi_allow_start =
          GetSystemParams()->test_site_params.azi_allow_start;
      double azi_allow_end = GetSystemParams()->test_site_params.azi_allow_end;
      // If the previous azimuth command is near the opposite side of the no-go,
      // we've snapped through the no-go, so we keep last valid azi command.
      if ((fabs(Wrap(azi_raw - azi_allow_start, -PI, PI)) <= snap_tol &&
           fabs(Wrap(azi_z1 - azi_allow_end, -PI, PI)) <= snap_tol) ||
          (fabs(Wrap(azi_raw - azi_allow_end, -PI, PI)) <= snap_tol &&
           fabs(Wrap(azi_z1 - azi_allow_start, -PI, PI)) <= snap_tol)) {
        azi_snap_through = true;
      }
    }
    Vec3 raw_wing_pos_g_cmd_azi_lim = *raw_wing_pos_g_cmd;
    // If azi has snapped through, we keep the azimuth the same but update
    // everything else.
    if (azi_snap_through) {
      double unused_azi, r, z;
      VecGToCyl(&raw_wing_pos_g_cmd_azi_lim, &unused_azi, &r, &z);
      CylToVecG(azi_z1, r, z, &raw_wing_pos_g_cmd_azi_lim);
    }
    CylindricalRateLimit(&raw_wing_pos_g_cmd_azi_lim, max_tangential_path_vel,
                         max_radial_path_vel, min_z_path_vel, max_z_path_vel,
                         *g_sys.ts, &state->wing_pos_g_cmd_z1);
  } else {
    state->wing_pos_g_cmd_z1 = *raw_wing_pos_g_cmd;
  }

  *wing_pos_g_cmd = state->wing_pos_g_cmd_z1;
}

// Calculates the velocity command by numerically differentiating the
// z position and using the winch velocity and the position to set the
// horizontal components.
static void CalcVelocityCommand(const Vec3 *raw_wing_pos_g_cmd,
                                const Vec3 *wing_pos_g_cmd, double winch_vel,
                                const FlightStatus *flight_status,
                                const Vec3 *wing_vel_g,
                                const HoverPathParams *params,
                                HoverPathState *state, Vec3 *wing_vel_g_cmd) {
  // TODO: I would prefer that neither raw_wing_pos_g_cmd nor
  // wing_vel_g be inputs to this function. These values are currently used to
  // implement special features active in HoverTransOut and
  // HoverPrepTransformGsDown.  The use of the current wing velocity wing_vel_g
  // could be removed in a refactoring by instead scaling the velocity error
  // signal.

  // Numerically differentiate wing_pos_g_cmd to form wing_vel_g_cmd.  A valid
  // velocity command is needed to prevent the integral term from winding up to
  // compensate for an invalid derivative term in the PID loops.
  DiffLpf2Vec3(wing_pos_g_cmd, params->velocity_cutoff_freq,
               params->velocity_damping_ratio, *g_sys.ts, wing_vel_g_cmd,
               state->wing_vel_g_cmd_zs);

  if (flight_status->flight_mode == kFlightModePilotHover) {
    state->wing_vel_g_cmd_z1 = *wing_vel_g;
  } else if (flight_status->flight_mode == kFlightModeHoverTransOut) {
    // Smoothly blend the velocity command to zero. Trans-out starts
    // at high velocities that we don't want to fight initially.
    state->wing_vel_g_cmd_z1.x =
        Crossfade(wing_vel_g->x, 0.0, flight_status->flight_mode_time, 0.0,
                  params->transout_vg_cmd_crossfade_duration);
    state->wing_vel_g_cmd_z1.y =
        Crossfade(wing_vel_g->y, 0.0, flight_status->flight_mode_time, 0.0,
                  params->transout_vg_cmd_crossfade_duration);

    state->wing_vel_g_cmd_z1.z =
        Crossfade(wing_vel_g->z * params->transout_vel_cmd_multiplier, 0.0,
                  flight_status->flight_mode_time, 0.0,
                  params->transout_vg_cmd_crossfade_duration);

    // Zero the azimuthal velocity component if it is positive (i.e.
    // if the kite has reversed direction after exiting crosswind).
    Vec3 vec_azi_g, vel_azi_g;
    // Calculate direction of increasing azimuth.
    Vec3Cross(&kVec3Z, wing_pos_g_cmd, &vec_azi_g);
    Vec3Normalize(&vec_azi_g, &vec_azi_g);
    // Scale the normalized azimuth vector by the component of
    // inertial velocity in that direction to get azimuthal velocity
    // and subtract.
    Vec3Scale(&vec_azi_g,
              fmax(Vec3Dot(&vec_azi_g, &state->wing_vel_g_cmd_z1), 0.0),
              &vel_azi_g);
    Vec3Sub(&state->wing_vel_g_cmd_z1, &vel_azi_g, &state->wing_vel_g_cmd_z1);

    state->wing_vel_g_cmd_z1.z = fmin(state->wing_vel_g_cmd_z1.z, 0.0);
  } else {
    // Because the position command is based on the actual position of the wing,
    // there could be an unintentional feedback loop on the radial velocity (we
    // don't control radial position).  For example, this unintentional feeback
    // may enter through the tether elevation controller, which modifies
    // raw_wing_pos_g_cmd.  Thus we overwrite the horizontal components of the
    // numerically differentiated wing_vel_g_cmd to avoid any unintentional
    // feedback.  Note that the winch based velocity isn't well-defined near the
    // origin, so we fade the velocity to zero near there.

    double wing_radial_velocity_ff;
    if (flight_status->flight_mode == kFlightModeHoverPrepTransformGsDown &&
        (-(state->wing_pos_g_cmd_z1.z - raw_wing_pos_g_cmd->z) >= 5.0)) {
      // During flight mode PrepTransformGsDown, the kite must descend from a
      // potentially high TransOut altitude to standard hover altitude.  At
      // these higher altitudes, the curvature of the sphere is more pronounced,
      // requiring a significant outward radial velocity to accompany the
      // downward velocity.  Here we compute an outward radial velocity command
      // that is consistent with motion on a sphere.

      double kite_elevation_ref =
          Asin(wing_pos_g_cmd->z / g_sys.tether->length);

      wing_radial_velocity_ff =
          Saturate(-wing_vel_g_cmd->z * tan(kite_elevation_ref), 0.0,
                   params->max_normal_radial_speed);

    } else {
      wing_radial_velocity_ff = winch_vel;
    }

    Vec3 wing_vel_g_ff;
    Vec3Scale(wing_pos_g_cmd,
              wing_radial_velocity_ff / Vec3NormBound(wing_pos_g_cmd, 1.0),
              &wing_vel_g_ff);
    wing_vel_g_ff.z = 0.0;

    // Remove the radial component of wing_vel_g_cmd.
    Vec3 radial_unit = *wing_pos_g_cmd;
    radial_unit.z = 0.0;
    Vec3Normalize(&radial_unit, &radial_unit);

    Vec3 radial_vel_cmd;
    Vec3Scale(&radial_unit, Vec3Dot(&radial_unit, wing_vel_g_cmd),
              &radial_vel_cmd);
    Vec3Sub(wing_vel_g_cmd, &radial_vel_cmd, wing_vel_g_cmd);
    assert(fabs(Vec3Dot(wing_vel_g_cmd, &radial_unit)) < 1e-3);

    // Add in the feed forward radial component.
    Vec3Add(wing_vel_g_cmd, &wing_vel_g_ff, wing_vel_g_cmd);

    // Limit how fast the velocity command can change.
    Vec3 min_acceleration_g;
    Vec3Scale(&params->max_acceleration_g, -1.0, &min_acceleration_g);
    RateLimitVec3(wing_vel_g_cmd, &min_acceleration_g,
                  &params->max_acceleration_g, *g_sys.ts,
                  &state->wing_vel_g_cmd_z1);
  }

  *wing_vel_g_cmd = state->wing_vel_g_cmd_z1;
}

void HoverPathStep(const Vec3 *tether_anchor_g, const VesselEstimate *vessel,
                   double horizontal_tension_cmd, double tether_elevation_cmd,
                   const TetherGroundAnglesEstimate *tether_ground_angles,
                   double payout, const Vec3 *wing_pos_g,
                   const Vec3 *wing_vel_g, const WindEstimate *wind_g,
                   const Playbook *playbook, double winch_pos, double winch_vel,
                   const FlightStatus *flight_status,
                   const HoverPathParams *params, HoverPathState *state,
                   Vec3 *wing_pos_g_cmd, Vec3 *wing_vel_g_cmd,
                   Vec3 *crosswind_loop_center_pos_g) {
  assert(horizontal_tension_cmd >= 0.0);
  assert(wing_pos_g != NULL);
  assert(wind_g != NULL);
  assert(playbook != NULL);
  assert(-g_sys.tether->length - 10.0 < winch_pos && winch_pos < 10.0);
  assert(flight_status != NULL);
  assert(params != NULL && state != NULL);
  assert(wing_pos_g_cmd != NULL && wing_vel_g_cmd != NULL);
  assert(crosswind_loop_center_pos_g != NULL);

  // Calculate the tether elevation error.
  const double tether_elevation_error_f = Lpf2(
      tether_elevation_cmd - tether_ground_angles->elevation_p,
      params->tether_elevation_error_fc, params->tether_elevation_error_zeta,
      *g_sys.ts, state->tether_elevation_error_zs);

  // Calculate the kite elevation command (used in PayOut and ReelIn).
  double radial_position_ff, vertical_position_ff, elevation_ff;
  double tether_elevation_cmd_g = tether_elevation_cmd + params->vessel_heel_ff;
  ConvertPayoutToPosition(payout, horizontal_tension_cmd,
                          tether_elevation_cmd_g, &radial_position_ff,
                          &vertical_position_ff, &elevation_ff);

  // TODO: Reconsider whether hypot is the best way to
  // eliminate the current kite position from this elevation
  // conversion.  Getting the norm_reel_pos_g from
  // -vertical_position_ff/sin(elevation_cmd) should result in
  // outputting the same cmd altitude as the feed forward estimate,
  // which would seem to squash the feedback portion. Using
  // radial_position_ff/cos(elevation_cmd) may have the effect of
  // increasing the gain.  Hypot may decrease the effective gain but
  // seems the best behaved.  radial_position_ff and
  // vertical_position_ff are the radial and z distances from the
  // tether anchor location, with +z direction consistent with that in
  // the ground frame.
  const double norm_reel_pos_g =
      hypot(radial_position_ff, vertical_position_ff);

  const double elevation_fb = ControlTetherElevation(
      tether_elevation_error_f, tether_ground_angles->elevation_valid,
      elevation_ff, tether_anchor_g, wing_pos_g, flight_status, params, state);

  const double elevation_cmd = elevation_ff + elevation_fb;

  // Reset the "fixed" position in pilot hover.  This position is used
  // in some flight plans to override another set point.
  if (flight_status->flight_mode == kFlightModePilotHover) {
    state->fixed_pos_g = *wing_pos_g;
  } else if (flight_status->flight_mode == kFlightModePerched) {
    // Record the current position as the perched position.
    state->perched_pos_g = *wing_pos_g;
  }

  // Use playbook to determine the potential loop geometry.
  // Wind_aloft_g and wind_g are equivalent during hover.
  CalcAccelStartAndCrosswindLoopCenterPosition(
      wind_g, playbook, &state->perched_pos_g, params,
      &state->accel_start_pos_g, crosswind_loop_center_pos_g);

  // TODO: Check vessel estimate validity.
  CalcRawPositionCommand(tether_anchor_g, &vessel->pos_g, flight_status,
                         elevation_cmd, norm_reel_pos_g, wind_g->dir_f,
                         wing_pos_g, &state->fixed_pos_g, &state->perched_pos_g,
                         &state->accel_start_pos_g, params, state,
                         &state->raw_wing_pos_g_cmd);

  SmoothRawPositionCommand(&state->raw_wing_pos_g_cmd,
                           flight_status->flight_mode, params, state,
                           wing_pos_g_cmd);

  CalcVelocityCommand(&state->raw_wing_pos_g_cmd, wing_pos_g_cmd, winch_vel,
                      flight_status, wing_vel_g, params, state, wing_vel_g_cmd);

  // Update telemetry.
  HoverTelemetry *ht = GetHoverTelemetry();
  ht->elevation_cmd = elevation_cmd;
  ht->elevation_ff = elevation_ff;
  ht->elevation_fb = elevation_fb;
  ht->raw_wing_pos_g_cmd = state->raw_wing_pos_g_cmd;
  ht->perched_pos_g = state->perched_pos_g;
  ht->tether_elevation_cmd = tether_elevation_cmd;
}

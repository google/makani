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

#include "control/crosswind/crosswind_power.h"

#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "common/c_math/filter.h"
#include "common/c_math/geometry.h"
#include "common/c_math/util.h"
#include "common/c_math/vec3.h"
#include "common/macros.h"
#include "control/common.h"
#include "control/control_params.h"
#include "control/control_telemetry.h"
#include "control/control_types.h"
#include "control/crosswind/crosswind_frame.h"
#include "control/crosswind/crosswind_playbook.h"
#include "control/crosswind/crosswind_types.h"
#include "control/crosswind/crosswind_util.h"
#include "control/estimator/estimator_types.h"
#include "control/ground_frame.h"
#include "control/system_params.h"
#include "control/system_types.h"

// Attempts to reduce the transient between transition-in and
// crosswind by selecting a path center where the current wing
// velocity and position align with the circular path.
static void SelectStartingAzimuthElevation(const Vec3 *wing_pos_g,
                                           const Vec3 *wing_vel_g,
                                           double path_azimuth_setpoint,
                                           double path_elevation_setpoint,
                                           double path_radius,
                                           const CrosswindPowerParams *params,
                                           double *azimuth, double *elevation) {
  assert(-PI <= path_azimuth_setpoint && path_azimuth_setpoint <= PI);
  assert(0.0 <= path_elevation_setpoint && path_elevation_setpoint <= PI / 2.0);
  assert(params->ele_min <= params->ele_max);
  assert(path_radius > 0.0);

  // Calculate offset from the current wing position to the ideal path
  // center.
  Vec3 wing_to_path_center_g;
  Vec3Cross(wing_pos_g, wing_vel_g, &wing_to_path_center_g);
  Vec3Normalize(&wing_to_path_center_g, &wing_to_path_center_g);
  Vec3Scale(&wing_to_path_center_g, path_radius, &wing_to_path_center_g);

  // Convert the ideal path center to azimuth and elevation values.
  Vec3 path_center_g;
  Vec3Add(&wing_to_path_center_g, wing_pos_g, &path_center_g);
  double unused_radius;
  VecGToSph(&path_center_g, azimuth, elevation, &unused_radius);

  // Saturate the azimuth and elevation values both by an allowed
  // angular offset from the starting azimuth and elevation set-points.
  double azimuth_offset = Wrap(*azimuth - path_azimuth_setpoint, -PI, PI);
  *azimuth = path_azimuth_setpoint + Saturate(azimuth_offset, -0.4, 0.4);
  *azimuth = Wrap(*azimuth, -PI, PI);

  double elevation_offset = Wrap(*elevation - path_elevation_setpoint, -PI, PI);
  *elevation = path_elevation_setpoint + Saturate(elevation_offset, -0.2, 0.2);
  *elevation = Saturate(*elevation, params->ele_min, params->ele_max);
}

void CrosswindPowerInit(const Vec3 *wing_pos_g, const Vec3 *wing_vel_g,
                        double wind_dir_f, const PlaybookEntry *playbook_entry,
                        const CrosswindPowerParams *params,
                        CrosswindPowerState *state) {
  assert(-PI <= wind_dir_f && wind_dir_f <= PI);
  assert(params->max_crosswind_y_position_for_slew >= 0.0);
  memset(state, 0, sizeof(*state));
  state->azi_setpoint =
      GetPlaybookEntryAzimuthWithLimits(wind_dir_f, playbook_entry);
  SelectStartingAzimuthElevation(
      wing_pos_g, wing_vel_g, state->azi_setpoint, playbook_entry->elevation,
      playbook_entry->path_radius_target, params, &state->raw_path_azimuth,
      &state->raw_path_elevation);
  SphToVecG(state->raw_path_azimuth, state->raw_path_elevation,
            g_sys.tether->length, &state->path_center_g_z1);
  state->path_type = kCrosswindPathNormal;

  // Check that the loop angle is provided on an equispaced grid. This is
  // important for getting a smooth derivative from SetAirspeedDerivative.
  state->dloop_angle = playbook_entry->lookup_loop_angle[1] -
                       playbook_entry->lookup_loop_angle[0];
  for (int32_t i = 0; i < ARRAYSIZE(playbook_entry->lookup_loop_angle) - 1;
       ++i) {
    double dloop_angle = playbook_entry->lookup_loop_angle[i + 1] -
                         playbook_entry->lookup_loop_angle[i];
    assert(fabs(dloop_angle - state->dloop_angle) < 1e-12);
  }

  state->airspeed_cmd_z1 = 0.0;
}

static void UpdatePathCenter(double wind_dir_f,
                             const PlaybookEntry *playbook_entry,
                             FlightMode flight_mode,
                             const CrosswindPowerParams *params,
                             CrosswindPowerState *state, Vec3 *path_center_g) {
  assert(-PI <= wind_dir_f && wind_dir_f <= PI);
  assert(0 <= flight_mode && flight_mode < kNumFlightModes);
  assert(path_center_g != NULL);
  assert(params->ele_min <= params->ele_max);

  if (flight_mode == kFlightModeCrosswindNormal) {
    // Update the path azimuth and elevation when in crosswind normal.

    // Calculate the set-point for the center of the flight path.
    // Some test sites or equipment have azimuth limitations.
    state->azi_setpoint =
        GetPlaybookEntryAzimuthWithLimits(wind_dir_f, playbook_entry);

    // Rate limit how fast elevation and azimuth set-point can change.
    RateLimitCircular(state->azi_setpoint, -params->azi_rate_lim,
                      params->azi_rate_lim, -PI, PI, *g_sys.ts,
                      &state->raw_path_azimuth);
    RateLimitCircular(playbook_entry->elevation, -params->ele_rate_lim,
                      params->ele_rate_lim, -PI, PI, *g_sys.ts,
                      &state->raw_path_elevation);
  } else if (flight_mode == kFlightModeCrosswindPrepTransOut) {
    // Update only the path elevation in prep transout.
    RateLimitCircular(params->transout_elevation_cmd, -params->ele_rate_lim,
                      params->ele_rate_lim, -PI, PI, *g_sys.ts,
                      &state->raw_path_elevation);
  }

  CrosswindPowerGetPathCenter(state, path_center_g);
}

// Chooses which path to use in the path controller.
static CrosswindPathType UpdatePathType(FlightMode flight_mode,
                                        double loop_angle,
                                        double path_radius_target_error,
                                        double kite_altitude,
                                        const CrosswindPowerParams *params,
                                        CrosswindPowerState *state) {
  if (flight_mode == kFlightModeCrosswindPrepTransOut) {
    // We change from kCrosswindPathNormal to PrepareTransitionOut near the
    // bottom of the loop. Over the rest of the loop, we leave the path type
    // unmodified. This handles situations where the vehicle passes loop_angle
    // == 0 during trans-out or we enter kFlightModeCrosswindPrepTransOut in
    // what would otherwise be the final quarter loop.
    // TODO: Wait for reaching azi command.
    if (params->loop_angle_path_switch_min < loop_angle &&
        loop_angle <= params->loop_angle_path_switch_max &&
        ((fabs(params->transout_elevation_cmd - state->raw_path_elevation) <
              0.01 &&
          fabs(path_radius_target_error) <
              params->transout_path_radius_target_threshold) ||
         kite_altitude <= params->min_transout_altitude)) {
      state->path_type = kCrosswindPathPrepareTransitionOut;
    }
  } else {
    state->path_type = kCrosswindPathNormal;
  }

  return state->path_type;
}

// TODO: This assumes a specific loop direction.
static double SetAirspeed(double loop_angle, CrosswindPathType path_type,
                          double wind_speed, const FlightStatus *flight_status,
                          const PlaybookEntry *playbook_entry,
                          const CrosswindPowerParams *params,
                          const CrosswindPowerState *state) {
  assert(0.0 <= loop_angle && loop_angle <= 2.0 * PI);

  double normal_airspeed_cmd = CircularInterp1(
      playbook_entry->lookup_loop_angle, playbook_entry->airspeed_lookup,
      ARRAYSIZE(playbook_entry->lookup_loop_angle), loop_angle);
  normal_airspeed_cmd =
      Saturate(normal_airspeed_cmd, params->min_airspeed, params->max_airspeed);
  double airspeed_cmd = normal_airspeed_cmd;

  // TODO: Use path_type rather than flight_mode to decide when to
  // change the airspeed command.
  if (flight_status->flight_mode == kFlightModeCrosswindPrepTransOut) {
    double transout_airspeed_cmd = CircularInterp1(
        playbook_entry->lookup_loop_angle,
        playbook_entry->transout_airspeed_lookup,
        ARRAYSIZE(playbook_entry->lookup_loop_angle), loop_angle);
    transout_airspeed_cmd = Saturate(
        transout_airspeed_cmd, params->min_airspeed, params->max_airspeed);
    airspeed_cmd = Crossfade(normal_airspeed_cmd, transout_airspeed_cmd,
                             flight_status->flight_mode_time, 0.0,
                             params->transout_airspeed_crossfade_time);
  }

  if (path_type == kCrosswindPathPrepareTransitionOut) {
    // When the path type switches, the airspeed command becomes a sine-shaped
    // profile with loop angle. The initial condition is set by the airspeed
    // command at the previous timestep.
    double corner_angle =
        Crossfade(params->low_wind_transout_airspeed_corner_angle,
                  params->high_wind_transout_airspeed_corner_angle, wind_speed,
                  params->corner_angle_low_wind_speed,
                  params->corner_angle_high_wind_speed);

    double sine_angle =
        Crossfade(0.0, PI / 2.0, loop_angle, corner_angle, PI / 2.0);

    double decelerating_airspeed_cmd =
        (state->airspeed_cmd_z1 - params->transout_airspeed_target) *
            sin(sine_angle) +
        params->transout_airspeed_target;

    airspeed_cmd = Crossfade(
        decelerating_airspeed_cmd, airspeed_cmd, Wrap(loop_angle, -PI, PI),
        params->loop_angle_path_switch_min -
            params->transout_path_switch_crossfade_distance,
        params->loop_angle_path_switch_min);

    // Beyond the deceleration target, apply a final crossfade deceleration.
    airspeed_cmd = Crossfade(params->transout_final_airspeed_target,
                             airspeed_cmd, Wrap(loop_angle, -PI, PI),
                             params->transout_final_airspeed_crossfade_angle,
                             corner_angle);
  }

  return airspeed_cmd;
}

// Return the derivative of airspeed_cmd with respect to loop_angle.
//
// For this to work properly, SetAirspeed must be stateless and "modeless".  The
// latter assumption is currently broken by dependence on state, flight mode,
// and path type, because the first-order difference estimate of the derivative
// is computed using a macroscopic change in loop angle, matching the loop angle
// interval between which Playbook does linear interpolation of the airspeed
// schedule.
//
// Approaches to fixing this include:
// 1. Have GetAirspeedDerivative itself dispatch on mode.  For example, the
// derivative of the final PrepTransOut airspeed schedule is available
// analytically.
// 2. Rather than converting Playbook's spline-interpolation into a linear
// interpolation table in the configuration system, evaluate the spline
// directly here in SetAirspeed, and calculate the derivative analytically
// in GetAirspeedDerivative.

static double GetAirspeedDerivative(double loop_angle, double dloop_angle,
                                    CrosswindPathType path_type,
                                    double wind_speed,
                                    const FlightStatus *flight_status,
                                    const PlaybookEntry *playbook_entry,
                                    const CrosswindPowerParams *params,
                                    const CrosswindPowerState *state) {
  // We take the derivative numerically so that all saturations in SetAirspeed
  // are automatically accounted for.
  return (SetAirspeed(Wrap(loop_angle + dloop_angle, 0.0, 2.0 * PI), path_type,
                      wind_speed, flight_status, playbook_entry, params,
                      state) -
          SetAirspeed(Wrap(loop_angle - dloop_angle, 0.0, 2.0 * PI), path_type,
                      wind_speed, flight_status, playbook_entry, params,
                      state)) /
         (2.0 * dloop_angle);
}

double SetAlpha(double loop_angle, const PlaybookEntry *playbook_entry) {
  return CircularInterp1(playbook_entry->lookup_loop_angle,
                         playbook_entry->alpha_lookup,
                         ARRAYSIZE(playbook_entry->alpha_lookup), loop_angle);
}

static double SetBeta(double loop_angle, const PlaybookEntry *playbook_entry) {
  return CircularInterp1(playbook_entry->lookup_loop_angle,
                         playbook_entry->beta_lookup,
                         ARRAYSIZE(playbook_entry->beta_lookup), loop_angle);
}

void CrosswindPowerGetPathCenter(const CrosswindPowerState *state,
                                 Vec3 *path_center_g) {
  assert(state != NULL && path_center_g != NULL);
  assert(-PI <= state->raw_path_elevation && state->raw_path_elevation <= PI);
  // TODO: Add an assert for the path azimuth being in
  // [-pi, pi) once all the wrapping issues are resolved.

  SphToVecG(state->raw_path_azimuth, state->raw_path_elevation,
            g_sys.tether->length, path_center_g);
}

// TODO: This code does not check state_est->wind_g.valid.
void CrosswindPowerStep(
    const FlightStatus *flight_status, const StateEstimate *state_est,
    double path_radius_target_error, const CrosswindFlags *flags,
    const CrosswindPowerParams *params, const PlaybookEntry *playbook_entry,
    CrosswindPowerState *state, CrosswindPathType *path_type,
    Vec3 *raw_path_center_g, Vec3 *path_center_g, double *airspeed_cmd,
    double *d_airspeed_d_loopangle, double *alpha_nom, double *beta_nom) {
  assert(flight_status != NULL && state_est != NULL && flags != NULL &&
         params != NULL && state != NULL && path_type != NULL &&
         path_center_g != NULL && airspeed_cmd != NULL && alpha_nom != NULL);

  UpdatePathCenter(state_est->wind_aloft_g.dir_f_playbook, playbook_entry,
                   flight_status->flight_mode, params, state,
                   raw_path_center_g);
  SlewAzimuthWhenNecessary(&state_est->Xg, raw_path_center_g,
                           params->max_crosswind_y_position_for_slew,
                           &state->path_center_g_z1);
  *path_center_g = state->path_center_g_z1;

  double loop_angle = CalcLoopAngle(path_center_g, &state_est->Xg);
  *path_type =
      UpdatePathType(flight_status->flight_mode, loop_angle,
                     path_radius_target_error, -state_est->Xg.z, params, state);

  *airspeed_cmd =
      SetAirspeed(loop_angle, *path_type, state_est->wind_aloft_g.speed_f,
                  flight_status, playbook_entry, params, state);
  if (*path_type == kCrosswindPathNormal) {
    state->airspeed_cmd_z1 = *airspeed_cmd;
  }

  *alpha_nom = SetAlpha(loop_angle, playbook_entry);
  *beta_nom = SetBeta(loop_angle, playbook_entry);

  *d_airspeed_d_loopangle =
      GetAirspeedDerivative(loop_angle, state->dloop_angle, *path_type,
                            state_est->wind_aloft_g.speed_f, flight_status,
                            playbook_entry, params, state);

  // Update telemetry.
  CrosswindTelemetry *cwt = GetCrosswindTelemetry();
  cwt->azi_offset = Wrap(
      state->azi_setpoint - state_est->wind_aloft_g.dir_f_playbook, -PI, PI);
  cwt->azi_target = state->azi_setpoint;
  cwt->elevation = state->raw_path_elevation;
  cwt->path_type = (int32_t)*path_type;
  cwt->path_center_g = *path_center_g;
  cwt->loop_angle = loop_angle;
}

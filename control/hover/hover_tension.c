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

#include "control/hover/hover_tension.h"

#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "common/c_math/filter.h"
#include "common/c_math/util.h"
#include "common/macros.h"
#include "control/common.h"
#include "control/control_params.h"
#include "control/control_telemetry.h"
#include "control/control_types.h"
#include "control/estimator/estimator_types.h"
#include "control/ground_frame.h"
#include "control/system_params.h"
#include "control/system_types.h"
#include "control/tether_util.h"

COMPILE_ASSERT(ARRAYSIZE(((HoverTensionParams *)NULL)->payout_table) ==
                   ARRAYSIZE(((HoverTensionParams *)NULL)->min_pitch_table),
               payout_and_min_pitch_tables_must_have_same_length);

COMPILE_ASSERT(ARRAYSIZE(((HoverTensionParams *)NULL)->payout_table) ==
                   ARRAYSIZE(((HoverTensionParams *)NULL)->max_pitch_table),
               payout_and_max_pitch_tables_must_have_same_length);

static bool ValidateState(const HoverTensionParams *params,
                          const HoverTensionState *state) {
  double int_pitch_min = fmin(params->tension_hard_pid.int_output_min,
                              params->tension_soft_pid.int_output_min);
  double int_pitch_max = fmax(params->tension_hard_pid.int_output_max,
                              params->tension_soft_pid.int_output_max);
  if (!((int_pitch_min <= state->int_pitch &&
         state->int_pitch <= int_pitch_max) ||
        IsApproximatelyEqual(state->int_pitch, int_pitch_min) ||
        IsApproximatelyEqual(state->int_pitch, int_pitch_max))) {
    assert(!(bool)"int_pitch is outside the integrator saturations.");
    return false;
  }

  if (!(-PI <= state->additional_pitch_cmd_z1 &&
        state->additional_pitch_cmd_z1 <= 0.0)) {
    assert(!(bool)"additional_pitch_cmd_z1 is outside [-pi, 0].");
    return false;
  }

  return true;
}

void HoverTensionInit(const HoverTensionParams *params,
                      HoverTensionState *state) {
  assert(params != NULL && state != NULL);
  assert(HoverTensionValidateParams(params));

  memset(state, 0, sizeof(*state));
  state->additional_pitch_cmd_z1 = 0.0;
  state->int_pitch = 0.0;
  state->horizontal_tension_cmd_z1 = params->tension_min_set_point;
  state->horizontal_tension_pilot_offset_target = 0.0;
  state->horizontal_tension_pilot_offset = 0.0;
  for (int32_t i = 0;
       i < ARRAYSIZE(state->horizontal_tension_pilot_offset_filter_state);
       ++i) {
    state->horizontal_tension_pilot_offset_filter_state[i] = 0.0;
  }
  state->joystick_roll_z1 = 0.0;
  state->cycles_above_roll_threshold = 0;
  state->horizontal_tension_increment_enabled = true;

  if (!ValidateState(params, state)) assert(false);
}

// These checks are intentionally loose, so that they may apply to a
// wide range of systems.  They are only meant to catch serious errors
// in the parameters.
bool HoverTensionValidateParams(const HoverTensionParams *params) {
  if (!(params->tension_min_set_point > 100.0)) {
    assert(
        !(bool)"tension_min_set_point must be positive and reasonably large.");
    return false;
  }

  if (!(params->tension_hard_pid.kp >= 0.0 &&
        params->tension_hard_pid.ki >= 0.0 &&
        fabs(params->tension_hard_pid.kd) < DBL_EPSILON)) {
    assert(!(
        bool)"tension_hard_pid must have non-negative P and I gains and"
             " zero D gain.");
    return false;
  }

  if (!(params->tension_hard_pid.int_output_min <
        params->tension_hard_pid.int_output_max)) {
    assert(!(
        bool)"tension_hard_pid.int_output_max must be greater than"
             " tension_hard_pid.int_output_min.");
    return false;
  }

  if (!(params->tension_soft_pid.kp >= 0.0 &&
        params->tension_soft_pid.ki >= 0.0 &&
        fabs(params->tension_soft_pid.kd) < DBL_EPSILON)) {
    assert(!(
        bool)"tension_soft_pid must have non-negative P and I gains and"
             " zero D gain.");
    return false;
  }

  if (!(params->tension_soft_pid.int_output_min <
        params->tension_soft_pid.int_output_max)) {
    assert(!(
        bool)"tension_soft_pid.int_output_max must be greater than"
             " tension_soft_pid.int_output_min.");
    return false;
  }

  if (!(params->hard_spring_payout >= 0.0 &&
        params->soft_spring_payout > params->hard_spring_payout)) {
    assert(!(
        bool)"hard_spring_payout must be non-negative and soft_spring_payout"
             " must be larger.");
    return false;
  }

  for (int32_t i = 1; i < HOVER_TENSION_PITCH_LIMIT_TABLE_LENGTH; ++i) {
    if (!(params->payout_table[i] > params->payout_table[i - 1])) {
      assert(!(bool)"payout_table must be strictly increasing.");
      return false;
    }
  }

  for (int32_t i = 0; i < HOVER_TENSION_PITCH_LIMIT_TABLE_LENGTH; ++i) {
    if (!(params->min_pitch_table[i] <= params->max_pitch_table[i])) {
      assert(!(bool)"min_pitch_table must be less than max_pitch_table.");
      return false;
    }
  }

  if (!(0.0 <= params->hover_drag_coeff && params->hover_drag_coeff < 5.0)) {
    assert(!(
        bool)"hover_drag_coeff must be non-negative and have a reasonable"
             " value.");
    return false;
  }

  if (!(params->max_pilot_extra_tension >= 0.0 &&
        params->max_pilot_extra_tension <= 10e3)) {
    assert(!(
        bool)"max_pilot_extra_tension must be non-negative and have a "
             "reasonable"
             " value.");
    return false;
  }

  return true;
}

// Estimates the apparent wind speed in the ground frame, given the
// wind speed, position, and winch velocity.  This is used rather than
// real apparent wind speed in CalcHorizontalAeroForce because:
//
// - When transitioning out of crosswind, the real apparent wind speed
//   is large compared to the wind speed and can cause excessive
//   pitch-back angle.
//
// - The real apparent wind speed creates a weak feedback loop between
//   height and horizontal position.
//
// NOTE: We've discussed moving this function to the
// estimator, but decided to leave it here because it is simple and
// very hover-specific.
static void EstimateApparentWind(const Vec3 *wing_pos_v_g, const Vec3 *wind_g,
                                 double winch_vel, Vec3 *apparent_wind_g) {
  // Assume the wing is only moving in x and y.
  Vec3 xy_wing_pos_v_g = {wing_pos_v_g->x, wing_pos_v_g->y, 0.0};
  double norm_xy_wing_pos_v_g = Vec3Norm(&xy_wing_pos_v_g);

  // The wing velocity is estimated to be the same magnitude as the
  // winch velocity, along a vector that passes through the ground
  // station z-axis and the wing parallel to the ground.  However, if
  // the wing is too close to the origin, this velocity is not
  // well-defined, so we just assume the wing is not moving.
  double scale = Crossfade(0.0, winch_vel / fmax(norm_xy_wing_pos_v_g, 1.0),
                           norm_xy_wing_pos_v_g, 0.0, 1.0);
  Vec3Scale(&xy_wing_pos_v_g, scale, apparent_wind_g);
  Vec3Sub(wind_g, apparent_wind_g, apparent_wind_g);
}

// Calculates the aerodynamic force on the wing from flat-plate drag
// projected onto the tether in the x-y plane.  This does not include
// tension from pitching back the rotor thrust.  This is used as an
// estimate of the baseline horizontal tether force.
static double CalcHorizontalAeroForce(const Vec3 *wing_pos_v_g,
                                      const Vec3 *wind_g, double winch_vel,
                                      const HoverTensionParams *params) {
  Vec3 apparent_wind_g;
  EstimateApparentWind(wing_pos_v_g, wind_g, winch_vel, &apparent_wind_g);

  // Only consider horizontal components of apparent wind.
  Vec3 apparent_wind_xy_g = {apparent_wind_g.x, apparent_wind_g.y, 0.0};
  Vec3 wing_pos_xy_v_g = {wing_pos_v_g->x, wing_pos_v_g->y, 0.0};
  double norm_wing_pos_xy_v_g = Vec3Norm(&wing_pos_xy_v_g);
  double wind_along_flat_tether =
      Vec3Dot(&apparent_wind_xy_g, &wing_pos_xy_v_g) /
      fmax(norm_wing_pos_xy_v_g, 1.0);

  // Calculate the drag force along the tether.  Note that
  // hover_drag_coeff should take into account that the wing is not
  // normal to the tether.
  return 0.5 * g_sys.phys->rho * g_sys.wing->A * params->hover_drag_coeff *
         Sign(wind_along_flat_tether) * wind_along_flat_tether *
         wind_along_flat_tether;
}

double HoverTensionSetHorizontalTension(const Vec3 *wing_pos_v_g,
                                        const Vec3 *wind_g, double winch_vel,
                                        bool hold_horizontal_tension_cmd,
                                        const HoverTensionParams *params,
                                        HoverTensionState *state) {
  assert(wing_pos_v_g != NULL && wind_g != NULL && params != NULL);
  assert(fabs(winch_vel) < 30.0);

  // The philosophy here is that the horizontal tension command should be (1)
  // the minimum horizontal tension command, or (2) the horizontal tension
  // provided by the wind, whichever is greater.  The intent is to not "fight"
  // the tension provided by the wind.
  double horizontal_aero_force =
      CalcHorizontalAeroForce(wing_pos_v_g, wind_g, winch_vel, params);
  double horizontal_tension_cmd = horizontal_aero_force;

  if (hold_horizontal_tension_cmd) {
    horizontal_tension_cmd = state->horizontal_tension_cmd_z1;
  }

  horizontal_tension_cmd = RateLimit(
      horizontal_tension_cmd, -params->horizontal_tension_cmd_rate_limit,
      params->horizontal_tension_cmd_rate_limit, *g_sys.ts,
      &state->horizontal_tension_cmd_z1);

  // Reapply the minimum tension command, in case it was violated by one of the
  // modifications above.
  horizontal_tension_cmd =
      fmax(params->tension_min_set_point, horizontal_tension_cmd);
  state->horizontal_tension_cmd_z1 = horizontal_tension_cmd;

  return horizontal_tension_cmd;
}

// Calculates the expected airframe pitch required to attain the
// desired horizontal tension command.
static double CalcFeedForwardPitch(double horizontal_tension_cmd,
                                   double tether_elevation_cmd,
                                   const Vec3 *wing_pos_v_g, const Vec3 *wind_g,
                                   double payout, double winch_vel,
                                   const HoverTensionParams *params) {
  assert(-1.0 < payout && payout < g_sys.tether->length + 10.0);
  assert(fabs(winch_vel) < 30.0);

  const double horizontal_aero_force =
      CalcHorizontalAeroForce(wing_pos_v_g, wind_g, winch_vel, params);

  const double payed_tether_mass = payout * g_sys.tether->linear_density;

  const double vertical_force_on_ground_station =
      horizontal_tension_cmd * tan(tether_elevation_cmd);

  return atan2(horizontal_tension_cmd - horizontal_aero_force,
               (g_sys.wing->m + payed_tether_mass) * g_sys.phys->g +
                   vertical_force_on_ground_station);
}

// Calculates the allowed pitch limits based on the distance from the
// perch.  Far from the perch, large pitch-back angles are fine for
// keeping the tether off the ground.  Near the perch, we avoid large
// pitch-back angles to preserve perching geometry.
static void CalcPitchLimits(double payout, const HoverTensionParams *params,
                            double *min_pitch, double *max_pitch) {
  assert(payout > -1.0);

  *min_pitch = Interp1(params->payout_table, params->min_pitch_table,
                       HOVER_TENSION_PITCH_LIMIT_TABLE_LENGTH, payout,
                       kInterpOptionSaturate);
  *max_pitch = Interp1(params->payout_table, params->max_pitch_table,
                       HOVER_TENSION_PITCH_LIMIT_TABLE_LENGTH, payout,
                       kInterpOptionSaturate);

  assert(*min_pitch <= *max_pitch);
}

// Calculates what the measured tension should be at the wing, which
// includes both the horizontal tension force and the tension due to
// the weight of the tether.
static double CalcTensionAtWing(double horizontal_tension,
                                const Vec3 *wing_pos_g) {
  assert(horizontal_tension >= 0.0);

  double r = fmax(Vec3XyNorm(wing_pos_g), 1.0);
  TetherParabola tether_parabola;
  TensionAndPointToParabola(horizontal_tension, r, -wing_pos_g->z,
                            g_sys.phys->g * g_sys.tether->linear_density,
                            &tether_parabola);
  return horizontal_tension /
         Saturate(cos(ParabolaAngle(&tether_parabola, r)), 0.1, 1.0);
}

// In the acceleration mode, pitch forward as the angle of inclination
// increases.
static double ModifyPitchCommandInAccel(double pitch_cmd,
                                        const Vec3 *wing_pos_g,
                                        FlightMode flight_mode,
                                        const HoverTensionParams *params,
                                        HoverTensionState *state) {
  assert(-PI <= pitch_cmd && pitch_cmd <= PI);
  assert(0 <= flight_mode && flight_mode < kNumFlightModes);

  double additional_pitch_cmd = 0.0;
  if (flight_mode == kFlightModeHoverAccel) {
    double elevation = VecGToElevation(wing_pos_g);
    additional_pitch_cmd = -fmax(0.0, elevation);
  }
  additional_pitch_cmd =
      RateLimit(additional_pitch_cmd, -params->additional_pitch_cmd_rate_limit,
                params->additional_pitch_cmd_rate_limit, *g_sys.ts,
                &state->additional_pitch_cmd_z1);
  return pitch_cmd + additional_pitch_cmd;
}

static void UpdateHorizontalTensionPilotOffsetTarget(
    const HoverTensionParams *params, const JoystickEstimate *joystick,
    FlightMode flight_mode, HoverTensionState *state) {
  const double roll = joystick->data.roll;

  if (flight_mode == kFlightModePilotHover) {
    // In PilotHover, the offset target is set to 0. The filter in the caller
    // will ensure a smooth transition.
    //
    // The increment is disabled in case, on return to auto-hover, the roll
    // happens to be temporarily above the threshold. The cycle counter reset
    // should be unnecessary, but seems like the most consistent thing to do.
    state->horizontal_tension_pilot_offset_target = 0.0;
    state->horizontal_tension_increment_enabled = false;
    state->cycles_above_roll_threshold = 0;
  } else if (!joystick->valid) {
    // No-op, and we won't update joystick_roll_z1 below.
  } else if (fabs(roll) < params->horizontal_tension_joystick_roll_threshold ||
             roll * state->joystick_roll_z1 < 0.0) {
    // Reset the counter and re-enable the increment if the roll stick drops
    // below the threshold or (edge case) jumps between the postive and negative
    // extremes.
    state->cycles_above_roll_threshold = 0;
    state->horizontal_tension_increment_enabled = true;
  } else {
    // The joystick roll exceeds the threshold and matches its sign on the
    // previous cycle. Update the counter and, if appropriate, apply the tension
    // increment.
    if (state->cycles_above_roll_threshold <
        params->horizontal_tension_num_cycles_for_increment) {
      ++state->cycles_above_roll_threshold;
    }
    if (state->cycles_above_roll_threshold ==
            params->horizontal_tension_num_cycles_for_increment &&
        state->horizontal_tension_increment_enabled) {
      state->horizontal_tension_increment_enabled = false;

      state->horizontal_tension_pilot_offset_target =
          Saturate(state->horizontal_tension_pilot_offset_target +
                       Sign(roll) * params->horizontal_tension_pilot_increment,
                   -params->horizontal_tension_max_pilot_offset,
                   params->horizontal_tension_max_pilot_offset);
    }
  }

  if (joystick->valid) {
    state->joystick_roll_z1 = roll;
  }
}

// Gets the horizontal tension offset applied by the pilot.
//
// If the joystick roll exceeds +/- a threshold value for a set number of
// cycles, the offset is incremented. The roll stick must then return below the
// threshold before another increment can be applied.
static double GetHorizontalTensionPilotOffset(const HoverTensionParams *params,
                                              const JoystickEstimate *joystick,
                                              FlightMode flight_mode,
                                              HoverTensionState *state) {
  UpdateHorizontalTensionPilotOffsetTarget(params, joystick, flight_mode,
                                           state);
  const double target = state->horizontal_tension_pilot_offset_target;
  RateLimit(target, -params->horizontal_tension_cmd_rate_limit,
            params->horizontal_tension_cmd_rate_limit, *g_sys.ts,
            &state->horizontal_tension_pilot_offset);
  state->horizontal_tension_pilot_offset =
      Lpf2(target, params->horizontal_tension_pilot_offset_fc,
           params->horizontal_tension_pilot_offset_zeta, *g_sys.ts,
           state->horizontal_tension_pilot_offset_filter_state);
  return state->horizontal_tension_pilot_offset;
}

double HoverTensionStep(
    double horizontal_tension_cmd, double tether_elevation_cmd,
    const TetherForceEstimate *tether_force, const Vec3 *wing_pos_g,
    const Vec3 *hover_origin_g, const Vec3 *wind_g, double payout,
    double winch_vel, bool below_half_thrust, const FlightStatus *flight_status,
    const JoystickEstimate *joystick, const HoverTensionParams *params,
    HoverTensionState *state) {
  assert(horizontal_tension_cmd >= 0.0);
  assert(tether_force != NULL && wing_pos_g != NULL && wind_g != NULL);
  assert(-1.0 <= payout && payout < g_sys.tether->length + 10.0);
  assert(fabs(winch_vel) < 30.0);
  assert(0 <= flight_status->flight_mode &&
         flight_status->flight_mode < kNumFlightModes);
  assert(params != NULL && state != NULL && ValidateState(params, state));
  assert(params->horizontal_tension_cmd_rate_limit > 0.0);

  // Add pilot-controlled horizontal tension offset.
  {
    const double pilot_offset = GetHorizontalTensionPilotOffset(
        params, joystick, flight_status->flight_mode, state);
    horizontal_tension_cmd += pilot_offset;
    GetControlTelemetry()->hover.horizontal_tension_pilot_offset = pilot_offset;
  }

  Vec3 wing_pos_v_g;
  Vec3Sub(wing_pos_g, hover_origin_g, &wing_pos_v_g);
  double pitch_ff =
      CalcFeedForwardPitch(horizontal_tension_cmd, tether_elevation_cmd,
                           &wing_pos_v_g, wind_g, payout, winch_vel, params);

  // Reset integrator while perched.  Hold integrator in some dynamic
  // modes, modes that contact the perch, or if below half thrust.
  // Otherwise, allow the integrator to integrate. HoverTransOut has
  // a constant pitch command and so the integrator is reset.
  IntegratorMode int_mode;
  if (!AnyHoverFlightMode(flight_status->flight_mode) ||
      (flight_status->flight_mode == kFlightModeHoverTransOut)) {
    int_mode = kIntegratorModeReset;
  } else if (below_half_thrust ||
             ((flight_status->flight_mode == kFlightModeHoverAscend ||
               flight_status->flight_mode == kFlightModeHoverDescend)) ||
             flight_status->flight_mode == kFlightModeHoverAccel ||
             !tether_force->valid) {
    int_mode = kIntegratorModeHold;
  } else {
    int_mode = kIntegratorModeIntegrate;
  }

  PidParams tension_pid;
  double tension_cmd, tension_error;
  // Crossfade between hard, short tether PID gains and soft, long tether PID
  // gains.
  CrossfadePidParams(&params->tension_hard_pid, &params->tension_soft_pid,
                     payout, params->hard_spring_payout,
                     params->soft_spring_payout, &tension_pid);
  tension_cmd = CalcTensionAtWing(horizontal_tension_cmd, wing_pos_g);
  tension_error = tension_cmd - tether_force->sph.tension;

  // Zero the tension integrator when first entering PrepTransformGsDown
  // to return the kite to a nominal pitch angle after TransOut.
  // TODO: fade this initialization in after we settle on a
  // new TransOut scheme.
  if ((flight_status->flight_mode == kFlightModeHoverPrepTransformGsDown) &&
      (flight_status->last_flight_mode == kFlightModeHoverTransOut) &&
      (flight_status->flight_mode_first_entry)) {
    state->int_pitch = 0.0;
  }

  // NOTE: The derivative term is handled by the radial
  // position controller.  See generate_hover_controllers.m.
  double pitch_fb = Pid(tension_error, 0.0, *g_sys.ts, int_mode, &tension_pid,
                        &state->int_pitch);

  double pitch_min, pitch_max;
  CalcPitchLimits(payout, params, &pitch_min, &pitch_max);
  double pitch_cmd = Saturate(pitch_ff + pitch_fb, pitch_min, pitch_max);

  // Back-solve the integrator to avoid wind-up when the pitch limit is
  // saturated.
  state->int_pitch -= (pitch_ff + pitch_fb) - pitch_cmd;
  state->int_pitch = Saturate(state->int_pitch, tension_pid.int_output_min,
                              tension_pid.int_output_max);

  // Modify the pitch command in the acceleration flight mode.
  pitch_cmd = ModifyPitchCommandInAccel(
      pitch_cmd, wing_pos_g, flight_status->flight_mode, params, state);

  // Update telemetry.
  HoverTelemetry *ht = GetHoverTelemetry();
  ht->horizontal_tension_cmd = horizontal_tension_cmd;
  ht->tension_cmd = tension_cmd;
  ht->pitch_ff = pitch_ff;
  ht->pitch_fb = pitch_fb;
  ht->pitch_min = pitch_min;
  ht->pitch_max = pitch_max;
  ht->pitch_cmd = pitch_cmd;
  ht->int_pitch = state->int_pitch;

  return pitch_cmd;
}

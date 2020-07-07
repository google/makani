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

#include "control/hover/hover_altitude.h"

#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <string.h>

#include "common/c_math/filter.h"
#include "common/c_math/util.h"
#include "control/common.h"
#include "control/control_params.h"
#include "control/control_telemetry.h"
#include "control/control_types.h"
#include "control/hover/hover_types.h"
#include "control/system_params.h"
#include "control/system_types.h"

// Returns true if the state appears to be valid.
static bool ValidateState(const HoverAltitudeParams *params,
                          const HoverAltitudeState *state) {
  if (!((params->low_altitude_pid.int_output_min <= state->int_thrust ||
         IsApproximatelyEqual(params->low_altitude_pid.int_output_min,
                              state->int_thrust)) &&
        (state->int_thrust <= params->low_altitude_pid.int_output_max ||
         IsApproximatelyEqual(state->int_thrust,
                              params->low_altitude_pid.int_output_max)))) {
    assert(!(bool)"int_thrust is outside the integrator saturations.");
    return false;
  }

  return true;
}

void HoverAltitudeInit(double previous_thrust_cmd,
                       const HoverAltitudeParams *params,
                       HoverAltitudeState *state) {
  assert(params != NULL && state != NULL);
  assert(HoverAltitudeValidateParams(params));

  memset(state, 0, sizeof(*state));
  state->thrust_cmd_z1 = previous_thrust_cmd;
  state->int_thrust = 0.0;
  state->int_boost = 0.0;

  if (!ValidateState(params, state)) assert(false);
}

bool HoverAltitudeValidateParams(const HoverAltitudeParams *params) {
  if (!(params->max_pilot_thrust_to_weight_ratio > 1.0 &&
        params->max_pilot_thrust_to_weight_ratio < 5.0)) {
    assert(!(
        bool)"max_pilot_thrust_to_weight_ratio must be greater than 1 and have"
             " a reasonably small value.");
    return false;
  }

  if (!(params->low_altitude_pid.kp >= 0.0 &&
        params->low_altitude_pid.ki >= 0.0 &&
        params->low_altitude_pid.kd >= 0.0)) {
    assert(!(bool)"low_altitude_pid must have non-negative gains.");
    return false;
  }

  if (!(params->high_altitude_pid.kp >= 0.0 &&
        params->high_altitude_pid.ki >= 0.0 &&
        params->high_altitude_pid.kd >= 0.0)) {
    assert(!(bool)"high_altitude_pid must have non-negative gains.");
    return false;
  }

  if (!(params->low_altitude_pid.int_output_min <=
        params->low_altitude_pid.int_output_max)) {
    assert(!(
        bool)"low_altitude_pid.int_output_max must be greater than or equal"
             " to low_altitude_pid.int_output_min.");
    return false;
  }

  if (!(params->low_altitude_pid.int_output_min ==
            params->high_altitude_pid.int_output_min &&
        params->low_altitude_pid.int_output_max ==
            params->high_altitude_pid.int_output_max)) {
    assert(!(
        bool)"low_altitude_pid and high_altitude_pid must have the same"
             " integrator limits.");
    return false;
  }

  if (!(params->max_thrust > 0.0)) {
    assert(!(bool)"max_thrust must be positive.");
    return false;
  }

  if (!(params->boost_output_min <= params->boost_output_max)) {
    assert(!(
        bool)"boost_output_max must be greater than or equal to"
             " boost_output_min");
  }

  if (!(params->boost_fc > 0.0)) {
    assert(!(bool)"boost_fc must be greater than zero.");
  }
  return true;
}

// Sets the thrust command based on the position of the joystick
// throttle.  Half throttle maps to a thrust-to-weight ratio of 1.0,
// while full throttle maps to the maximum thrust-to-weight ratio.
// This returns a thrust, so we multiply this thrust-to-weight ratio
// by the current feed-forward thrust, thrust_ff.
static double ConvertThrottleToThrust(double joystick_throttle,
                                      double thrust_ff,
                                      const HoverAltitudeParams *params) {
  assert(0.0 <= joystick_throttle && joystick_throttle <= 1.0);
  assert(thrust_ff >= 0.0);

  double delta = params->max_pilot_thrust_to_weight_ratio - 1.0;
  assert(0.0 < delta && delta < 1.0);
  double thrust_to_weight_ratio =
      Crossfade(1.0 - delta, 1.0 + delta, joystick_throttle, 0.0, 1.0);
  return thrust_to_weight_ratio * thrust_ff;
}

// Calculates the feed-forward thrust necessary to support both the
// wing and tether at a given payout.
//
// TODO: Use knowledge of parabola to calculate actual weight
// supported by the wing.
static double CalcFeedForwardThrust(double payout) {
  assert(-1.0 <= payout && payout < g_sys.tether->length + 10.0);

  // We saturate payout so it does not become a critical sensor.
  double payed_tether_mass = Saturate(payout, 0.0, g_sys.tether->length) *
                             g_sys.tether->linear_density;
  return (g_sys.wing->m + payed_tether_mass) * g_sys.phys->g;
}

double HoverAltitudeStep(double wing_pos_z_g_cmd, double wing_pos_z_g,
                         double wing_vel_z_g_cmd, double wing_vel_z_g,
                         double joystick_throttle, double payout,
                         const FlightStatus *flight_status, bool gain_ramp_done,
                         double thrust_z1, const HoverAltitudeParams *params,
                         HoverAltitudeState *state) {
  assert(0.0 <= joystick_throttle && joystick_throttle <= 1.0);
  assert(-1.0 <= payout && payout <= g_sys.tether->length + 10.0);
  assert(0 <= flight_status->flight_mode &&
         flight_status->flight_mode < kNumFlightModes);
  assert(thrust_z1 < 2.0 * g_sys.wing->m * g_sys.phys->g);
  assert(params != NULL && state != NULL && ValidateState(params, state));

  // Calculate the error signals and switch sign on wing position and
  // velocity so that positive is up.
  double altitude_error = -(wing_pos_z_g_cmd - wing_pos_z_g);
  double altitude_vel_error = -(wing_vel_z_g_cmd - wing_vel_z_g);

  // Select the integration mode.  Reset or hold the integrator if the
  // autonomous controller is not in control.  This can happen if the
  // wing is not in an autonomous hover mode or if the wing is on the
  // perch or hanging from constraints.  Since we detect the latter
  // cases based on low thrust, which could also potentially occur
  // during a large step command in altitude, we hold rather than
  // reset the integrator for these cases.
  IntegratorMode int_mode;
  if (!AnyAutoHoverFlightMode(flight_status->flight_mode)) {
    int_mode = kIntegratorModeReset;
  } else if (!gain_ramp_done ||
             flight_status->flight_mode == kFlightModeHoverAccel) {
    int_mode = kIntegratorModeHold;
  } else {
    int_mode = kIntegratorModeIntegrate;
  }

  // Set the 'boost' mode.  This is a second-order integrator (1/s^2)
  // term that we currently apply only in HoverAscend.  If there is a
  // need for closer altitude command tracking in other flight modes,
  // the boost could be enabled in those modes as well.  Care should
  // be taken to limit the boost's ability to increase the kite's
  // velocity in a downwards direction due to the risk of vortex ring
  // state.  Applying the boost in only HoverAscend mitigates that
  // risk.
  bool boost_enable;
  if (params->boost_enabled &&
      flight_status->flight_mode == kFlightModeHoverAscend) {
    boost_enable = true;
  } else {
    boost_enable = false;
  }

  IntegratorMode boost_mode;
  if (boost_enable) {
    boost_mode = int_mode;
    if (!state->boost_enable_z) {
      state->int_boost = state->int_thrust;
      state->int_thrust = 0.0;
    }
  } else {
    boost_mode = kIntegratorModeReset;
    if (state->boost_enable_z) {
      state->int_thrust += state->int_boost;
      state->int_boost = 0.0;
    }
  }

  state->boost_enable_z = boost_enable;

  // Schedule altitude gains based on the altitude command.  This
  // reduces the altitude gains at high altitude to avoid structural
  // vibrations and to avoid exciting modes in the tether.
  PidParams altitude_pid;
  CrossfadePidParams(&params->low_altitude_pid, &params->high_altitude_pid,
                     -wing_pos_z_g_cmd, params->low_altitude,
                     params->high_altitude, &altitude_pid);

  // Don't apply anti-windup until after the gain ramp is complete.
  double thrust_tracking_error =
      gain_ramp_done ? thrust_z1 - state->thrust_cmd_z1 : 0.0;

  // Fade in the thrust feedforward term at the start of HoverTransOut
  double thrust_ff_scale = 1.0;
  if (flight_status->flight_mode == kFlightModeHoverTransOut &&
      flight_status->last_flight_mode == kFlightModeCrosswindPrepTransOut) {
    thrust_ff_scale = Crossfade(0.0, 1.0, flight_status->flight_mode_time,
                                params->transout_thrust_fade_start,
                                params->transout_thrust_fade_end);
  }

  double thrust_ff = thrust_ff_scale * CalcFeedForwardThrust(payout);
  double thrust_fb =
      PidAntiWindup(altitude_error, altitude_vel_error, thrust_tracking_error,
                    *g_sys.ts, int_mode, &altitude_pid, &state->int_thrust);

  // Add a boost term, boost_fc * (2 PI) * thrust_fb / s.  Here
  // boost_fc is the frequency, in Hz, at which the boost term crosses
  // over the existing PID feedback.
  //
  // NOTE: For some reason, performance is much better if I
  // integrate thrust_fb here instead of int_thrust. Perhaps the
  // situation is better when integrating thrust_fb because we
  // effectively get integral control over velocity?
  Integrator(params->boost_fc * (2.0 * PI) * thrust_fb,
             params->boost_output_min, params->boost_output_max, *g_sys.ts,
             boost_mode, &state->int_boost);

  // Further saturate the boost integrator such that the sum of the
  // boost integrator and the regular integrator are within the
  // saturation limits of the regular integrator.  This prevents a
  // step change in thrust when we transfer the state of the boost
  // integrator to the regular integrator on change in flight mode.
  state->int_boost =
      Saturate(state->int_thrust + state->int_boost,
               altitude_pid.int_output_min, altitude_pid.int_output_max) -
      state->int_thrust;

  thrust_fb += state->int_boost;
  double thrust_cmd = thrust_ff + thrust_fb;

  // Modify thrust command based on flight mode.
  if (flight_status->flight_mode == kFlightModePilotHover) {
    // Overwrite thrust_cmd with the value from the throttle.
    thrust_cmd = ConvertThrottleToThrust(joystick_throttle, thrust_ff, params);

    // Because the pilot throttle is effectively acting like an
    // integrator during pilot hover, we set the actual integrator to
    // the extra thrust so there will be a smooth transition to
    // autonomous hover.
    state->int_thrust =
        Saturate(thrust_cmd - thrust_ff, altitude_pid.int_output_min,
                 altitude_pid.int_output_max);
    state->int_boost = 0.0;
  } else if (flight_status->flight_mode == kFlightModeHoverAccel) {
    // Command the maximum thrust during acceleration and
    // transition-in.
    thrust_cmd = params->max_thrust;
  }

  // Apply rate limit and update persistent state.  The update of
  // thrust_cmd_z1 must come before the final saturation for the
  // anti-windup to be effective. The rate limit is intended to only
  // be active in HoverAccel and at the beginning of HoverTransOut. In
  // the latter case the rate limit is used, in conjunction with
  // zeroing of the altitude error, to enforce continuity of thrust
  // command with CrosswindPrepTransOut.
  thrust_cmd =
      RateLimit(thrust_cmd, -params->max_thrust_rate, params->max_thrust_rate,
                *g_sys.ts, &state->thrust_cmd_z1);

  // Update telemetry.
  GetHoverTelemetry()->thrust_ff = thrust_ff;
  GetHoverTelemetry()->thrust_fb = thrust_fb;
  GetHoverTelemetry()->int_thrust = state->int_thrust;
  GetHoverTelemetry()->int_boost = state->int_boost;

  // Saturate and return thrust command.  We saturate the output here
  // to avoid overwhelming the output stage's thrust-moment solver
  // with an unrealistically high thrust command.
  return Saturate(thrust_cmd, 0.0, params->max_thrust);
}

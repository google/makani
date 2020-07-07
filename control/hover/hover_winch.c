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

#include "control/hover/hover_winch.h"

#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "common/c_math/filter.h"
#include "common/c_math/util.h"
#include "common/macros.h"
#include "control/control_params.h"
#include "control/control_types.h"
#include "control/estimator/estimator_types.h"
#include "control/hover/hover_types.h"
#include "control/sensor_types.h"
#include "control/sensor_util.h"
#include "control/system_params.h"

COMPILE_ASSERT(
    ARRAYSIZE(((HoverWinchParams *)NULL)->winch_position_pay_out_table) ==
        ARRAYSIZE(((HoverWinchParams *)NULL)->winch_speed_pay_out_table),
    winch_position_and_speed_pay_out_tables_must_have_same_length);

COMPILE_ASSERT(
    ARRAYSIZE(((HoverWinchParams *)NULL)->winch_position_reel_in_table) ==
        ARRAYSIZE(((HoverWinchParams *)NULL)->winch_speed_reel_in_table),
    winch_position_and_speed_reel_in_tables_must_have_same_length);

static bool ValidateState(const HoverWinchParams *params,
                          const HoverWinchState *state) {
  if (!(fabs(state->winch_vel_cmd_z1) <= params->max_winch_speed)) {
    assert(!(
        bool)"winch_vel_cmd_z1 is greater than the maximum allowed winch"
             " velocity.");
    return false;
  }

  return true;
}

void HoverWinchInit(const HoverWinchParams *params, HoverWinchState *state) {
  assert(params != NULL && state != NULL);
  assert(HoverWinchValidateParams(params));

  memset(state, 0, sizeof(*state));
  state->winch_vel_cmd_z1 = 0.0;

  if (!ValidateState(params, state)) assert(false);
}

// These checks are intentionally loose, so that they may apply to a
// wide range of systems.  They are only meant to catch serious errors
// in the parameters.
bool HoverWinchValidateParams(const HoverWinchParams *params) {
  for (int32_t i = 0; i < ARRAYSIZE(params->winch_speed_pay_out_table); ++i) {
    if (!(params->winch_position_pay_out_table[i] <= 0.0)) {
      assert(!(bool)"winch_position_pay_out_table must be non-positive.");
      return false;
    }

    if (i > 0 &&
        !(params->winch_position_pay_out_table[i] >
          params->winch_position_pay_out_table[i - 1])) {
      assert(
          !(bool)"winch_position_pay_out_table must be strictly increasing.");
      return false;
    }

    if (!(params->winch_speed_pay_out_table[i] >= 0.0 &&
          params->winch_speed_pay_out_table[i] <= params->max_winch_speed)) {
      assert(!(
          bool)"winch_speed_pay_out_table must be positive and less than"
               " max_winch_speed.");
      return false;
    }
  }

  for (int32_t i = 0; i < ARRAYSIZE(params->winch_speed_reel_in_table); ++i) {
    if (!(params->winch_position_reel_in_table[i] <= 0.0)) {
      assert(!(bool)"winch_position_reel_in_table must be non-positive.");
      return false;
    }

    if (i > 0 &&
        !(params->winch_position_reel_in_table[i] >
          params->winch_position_reel_in_table[i - 1])) {
      assert(
          !(bool)"winch_position_reel_in_table must be strictly increasing.");
      return false;
    }

    if (!(params->winch_speed_reel_in_table[i] >= 0.0 &&
          params->winch_speed_reel_in_table[i] <= params->max_winch_speed)) {
      assert(!(
          bool)"winch_speed_reel_in_table must be positive and less than"
               " max_winch_speed.");
      return false;
    }
  }

  if (!(params->contact_payout > 0.0 && params->contact_payout <= 10.0)) {
    assert(!(bool)"contact_payout must be positive and reasonably small.");
    return false;
  }

  if (!(params->contact_winch_speed > 0.0 &&
        params->contact_winch_speed <= 10.0)) {
    assert(!(bool)"contact_winch_speed must be positive and reasonably small.");
    return false;
  }

  if (!(params->max_winch_speed >= params->contact_winch_speed)) {
    assert(!(
        bool)"max_winch_speed must be greater than or equal to"
             " contact_winch_speed.");
    return false;
  }

  if (!(params->max_tension >= 1000.0)) {
    assert(!(bool)"max_tension must be positive and reasonably large.");
    return false;
  }

  return true;
}

// The lookup tables define winch velocity as a function of winch
// position.  In these tables a linear increase in velocity with
// respect to position would lead to exponential acceleration because
// the solution to the differential equation v(t) = x(t) is:
//
//   v(t) = exp(t)
//   a(t) = exp(t)
//
// Warping the table with sqrt() leads to constant acceleration
// profiles because the solution to the differential equation v(t) =
// sqrt(x(t)) is:
//
//   v(t) = t/2
//   a(t) = 1/2 (a constant!)
//
// This uses winch position rather than pay-out because it is
// important for the pay-out to stop exactly in crosswind position.
static double CalcPayoutWinchVelocity(const WinchEstimate *winch,
                                      const HoverWinchParams *params) {
  assert(fabs(winch->payout - winch->position - g_sys.tether->length) < 10.0 ||
         !winch->valid);

  // If the winch estimate is invalid, use the "contact" winch speed.
  if (!winch->valid) {
    return params->contact_winch_speed;
  } else {
    return Interp1WarpY(params->winch_position_pay_out_table,
                        params->winch_speed_pay_out_table,
                        HOVER_WINCH_PAY_OUT_TABLE_LENGTH, winch->position,
                        kInterpOptionSaturate, &Square, &Sqrt);
  }
}

// TODO: This code ignores winch->proximity_valid.
static double CalcReelInWinchVelocity(const WinchEstimate *winch,
                                      const HoverWinchParams *params) {
  // TODO(b/26382449): Instead of checking for kControlOptHardCodeInitialPayout
  // in this assert, consider also overriding winch->position.
  assert(fabs(winch->payout - winch->position - g_sys.tether->length) < 10.0 ||
         !winch->valid ||
         (*g_cont.control_opt & kControlOptHardCodeInitialPayout));

  // Slow reel-in when close to the perch (or the winch estimate is invalid).
  if (!winch->valid || winch->payout < params->contact_payout) {
    return -params->contact_winch_speed;
  } else {
    // See comment above about warping.  Also, the warping function
    // doesn't care about the sign of winch_speed_reel_in_tab, but
    // obviously the sign should be negative for reel-in.
    double winch_vel_cmd = -Interp1WarpY(
        params->winch_position_reel_in_table, params->winch_speed_reel_in_table,
        HOVER_WINCH_REEL_IN_TABLE_LENGTH, winch->position,
        kInterpOptionSaturate, &Square, &Sqrt);

    return winch_vel_cmd;
  }
}

// Returns the value of the joystick pitch, saturated to [-1, 1], and set to 0.0
// if the joystick estimate is invalid.
static double GetJoystickPitch(const JoystickEstimate *joystick) {
  assert(!joystick->valid ||
         (-1.0 <= joystick->data.pitch && joystick->data.pitch <= 1.0));
  return joystick->valid ? Saturate(joystick->data.pitch, -1.0, 1.0) : 0.0;
}

// Uses the pitch stick to control winch speed during the HoverInPlace flight
// plan. A positive pitch signals payout, while a negative pitch signals
// reel-in. The speed is scaled by the magnitude of the joystick pitch.
static double CalcHoverInPlaceWinchVelocity(const WinchEstimate *winch,
                                            const JoystickEstimate *joystick,
                                            const HoverWinchParams *params) {
  // Make (-0.05, 0.05) a deadband on the pitch scale. Modify the resulting
  // scale to make it continuously range from -1 to 1.
  double scale = GetJoystickPitch(joystick);
  if (scale < -0.05) {
    scale += 0.05;
  } else if (scale > 0.05) {
    scale -= 0.05;
  } else {
    scale = 0.0;
  }
  scale /= 0.95;

  double winch_vel_cmd = 0.0;
  if (scale <= 0.0) {
    winch_vel_cmd = -scale * CalcReelInWinchVelocity(winch, params);
  } else {
    winch_vel_cmd = scale * CalcPayoutWinchVelocity(winch, params);
  }
  return winch_vel_cmd;
}

// Scales the winch velocity command up or down, by up to 100%,
// according to the position of the joystick pitch stick.  Moving the
// pitch stick down increases velocity.  Returns the scaled winch
// velocity command.
static double ScaleWinchVelocityWithJoystick(double winch_vel_cmd,
                                             const JoystickEstimate *joystick) {
  return (1.0 + GetJoystickPitch(joystick)) * winch_vel_cmd;
}

// Applies final saturations, rate limits, and safety checks to the
// winch velocity command.
static double LimitWinchVelocity(double winch_vel_cmd,
                                 const WinchEstimate *winch,
                                 const TetherForceEstimate *tether,
                                 const HoverWinchParams *params,
                                 HoverWinchState *state) {
  // TODO(b/26382449): Instead of checking for kControlOptHardCodeInitialPayout
  // in this assert, consider also overriding winch->position.
  assert(fabs(winch->payout - winch->position - g_sys.tether->length) < 10.0 ||
         !winch->valid ||
         (*g_cont.control_opt & kControlOptHardCodeInitialPayout));

  double limited_winch_vel_cmd = Saturate(
      winch_vel_cmd, -params->max_winch_speed, params->max_winch_speed);

  limited_winch_vel_cmd =
      RateLimit(limited_winch_vel_cmd, -params->max_winch_accel,
                params->max_winch_accel, *g_sys.ts, &state->winch_vel_cmd_z1);

  // The following end-of-travel stop commands intentionally
  // circumvent the rate-limiting to ensure crisp stopping.

  // If the winch position is greater than zero, then stop pay-out
  // because the winch is beyond crosswind position.
  if (winch->valid && winch->position >= 0.0) {
    limited_winch_vel_cmd = fmin(0.0, limited_winch_vel_cmd);
  }

  // If the proximity sensor is active or if the tension is above the maximum
  // safe winch-in value (presumably because it is trying to reel the wing in
  // while perched), do not allow reel-in.
  //
  // TODO: This code ignores winch->proximity_valid.
  if (winch->proximity ||
      (tether->valid && tether->sph.tension > params->max_tension)) {
    limited_winch_vel_cmd = fmax(0.0, limited_winch_vel_cmd);
  }

  return limited_winch_vel_cmd;
}

double HoverWinchStep(FlightMode flight_mode, const StateEstimate *state_est,
                      const HoverWinchParams *params, HoverWinchState *state) {
  assert(0 <= flight_mode && flight_mode < kNumFlightModes);
  assert(state_est != NULL);
  assert(params != NULL && state != NULL && ValidateState(params, state));

  // Set base winch velocity based on flight mode and winch position.
  double winch_vel_cmd;
  bool scale_velocity_with_joystick = true;
  if (*g_cont.flight_plan == kFlightPlanStartDownwind) {
    // Note that
    // this is important even if a winch isn't installed because
    // winch_vel_cmd is used in the path and tension controllers.
    winch_vel_cmd = 0.0;
  } else if (flight_mode == kFlightModeHoverPayOut) {
    winch_vel_cmd = CalcPayoutWinchVelocity(&state_est->winch, params);
  } else if (flight_mode == kFlightModeHoverReelIn) {
    winch_vel_cmd = CalcReelInWinchVelocity(&state_est->winch, params);
  } else if (*g_cont.flight_plan == kFlightPlanHoverInPlace &&
             flight_mode == kFlightModeHoverFullLength) {
    scale_velocity_with_joystick = false;
    winch_vel_cmd = CalcHoverInPlaceWinchVelocity(&state_est->winch,
                                                  &state_est->joystick, params);
  } else {
    winch_vel_cmd = 0.0;
  }

  // The commanded winch speed should never be greater than the
  // maximum winch speed from the lookup tables alone.
  assert(fabs(winch_vel_cmd) <= params->max_winch_speed);

  // Adjust winch speed with the joystick. The speed command is scaled from 0 to
  // 2 times depending on the joystick pitch, except for during HoverInPlace, in
  // which the joystick pitch has already determined the speed.
  if (scale_velocity_with_joystick) {
    winch_vel_cmd =
        ScaleWinchVelocityWithJoystick(winch_vel_cmd, &state_est->joystick);
  }

  // Apply saturations, rate limits, and safety checks to winch
  // velocity command.
  winch_vel_cmd = LimitWinchVelocity(winch_vel_cmd, &state_est->winch,
                                     &state_est->tether_force_b, params, state);

  return winch_vel_cmd;
}

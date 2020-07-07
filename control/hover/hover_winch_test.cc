// Copyright 2020 Makani Technologies LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>

#include <stdint.h>
#include <string.h>

#include "common/macros.h"
#include "control/control_params.h"
#include "control/control_types.h"
#include "control/estimator/estimator_types.h"
#include "control/hover/hover_types.h"
#include "control/hover/hover_winch.h"
#include "control/sensor_types.h"
#include "control/sensor_util.h"
#include "control/system_params.h"

extern "C" {

bool ValidateState(const HoverWinchParams *params,
                   const HoverWinchState *state);

double CalcPayoutWinchVelocity(const WinchEstimate *winch,
                               const HoverWinchParams *params);

double CalcReelInWinchVelocity(const WinchEstimate *winch,
                               const HoverWinchParams *params);

double ScaleWinchVelocityWithJoystick(double winch_vel_cmd,
                                      const JoystickEstimate *joystick);

double LimitWinchVelocity(double winch_vel_cmd, const WinchEstimate *winch,
                          const TetherForceEstimate *tether,
                          const HoverWinchParams *params,
                          HoverWinchState *state);

}  // extern "C"

TEST(ValidateState, Default) {
  const HoverWinchParams &params = GetControlParams()->hover.winch;
  HoverWinchState state;
  HoverWinchInit(&params, &state);
  EXPECT_TRUE(ValidateState(&params, &state));
}

TEST(HoverWinchInit, SameInitialization) {
  HoverWinchState state0, state1;
  memset(&state0, 0, sizeof(state0));
  memset(&state1, 0xFF, sizeof(state1));
  HoverWinchInit(&GetControlParams()->hover.winch, &state0);
  HoverWinchInit(&GetControlParams()->hover.winch, &state1);
  EXPECT_TRUE(!memcmp(&state0, &state1, sizeof(state0)));
}

TEST(HoverWinchValidateParams, Default) {
  EXPECT_TRUE(HoverWinchValidateParams(&GetControlParams()->hover.winch));
}

TEST(CalcPayoutWinchVelocity, Normal) {
  const HoverWinchParams &params = GetControlParams()->hover.winch;
  WinchEstimate winch = WinchEstimate();

  // Default to contact winch speed if the estimate is invalid.
  winch.valid = false;
  winch.payout = 100.0;
  winch.position = winch.payout - g_sys.tether->length;
  EXPECT_NEAR(params.contact_winch_speed,
              CalcPayoutWinchVelocity(&winch, &params), 1e-9);

  // Otherwise, use the interpolated speed.
  winch.valid = true;
  for (int32_t i = 0; i < ARRAYSIZE(params.winch_position_pay_out_table); ++i) {
    winch.position = params.winch_position_pay_out_table[i];
    winch.payout = winch.position + g_sys.tether->length;
    if (winch.payout >= params.contact_payout) {
      EXPECT_NEAR(params.winch_speed_pay_out_table[i],
                  CalcPayoutWinchVelocity(&winch, &params), 1e-9);
    }
  }
}

TEST(CalcReelInWinchVelocity, Normal) {
  const HoverWinchParams &params = GetControlParams()->hover.winch;
  WinchEstimate winch = WinchEstimate();

  // Default to contact winch speed if the estimate is invalid, the
  // proximity sensor is active, or the payout is small.
  winch.valid = false;
  winch.payout = 100.0;
  winch.position = winch.payout - g_sys.tether->length;
  EXPECT_NEAR(-params.contact_winch_speed,
              CalcReelInWinchVelocity(&winch, &params), 1e-9);
  winch.payout = 0.0;
  winch.position = winch.payout - g_sys.tether->length;
  EXPECT_NEAR(-params.contact_winch_speed,
              CalcReelInWinchVelocity(&winch, &params), 1e-9);

  // Otherwise, use the interpolated speed.
  winch.valid = true;
  for (int32_t i = 0; i < ARRAYSIZE(params.winch_position_reel_in_table); ++i) {
    winch.position = params.winch_position_reel_in_table[i];
    winch.payout = winch.position + g_sys.tether->length;
    EXPECT_NEAR(-params.winch_speed_reel_in_table[i],
                CalcReelInWinchVelocity(&winch, &params), 1e-9);
  }
}

TEST(ScaleWinchVelocityWithJoystick, Normal) {
  JoystickEstimate joystick = JoystickEstimate();

  joystick.valid = true;
  joystick.data.pitch = 0.0;
  EXPECT_NEAR(1.0, ScaleWinchVelocityWithJoystick(1.0, &joystick), 1e-9);

  joystick.valid = false;
  joystick.data.pitch = 0.5;
  EXPECT_NEAR(1.0, ScaleWinchVelocityWithJoystick(1.0, &joystick), 1e-9);

  joystick.valid = true;
  joystick.data.pitch = 1.0;
  EXPECT_NEAR(2.0, ScaleWinchVelocityWithJoystick(1.0, &joystick), 1e-9);

  joystick.valid = true;
  joystick.data.pitch = -1.0;
  EXPECT_NEAR(0.0, ScaleWinchVelocityWithJoystick(1.0, &joystick), 1e-9);
}

TEST(LimitWinchVelocity, Normal) {
  const HoverWinchParams &params = GetControlParams()->hover.winch;
  HoverWinchState state = HoverWinchState();
  WinchEstimate winch = WinchEstimate();
  winch.position = winch.payout - g_sys.tether->length;
  TetherForceEstimate tether = TetherForceEstimate();

  // No limit with a reasonable winch velocity.
  state.winch_vel_cmd_z1 = 1.0;
  winch.proximity_valid = true;
  winch.proximity = false;
  EXPECT_NEAR(1.0, LimitWinchVelocity(1.0, &winch, &tether, &params, &state),
              1e-9);

  // Limit to a positive value with the proximity sensor active.
  state.winch_vel_cmd_z1 = -1.0;
  winch.proximity = true;
  EXPECT_NEAR(0.0, LimitWinchVelocity(-1.0, &winch, &tether, &params, &state),
              1e-9);
  state.winch_vel_cmd_z1 = 1.0;
  winch.proximity = true;
  EXPECT_NEAR(1.0, LimitWinchVelocity(1.0, &winch, &tether, &params, &state),
              1e-9);

  // Limit to a positive value with high tether forces.
  tether.valid = true;
  tether.sph.tension = params.max_tension + 1.0;
  state.winch_vel_cmd_z1 = -1.0;
  winch.proximity = false;
  EXPECT_NEAR(0.0, LimitWinchVelocity(-1.0, &winch, &tether, &params, &state),
              1e-9);
  state.winch_vel_cmd_z1 = 1.0;
  EXPECT_NEAR(1.0, LimitWinchVelocity(1.0, &winch, &tether, &params, &state),
              1e-9);

  tether.valid = true;
  tether.sph.tension = params.max_tension / 10.0;
  state.winch_vel_cmd_z1 = -1.0;
  winch.proximity = true;
  EXPECT_NEAR(0.0, LimitWinchVelocity(-1.0, &winch, &tether, &params, &state),
              1e-9);
  state.winch_vel_cmd_z1 = 1.0;
  EXPECT_NEAR(1.0, LimitWinchVelocity(1.0, &winch, &tether, &params, &state),
              1e-9);
}

// The winch controller should always return zero (assuming the rate
// limit isn't active), for all but the pay-out and reel-in flight
// modes.
TEST(HoverWinchStep, Zero) {
  const HoverWinchParams &params = GetControlParams()->hover.winch;
  StateEstimate state_est = StateEstimate();
  HoverWinchState state = HoverWinchState();

  for (int32_t i = 0; i < kNumFlightModes; ++i) {
    if (i != static_cast<int32_t>(kFlightModeHoverPayOut) &&
        i != static_cast<int32_t>(kFlightModeHoverReelIn)) {
      // Deactivate rate limit.
      state.winch_vel_cmd_z1 = 0.0;

      state_est.winch.valid = true;
      state_est.winch.payout = 100.0;
      state_est.winch.position = state_est.winch.payout - g_sys.tether->length;
      EXPECT_NEAR(0.0, HoverWinchStep(static_cast<FlightMode>(i), &state_est,
                                      &params, &state),
                  1e-9);
    }
  }
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

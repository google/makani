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

#include <math.h>
#include <stdint.h>
#include <string.h>

#include "common/macros.h"
#include "control/control_params.h"
#include "control/control_types.h"
#include "control/estimator/estimator_types.h"
#include "control/hover/hover_position.h"
#include "control/hover/hover_types.h"
#include "control/system_params.h"
#include "lib/util/test_util.h"

extern "C" {

bool ValidateState(const HoverPositionParams *params,
                   const HoverPositionState *state);

void CalcVelocityError(const Vec3 *wing_vel_g_cmd, const Vec3 *wing_vel_g,
                       const Mat3 *dcm_g2b, double tangential_kd,
                       const FlightStatus *flight_status,
                       const HoverPositionParams *params,
                       Vec3 *wing_vel_b_error);

void CalcPositionError(const Vec3 *wing_pos_g_cmd, const Vec3 *wing_pos_g,
                       const Vec3 *hover_origin_g, const Mat3 *dcm_g2b,
                       double tangential_kp, const HoverPositionParams *params,
                       Vec3 *wing_pos_b_error);

void CalcFeedForwardAngles(double pitch_cmd, const HoverPositionParams *params,
                           Vec3 *angles_ff);

void SchedulePid(double payout, double wing_pos_z_g,
                 const HoverPositionParams *params, PidParams *radial_pid,
                 PidParams *tangential_pid);

void ConvertJoystickToAngles(const JoystickData *joystick,
                             const HoverPositionParams *params,
                             Vec3 *angles_fb);

}  // extern "C"

TEST(ValidateState, Default) {
  const HoverPositionParams &params = GetControlParams()->hover.position;
  HoverPositionState state;
  HoverPositionInit(0.0, &params, &state);
  EXPECT_TRUE(ValidateState(&params, &state));
}

TEST(HoverPositionInit, SameInitialization) {
  HoverPositionState state0, state1;
  memset(&state0, 0, sizeof(state0));
  memset(&state1, 0xFF, sizeof(state1));
  HoverPositionInit(0.0, &GetControlParams()->hover.position, &state0);
  HoverPositionInit(0.0, &GetControlParams()->hover.position, &state1);
  EXPECT_TRUE(!memcmp(&state0, &state1, sizeof(state0)));
}

TEST(HoverPositionValidateParams, Default) {
  EXPECT_TRUE(HoverPositionValidateParams(&GetControlParams()->hover.position));
}

TEST(CalcVelocityError, Saturated) {
  HoverPositionParams params = HoverPositionParams();
  params.max_vel_angle_fb = 0.1;

  FlightStatus flight_status = FlightStatus();
  flight_status.flight_mode = kFlightModeHoverPayOut;

  Mat3 dcm_g2b;
  AngleToDcm(0.0, PI / 2.0, 0.0, kRotationOrderZyx, &dcm_g2b);

  Vec3 wing_vel_g_cmd = {3.0, -2.0, 1.0};
  Vec3 wing_vel_g = {-22.0, 5.0, -4.0};
  Vec3 wing_vel_b_error;
  CalcVelocityError(&wing_vel_g_cmd, &wing_vel_g, &dcm_g2b, 0.01,
                    &flight_status, &params, &wing_vel_b_error);

  Vec3 ans = {0.0, -7.0, 10.0};
  EXPECT_NEAR_VEC3(wing_vel_b_error, ans, 1e-9);
}

TEST(CalcPositionError, Saturated) {
  HoverPositionParams params = HoverPositionParams();
  params.max_pos_angle_fb = 0.1;

  Mat3 dcm_g2b;
  AngleToDcm(0.0, PI / 2.0, 0.0, kRotationOrderZyx, &dcm_g2b);

  Vec3 wing_pos_g_cmd = {1.0, -22.0, 3.0};
  Vec3 wing_pos_g = {-4.0, 0.0, -6.0};
  Vec3 hover_origin_g = kVec3Zero;
  Vec3 wing_pos_b_error;
  CalcPositionError(&wing_pos_g_cmd, &wing_pos_g, &hover_origin_g, &dcm_g2b,
                    0.01, &params, &wing_pos_b_error);

  Vec3 ans = {0.0, -10.0, 0.0};
  EXPECT_NEAR_VEC3(wing_pos_b_error, ans, 1e-9);
}

TEST(CalcFeedForwardAngles, ZeroFeedForward) {
  HoverPositionParams params = HoverPositionParams();
  params.eulers_ff.x = 0.0;
  params.eulers_ff.y = 0.0;
  params.eulers_ff.z = 0.0;
  Vec3 angles_ff;
  CalcFeedForwardAngles(0.0, &params, &angles_ff);
  EXPECT_NEAR_VEC3(angles_ff, kVec3Zero, 1e-9);
}

TEST(CalcFeedForwardAngles, SmallEulers) {
  HoverPositionParams params = HoverPositionParams();
  params.eulers_ff.x = 1e-6;
  params.eulers_ff.y = 2e-6;
  params.eulers_ff.z = 3e-6;
  Vec3 angles_ff;
  CalcFeedForwardAngles(0.0, &params, &angles_ff);
  EXPECT_NEAR(angles_ff.x, params.eulers_ff.x, 1e-9);
  EXPECT_NEAR(angles_ff.y, params.eulers_ff.y, 1e-9);
  EXPECT_NEAR(angles_ff.z, params.eulers_ff.z, 1e-9);
}

TEST(CalcFeedForwardAngles, JustPitchCommand) {
  HoverPositionParams params = HoverPositionParams();
  params.eulers_ff.x = 0.0;
  params.eulers_ff.y = 0.0;
  params.eulers_ff.z = 0.0;
  Vec3 angles_ff;
  CalcFeedForwardAngles(0.1, &params, &angles_ff);
  EXPECT_NEAR(angles_ff.x, 0.0, 1e-9);
  EXPECT_NEAR(angles_ff.y, 0.1, 1e-9);
  EXPECT_NEAR(angles_ff.z, 0.0, 1e-9);
}

TEST(SchedulePid, ZeroPayout) {
  const HoverPositionParams &params = GetControlParams()->hover.position;

  PidParams radial_pid, tangential_pid;
  SchedulePid(0.0, 0.0, &params, &radial_pid, &tangential_pid);
  EXPECT_NEAR(radial_pid.kp, params.short_tether_radial_pid.kp, 1e-9);
  EXPECT_NEAR(radial_pid.ki, params.short_tether_radial_pid.ki, 1e-9);
  EXPECT_NEAR(radial_pid.kd, params.short_tether_radial_pid.kd, 1e-9);
  EXPECT_NEAR(radial_pid.int_output_min,
              params.short_tether_radial_pid.int_output_min, 1e-9);
  EXPECT_NEAR(radial_pid.int_output_max,
              params.short_tether_radial_pid.int_output_max, 1e-9);
  EXPECT_NEAR(tangential_pid.kp, params.short_tether_tangential_pid.kp, 1e-9);
  EXPECT_NEAR(tangential_pid.ki, params.short_tether_tangential_pid.ki, 1e-9);
  EXPECT_NEAR(tangential_pid.kd, params.short_tether_tangential_pid.kd, 1e-9);
  EXPECT_NEAR(tangential_pid.int_output_min,
              params.short_tether_tangential_pid.int_output_min, 1e-9);
  EXPECT_NEAR(tangential_pid.int_output_max,
              params.short_tether_tangential_pid.int_output_max, 1e-9);
}

TEST(SchedulePid, FullPayout) {
  const HoverPositionParams &params = GetControlParams()->hover.position;

  PidParams radial_pid, tangential_pid;
  SchedulePid(g_sys.tether->length, -g_sys.tether->length, &params, &radial_pid,
              &tangential_pid);
  EXPECT_NEAR(radial_pid.kp, params.long_tether_radial_pid.kp, 1e-9);
  EXPECT_NEAR(radial_pid.ki, params.long_tether_radial_pid.ki, 1e-9);
  EXPECT_NEAR(radial_pid.kd, params.long_tether_radial_pid.kd, 1e-9);
  EXPECT_NEAR(radial_pid.int_output_min,
              params.long_tether_radial_pid.int_output_min, 1e-9);
  EXPECT_NEAR(radial_pid.int_output_max,
              params.long_tether_radial_pid.int_output_max, 1e-9);
  EXPECT_NEAR(tangential_pid.kp,
              params.high_altitude_long_tether_tangential_pid.kp, 1e-9);
  EXPECT_NEAR(tangential_pid.ki,
              params.high_altitude_long_tether_tangential_pid.ki, 1e-9);
  EXPECT_NEAR(tangential_pid.kd,
              params.high_altitude_long_tether_tangential_pid.kd, 1e-9);
  EXPECT_NEAR(tangential_pid.int_output_min,
              params.high_altitude_long_tether_tangential_pid.int_output_min,
              1e-9);
  EXPECT_NEAR(tangential_pid.int_output_max,
              params.high_altitude_long_tether_tangential_pid.int_output_max,
              1e-9);
}

TEST(ConvertJoystickToAngles, Normal) {
  HoverPositionParams params = HoverPositionParams();
  params.k_pilot.x = 1.0;
  params.k_pilot.y = -2.0;
  params.k_pilot.z = 3.0;

  JoystickData joystick = JoystickData();
  joystick.roll = -0.1;
  joystick.pitch = -0.2;
  joystick.yaw = 0.3;

  Vec3 angles_fb;
  ConvertJoystickToAngles(&joystick, &params, &angles_fb);

  Vec3 ans = {-0.1, 0.4, 0.9};
  EXPECT_NEAR_VEC3(angles_fb, ans, 1e-9);
}

TEST(HoverPositionStep, ZeroFeedback) {
  const HoverPositionParams &params = GetControlParams()->hover.position;
  HoverPositionState state = HoverPositionState();

  FlightStatus flight_status = FlightStatus();
  flight_status.flight_mode = kFlightModeHoverPayOut;

  JoystickData joystick = JoystickData();
  joystick.roll = 0.0;
  joystick.pitch = 0.0;
  joystick.yaw = 0.0;
  Vec3 wing_pos_g = {-10.0, 0.0, 0.0};
  Vec3 hover_origin_g = {0.0, 0.0, 0.0};
  Vec3 wing_vel_g = kVec3Zero;
  Vec3 wing_pqr = kVec3Zero;
  Vec3 wing_angles = kVec3Zero;
  Vec3 angles_cmd;
  Vec3 pqr_cmd;
  HoverPositionStep(&wing_pos_g, &wing_pos_g, &wing_vel_g, &wing_vel_g,
                    &hover_origin_g, &wing_pqr, &wing_angles, 0.0, 0.0,
                    &joystick, &kMat3Identity, 10.0, &flight_status, 0.0,
                    &params, &state, &pqr_cmd, &angles_cmd);

  Vec3 angles_ff;
  CalcFeedForwardAngles(0.0, &params, &angles_ff);
  EXPECT_NEAR_VEC3(angles_cmd, angles_ff, 1e-9);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

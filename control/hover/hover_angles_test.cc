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
#include <string.h>

#include "common/c_math/mat3.h"
#include "common/c_math/util.h"
#include "common/c_math/vec3.h"
#include "control/control_params.h"
#include "control/control_types.h"
#include "control/hover/hover.h"
#include "control/hover/hover_angles.h"
#include "control/system_params.h"
#include "lib/util/test_util.h"

extern "C" {

bool ValidateState(const HoverAnglesParams *params,
                   const HoverAnglesState *state);

void CalcAnglesError(const Vec3 *angles_cmd, const Vec3 *angles,
                     Vec3 *angles_error);

double ModifyPitchMomentDuringPerching(bool perching, double pitch_moment,
                                       double pitch_error,
                                       const HoverAnglesParams *params,
                                       double *extra_pitch_moment_z1);

}  // extern "C"

TEST(ValidateState, Default) {
  const HoverAnglesParams &params = GetControlParams()->hover.angles;
  HoverAnglesState state;
  HoverAnglesInit(&params, &state);
  EXPECT_TRUE(ValidateState(&params, &state));
}

TEST(HoverAnglesInit, SameInitialization) {
  HoverAnglesState state0, state1;
  memset(&state0, 0, sizeof(state0));
  memset(&state1, 0xFF, sizeof(state1));
  HoverAnglesInit(&GetControlParams()->hover.angles, &state0);
  HoverAnglesInit(&GetControlParams()->hover.angles, &state1);
  EXPECT_TRUE(!memcmp(&state0, &state1, sizeof(state0)));
}

TEST(HoverAnglesValidateParams, Default) {
  EXPECT_TRUE(HoverAnglesValidateParams(&GetControlParams()->hover.angles));
}

TEST(HoverAnglesGetAnglesError, Normal) {
  HoverAnglesState state = HoverAnglesState();
  state.angles_error.x = 1.0;
  state.angles_error.y = 2.0;
  state.angles_error.z = 3.0;
  Vec3 angles_error;
  HoverAnglesGetAnglesError(&state, &angles_error);
  EXPECT_NEAR_VEC3(angles_error, state.angles_error, 1e-9);
}

TEST(HoverAnglesGetAngles, Home) {
  Vec3 wing_pos_g = {-140.0, 0.0, 0.0};
  Vec3 hover_origin_g = kVec3Zero;
  Mat3 dcm_g2b = kMat3Identity;
  Vec3 hover_angles = kVec3Zero;
  HoverAnglesGetAngles(&wing_pos_g, &hover_origin_g, &dcm_g2b, &hover_angles);

  EXPECT_NEAR(0.0, hover_angles.x, 1e-9);
  EXPECT_NEAR(-M_PI / 2.0, hover_angles.y, 1e-9);
  EXPECT_NEAR(0.0, hover_angles.z, 1e-9);
}

TEST(HoverAnglesGetAngles, HoverHome) {
  Vec3 wing_pos_g = {-140.0, 0.0, 0.0};
  Vec3 hover_origin_g = kVec3Zero;
  Mat3 dcm_g2b = {{{0.0, 0.0, -1.0}, {0.0, 1.0, 0.0}, {1.0, 0.0, 0.0}}};
  Vec3 hover_angles = kVec3Zero;
  HoverAnglesGetAngles(&wing_pos_g, &hover_origin_g, &dcm_g2b, &hover_angles);

  EXPECT_NEAR(0.0, hover_angles.x, 1e-9);
  EXPECT_NEAR(0.0, hover_angles.y, 1e-9);
  EXPECT_NEAR(0.0, hover_angles.z, 1e-9);
}

TEST(HoverAnglesGetAngles, HoverHomePosY) {
  Vec3 wing_pos_g = {-140.0, 140.0, 0.0};
  Vec3 hover_origin_g = kVec3Zero;
  Mat3 dcm_g2b = {{{0.0, 0.0, -1.0}, {0.0, 1.0, 0.0}, {1.0, 0.0, 0.0}}};
  Vec3 hover_angles = kVec3Zero;
  HoverAnglesGetAngles(&wing_pos_g, &hover_origin_g, &dcm_g2b, &hover_angles);

  EXPECT_NEAR(-M_PI / 4.0, hover_angles.x, 1e-9);
  EXPECT_NEAR(0.0, hover_angles.y, 1e-9);
  EXPECT_NEAR(0.0, hover_angles.z, 1e-9);
}

TEST(HoverAnglesGetAngles, HoverHomeNegY) {
  Vec3 wing_pos_g = {-140.0, -140.0, 0.0};
  Vec3 hover_origin_g = kVec3Zero;
  Mat3 dcm_g2b = {{{0.0, 0.0, -1.0}, {0.0, 1.0, 0.0}, {1.0, 0.0, 0.0}}};
  Vec3 hover_angles = kVec3Zero;
  HoverAnglesGetAngles(&wing_pos_g, &hover_origin_g, &dcm_g2b, &hover_angles);

  EXPECT_NEAR(M_PI / 4.0, hover_angles.x, 1e-9);
  EXPECT_NEAR(0.0, hover_angles.y, 1e-9);
  EXPECT_NEAR(0.0, hover_angles.z, 1e-9);
}

TEST(HoverAnglesGetAngles, PitchBack90) {
  Vec3 wing_pos_g = {-140.0, 0.0, 0.0};
  Vec3 hover_origin_g = kVec3Zero;
  Mat3 dcm_g2b = {{{-1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, -1.0}}};
  Vec3 hover_angles = kVec3Zero;
  HoverAnglesGetAngles(&wing_pos_g, &hover_origin_g, &dcm_g2b, &hover_angles);

  EXPECT_NEAR(0.0, hover_angles.x, 1e-9);
  EXPECT_NEAR(M_PI / 2.0, hover_angles.y, 1e-9);
  EXPECT_NEAR(0.0, hover_angles.z, 1e-9);
}

TEST(HoverAnglesGetAngles, PitchForward45) {
  Vec3 wing_pos_g = {-140.0, 0.0, 0.0};
  Vec3 hover_origin_g = kVec3Zero;
  Mat3 dcm_g2b = {{{1.0 / sqrt(2.0), 0.0, -1.0 / sqrt(2.0)},
                   {0.0, 1.0, 0.0},
                   {1.0 / sqrt(2.0), 0.0, 1.0 / sqrt(2.0)}}};
  Vec3 hover_angles = kVec3Zero;
  HoverAnglesGetAngles(&wing_pos_g, &hover_origin_g, &dcm_g2b, &hover_angles);

  EXPECT_NEAR(0.0, hover_angles.x, 1e-9);
  EXPECT_NEAR(-M_PI / 4.0, hover_angles.y, 1e-9);
  EXPECT_NEAR(0.0, hover_angles.z, 1e-9);
}

TEST(HoverAnglesGetAngles, PitchForward180) {
  Vec3 wing_pos_g = {-140.0, 0.0, 0.0};
  Vec3 hover_origin_g = kVec3Zero;
  Mat3 dcm_g2b = {{{0.0, 0.0, 1.0}, {0.0, 1.0, 0.0}, {-1.0, 0.0, 0.0}}};
  Vec3 hover_angles;
  HoverAnglesGetAngles(&wing_pos_g, &hover_origin_g, &dcm_g2b, &hover_angles);

  EXPECT_NEAR(0.0, hover_angles.x, 1e-9);
  EXPECT_NEAR(M_PI, fabs(hover_angles.y), 1e-9);  // pi or -pi are acceptable.
  EXPECT_NEAR(0.0, hover_angles.z, 1e-9);
}

TEST(HoverAnglesGetAngles, HoverHome2) {
  Vec3 wing_pos_g = {-140.0, 0.0, 0.0};
  Vec3 hover_origin_g = kVec3Zero;
  Vec3 hover_angles = kVec3Zero;
  Mat3 dcm_g2b;
  AngleToDcm(0.0, M_PI / 2.0, 0.0, kRotationOrderZyx, &dcm_g2b);

  HoverAnglesGetAngles(&wing_pos_g, &hover_origin_g, &dcm_g2b, &hover_angles);

  EXPECT_NEAR(0.0, hover_angles.x, 1e-9);
  EXPECT_NEAR(0.0, hover_angles.y, 1e-9);
  EXPECT_NEAR(0.0, hover_angles.z, 1e-9);
}

TEST(HoverAnglesGetAngles, PitchBack45) {
  Vec3 wing_pos_g = {-140.0, 0.0, 0.0};
  Vec3 hover_origin_g = kVec3Zero;
  Vec3 hover_angles = kVec3Zero;
  Mat3 dcm_g2b;
  AngleToDcm(0.0, M_PI * 3.0 / 4.0, 0.0, kRotationOrderZyx, &dcm_g2b);

  HoverAnglesGetAngles(&wing_pos_g, &hover_origin_g, &dcm_g2b, &hover_angles);

  EXPECT_NEAR(0.0, hover_angles.x, 1e-9);
  EXPECT_NEAR(M_PI / 4.0, hover_angles.y, 1e-9);
  EXPECT_NEAR(0.0, hover_angles.z, 1e-9);
}

TEST(HoverAnglesGetAngles, YawRight45) {
  Vec3 wing_pos_g = {-140.0, 0.0, 0.0};
  Vec3 hover_origin_g = kVec3Zero;
  Vec3 hover_angles = kVec3Zero;
  Mat3 dcm_g2b;
  AngleToDcm(0.0, M_PI / 2.0, M_PI / 4.0, kRotationOrderXyz, &dcm_g2b);

  HoverAnglesGetAngles(&wing_pos_g, &hover_origin_g, &dcm_g2b, &hover_angles);

  EXPECT_NEAR(0.0, hover_angles.x, 1e-9);
  EXPECT_NEAR(0.0, hover_angles.y, 1e-9);
  EXPECT_NEAR(M_PI / 4.0, hover_angles.z, 1e-9);
}

TEST(HoverAnglesGetAngles, YawLeft45) {
  Vec3 wing_pos_g = {-140.0, 0.0, 0.0};
  Vec3 hover_origin_g = kVec3Zero;
  Vec3 hover_angles = kVec3Zero;
  Mat3 dcm_g2b;
  AngleToDcm(0.0, M_PI / 2.0, -M_PI / 4.0, kRotationOrderXyz, &dcm_g2b);

  HoverAnglesGetAngles(&wing_pos_g, &hover_origin_g, &dcm_g2b, &hover_angles);

  EXPECT_NEAR(0.0, hover_angles.x, 1e-9);
  EXPECT_NEAR(0.0, hover_angles.y, 1e-9);
  EXPECT_NEAR(-M_PI / 4.0, hover_angles.z, 1e-9);
}

TEST(HoverAnglesGetAngles, YawRight135) {
  Vec3 wing_pos_g = {-140.0, 0.0, 0.0};
  Vec3 hover_origin_g = kVec3Zero;
  Vec3 hover_angles = kVec3Zero;
  Mat3 dcm_g2b;
  AngleToDcm(0.0, M_PI / 2.0, 3.0 * M_PI / 4.0, kRotationOrderXyz, &dcm_g2b);

  HoverAnglesGetAngles(&wing_pos_g, &hover_origin_g, &dcm_g2b, &hover_angles);

  EXPECT_NEAR(0.0, hover_angles.x, 1e-9);
  EXPECT_NEAR(0.0, hover_angles.y, 1e-9);
  EXPECT_NEAR(M_PI * 3.0 / 4.0, hover_angles.z, 1e-9);
}

TEST(HoverAnglesGetAngles, YawLeft135) {
  Vec3 wing_pos_g = {-140.0, 0.0, 0.0};
  Vec3 hover_origin_g = kVec3Zero;
  Vec3 hover_angles = kVec3Zero;
  Mat3 dcm_g2b;
  AngleToDcm(0.0, M_PI / 2.0, -3.0 * M_PI / 4.0, kRotationOrderXyz, &dcm_g2b);

  HoverAnglesGetAngles(&wing_pos_g, &hover_origin_g, &dcm_g2b, &hover_angles);

  EXPECT_NEAR(0.0, hover_angles.x, 1e-9);
  EXPECT_NEAR(0.0, hover_angles.y, 1e-9);
  EXPECT_NEAR(-M_PI * 3.0 / 4.0, hover_angles.z, 1e-9);
}

TEST(CalcAnglesError, Zeros) {
  Vec3 angles_error;
  CalcAnglesError(&kVec3Zero, &kVec3Zero, &angles_error);

  EXPECT_NEAR_VEC3(angles_error, kVec3Zero, 1e-9);
}

TEST(CalcAnglesError, Same) {
  Vec3 angles_cmd = {0.1, -0.2, 0.3};
  Vec3 angles_error;
  CalcAnglesError(&angles_cmd, &angles_cmd, &angles_error);

  EXPECT_NEAR_VEC3(angles_error, kVec3Zero, 1e-9);
}

TEST(CalcAnglesError, ReuseInput) {
  Vec3 angles_cmd = {0.1, -0.2, 0.3};
  CalcAnglesError(&angles_cmd, &angles_cmd, &angles_cmd);

  EXPECT_NEAR_VEC3(angles_cmd, kVec3Zero, 1e-9);
}

// An example of a situation where it's clear that you can't add
// axis-angle vectors occurs when you consider pi/2 rotations about
// different axes.  Assume the angle command is a pi/2 rotation about
// the x-axis, and the measured angle is a pi/2 rotation about the
// z-axis.  Then the axis-angle rotation from measurement to command
// is a 2*pi/3 rotation about the <1, 1, -1> axis.
TEST(CalcAnglesError, LargeAngle) {
  Vec3 angles_cmd = {M_PI / 2.0, 0.0, 0.0};
  Vec3 angles = {0.0, 0.0, M_PI / 2.0};
  Vec3 angles_error;
  CalcAnglesError(&angles_cmd, &angles, &angles_error);

  double angle_ans = 2.0 * M_PI / 3.0;
  Vec3 axis_ans = {1.0 / sqrt(3.0), 1.0 / sqrt(3.0), -1.0 / sqrt(3.0)};
  Vec3 angles_error_ans;
  Vec3Scale(&axis_ans, angle_ans, &angles_error_ans);
  EXPECT_NEAR_VEC3(angles_error_ans, angles_error, 1e-9);
}

TEST(ModifyPitchMomentDuringPerching, FullPerchContactPitchMoment) {
  const HoverAnglesParams &params = GetControlParams()->hover.angles;

  // We expect the full perch_contact_pitch_moment at zero payout,
  // assuming we are not meeting the angle command.
  double extra_pitch_moment_z1 = 0.0;

  for (double t = 0.0; t < 5.0; t += *g_sys.ts) {
    ModifyPitchMomentDuringPerching(true, 0.0, 0.2, &params,
                                    &extra_pitch_moment_z1);
  }
  EXPECT_EQ(params.perch_contact_total_pitch_moment_max,
            ModifyPitchMomentDuringPerching(true, 0.0, 0.2, &params,
                                            &extra_pitch_moment_z1));
}

// TODO: This no longer tests the behavior during perching,
// which is only active during Ascend and Descend flight modes.
TEST(HoverAnglesStep, ZeroFeedback) {
  const HoverAnglesParams &params = GetControlParams()->hover.angles;
  const ExperimentState experiment_state = ExperimentState();
  FlightStatus flight_status = FlightStatus();
  HoverAnglesState state = HoverAnglesState();
  VesselEstimate vessel = VesselEstimate();
  Mat3Scale(&kMat3Identity, 1.0, &vessel.dcm_g2p);

  WinchEstimate winch = WinchEstimate();
  Vec3 moment_ff = {0.0, -params.nominal_elevator_pitch_moment, 0.0};

  Vec3 moment_cmd;
  double aileron_roll_moment, elevator_pitch_moment, rudder_yaw_moment;
  winch.payout = 100.0;
  flight_status.flight_mode = kFlightModeCrosswindNormal;
  HoverAnglesStep(&kVec3Zero, &kVec3Zero, &kVec3Zero, &kVec3Zero, &winch,
                  &vessel, false, &flight_status, &params, &experiment_state,
                  &state, &moment_cmd, &aileron_roll_moment,
                  &elevator_pitch_moment, &rudder_yaw_moment);
  EXPECT_NEAR_VEC3(moment_ff, moment_cmd, 1e-9);
  EXPECT_NEAR(params.nominal_elevator_pitch_moment, elevator_pitch_moment,
              1e-9);
  EXPECT_NEAR(0.0, rudder_yaw_moment, 1e-9);
  EXPECT_NEAR(0.0, aileron_roll_moment, 1e-9);

  winch.payout = 0.0;
  flight_status.flight_mode = kFlightModePilotHover;
  HoverAnglesStep(&kVec3Zero, &kVec3Zero, &kVec3Zero, &kVec3Zero, &winch,
                  &vessel, false, &flight_status, &params, &experiment_state,
                  &state, &moment_cmd, &aileron_roll_moment,
                  &elevator_pitch_moment, &rudder_yaw_moment);

  EXPECT_NEAR_VEC3(kVec3Zero, moment_cmd, 1e-9);
  EXPECT_NEAR(params.nominal_elevator_pitch_moment, elevator_pitch_moment,
              1e-9);
  EXPECT_NEAR(0.0, rudder_yaw_moment, 1e-9);
  EXPECT_NEAR(0.0, aileron_roll_moment, 1e-9);

  winch.payout = 100.0;
  HoverAnglesStep(&kVec3Zero, &kVec3Zero, &kVec3Zero, &kVec3Zero, &winch,
                  &vessel, true, &flight_status, &params, &experiment_state,
                  &state, &moment_cmd, &aileron_roll_moment,
                  &elevator_pitch_moment, &rudder_yaw_moment);

  EXPECT_NEAR_VEC3(moment_ff, moment_cmd, 1e-9);
  EXPECT_NEAR(params.nominal_elevator_pitch_moment, elevator_pitch_moment,
              1e-9);
  EXPECT_NEAR(0.0, rudder_yaw_moment, 1e-9);
  EXPECT_NEAR(0.0, aileron_roll_moment, 1e-9);

  HoverAnglesStep(&kVec3Zero, &kVec3Zero, &kVec3Zero, &kVec3Zero, &winch,
                  &vessel, false, &flight_status, &params, &experiment_state,
                  &state, &moment_cmd, &aileron_roll_moment,
                  &elevator_pitch_moment, &rudder_yaw_moment);

  EXPECT_NEAR_VEC3(moment_ff, moment_cmd, 1e-9);
  EXPECT_NEAR(params.nominal_elevator_pitch_moment, elevator_pitch_moment,
              1e-9);
  EXPECT_NEAR(0.0, rudder_yaw_moment, 1e-9);
  EXPECT_NEAR(0.0, aileron_roll_moment, 1e-9);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

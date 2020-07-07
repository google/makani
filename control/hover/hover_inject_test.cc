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

#include "common/c_math/vec3.h"
#include "control/control_params.h"
#include "control/control_types.h"
#include "control/hover/hover.h"
#include "control/hover/hover_inject.h"
#include "lib/util/test_util.h"

extern "C" {

double GetInjectTime(const FlightStatus *flight_status,
                     const HoverInjectParams *params);

void CalcVec3StepSignal(double t, const Vec3 *amplitude, const Vec3 *start_time,
                        const Vec3 *stop_time, Vec3 *signal);

}  // extern "C"

TEST(HoverInjectInit, Nothing) {
  HoverInjectInit(&GetControlParams()->hover.inject);
}

TEST(HoverInjectValidateParams, Default) {
  EXPECT_TRUE(HoverInjectValidateParams(&GetControlParams()->hover.inject));
}

TEST(HoverInjectIsEnabled, Default) {
  HoverInjectParams params = GetControlParams()->hover.inject;
  FlightStatus flight_status;
  flight_status.flight_mode = kFlightModeHoverFullLength;

  for (int32_t i = 0; i < kNumFlightPlans; ++i) {
    GetControlParamsUnsafe()->flight_plan = static_cast<FlightPlan>(i);
    for (int32_t j = 0; j < kNumFlightModes; ++j) {
      flight_status.flight_mode = static_cast<FlightMode>(j);

      if (static_cast<FlightPlan>(i) == kFlightPlanHoverInPlace &&
          static_cast<FlightMode>(j) == kFlightModeHoverFullLength) {
        params.use_signal_injection = true;
        EXPECT_TRUE(HoverInjectIsEnabled(&flight_status, &params));
        params.use_signal_injection = false;
        EXPECT_FALSE(HoverInjectIsEnabled(&flight_status, &params));
      } else {
        params.use_signal_injection = true;
        EXPECT_FALSE(HoverInjectIsEnabled(&flight_status, &params));
        params.use_signal_injection = false;
        EXPECT_FALSE(HoverInjectIsEnabled(&flight_status, &params));
      }
    }
  }
}

TEST(GetInjectTime, Normal) {
  HoverInjectParams params = GetControlParams()->hover.inject;
  params.use_signal_injection = true;
  FlightStatus flight_status = FlightStatus();
  flight_status.flight_mode_time = 1.0;

  for (int32_t i = 0; i < kNumFlightPlans; ++i) {
    GetControlParamsUnsafe()->flight_plan = static_cast<FlightPlan>(i);
    for (int32_t j = 0; j < kNumFlightModes; ++j) {
      flight_status.flight_mode = static_cast<FlightMode>(j);

      if (static_cast<FlightPlan>(i) == kFlightPlanHoverInPlace &&
          static_cast<FlightMode>(j) == kFlightModeHoverFullLength) {
        EXPECT_EQ(GetInjectTime(&flight_status, &params), 1.0);
      } else {
        EXPECT_EQ(GetInjectTime(&flight_status, &params), 0.0);
      }
    }
  }
}

TEST(CalcVec3StepSignal, Normal) {
  Vec3 amplitude = {10.0, 20.0, -30.0};
  Vec3 start_time = {1.0, 1.2, 3.0};
  Vec3 stop_time = {1.5, 2.1, 3.1};

  Vec3 signal;
  CalcVec3StepSignal(0.9, &amplitude, &start_time, &stop_time, &signal);
  EXPECT_EQ(signal.x, 0.0);
  EXPECT_EQ(signal.y, 0.0);
  EXPECT_EQ(signal.z, 0.0);

  CalcVec3StepSignal(1.1, &amplitude, &start_time, &stop_time, &signal);
  EXPECT_EQ(signal.x, 10.0);
  EXPECT_EQ(signal.y, 0.0);
  EXPECT_EQ(signal.z, 0.0);

  CalcVec3StepSignal(1.3, &amplitude, &start_time, &stop_time, &signal);
  EXPECT_EQ(signal.x, 10.0);
  EXPECT_EQ(signal.y, 20.0);
  EXPECT_EQ(signal.z, 0.0);

  CalcVec3StepSignal(1.6, &amplitude, &start_time, &stop_time, &signal);
  EXPECT_EQ(signal.x, 0.0);
  EXPECT_EQ(signal.y, 20.0);
  EXPECT_EQ(signal.z, 0.0);

  CalcVec3StepSignal(2.2, &amplitude, &start_time, &stop_time, &signal);
  EXPECT_EQ(signal.x, 0.0);
  EXPECT_EQ(signal.y, 0.0);
  EXPECT_EQ(signal.z, 0.0);

  CalcVec3StepSignal(3.05, &amplitude, &start_time, &stop_time, &signal);
  EXPECT_EQ(signal.x, 0.0);
  EXPECT_EQ(signal.y, 0.0);
  EXPECT_EQ(signal.z, -30.0);
}

TEST(HoverInjectPositionSignal, Normal) {
  FlightStatus flight_status = FlightStatus();
  flight_status.flight_mode = kFlightModeHoverFullLength;
  GetControlParamsUnsafe()->flight_plan = kFlightPlanHoverInPlace;

  HoverInjectParams params = HoverInjectParams();
  params.use_signal_injection = true;
  params.position_amplitude = {2.2, -1.0, 1.0};
  params.position_start_time = {1.0, 0.0, 3.0};
  params.position_stop_time = {1.6, 1.2, 3.3};

  Vec3 hover_origin_g = kVec3Zero;
  Vec3 wing_pos_g_cmd = {-1.0, 0.0, 0.0};
  Vec3 wing_pos_g_cmd_out;

  flight_status.flight_mode_time = 1.5;
  HoverInjectPositionSignal(&flight_status, &wing_pos_g_cmd, &hover_origin_g,
                            &params, &wing_pos_g_cmd_out);
  EXPECT_NEAR(wing_pos_g_cmd_out.x, -1.0, 1e-9);
  EXPECT_NEAR(wing_pos_g_cmd_out.y, 0.0, 1e-9);
  EXPECT_NEAR(wing_pos_g_cmd_out.z, -2.2, 1e-9);

  flight_status.flight_mode_time = 1.05;
  HoverInjectPositionSignal(&flight_status, &wing_pos_g_cmd, &hover_origin_g,
                            &params, &wing_pos_g_cmd_out);
  EXPECT_NEAR(wing_pos_g_cmd_out.x, -1.0, 1e-9);
  EXPECT_NEAR(wing_pos_g_cmd_out.y, -1.0, 1e-9);
  EXPECT_NEAR(wing_pos_g_cmd_out.z, -2.2, 1e-9);

  flight_status.flight_mode_time = 3.1;
  HoverInjectPositionSignal(&flight_status, &wing_pos_g_cmd, &hover_origin_g,
                            &params, &wing_pos_g_cmd_out);
  EXPECT_NEAR(wing_pos_g_cmd_out.x, 0.0, 1e-9);
  EXPECT_NEAR(wing_pos_g_cmd_out.y, 0.0, 1e-9);
  EXPECT_NEAR(wing_pos_g_cmd_out.z, 0.0, 1e-9);
}

TEST(HoverInjectAnglesSignal, Normal) {
  FlightStatus flight_status = FlightStatus();
  flight_status.flight_mode = kFlightModeHoverFullLength;
  GetControlParamsUnsafe()->flight_plan = kFlightPlanHoverInPlace;

  HoverInjectParams params = HoverInjectParams();
  params.use_signal_injection = true;
  params.angles_amplitude = {0.22, -0.1, 0.1};
  params.angles_start_time = {1.0, 0.0, 3.0};
  params.angles_stop_time = {1.6, 1.2, 3.3};

  Vec3 angles;

  flight_status.flight_mode_time = 1.5;
  HoverInjectAnglesSignal(&flight_status, &kVec3Zero, &params, &angles);
  EXPECT_NEAR(angles.x, 0.22, 1e-9);
  EXPECT_NEAR(angles.y, 0.0, 1e-9);
  EXPECT_NEAR(angles.z, 0.0, 1e-9);

  flight_status.flight_mode_time = 1.05;
  HoverInjectAnglesSignal(&flight_status, &kVec3Zero, &params, &angles);
  EXPECT_NEAR(angles.x, 0.22, 1e-9);
  EXPECT_NEAR(angles.y, -0.1, 1e-9);
  EXPECT_NEAR(angles.z, 0.0, 1e-9);

  flight_status.flight_mode_time = 3.1;
  HoverInjectAnglesSignal(&flight_status, &kVec3Zero, &params, &angles);
  EXPECT_NEAR(angles.x, 0.0, 1e-9);
  EXPECT_NEAR(angles.y, 0.0, 1e-9);
  EXPECT_NEAR(angles.z, 0.1, 1e-9);
}

TEST(HoverInjectOutputSignal, Normal) {
  FlightStatus flight_status = FlightStatus();
  flight_status.flight_mode = kFlightModeHoverFullLength;
  GetControlParamsUnsafe()->flight_plan = kFlightPlanHoverInPlace;

  HoverInjectParams params = HoverInjectParams();
  params.use_signal_injection = true;

  params.elevator_amplitude = 0.22;
  params.elevator_start_time = 1.0;
  params.elevator_stop_time = 1.6;

  params.rudder_amplitude = 0.22;
  params.rudder_start_time = params.elevator_start_time;
  params.rudder_stop_time = params.elevator_stop_time;

  params.blown_flaps_amplitude = 0.137;
  params.blown_flaps_period =
      params.elevator_stop_time - params.elevator_start_time;
  params.blown_flaps_start_time = params.elevator_start_time;
  params.blown_flaps_stop_time = params.elevator_stop_time;

  ControlOutput control_output = ControlOutput();

  // At half period, the elevator should be at a maximum and the
  // rudder and ailerons, which use the full triangle wave, should
  // have returned to zero.
  double period = params.elevator_stop_time - params.elevator_start_time;
  flight_status.flight_mode_time = params.elevator_start_time + period / 2.0;

  HoverInjectOutputSignal(&flight_status, &control_output, &params,
                          &control_output);
  EXPECT_NEAR(control_output.flaps[kFlapEle], params.elevator_amplitude, 1e-9);
  EXPECT_NEAR(control_output.flaps[kFlapRud], 0.0, 1e-9);
  EXPECT_NEAR(control_output.flaps[kFlapA1], 0.0, 1e-9);
  EXPECT_NEAR(control_output.flaps[kFlapA2], 0.0, 1e-9);
  EXPECT_NEAR(control_output.flaps[kFlapA4], 0.0, 1e-9);
  EXPECT_NEAR(control_output.flaps[kFlapA5], 0.0, 1e-9);
  EXPECT_NEAR(control_output.flaps[kFlapA7], 0.0, 1e-9);
  EXPECT_NEAR(control_output.flaps[kFlapA8], 0.0, 1e-9);

  // Evaluate the injection at quarter-period.
  control_output = ControlOutput();
  flight_status.flight_mode_time = params.elevator_start_time + period / 4.0;
  HoverInjectOutputSignal(&flight_status, &control_output, &params,
                          &control_output);

  EXPECT_NEAR(control_output.flaps[kFlapRud], params.rudder_amplitude, 1e-9);
  EXPECT_NEAR(control_output.flaps[kFlapA1], 0.0, 1e-9);
  EXPECT_NEAR(control_output.flaps[kFlapA2], 0.0, 1e-9);
  EXPECT_NEAR(control_output.flaps[kFlapA4], -params.blown_flaps_amplitude,
              1e-9);
  EXPECT_NEAR(control_output.flaps[kFlapA5], params.blown_flaps_amplitude,
              1e-9);
  EXPECT_NEAR(control_output.flaps[kFlapA7], 0.0, 1e-9);
  EXPECT_NEAR(control_output.flaps[kFlapA8], 0.0, 1e-9);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

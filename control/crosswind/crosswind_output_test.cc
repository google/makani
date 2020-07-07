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

#include "avionics/common/avionics_messages.h"
#include "control/control_params.h"
#include "control/control_types.h"
#include "control/crosswind/crosswind_output.h"
#include "lib/util/test_util.h"

extern "C" {

void GetRudderLimits(double airspeed, double beta,
                     const CrosswindOutputParams *params,
                     double *rudder_lower_limit, double *rudder_upper_limit);

double GetDetwistCommand(LoopDirection loop_dir, double loop_angle,
                         int32_t *rev_count, double *detwist_loop_angle);

}  // extern "C"

// Confirms that CrosswindOutputStep sets all fields of the
// ControlOutput structure.
TEST(CrosswindOutputStep, SetAllValues) {
  StateEstimate state_est = StateEstimate();
  ThrustMoment zero_thrust_moment = ThrustMoment();
  Deltas zero_deltas = Deltas();
  Deltas deltas_available;
  CrosswindOutputParams params = GetControlParams()->crosswind.output;
  CrosswindOutputState state = CrosswindOutputState();

  ControlOutput control_output_0;
  memset(&control_output_0, 0xFF, sizeof(control_output_0));

  Vec3 path_center_g = kVec3Ones;

  ControlOutput control_output = control_output_0;
  CrosswindOutputStep(kLoopDirectionCcw, 0.0, false, &zero_thrust_moment,
                      &zero_deltas, &state_est, &path_center_g, &params, &state,
                      &control_output, &deltas_available);

  // Note that this check relies on the output not having an actual
  // value composed of 0xFF.
  EXPECT_NE(control_output.sync.sequence, control_output_0.sync.sequence);
  EXPECT_NE(control_output.sync.sequence, control_output_0.sync.flight_mode);
  for (int32_t i = 0; i < kNumFlaps; ++i) {
    EXPECT_NE(control_output.flaps[i], control_output_0.flaps[i]);
  }
  for (int32_t i = 0; i < kNumMotors; ++i) {
    EXPECT_NE(control_output.motor_speed_upper_limit[i],
              control_output_0.motor_speed_upper_limit[i]);
    EXPECT_NE(control_output.motor_speed_lower_limit[i],
              control_output_0.motor_speed_lower_limit[i]);
    EXPECT_NE(control_output.motor_torque[i], control_output_0.motor_torque[i]);
  }
  EXPECT_NE(control_output.winch_vel_cmd, control_output_0.winch_vel_cmd);
  EXPECT_NE(control_output.detwist_cmd, control_output_0.detwist_cmd);
  EXPECT_NE(control_output.stop_motors, control_output_0.stop_motors);
  EXPECT_NE(control_output.run_motors, control_output_0.run_motors);
  EXPECT_NE(control_output.tether_release, control_output_0.tether_release);
  EXPECT_NE(control_output.gs_azi_cmd.target,
            control_output_0.gs_azi_cmd.target);
  EXPECT_NE(control_output.gs_azi_cmd.dead_zone,
            control_output_0.gs_azi_cmd.dead_zone);
  EXPECT_NE(control_output.gs_unpause_transform,
            control_output_0.gs_unpause_transform);
  EXPECT_NE(control_output.hold_gs_azi_cmd, control_output_0.hold_gs_azi_cmd);
}

TEST(CrosswindOutputValidateParams, Passes) {
  const CrosswindOutputParams *params = &GetControlParams()->crosswind.output;
  EXPECT_TRUE(CrosswindOutputValidateParams(params));
}

TEST(GetRudderLimits, TableRecovery) {
  const CrosswindOutputParams *params = &GetControlParams()->crosswind.output;
  for (int i = 0; i < CROSSWIND_RUDDER_LIMIT_AIRSPEEDS; ++i) {
    const double airspeed = params->rudder_limit_airspeeds[i];
    for (int j = 0; j < CROSSWIND_RUDDER_LIMIT_BETAS; ++j) {
      const double beta = params->rudder_limit_betas[j];
      double rudder_lower_limit, rudder_upper_limit;
      GetRudderLimits(airspeed, beta, params, &rudder_lower_limit,
                      &rudder_upper_limit);
      EXPECT_NEAR(rudder_lower_limit, params->rudder_limits_lower[i][j], 1e-9);
      EXPECT_NEAR(rudder_upper_limit, params->rudder_limits_upper[i][j], 1e-9);
    }
  }
}

class GetDetwistCommandTest : public ::testing::Test {
 protected:
  GetDetwistCommandTest() : rev_count_(0), detwist_loop_angle_(0.0) {}

  double GetCommand(LoopDirection loop_dir, double absolute_angle) {
    double loop_angle = Wrap(absolute_angle, 0.0, 2.0 * PI);
    return GetDetwistCommand(loop_dir, loop_angle, &rev_count_,
                             &detwist_loop_angle_);
  }

  const double wraparound_ = (double)TETHER_DETWIST_REVS * 2.0 * PI;
  const double tolerance_ = wraparound_ * DBL_EPSILON;
  int32_t rev_count_;
  double detwist_loop_angle_;
};

TEST_F(GetDetwistCommandTest, SimpleClockwise) {
  for (double angle = 0.0; angle > -5.0 * wraparound_; angle -= 0.1) {
    ASSERT_NEAR(Wrap(angle, 0.0, wraparound_),
                GetCommand(kLoopDirectionCw, angle), tolerance_);
  }
}

TEST_F(GetDetwistCommandTest, SimpleCounterClockwise) {
  for (double angle = 0.0; angle < 5.0 * wraparound_; angle += 0.1) {
    ASSERT_NEAR(Wrap(angle, 0.0, wraparound_),
                GetCommand(kLoopDirectionCcw, angle), tolerance_);
  }
}

TEST_F(GetDetwistCommandTest, WrongDirectionClockwise) {
  // Check that moving up to PI in the wrong direction doesn't change the
  // command.
  for (double angle = 0.0; angle < PI; angle += 0.1) {
    ASSERT_NEAR(0.0, GetCommand(kLoopDirectionCw, angle), tolerance_);
  }
}

TEST_F(GetDetwistCommandTest, WrongDirectionCounterClockwise) {
  for (double angle = 0.0; angle > -PI; angle -= 0.1) {
    ASSERT_NEAR(0.0, GetCommand(kLoopDirectionCcw, angle), tolerance_);
  }
}

TEST_F(GetDetwistCommandTest, DebounceClockwise) {
  double angle = 0.0;
  double min_angle = 0.0;
  while (angle > -5.0 * wraparound_) {
    for (int32_t i = 0; i < 4; ++i) {
      angle -= 0.1;
      min_angle = fmin(min_angle, angle);
      ASSERT_NEAR(Wrap(min_angle, 0.0, wraparound_),
                  GetCommand(kLoopDirectionCw, angle), tolerance_);
    }
    angle += 0.22;
  }
}

TEST_F(GetDetwistCommandTest, DebounceCounterClockwise) {
  double angle = 0.0;
  double max_angle = 0.0;
  while (angle < 5.0 * wraparound_) {
    for (int32_t i = 0; i < 4; ++i) {
      angle += 0.1;
      max_angle = fmax(max_angle, angle);
      ASSERT_NEAR(Wrap(max_angle, 0.0, wraparound_),
                  GetCommand(kLoopDirectionCcw, angle), tolerance_);
    }
    angle -= 0.22;
  }
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

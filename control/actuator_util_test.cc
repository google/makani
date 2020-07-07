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

#include <limits>

#include "control/actuator_util.h"
#include "control/control_params.h"
#include "control/control_types.h"
#include "control/simple_aero.h"
#include "control/system_params.h"
#include "control/system_types.h"
#include "lib/util/test_util.h"
#include "system/labels.h"

using ::test_util::Rand;

extern "C" {

void ThrustMomentToThrusts(const ThrustMoment *thrust_moment,
                           const ThrustMoment *weights, const Vec *min_thrusts,
                           const Vec *max_thrusts,
                           double total_power_limit_thrust,
                           StackingState stacking_state,
                           const RotorControlParams *params, Vec *thrusts,
                           ThrustMoment *available_thrust_moment);

}  // extern "C"

TEST(ThrustMomentToThrusts, LevelHover) {
  RotorControlParams params = *g_cont.rotor_control;
  // Matrix for a vehicle with motor positions and center-of-mass that
  // are symmetric about the x-z plane.
  double comm_and_diff_thrusts_to_thrust_moment[4][5] = {
      {8.0, 0.0, 0.0, 0.0, 0.0},
      {-0.10, 0.37, 0.37, 0.37, 0.37},
      {0.73, 2.82, 2.82, 2.82, 2.82},
      {0.0, -7.27, -2.42, 2.42, 7.27}};
  memcpy(&params.comm_and_diff_thrusts_to_thrust_moment,
         comm_and_diff_thrusts_to_thrust_moment,
         sizeof(params.comm_and_diff_thrusts_to_thrust_moment));

  VEC_INIT(kNumMotors, min_thrusts, {0});
  VEC_INIT(kNumMotors, max_thrusts, {0});
  VEC_INIT(kNumMotors, thrusts, {0});
  const double max_weight = 1.3 * g_sys.phys->g * g_sys.wing->m;
  for (int32_t i = 0; i < kNumMotors; ++i) {
    *VecPtr(&min_thrusts, i) = 0.0;
    *VecPtr(&max_thrusts, i) = max_weight / kNumMotors;
  }

  ThrustMoment thrust_moment = {max_weight / 1.3, {0.0, 0.0, 0.0}};
  ThrustMoment weights = {1.0, {1e-9, 0.1, 0.1}};
  ThrustMoment available_thrust_moment;
  ThrustMomentToThrusts(&thrust_moment, &weights, &min_thrusts, &max_thrusts,
                        max_weight, kStackingStateNormal, &params, &thrusts,
                        &available_thrust_moment);

  // Check that bottom rotors deliver equal thrust.
  for (MotorLabel m : {kMotorSbi, kMotorPbi, kMotorPbo}) {
    EXPECT_NEAR(VecGet(&thrusts, kMotorSbo), VecGet(&thrusts, m), 1e-9);
  }

  // Check that top rotors deliver equal thrust.
  for (MotorLabel m : {kMotorPti, kMotorSti, kMotorSto}) {
    EXPECT_NEAR(VecGet(&thrusts, kMotorPto), VecGet(&thrusts, m), 1e-9);
  }

  // We don't check for:
  //   available_thrust_moment.moment.x == thrust_moment.moment.x
  // because the rotors aren't necessarily capable of delivering the
  // requested roll moment if the propellers all spin the same
  // direction.
  EXPECT_NEAR(available_thrust_moment.thrust, thrust_moment.thrust, 1e-3);
  EXPECT_NEAR(available_thrust_moment.moment.y, thrust_moment.moment.y, 1e-3);
  EXPECT_NEAR(available_thrust_moment.moment.z, thrust_moment.moment.z, 1e-3);
}

TEST(ThrustMomentToThrusts, LevelHoverPrioritizePitchYaw) {
  RotorControlParams params = *g_cont.rotor_control;
  // This test depends on a matrix with no coupling between the common
  // mode thrust and the pitch and yaw moments.
  double comm_and_diff_thrusts_to_thrust_moment[4][5] = {
      {8.0, 0.0, 0.0, 0.0, 0.0},
      {-0.10, 0.37, 0.37, 0.37, 0.37},
      {0.0, 2.82, 2.82, 2.82, 2.82},
      {0.0, -6.27, -3.45, 3.45, 6.27}};
  memcpy(&params.comm_and_diff_thrusts_to_thrust_moment,
         comm_and_diff_thrusts_to_thrust_moment,
         sizeof(params.comm_and_diff_thrusts_to_thrust_moment));

  const double max_weight = 1.3 * g_sys.phys->g * g_sys.wing->m;
  VEC_INIT(kNumMotors, min_thrusts, {0});
  VEC_INIT(kNumMotors, max_thrusts, {0});
  VEC_INIT(kNumMotors, thrusts, {0});
  for (int32_t i = 0; i < kNumMotors; ++i) {
    *VecPtr(&min_thrusts, i) = 0.0;
    *VecPtr(&max_thrusts, i) = max_weight / kNumMotors;
  }

  for (int32_t i = 0; i < 222; ++i) {
    ThrustMoment weights = {1.0, {1e-9, 1e-9, Rand(0.1, 100.0)}};
    ThrustMoment available_thrust_moment;
    ThrustMoment thrust_moment = {max_weight, {0.0, 0.0, 1.0}};
    ThrustMomentToThrusts(&thrust_moment, &weights, &min_thrusts, &max_thrusts,
                          max_weight, kStackingStateNormal, &params, &thrusts,
                          &available_thrust_moment);

    double thrust_per_yaw_moment_trade =
        weights.moment.z / weights.thrust *
        (comm_and_diff_thrusts_to_thrust_moment[3][3] +
         comm_and_diff_thrusts_to_thrust_moment[3][4]) /
        4.0;
    EXPECT_NEAR((thrust_moment.thrust - available_thrust_moment.thrust) /
                    thrust_per_yaw_moment_trade,
                thrust_moment.moment.z - available_thrust_moment.moment.z,
                1e-6);
  }

  for (int32_t i = 0; i < 222; ++i) {
    ThrustMoment weights = {1.0, {1e-9, Rand(0.1, 100.0), 1.0}};
    ThrustMoment available_thrust_moment;
    ThrustMoment thrust_moment = {max_weight, {0.0, 1.0, 0.0}};
    ThrustMomentToThrusts(&thrust_moment, &weights, &min_thrusts, &max_thrusts,
                          max_weight, kStackingStateNormal, &params, &thrusts,
                          &available_thrust_moment);

    double thrust_per_pitch_moment_trade =
        weights.moment.y / weights.thrust *
        (comm_and_diff_thrusts_to_thrust_moment[2][3] +
         comm_and_diff_thrusts_to_thrust_moment[2][4]) /
        4.0;
    // We use only the magnitude of the torque error here as whether
    // the top row of motors or bottom row of motors saturates depends
    // on both the magnitude of the pitch request and the motor
    // positions, so we use only the magnitude of the torque error.
    EXPECT_NEAR((thrust_moment.thrust - available_thrust_moment.thrust) /
                    thrust_per_pitch_moment_trade,
                fabs(thrust_moment.moment.y - available_thrust_moment.moment.y),
                1e-6);
  }
}

// Test that requesting zero thrust, roll moment and pitch moment
// generates the minimum possible rotors speeds.  Pitch moment is
// excluded as a the current motor locations require unequal rotor
// speeds between the top and bottom rows to realize a zero pitch
// moment.
TEST(MixRotors, ZeroSpeedMinimumThrustNoXZMoment) {
  ThrustMoment thrust_moment = {0.0, {0.0, 0.0, 0.0}};
  ThrustMoment weights = {1.0, {1.0, 1e-9, 1.0}};

  double rotors[kNumMotors];
  double v_app_locals[kNumMotors];
  ThrustMoment available_thrust_moment;
  MixRotors(&thrust_moment, &weights, 0.0, &kVec3Zero, kStackingStateNormal,
            true, g_sys.phys->rho, g_sys.rotors, g_cont.rotor_control, rotors,
            &available_thrust_moment, v_app_locals);

  for (int32_t i = 0; i < kNumMotors; ++i) {
    EXPECT_NEAR(rotors[i], g_cont.rotor_control->idle_speed, 1e-9);
  }
  EXPECT_NEAR(available_thrust_moment.thrust, 0.0, 1e-3);
  EXPECT_NEAR(available_thrust_moment.moment.x, 0.0, 1e-9);
  EXPECT_NEAR(available_thrust_moment.moment.z, 0.0, 1e-9);
}

// Around commit c33e68d7, there was an issue where the motor commands
// became erratic at very high airspeeds.  The issue was that the
// constraints inside ThrustMomentToThrusts were being modified to
// find a valid initial solution to the constrained least squares
// problem.  This modification in constrains meant that an
// unachievable thrust was being passed to ThrustToOmega.  This issue
// was fixed with a modification to ThrustToOmega.
TEST(MixRotors, NoErraticValuesAtHighAirspeeds) {
  ThrustMoment weights = {1e-3, {1e-4, 1.0, 1.0}};
  ThrustMoment thrust_moment_avail;
  Vec3 pqr = {-0.09, 0.04, -0.56};
  for (double thrust = -182000.0; thrust > -192000.0; thrust -= 100.0) {
    ThrustMoment thrust_moment = {thrust, {0.0, -2000.0, 900.0}};
    double v_freestream = 90.0;
    double rotors[kNumMotors];
    double v_app_locals[kNumMotors];
    MixRotors(&thrust_moment, &weights, v_freestream, &pqr,
              kStackingStateNormal, false, g_sys.phys->rho, g_sys.rotors,
              g_cont.rotor_control, rotors, &thrust_moment_avail, v_app_locals);

    thrust_moment.thrust += Rand(-1.0, 1.0);
    thrust_moment.moment.x += Rand(-1.0, 1.0);
    thrust_moment.moment.y += Rand(-1.0, 1.0);
    thrust_moment.moment.z += Rand(-1.0, 1.0);
    v_freestream += Rand(-1.0, 1.0);
    double rotors_perturb[kNumMotors];
    MixRotors(&thrust_moment, &weights, v_freestream, &pqr,
              kStackingStateNormal, false, g_sys.phys->rho, g_sys.rotors,
              g_cont.rotor_control, rotors_perturb, &thrust_moment_avail,
              v_app_locals);

    for (int32_t i = 0; i < kNumMotors; ++i) {
      EXPECT_NEAR(rotors[i], rotors_perturb[i], 10.0);
    }
  }
}

TEST(MixFlaps, Zeros) {
  double offsets[kNumFlaps] = {0};
  double flaps[kNumFlaps] = {0};
  Deltas deltas = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  Deltas deltas_avail;

  double upper_limits[kNumFlaps];
  double lower_limits[kNumFlaps];
  for (int i = 0; i < kNumFlaps; i++) {
    upper_limits[i] = 100.0;
    lower_limits[i] = -100.0;
  }

  MixFlaps(&deltas, offsets, lower_limits, upper_limits, flaps, &deltas_avail);
  for (int32_t i = 0; i < kNumFlaps; ++i) {
    EXPECT_NEAR(flaps[i], 0.0, std::numeric_limits<double>::epsilon());
  }
}

TEST(MixFlaps, Limits) {
  double lower_limits[kNumFlaps] = {1.0,  1.0,  -1.0, -1.0,
                                    -2.0, -2.0, -3.0, -4.0};
  double upper_limits[kNumFlaps] = {2.0, 2.0, 1.0, 1.0, 3.0, 3.0, 4.0, -1.0};
  double flaps[kNumFlaps] = {0};
  Deltas deltas = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  Deltas deltas_avail;

  double offsets_low[kNumFlaps] = {-10.0, -10.0, -10.0, -10.0,
                                   -10.0, -10.0, -10.0, -10.0};
  MixFlaps(&deltas, offsets_low, lower_limits, upper_limits, flaps,
           &deltas_avail);
  for (int32_t i = 0; i < kNumFlaps; ++i) {
    EXPECT_GE(flaps[i], lower_limits[i]);
    EXPECT_LE(flaps[i], upper_limits[i]);
  }

  double offsets_high[kNumFlaps] = {10.0, 10.0, 10.0, 10.0,
                                    10.0, 10.0, 10.0, 10.0};
  MixFlaps(&deltas, offsets_high, lower_limits, upper_limits, flaps,
           &deltas_avail);
  for (int32_t i = 0; i < kNumFlaps; ++i) {
    EXPECT_GE(flaps[i], lower_limits[i]);
    EXPECT_LE(flaps[i], upper_limits[i]);
  }
}

TEST(MixFlaps, Simple) {
  double offsets[kNumFlaps] = {0};
  double flaps[kNumFlaps] = {0};
  Deltas deltas = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};

  const double lower_limits[kNumFlaps] = {-10.0, -10.0, -10.0, -10.0,
                                          -10.0, -10.0, -10.0, -10.0};
  const double upper_limits[kNumFlaps] = {10.0, 10.0, 10.0, 10.0,
                                          10.0, 10.0, 10.0, 10.0};

  Deltas deltas_avail;
  MixFlaps(&deltas, offsets, lower_limits, upper_limits, flaps, &deltas_avail);

  EXPECT_NEAR(deltas.aileron, deltas_avail.aileron, 0.01);
  EXPECT_NEAR(deltas.inboard_flap, deltas_avail.inboard_flap, 0.01);
  EXPECT_NEAR(deltas.midboard_flap, deltas_avail.midboard_flap, 0.01);
  EXPECT_NEAR(deltas.outboard_flap, deltas_avail.outboard_flap, 0.01);
  EXPECT_NEAR(deltas.elevator, deltas_avail.elevator, 0.01);
  EXPECT_NEAR(deltas.rudder, deltas_avail.rudder, 0.01);
}

TEST(FlapAnglesToServoAngles, Linear) {
  const ServoParams *servo_params = GetSystemParams()->servos;
  double flaps[kNumFlaps] = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
  double servos[kNumServos];
  FlapAnglesToServoAngles(flaps, servo_params, servos);
  // Skipping the rudder as it is nonlinear.
  for (int32_t i = 0; i < kServoR1; ++i) {
    EXPECT_NEAR(0.5, servos[i], std::numeric_limits<double>::epsilon());
  }
}

TEST(FlapAnglesToServoAngles, Zero) {
  const ServoParams *servo_params = GetSystemParams()->servos;
  double flaps[kNumFlaps] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  double servos[kNumServos];
  FlapAnglesToServoAngles(flaps, servo_params, servos);
  for (int32_t i = 0; i < kNumServos; ++i) {
    EXPECT_NEAR(0.0, servos[i], std::numeric_limits<double>::epsilon());
  }
}

TEST(FlapAnglesToServoAngles, Inverse) {
  const ServoParams *servo_params = GetSystemParams()->servos;
  double flaps[kNumFlaps];
  double servos[kNumServos];
  double flaps_out[kNumFlaps];
  for (double angle = -0.4; angle < 0.41; angle += 0.05) {
    for (int32_t i = 0; i < kNumFlaps; ++i) {
      flaps[i] = angle;
    }
    FlapAnglesToServoAngles(flaps, servo_params, servos);
    ServoAnglesToFlapAngles(servos, servo_params, flaps_out);
    for (int32_t i = 0; i < kNumFlaps; ++i) {
      EXPECT_NEAR(flaps[i], flaps_out[i],
                  std::numeric_limits<double>::epsilon());
    }
  }
}

TEST(FlapAnglesToServoAngles, Varied) {
  const ServoParams *servo_params = GetSystemParams()->servos;
  double flaps[kNumFlaps];
  double servos[kNumServos];
  double flaps_out[kNumFlaps];
  for (int32_t i = 0; i < kNumFlaps; ++i) {
    flaps[i] = -0.4 + i * 0.05;
  }
  FlapAnglesToServoAngles(flaps, servo_params, servos);
  ServoAnglesToFlapAngles(servos, servo_params, flaps_out);
  for (int32_t i = 0; i < kNumFlaps; ++i) {
    EXPECT_NEAR(flaps[i], flaps_out[i], std::numeric_limits<double>::epsilon());
  }
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

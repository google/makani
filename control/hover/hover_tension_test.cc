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
#include "control/hover/hover_tension.h"
#include "control/hover/hover_types.h"
#include "lib/util/test_util.h"

extern "C" {

void EstimateApparentWind(const Vec3 *wing_pos_g, const Vec3 *wind_g,
                          double winch_vel, Vec3 *apparent_wind_g);

double CalcHorizontalAeroForce(const Vec3 *wing_pos_g, const Vec3 *wind_g,
                               double winch_vel,
                               const HoverTensionParams *params);

double CalcFeedForwardPitch(double horizontal_tension_cmd,
                            double tether_elevation_cmd, const Vec3 *wing_pos_g,
                            const Vec3 *wind_g, double payout, double winch_vel,
                            const HoverTensionParams *params);

void CalcPitchLimits(double payout, const HoverTensionParams *params,
                     double *min_pitch, double *max_pitch);

double CalcTensionAtWing(double horizontal_tension, const Vec3 *wing_pos_g);

double ModifyPitchCommandInAccel(double pitch_cmd, const Vec3 *wing_pos_g,
                                 FlightMode flight_mode,
                                 const HoverTensionParams *params,
                                 HoverTensionState *state);

double GetHorizontalTensionPilotOffset(const HoverTensionParams *params,
                                       const JoystickEstimate *joystick,
                                       FlightMode flight_mode,
                                       HoverTensionState *state);

}  // extern "C"

const Vec3 kWingPosGValues[] = {
    {0.0, 0.0, 0.0},    {-100.0, 0.0, 0.0}, {100.0, 0.0, 0.0},
    {0.0, -100.0, 0.0}, {0.0, 100.0, 0.0},  {0.0, 0.0, -100.0},
    {0.0, 0.0, 100.0},  {10.0, 20.0, 30.0}, {-1000.0, 0.0, 0.0},
};

const Vec3 kWindGValues[] = {
    {-10.0, 0.0, 0.0}, {10.0, 0.0, 0.0}, {0.0, -10.0, 0.0}, {0.0, 10.0, 0.0},
    {0.0, 0.0, -10.0}, {0.0, 0.0, 10.0}, {1.0, 2.0, 3.0},
};

const double kPayoutValues[] = {0.0, 1.0, 10.0, 100.0};
const double kWinchVelValues[] = {-5.0, -1.0, 0.0, 1.0, 5.0};

TEST(HoverTensionInit, SameInitialization) {
  HoverTensionState state0, state1;
  memset(&state0, 0, sizeof(state0));
  memset(&state1, 0xFF, sizeof(state1));
  HoverTensionInit(&GetControlParams()->hover.tension, &state0);
  HoverTensionInit(&GetControlParams()->hover.tension, &state1);
  EXPECT_TRUE(!memcmp(&state0, &state1, sizeof(state0)));
}

TEST(HoverTensionValidateParams, Default) {
  EXPECT_TRUE(HoverTensionValidateParams(&GetControlParams()->hover.tension));
}

TEST(EstimateApparentWind, NotMoving) {
  for (int32_t i = 0; i < ARRAYSIZE(kWingPosGValues); ++i) {
    for (int32_t j = 0; j < ARRAYSIZE(kWindGValues); ++j) {
      Vec3 apparent_wind_g;
      EstimateApparentWind(&kWingPosGValues[i], &kWindGValues[j], 0.0,
                           &apparent_wind_g);
      EXPECT_NEAR_VEC3(apparent_wind_g, kWindGValues[j], 1e-9);
    }
  }
}

TEST(EstimateApparentWind, ZeroWingPosition) {
  Vec3 wing_pos_g = {0.0, 0.0, 0.0};

  for (int32_t i = 0; i < ARRAYSIZE(kWindGValues); ++i) {
    Vec3 apparent_wind_g;
    EstimateApparentWind(&wing_pos_g, &kWindGValues[i], 0.0, &apparent_wind_g);
    EXPECT_NEAR_VEC3(apparent_wind_g, kWindGValues[i], 1e-9);
  }
}

TEST(CalcHorizontalAeroForce, NoDrag) {
  const HoverTensionParams &params = GetControlParams()->hover.tension;
  for (int32_t i = 0; i < ARRAYSIZE(kWingPosGValues); ++i) {
    double drag =
        CalcHorizontalAeroForce(&kWingPosGValues[i], &kVec3Zero, 0.0, &params);
    EXPECT_NEAR(0.0, drag, 1e-9);
  }
}

TEST(HoverTensionSetHorizontalTension, Normal) {
  const HoverTensionParams &params = GetControlParams()->hover.tension;
  HoverTensionState state = HoverTensionState();
  for (int32_t i = 0; i < ARRAYSIZE(kWingPosGValues); ++i) {
    EXPECT_NEAR(
        HoverTensionSetHorizontalTension(&kWingPosGValues[i], &kVec3Zero, 0.0,
                                         false, &params, &state),
        params.tension_min_set_point, 1e-9);
  }
}

TEST(CalcFeedForwardPitch, CheckSigns) {
  const HoverTensionParams &params = GetControlParams()->hover.tension;

  double tension_0 = params.tension_min_set_point + 1000.0;
  Vec3 wind_g_0 = {-10.0, 0.0, 0.0};
  Vec3 wing_pos_g_0 = {-100.0, 0.0, 0.0};
  double payout_0 = 100.0;
  double tether_elevation_cmd = 0.0;
  double pitch_ff_0 =
      CalcFeedForwardPitch(tension_0, tether_elevation_cmd, &wing_pos_g_0,
                           &wind_g_0, payout_0, 0.0, &params);

  // Increasing/decreasing the tension command should
  // increase/decrease the pitch.
  EXPECT_GT(
      CalcFeedForwardPitch(tension_0 + 1.0, tether_elevation_cmd, &wing_pos_g_0,
                           &wind_g_0, payout_0, 0.0, &params),
      pitch_ff_0);
  EXPECT_LT(
      CalcFeedForwardPitch(tension_0 - 1.0, tether_elevation_cmd, &wing_pos_g_0,
                           &wind_g_0, payout_0, 0.0, &params),
      pitch_ff_0);

  // Moving the wing along the wind vector shouldn't have an effect.
  Vec3 wing_pos_g = {-99.0, 0.0, 0.0};
  EXPECT_NEAR(CalcFeedForwardPitch(tension_0, tether_elevation_cmd, &wing_pos_g,
                                   &wind_g_0, payout_0, 0.0, &params),
              pitch_ff_0, 1e-9);

  // Increasing/decreasing the wind speed should reduce/increase the
  // pitch.
  Vec3 higher_wind_g = {-11.0, 0.0, 0.0};
  Vec3 lower_wind_g = {-9.0, 0.0, 0.0};
  EXPECT_GT(CalcFeedForwardPitch(tension_0, tether_elevation_cmd, &wing_pos_g_0,
                                 &lower_wind_g, payout_0, 0.0, &params),
            pitch_ff_0);
  EXPECT_LT(CalcFeedForwardPitch(tension_0, tether_elevation_cmd, &wing_pos_g_0,
                                 &higher_wind_g, payout_0, 0.0, &params),
            pitch_ff_0);

  // Increasing/decreasing the payout should decrease/increase the
  // magnitude of the pitch because the wing is thrusting more/less to
  // counteract the tether weight.  Note that it is important to take
  // the magnitude because the relationship reverses when the
  // horizontal aerodynamic force is greater than the horizontal
  // tension command.
  EXPECT_GT(
      fabs(CalcFeedForwardPitch(tension_0, tether_elevation_cmd, &wing_pos_g_0,
                                &wind_g_0, payout_0 - 50.0, 0.0, &params)),
      fabs(pitch_ff_0));
  EXPECT_LT(
      fabs(CalcFeedForwardPitch(tension_0, tether_elevation_cmd, &wing_pos_g_0,
                                &wind_g_0, payout_0 + 50.0, 0.0, &params)),
      fabs(pitch_ff_0));
}

TEST(CalcPitchLimits, CheckSaturated) {
  const HoverTensionParams &params = GetControlParams()->hover.tension;

  double min_pitch, max_pitch;
  CalcPitchLimits(params.payout_table[0] - 0.1, &params, &min_pitch,
                  &max_pitch);
  EXPECT_NEAR(params.min_pitch_table[0], min_pitch, 1e-9);
  EXPECT_NEAR(params.max_pitch_table[0], max_pitch, 1e-9);

  int32_t n = HOVER_TENSION_PITCH_LIMIT_TABLE_LENGTH;
  CalcPitchLimits(params.payout_table[n - 1] + 1.0, &params, &min_pitch,
                  &max_pitch);
  EXPECT_NEAR(params.min_pitch_table[n - 1], min_pitch, 1e-9);
  EXPECT_NEAR(params.max_pitch_table[n - 1], max_pitch, 1e-9);
}

// When there's no tether weight, the horizontal tension should be
// close to the full tension.
TEST(CalcTensionAtWing, AtOrigin) {
  EXPECT_NEAR(1000.0, CalcTensionAtWing(1000.0, &kVec3Zero), 1e-1);
}

TEST(ModifyPitchCommandInAccel, NoModificationOutsideAccelTransIn) {
  const HoverTensionParams &params = GetControlParams()->hover.tension;
  HoverTensionState state = HoverTensionState();
  for (int32_t i = 0; i < kNumFlightModes; ++i) {
    if (static_cast<FlightMode>(i) != kFlightModeHoverAccel &&
        static_cast<FlightMode>(i) != kFlightModeTransIn) {
      double pitch_in = 0.22;
      Vec3 wing_pos_g = {-100.0, -100.0, -100.0};
      double pitch_cmd = ModifyPitchCommandInAccel(
          pitch_in, &wing_pos_g, static_cast<FlightMode>(i), &params, &state);
      EXPECT_EQ(pitch_in, pitch_cmd);
    }
  }
}

TEST(ModifyPitchCommandInAccel, NegativePitchInAccel) {
  const HoverTensionParams &params = GetControlParams()->hover.tension;
  HoverTensionState state = HoverTensionState();
  for (double z = 0.0; z > -100.0; z -= 10.0) {
    Vec3 wing_pos_g = {-100.0 - z, 0.0, z};
    double pitch_cmd = ModifyPitchCommandInAccel(
        0.0, &wing_pos_g, kFlightModeHoverAccel, &params, &state);
    EXPECT_LE(pitch_cmd, 0.0);
    EXPECT_GE(pitch_cmd, -M_PI / 2.0);
  }
}

// The output of GetHorizontalTensionPilotOffset is filtered, so applying tests
// to the target value stored in its state is simpler.
TEST(GetHorizontalTensionPilotOffset, PilotOffset) {
  const HoverTensionParams &params = GetControlParams()->hover.tension;
  const double roll_threshold =
      params.horizontal_tension_joystick_roll_threshold;
  const double increment = params.horizontal_tension_pilot_increment;
  const int32_t num_cycles = params.horizontal_tension_num_cycles_for_increment;

  auto joystick = JoystickEstimate();
  joystick.valid = true;
  joystick.data.roll = 0.0;
  auto state = HoverTensionState();
  state.joystick_roll_z1 = 0.0;
  state.cycles_above_roll_threshold = 0;
  state.horizontal_tension_increment_enabled = true;
  const FlightMode flight_mode = kFlightModeHoverPayOut;

  // The roll stick moves above the threshold. The offset is updated exactly
  // once, after num_cycles.
  joystick.data.roll = roll_threshold + 0.01;
  for (int32_t i = 1; i < num_cycles; ++i) {
    GetHorizontalTensionPilotOffset(&params, &joystick, flight_mode, &state);
    EXPECT_NEAR(state.horizontal_tension_pilot_offset_target, 0.0, DBL_EPSILON);
  }
  GetHorizontalTensionPilotOffset(&params, &joystick, flight_mode, &state);
  EXPECT_NEAR(state.horizontal_tension_pilot_offset_target, increment,
              DBL_EPSILON);
  for (int32_t i = 1; i < 5 * num_cycles; ++i) {
    GetHorizontalTensionPilotOffset(&params, &joystick, flight_mode, &state);
    EXPECT_NEAR(state.horizontal_tension_pilot_offset_target, increment,
                DBL_EPSILON);
  }

  // The roll stick moves below the threshold for one cycle. Then the increment
  // process may be repeated.
  joystick.data.roll = 0.0;
  GetHorizontalTensionPilotOffset(&params, &joystick, flight_mode, &state);
  EXPECT_NEAR(state.horizontal_tension_pilot_offset_target, increment,
              DBL_EPSILON);
  joystick.data.roll = roll_threshold + 0.01;
  for (int32_t i = 1; i < num_cycles; ++i) {
    GetHorizontalTensionPilotOffset(&params, &joystick, flight_mode, &state);
    EXPECT_NEAR(state.horizontal_tension_pilot_offset_target, increment,
                DBL_EPSILON);
  }
  GetHorizontalTensionPilotOffset(&params, &joystick, flight_mode, &state);
  EXPECT_NEAR(state.horizontal_tension_pilot_offset_target, 2.0 * increment,
              DBL_EPSILON);

  // Now decrement once.
  joystick.data.roll = 0.0;
  GetHorizontalTensionPilotOffset(&params, &joystick, flight_mode, &state);
  EXPECT_NEAR(state.horizontal_tension_pilot_offset_target, 2.0 * increment,
              DBL_EPSILON);
  joystick.data.roll = -roll_threshold - 0.01;
  for (int32_t i = 1; i < num_cycles; ++i) {
    GetHorizontalTensionPilotOffset(&params, &joystick, flight_mode, &state);
    EXPECT_NEAR(state.horizontal_tension_pilot_offset_target, 2.0 * increment,
                DBL_EPSILON);
  }
  GetHorizontalTensionPilotOffset(&params, &joystick, flight_mode, &state);
  EXPECT_NEAR(state.horizontal_tension_pilot_offset_target, increment,
              DBL_EPSILON);

  // Finally, remove the offset in PilotHover.
  joystick.data.roll = 0.0;
  GetHorizontalTensionPilotOffset(&params, &joystick, kFlightModePilotHover,
                                  &state);
  EXPECT_NEAR(state.horizontal_tension_pilot_offset_target, 0.0, DBL_EPSILON);
}

// Confirm that the feedback term is zero when the commanded and
// measured tensions are equal.
TEST(HoverTensionStep, ZeroFeedback) {
  HoverTensionParams params = GetControlParams()->hover.tension;
  // Remove the pitch limits because they interfere with the test.
  for (int32_t i = 0; i < HOVER_TENSION_PITCH_LIMIT_TABLE_LENGTH; ++i) {
    params.min_pitch_table[i] = -1.0;
    params.max_pitch_table[i] = 1.0;
  }

  TetherForceEstimate tether_force = TetherForceEstimate();
  tether_force.valid = true;
  tether_force.sph.tension = 9000.0;

  FlightStatus flight_status = FlightStatus();
  flight_status.flight_mode = kFlightModePilotHover;
  flight_status.last_flight_mode = kFlightModePerched;

  HoverTensionState state = HoverTensionState();
  JoystickEstimate joystick = JoystickEstimate();
  const double tether_elevation_cmd = 0.0;

  state.horizontal_tension_cmd_z1 = tether_force.sph.tension;
  // The wing position must be zero here so there is no tether weight
  // and the horizontal tension command is the same as the tension at
  // the wing command.
  Vec3 wing_pos_g = kVec3Zero;
  Vec3 vessel_g = kVec3Zero;
  for (int32_t i = 0; i < ARRAYSIZE(kWindGValues); ++i) {
    for (int32_t j = 0; j < ARRAYSIZE(kPayoutValues); ++j) {
      for (int32_t k = 0; k < ARRAYSIZE(kWinchVelValues); ++k) {
        double pitch_cmd = HoverTensionStep(
            tether_force.sph.tension, tether_elevation_cmd, &tether_force,
            &wing_pos_g, &vessel_g, &kWindGValues[i], kPayoutValues[j],
            kWinchVelValues[k], false, &flight_status, &joystick, &params,
            &state);
        double pitch_ff = CalcFeedForwardPitch(
            tether_force.sph.tension, tether_elevation_cmd, &wing_pos_g,
            &kWindGValues[i], kPayoutValues[j], kWinchVelValues[k], &params);
        EXPECT_NEAR(pitch_cmd, pitch_ff, 1e-7);
      }
    }
  }
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

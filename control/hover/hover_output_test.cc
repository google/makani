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

#include "control/common.h"
#include "control/control_params.h"
#include "control/control_types.h"
#include "control/hover/hover_output.h"
#include "control/sensor_util.h"
#include "control/system_params.h"
#include "lib/util/test_util.h"

extern "C" {

bool ValidateState(const HoverOutputParams *params,
                   const HoverOutputState *state);

double UpdateGainRampScale(FlightMode flight_mode,
                           const JoystickEstimate *joystick,
                           bool below_gain_ramp_down_thrust_threshold,
                           const HoverOutputParams *params,
                           HoverOutputState *state);

void ScaleThrustMoment(const ThrustMoment *thrust_moment_in, double scale,
                       ThrustMoment *thrust_moment_out);

double CalcAlphaAtElevator(const Vec3 *apparent_wind_b,
                           const HoverOutputParams *params);

double CalcElevatorDeflection(double elevator_pitch_moment,
                              const ApparentWindEstimate *apparent_wind,
                              const FlightStatus *flight_status,
                              const HoverOutputParams *params,
                              HoverOutputState *state);

void ConvertMomentsToDeltas(const Vec3 *moment_cmd,
                            double elevator_pitch_moment,
                            double rudder_yaw_moment,
                            const ApparentWindEstimate *apparent_wind,
                            const FlightStatus *flight_status,
                            const HoverOutputParams *params,
                            HoverOutputState *state, Deltas *deltas);

}  // extern "C"

TEST(ValidateState, Default) {
  const HoverOutputParams &params = GetControlParams()->hover.output;
  HoverOutputState state;
  HoverOutputInit(0.0, 0.0, &kVec3Zero, &params, &state);
  EXPECT_TRUE(ValidateState(&params, &state));
}

TEST(HoverOutputInit, SameInitialization) {
  HoverOutputState state0, state1;
  memset(&state0, 0, sizeof(state0));
  memset(&state1, 0xFF, sizeof(state1));
  HoverOutputInit(0.0, 0.0, &kVec3Zero, &GetControlParams()->hover.output,
                  &state0);
  HoverOutputInit(0.0, 0.0, &kVec3Zero, &GetControlParams()->hover.output,
                  &state1);
  EXPECT_TRUE(!memcmp(&state0, &state1, sizeof(state0)));
}

TEST(HoverOutputValidateParams, Default) {
  EXPECT_TRUE(HoverOutputValidateParams(&GetControlParams()->hover.output));
}

TEST(HoverOutputIsGainRampDone, Normal) {
  HoverOutputState state = HoverOutputState();
  state.gain_ramp_state = kGainStateFull;
  EXPECT_TRUE(HoverOutputIsGainRampDone(&state));
}

TEST(HoverOutputGetLastThrust, Normal) {
  HoverOutputState state = HoverOutputState();
  state.thrust_moment_out.thrust = 100.0;
  EXPECT_NEAR(HoverOutputGetLastThrust(&state), 100.0, 1e-9);
}

TEST(UpdateGainRampScale, ApplySlowRampStartLatched) {
  const HoverOutputParams &params = GetControlParams()->hover.output;

  JoystickEstimate joystick = JoystickEstimate();
  joystick.valid = true;
  joystick.data.throttle = 0.5;

  HoverOutputState state;
  HoverOutputInit(joystick.data.throttle, 0.0, &kVec3Zero, &params, &state);
  state.gain_ramp_state = kGainStateZeroLatched;

  // Get to kGainStateRampUp state, so scale will be greater than 0.0.
  UpdateGainRampScale(kFlightModeHoverAscend, &joystick, false, &params,
                      &state);

  for (double t = 0.0; t < params.gain_ramp_time - 0.02; t += 0.01) {
    double scale = UpdateGainRampScale(kFlightModeHoverAscend, &joystick, false,
                                       &params, &state);
    EXPECT_TRUE(0.0 < scale && scale < 1.0);
    EXPECT_FALSE(HoverOutputIsGainRampDone(&state));
  }

  joystick.data.throttle = 1.0;
  EXPECT_NEAR(UpdateGainRampScale(kFlightModeHoverAscend, &joystick, false,
                                  &params, &state),
              1.0, 1e-9);
  for (double t = 0.0; t < 5.0; t += 0.01) {
    EXPECT_EQ(UpdateGainRampScale(kFlightModeHoverAscend, &joystick, false,
                                  &params, &state),
              1.0);
    EXPECT_TRUE(HoverOutputIsGainRampDone(&state));
  }
}

TEST(UpdateGainRampScale, ApplySlowRampStartUnlatched) {
  const HoverOutputParams &params = GetControlParams()->hover.output;

  JoystickEstimate joystick = JoystickEstimate();
  joystick.valid = true;
  joystick.data.throttle = 0.0;

  HoverOutputState state;
  HoverOutputInit(joystick.data.throttle, 0.0, &kVec3Zero, &params, &state);
  state.gain_ramp_state = kGainStateEStopped;

  for (double t = 0.0;
       t < g_cont.joystick_control->e_stop_throttle_latch_time - 0.01;
       t += 0.01) {
    EXPECT_NEAR(UpdateGainRampScale(kFlightModeHoverAscend, &joystick, false,
                                    &params, &state),
                0.0, 1e-9);
    EXPECT_FALSE(HoverOutputIsGainRampDone(&state));
  }

  // Get to kGainStateRampUp state, so scale will be greater than 0.0.
  joystick.data.throttle = 0.5;
  UpdateGainRampScale(kFlightModeHoverAscend, &joystick, false, &params,
                      &state);

  for (double t = 0.0; t < params.gain_ramp_time - 0.01; t += 0.01) {
    double scale = UpdateGainRampScale(kFlightModeHoverAscend, &joystick, false,
                                       &params, &state);
    EXPECT_TRUE(0.0 <= scale && scale <= 1.0);
    EXPECT_FALSE(HoverOutputIsGainRampDone(&state));
  }

  joystick.data.throttle = 1.0;

  // On this iteration, the gain ramp scale should only differ from
  // 1.0 due to floating point rounding errors.
  EXPECT_NEAR(UpdateGainRampScale(kFlightModeHoverAscend, &joystick, false,
                                  &params, &state),
              1.0, 1e-9);

  // On subsequent iterations, gain_ramp_scale should be exactly 1.0.
  for (double t = 0.0; t < 5.0; t += 0.01) {
    EXPECT_EQ(UpdateGainRampScale(kFlightModeHoverAscend, &joystick, false,
                                  &params, &state),
              1.0);
    EXPECT_TRUE(HoverOutputIsGainRampDone(&state));
  }
}

TEST(UpdateGainRampScale, DoNotApplySlowRamp) {
  const HoverOutputParams &params = GetControlParams()->hover.output;

  JoystickEstimate joystick = JoystickEstimate();
  joystick.valid = true;
  joystick.data.throttle = 0.0;

  HoverOutputState state;
  HoverOutputInit(joystick.data.throttle, 0.0, &kVec3Zero, &params, &state);
  state.gain_ramp_state = kGainStateEStopped;

  for (double t = 0.0;
       t < g_cont.joystick_control->e_stop_throttle_latch_time - 0.1;
       t += 0.01) {
    EXPECT_NEAR(UpdateGainRampScale(kFlightModeHoverAscend, &joystick, false,
                                    &params, &state),
                0.0, 1e-9);
    EXPECT_FALSE(HoverOutputIsGainRampDone(&state));
  }

  // Get to kGainStateFull state, so scale will be 1.0.
  joystick.data.throttle = 0.5;
  UpdateGainRampScale(kFlightModeHoverAscend, &joystick, false, &params,
                      &state);

  for (double t = 0.0; t < 5.0; t += 0.01) {
    EXPECT_NEAR(UpdateGainRampScale(kFlightModeHoverAscend, &joystick, false,
                                    &params, &state),
                1.0, 1e-9);
    EXPECT_TRUE(HoverOutputIsGainRampDone(&state));
  }
}

TEST(UpdateGainRampScale, JoystickInvalid) {
  const HoverOutputParams &params = GetControlParams()->hover.output;

  JoystickEstimate joystick = JoystickEstimate();
  joystick.valid = false;

  HoverOutputState state;
  HoverOutputInit(joystick.data.throttle, 0.0, &kVec3Zero, &params, &state);

  ControlParams *control_params = GetControlParamsUnsafe();

  // When kControlOptHoverThrottleEStop is enabled, an invalid
  // joystick will result in the faulted gain ramp state.
  for (int32_t j = 0; j < kNumFlightModes; ++j) {
    FlightMode flight_mode = static_cast<FlightMode>(j);
    for (int32_t k = 0; k < kNumGainStates; ++k) {
      // Enable kControlOptHoverThrottleEStop.
      state.gain_ramp_state = static_cast<GainState>(k);
      control_params->control_opt = static_cast<ControlOption>(
          control_params->control_opt | kControlOptHoverThrottleEStop);

      UpdateGainRampScale(flight_mode, &joystick, false, &params, &state);
      EXPECT_EQ(state.gain_ramp_state, kGainStateRampDown);

      // Disable kControlOptHoverThrottleEStop.
      state.gain_ramp_state = static_cast<GainState>(k);
      control_params->control_opt = static_cast<ControlOption>(
          control_params->control_opt & ~kControlOptHoverThrottleEStop);

      UpdateGainRampScale(flight_mode, &joystick, false, &params, &state);

      if (static_cast<GainState>(k) != kGainStateRampDown) {
        EXPECT_NE(state.gain_ramp_state, kGainStateRampDown);
      }
    }
  }
}

TEST(UpdateGainRampScale, ApplySlowRampDown) {
  const HoverOutputParams &params = GetControlParams()->hover.output;

  JoystickEstimate joystick = JoystickEstimate();
  joystick.valid = true;
  joystick.data.throttle = 0.5;

  HoverOutputState state;
  HoverOutputInit(joystick.data.throttle, 0.0, &kVec3Zero, &params, &state);
  state.gain_ramp_state = kGainStateFull;

  // Verify that there is no transition in
  // kFlightModeHoverDescend when we are above the ramp-down thrust threshold.
  double scale = UpdateGainRampScale(kFlightModeHoverDescend, &joystick, false,
                                     &params, &state);
  EXPECT_EQ(1.0, scale);
  EXPECT_EQ(kGainStateFull, state.gain_ramp_state);

  // Ramp-down should occur in kFlightModeHoverDescend when thrust is below
  // the ramp-down thrust threshold.
  double scale_z1;
  for (double t = 0.0; t < 2.0; t += 0.01) {
    scale_z1 = scale;
    scale = UpdateGainRampScale(kFlightModeHoverDescend, &joystick, true,
                                &params, &state);
    EXPECT_LE(0.0, scale);
    EXPECT_LE(scale, 1.0);
    EXPECT_LE(scale, scale_z1);
    EXPECT_EQ(kGainStateRampDown, state.gain_ramp_state);
  }
}

// The below_gain_ramp_down_thrust_threshold argument to UpdateGainRampScale
// should have no effect in flight modes other than kFlightModeHoverDescend.
TEST(UpdateGainRampScale, BelowGainRampDownThrustThreshold) {
  const HoverOutputParams &params = GetControlParams()->hover.output;
  ControlParams *control_params = GetControlParamsUnsafe();

  for (int32_t i = 0; i < kNumFlightPlans; ++i) {
    control_params->flight_plan = static_cast<FlightPlan>(i);

    for (int32_t j = 0; j < kNumFlightModes; ++j) {
      const FlightMode flight_mode = static_cast<FlightMode>(j);

      if (flight_mode == kFlightModeHoverDescend) {
        continue;
      }

      for (int32_t k = 0; k < kNumGainStates; ++k) {
        JoystickEstimate joystick = JoystickEstimate();
        joystick.valid = true;
        joystick.data.throttle = 0.5;

        HoverOutputState state1;
        HoverOutputInit(joystick.data.throttle, 0.0, &kVec3Zero, &params,
                        &state1);
        state1.gain_ramp_state = static_cast<GainState>(k);

        HoverOutputState state2 = state1;

        UpdateGainRampScale(flight_mode, &joystick, true, &params, &state1);
        UpdateGainRampScale(flight_mode, &joystick, false, &params, &state2);

        EXPECT_EQ(state1.gain_ramp_latch_timer, state2.gain_ramp_latch_timer);
        EXPECT_EQ(state1.gain_ramp_scale, state2.gain_ramp_scale);
        EXPECT_EQ(state1.gain_ramp_state, state2.gain_ramp_state);
      }
    }
  }
}

TEST(ScaleThrustMoment, Zero) {
  ThrustMoment thrust_moment_in = {1.0, {2.0, 3.0, 4.0}};
  ThrustMoment thrust_moment_out;
  ScaleThrustMoment(&thrust_moment_in, 0.0, &thrust_moment_out);
  EXPECT_NEAR(thrust_moment_out.thrust, 0.0, 1e-9);
  EXPECT_NEAR_VEC3(thrust_moment_out.moment, kVec3Zero, 1e-9);
}

TEST(CalcAlphaAtElevator, SameAlphaAtZeroPropwash) {
  HoverOutputParams params = GetControlParams()->hover.output;
  params.propwash_b = kVec3Zero;

  for (double wind_speed = -10.0; wind_speed < 10.0; wind_speed += 1.0) {
    Vec3 apparent_wind_b = {1.0, 2.0, wind_speed};
    ApparentWindSph apparent_wind_sph;
    ApparentWindCartToSph(&apparent_wind_b, &apparent_wind_sph);
    EXPECT_NEAR(CalcAlphaAtElevator(&apparent_wind_b, &params),
                apparent_wind_sph.alpha, 1e-9);
  }
}

TEST(CalcAlphaAtElevator, SameAlphaHighLowWindSpeeds) {
  HoverOutputParams params = GetControlParams()->hover.output;

  Vec3 apparent_wind_b = {1.0, 2.0, 25.0};

  ApparentWindSph apparent_wind_sph;
  ApparentWindCartToSph(&apparent_wind_b, &apparent_wind_sph);
  EXPECT_NEAR(CalcAlphaAtElevator(&apparent_wind_b, &params),
              apparent_wind_sph.alpha, 1e-9);

  apparent_wind_b.x = 1.0;
  apparent_wind_b.y = 2.0;
  apparent_wind_b.z = -25.0;
  ApparentWindCartToSph(&apparent_wind_b, &apparent_wind_sph);
  EXPECT_NEAR(CalcAlphaAtElevator(&apparent_wind_b, &params),
              apparent_wind_sph.alpha, 1e-9);
}

TEST(CalcElevatorDeflection, AlignWithPropwashInLowWindWithZeroFeedback) {
  HoverOutputParams params = GetControlParams()->hover.output;
  HoverOutputState state = HoverOutputState();

  ApparentWindEstimate apparent_wind = ApparentWindEstimate();
  apparent_wind.vector.x = -0.2;
  apparent_wind.vector.y = 0.1;
  apparent_wind.vector.z = -(params.zero_propwash_wind_speed - 1.0);

  FlightStatus flight_status = FlightStatus();
  flight_status.flight_mode = kFlightModeHoverPayOut;
  flight_status.flight_mode_time = 0.0;

  double propwash_angle = atan2(-params.propwash_b.z, -params.propwash_b.x);

  double delta_ele = 0.0;
  double last_delta_ele;
  do {
    last_delta_ele = delta_ele;
    delta_ele = CalcElevatorDeflection(0.0, &apparent_wind, &flight_status,
                                       &params, &state);
  } while (fabs(delta_ele - last_delta_ele) > 1e-9);

  // Relaxed tolerance because the feed-forward term is low-pass
  // filtered.
  EXPECT_NEAR(-propwash_angle, delta_ele, 1e-6);
}

TEST(ConvertMomentsToDeltas, DoesNotChangeElevatorCalculation) {
  HoverOutputParams params = GetControlParams()->hover.output;
  HoverOutputState state = HoverOutputState();

  double elevator_pitch_moment = 1.1;
  ApparentWindEstimate apparent_wind = ApparentWindEstimate();

  for (int32_t flight_mode = 0; flight_mode < kNumFlightModes; ++flight_mode) {
    FlightStatus flight_status = FlightStatus();
    flight_status.flight_mode = static_cast<FlightMode>(flight_mode);
    flight_status.flight_mode_time = 0.0;

    // Reset the relevant state before each call to CalcElevatorDeflection.
    state.delta_ele_ff_z1 = 0.0;
    double delta_ele = CalcElevatorDeflection(
        elevator_pitch_moment, &apparent_wind, &flight_status, &params, &state);

    state.delta_ele_ff_z1 = 0.0;
    Deltas deltas;
    ConvertMomentsToDeltas(&kVec3Zero, elevator_pitch_moment, 0.0,
                           &apparent_wind, &flight_status, &params, &state,
                           &deltas);
    EXPECT_NEAR(delta_ele, deltas.elevator, 1e-9);
  }
}

TEST(ConvertMomentsToDeltas, CheckFlapSigns) {
  HoverOutputParams params = GetControlParams()->hover.output;
  HoverOutputState state = HoverOutputState();
  ApparentWindEstimate apparent_wind = ApparentWindEstimate();

  Vec3 moment_cmd;
  Deltas deltas;
  for (int32_t flight_mode = 0; flight_mode < kNumFlightModes; ++flight_mode) {
    FlightStatus flight_status = FlightStatus();
    flight_status.flight_mode = static_cast<FlightMode>(flight_mode);
    flight_status.flight_mode_time = 0.0;

    // Check aileron signs.
    moment_cmd = kVec3Zero;
    moment_cmd.x = 1.0;
    ConvertMomentsToDeltas(&moment_cmd, 0.0, 0.0, &apparent_wind,
                           &flight_status, &params, &state, &deltas);
    EXPECT_LE(deltas.aileron, 0.0);

    moment_cmd.x = -1.0;
    ConvertMomentsToDeltas(&moment_cmd, 0.0, 0.0, &apparent_wind,
                           &flight_status, &params, &state, &deltas);
    EXPECT_GE(deltas.aileron, 0.0);

    // Check rudder signs.
    moment_cmd = kVec3Zero;
    moment_cmd.z = 1.0;
    ConvertMomentsToDeltas(&moment_cmd, 0.0, 0.0, &apparent_wind,
                           &flight_status, &params, &state, &deltas);
    EXPECT_LE(deltas.rudder, 0.0);

    moment_cmd.z = -1.0;
    ConvertMomentsToDeltas(&moment_cmd, 0.0, 0.0, &apparent_wind,
                           &flight_status, &params, &state, &deltas);
    EXPECT_GE(deltas.rudder, 0.0);
  }
}

// Confirms that HoverOutputStep sets all fields of the ControlOutput
// structure.
TEST(HoverOutputStep, SetAllValues) {
  StateEstimate state_est = StateEstimate();
  ThrustMoment zero_thrust_moment = ThrustMoment();
  HoverOutputParams params = GetControlParams()->hover.output;
  Vec3 crosswind_loop_center_g = kVec3Ones;
  HoverOutputState state = HoverOutputState();

  ControlOutput control_output_0;
  memset(&control_output_0, 0xFF, sizeof(control_output_0));
  for (int32_t flight_mode = 0; flight_mode < kNumFlightModes; ++flight_mode) {
    FlightStatus flight_status = FlightStatus();
    flight_status.flight_mode = static_cast<FlightMode>(flight_mode);
    flight_status.flight_mode_time = 0.0;

    if (!AnyHoverFlightMode(flight_status.flight_mode)) {
      continue;
    }

    ControlOutput control_output = control_output_0;
    HoverOutputStep(&flight_status, &state_est, crosswind_loop_center_g,
                    &zero_thrust_moment, 0.0, 0.0, 0.0, 22.0, &params, &state,
                    &control_output);

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
      EXPECT_NE(control_output.motor_torque[i],
                control_output_0.motor_torque[i]);
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
}

// Note that holding the actual value of gs_azi_cmd is performed by
// ControlOutputStep, not HoverOutputStep.
TEST(HoverOutputStep, HoldGsTargetAppropriately) {
  FlightStatus flight_status = FlightStatus();
  StateEstimate state_est = StateEstimate();
  ThrustMoment zero_thrust_moment = ThrustMoment();
  Vec3 crosswind_loop_center_g = kVec3Ones;
  HoverOutputParams params = GetControlParams()->hover.output;
  HoverOutputState state = HoverOutputState();
  ControlOutput control_output;

  state_est.joystick.valid = true;
  state_est.joystick.data.throttle = 0.85;
  state_est.joystick.throttle_f = 0.85;
  state_est.vessel.pos_g = kVec3Zero;
  state_est.vessel.dcm_g2v = kMat3Identity;
  state.gain_ramp_state = kGainStateFull;
  flight_status.flight_mode = kFlightModeHoverPayOut;

  // Typically in reel, the targeting data is not held.
  state_est.Xg = {
      0.0, 5.0 * GetSystemParams()->ground_station.gs02.anchor_arm_length, 0.0};
  state_est.gs_mode = kGroundStationModeReel;
  HoverOutputStep(&flight_status, &state_est, crosswind_loop_center_g,
                  &zero_thrust_moment, 0.0, 0.0, 0.0, 22.0, &params, &state,
                  &control_output);
  EXPECT_FALSE(control_output.hold_gs_azi_cmd);

  // Hold the targeting data when perched.
  flight_status.flight_mode = kFlightModePerched;
  HoverOutputStep(&flight_status, &state_est, crosswind_loop_center_g,
                  &zero_thrust_moment, 0.0, 0.0, 0.0, 22.0, &params, &state,
                  &control_output);
  EXPECT_TRUE(control_output.hold_gs_azi_cmd);

  // Hold the targeting data when gain ramping.
  flight_status.flight_mode = kFlightModeHoverDescend;
  state.gain_ramp_state = kGainStateRampDown;
  HoverOutputStep(&flight_status, &state_est, crosswind_loop_center_g,
                  &zero_thrust_moment, 0.0, 0.0, 0.0, 22.0, &params, &state,
                  &control_output);
  EXPECT_TRUE(control_output.hold_gs_azi_cmd);

  // When the kite's xy-position is too close to the origin, we hold the
  // targeting data as a safety measure.
  state_est.Xg = kVec3Zero;
  state_est.gs_mode = kGroundStationModeReel;
  state.gain_ramp_state = kGainStateFull;
  HoverOutputStep(&flight_status, &state_est, crosswind_loop_center_g,
                  &zero_thrust_moment, 0.0, 0.0, 0.0, 22.0, &params, &state,
                  &control_output);
  EXPECT_TRUE(control_output.hold_gs_azi_cmd);

  // The targeting data is held during HighTension.
  state_est.gs_mode = kGroundStationModeHighTension;
  HoverOutputStep(&flight_status, &state_est, crosswind_loop_center_g,
                  &zero_thrust_moment, 0.0, 0.0, 0.0, 22.0, &params, &state,
                  &control_output);
  EXPECT_TRUE(control_output.hold_gs_azi_cmd);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

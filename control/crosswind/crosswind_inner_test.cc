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

#include "control/control_params.h"
#include "control/crosswind/crosswind_inner.h"
#include "lib/util/test_util.h"

extern "C" {

void ScheduleLongitudinalGains(
    double airspeed, const CrosswindInnerParams *params,
    double longitudinal_gains[][kNumCrosswindLongitudinalStates]);

double CalcLongitudinalFeedForward(double alpha_cmd, double dCL_cmd,
                                   const CrosswindInnerParams *params,
                                   double *delta_flap);

double CalcLongitudinalFeedback(
    double alpha_cmd, double alpha, double q_cmd, double q,
    double int_alpha_error,
    double longitudinal_gains[][kNumCrosswindLongitudinalStates],
    double *moment_y);

double InitAlphaErrorIntegrator(double delta_elevator_0, double airspeed,
                                double alpha_cmd, double alpha, double q_cmd,
                                double q, const CrosswindInnerParams *params);

void ScheduleLateralGains(double airspeed, const CrosswindInnerParams *params,
                          double lateral_gains[][kNumCrosswindLateralStates]);

void CalcLateralFeedback(double tether_roll_cmd, double tether_roll,
                         double beta_cmd, double beta, const Vec3 *pqr_cmd,
                         const Vec3 *pqr, double int_tether_roll_error,
                         double int_beta_error,
                         double lateral_gains[][kNumCrosswindLateralStates],
                         double *delta_aileron, double *delta_rudder,
                         double *moment_z);

void InitLateralIntegrators(double delta_aileron_0, double delta_rudder_0,
                            double airspeed, double tether_roll_cmd,
                            double tether_roll, double beta_cmd, double beta,
                            const Vec3 *pqr_cmd, const Vec3 *pqr,
                            const CrosswindInnerParams *params,
                            double *int_tether_roll_error,
                            double *int_beta_error);

}  // extern "C"

TEST(ScheduleLongitudinalGains, ZeroAirspeed) {
  CrosswindInnerParams params = GetControlParams()->crosswind.inner;

  double longitudinal_gains[kNumCrosswindLongitudinalInputs]
                           [kNumCrosswindLongitudinalStates];
  ScheduleLongitudinalGains(0.0, &params, longitudinal_gains);

  for (int32_t i = 0; i < kNumCrosswindLongitudinalInputs; ++i) {
    for (int32_t j = 0; j < kNumCrosswindLongitudinalStates; ++j) {
      EXPECT_NEAR(longitudinal_gains[i][j],
                  params.longitudinal_gains_min_airspeed[i][j], 1e-9);
    }
  }
}

TEST(ScheduleLongitudinalGains, ReallyHighAirspeed) {
  CrosswindInnerParams params = GetControlParams()->crosswind.inner;

  double longitudinal_gains[kNumCrosswindLongitudinalInputs]
                           [kNumCrosswindLongitudinalStates];
  ScheduleLongitudinalGains(340.0, &params, longitudinal_gains);

  for (int32_t i = 0; i < kNumCrosswindLongitudinalInputs; ++i) {
    for (int32_t j = 0; j < kNumCrosswindLongitudinalStates; ++j) {
      EXPECT_NEAR(longitudinal_gains[i][j],
                  params.longitudinal_gains_max_airspeed[i][j], 1e-9);
    }
  }
}

TEST(ScheduleLongitudinalGains, NominalAirspeed) {
  CrosswindInnerParams params = GetControlParams()->crosswind.inner;

  double longitudinal_gains[kNumCrosswindLongitudinalInputs]
                           [kNumCrosswindLongitudinalStates];
  ScheduleLongitudinalGains(
      params.airspeed_table[kCrosswindInnerNominalAirspeed], &params,
      longitudinal_gains);

  for (int32_t i = 0; i < kNumCrosswindLongitudinalInputs; ++i) {
    for (int32_t j = 0; j < kNumCrosswindLongitudinalStates; ++j) {
      EXPECT_NEAR(longitudinal_gains[i][j],
                  params.longitudinal_gains_nominal_airspeed[i][j], 1e-9);
    }
  }
}

TEST(InitAlphaErrorIntegrator, MatchInitialDeflection) {
  const CrosswindInnerParams params = GetControlParams()->crosswind.inner;

  // Set initial parameters.  The commands must be close to the
  // measured values so the required integrator values are not
  // saturated.
  const double delta_elevator_0 = 0.05;
  const double airspeed = 55.0;
  const double alpha_cmd = 0.01;
  const double alpha = 0.011;
  const double q_cmd = -0.001;
  const double q = 0.0;

  double int_alpha_error = InitAlphaErrorIntegrator(
      delta_elevator_0, airspeed, alpha_cmd, alpha, q_cmd, q, &params);

  double longitudinal_gains[kNumCrosswindLongitudinalInputs]
                           [kNumCrosswindLongitudinalStates];
  ScheduleLongitudinalGains(airspeed, &params, longitudinal_gains);

  double unused_delta_flap;
  double delta_elevator_ff =
      CalcLongitudinalFeedForward(alpha_cmd, 0.0, &params, &unused_delta_flap);
  double unused_moment_y;
  double delta_elevator_fb =
      CalcLongitudinalFeedback(alpha_cmd, alpha, q_cmd, q, int_alpha_error,
                               longitudinal_gains, &unused_moment_y);

  EXPECT_NEAR(delta_elevator_0, delta_elevator_ff + delta_elevator_fb, 1e-3);
}

TEST(ScheduleLateralGains, ZeroAirspeed) {
  CrosswindInnerParams params = GetControlParams()->crosswind.inner;

  double lateral_gains[kNumCrosswindLateralInputs][kNumCrosswindLateralStates];
  ScheduleLateralGains(0.0, &params, lateral_gains);

  for (int32_t i = 0; i < kNumCrosswindLateralInputs; ++i) {
    for (int32_t j = 0; j < kNumCrosswindLateralStates; ++j) {
      EXPECT_NEAR(lateral_gains[i][j], params.lateral_gains_min_airspeed[i][j],
                  1e-9);
    }
  }
}

TEST(ScheduleLateralGains, ReallyHighAirspeed) {
  CrosswindInnerParams params = GetControlParams()->crosswind.inner;

  double lateral_gains[kNumCrosswindLateralInputs][kNumCrosswindLateralStates];
  ScheduleLateralGains(340.0, &params, lateral_gains);

  for (int32_t i = 0; i < kNumCrosswindLateralInputs; ++i) {
    for (int32_t j = 0; j < kNumCrosswindLateralStates; ++j) {
      EXPECT_NEAR(lateral_gains[i][j], params.lateral_gains_max_airspeed[i][j],
                  1e-9);
    }
  }
}

TEST(ScheduleLateralGains, NominalAirspeed) {
  CrosswindInnerParams params = GetControlParams()->crosswind.inner;

  double lateral_gains[kNumCrosswindLateralInputs][kNumCrosswindLateralStates];
  ScheduleLateralGains(params.airspeed_table[kCrosswindInnerNominalAirspeed],
                       &params, lateral_gains);

  for (int32_t i = 0; i < kNumCrosswindLateralInputs; ++i) {
    for (int32_t j = 0; j < kNumCrosswindLateralStates; ++j) {
      EXPECT_NEAR(lateral_gains[i][j],
                  params.lateral_gains_nominal_airspeed[i][j], 1e-9);
    }
  }
}

TEST(InitLateralIntegrators, MatchInitialDeflections) {
  const CrosswindInnerParams params = GetControlParams()->crosswind.inner;

  // Set initial parameters.  The commands must be close to the
  // measured values so the required integrator values are not
  // saturated.
  const double delta_aileron_0 = 0.1;
  const double delta_rudder_0 = -0.1;
  const double airspeed = 55.0;
  const double tether_roll_cmd = 0.1;
  const double tether_roll = 0.11;
  const double beta_cmd = 0.1;
  const double beta = 0.13;
  const Vec3 pqr_cmd = {-0.01, 0.01, 0.02};
  const Vec3 pqr = {0.0, 0.0, 0.0};

  double int_tether_roll_error, int_beta_error;
  InitLateralIntegrators(delta_aileron_0, delta_rudder_0, airspeed,
                         tether_roll_cmd, tether_roll, beta_cmd, beta, &pqr_cmd,
                         &pqr, &params, &int_tether_roll_error,
                         &int_beta_error);

  // The following four assertions test the initial parameters
  // (above); if either integrator value is saturating, the test
  // conditions should be changed.
  EXPECT_NE(int_tether_roll_error, params.int_tether_roll_min);
  EXPECT_NE(int_tether_roll_error, params.int_tether_roll_max);
  EXPECT_NE(int_beta_error, params.int_beta_min);
  EXPECT_NE(int_beta_error, params.int_beta_max);

  double lateral_gains[kNumCrosswindLateralInputs][kNumCrosswindLateralStates];
  ScheduleLateralGains(airspeed, &params, lateral_gains);

  double delta_aileron, delta_rudder, unused_moment_z;
  CalcLateralFeedback(tether_roll_cmd, tether_roll, beta_cmd, beta, &pqr_cmd,
                      &pqr, int_tether_roll_error, int_beta_error,
                      lateral_gains, &delta_aileron, &delta_rudder,
                      &unused_moment_z);

  EXPECT_NEAR(delta_aileron_0, delta_aileron, 1e-3);
  EXPECT_NEAR(delta_rudder_0, delta_rudder, 1e-3);
}

TEST(CrosswindInnerInit, SameInitialization) {
  StateEstimate state_estimate = StateEstimate();
  Deltas deltas = Deltas();
  CrosswindInnerState state0, state1;
  memset(&state0, 0, sizeof(state0));
  memset(&state1, 0xFF, sizeof(state1));
  CrosswindInnerInit(&state_estimate, &deltas, 0.0, 0.0, 0.0,
                     &GetControlParams()->crosswind.inner, &state0);
  CrosswindInnerInit(&state_estimate, &deltas, 0.0, 0.0, 0.0,
                     &GetControlParams()->crosswind.inner, &state1);
  EXPECT_TRUE(!memcmp(&state0, &state1, sizeof(state0)));
}

TEST(CrosswindInnerValidateParams, Default) {
  const CrosswindInnerParams params = GetControlParams()->crosswind.inner;
  EXPECT_TRUE(CrosswindInnerValidateParams(&params));
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

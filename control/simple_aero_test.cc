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

#include <float.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <limits>

#include "common/c_math/vec3.h"
#include "control/control_params.h"
#include "control/control_types.h"
#include "control/simple_aero.h"
#include "control/system_types.h"
#include "lib/util/test_util.h"

using ::test_util::Rand;

extern "C" {

double CalcThrustCoeff(double J, const SimpleRotorModelParams *params);

}  // extern "C"

TEST(AlphaToCL, Recovery) {
  SimpleAeroModelParams params;
  params.dCL_dalpha = 5.2;
  params.CL_0 = 0.1;

  for (int32_t i = 0; i < 22; ++i) {
    double CL_in = Rand(-1.0, 1.0);
    double CL_out = AlphaToCL(CLToAlpha(CL_in, &params), &params);
    EXPECT_NEAR(CL_out, CL_in, 1e-9);
  }
}

TEST(BetaToCY, Recovery) {
  SimpleAeroModelParams params;
  params.dCY_dbeta = -5.8;
  params.CY_0 = -0.15;
  for (int32_t i = 0; i < 22; ++i) {
    double CY_in = Rand(-1.0, 1.0);
    double CY_out = BetaToCY(CYToBeta(CY_in, &params), &params);
    EXPECT_NEAR(CY_out, CY_in, 1e-9);
  }
}

#if !defined(NDEBUG)

TEST(JToOmega, ZeroDeath) {
  ASSERT_DEATH(JToOmega(0.0, 1.0, 1.0), "");
  ASSERT_DEATH(JToOmega(1.0, 1.0, 0.0), "");
}

#endif  // !defined(NDEBUG)

TEST(JToOmega, Ones) {
  EXPECT_NEAR(JToOmega(1.0, 1.0, 1.0), 2.0 * M_PI,
              std::numeric_limits<double>::epsilon());
}

TEST(CalcLocalAirspeed, Simple) {
  Vec3 pos = {0.0, 1.0, 0.0};
  Vec3 pqr = {0.0, 0.0, 1.0};
  double ans = CalcLocalAirspeed(1.0, 0.0, &pos, &pqr);
  EXPECT_NEAR(ans, 0.0, 1e-9);
}

#if !defined(NDEBUG)

TEST(CalcLocalAirspeed, NegativeDeath) {
  ASSERT_DEATH(CalcLocalAirspeed(1.0, 2.0, &kVec3Zero, &kVec3Zero), "");
}

#endif  // !defined(NDEBUG)

TEST(OmegaToThrust, Static) {
  SimpleRotorModelParams params = {
      {0.046, -0.125, -0.34}, 0.66, 0.95, 2.32, pow(2.32, 4.0)};
  double air_density = 1.2;
  for (int32_t i = 0; i < 22222; ++i) {
    double omega = Rand(0.0, 1000.0);
    double omega_out =
        ThrustToOmega(OmegaToThrust(omega, 0.0, air_density, &params), 0.0,
                      air_density, &params);
    EXPECT_NEAR(omega, omega_out, 1e-3);
  }
}

TEST(OmegaToThrust, Invert) {
  SimpleRotorModelParams params = {
      {0.046, -0.125, -0.34}, 0.66, 0.95, 2.32, pow(2.32, 4.0)};
  for (int32_t i = 0; i < 22222; ++i) {
    double omega = Rand(10.0, 1000.0);
    double v_inf = Rand(-50.0, 100.0);
    double J = 2.0 * PI * v_inf / (omega * 0.78);
    double air_density = 1.2;
    // For the above simple rotor model parameters, dTdw goes negative
    // at J = 1.08.
    if (0.0 < J && J < params.J_max) {
      double omega_out =
          ThrustToOmega(OmegaToThrust(omega, v_inf, air_density, &params),
                        v_inf, air_density, &params);
      EXPECT_NEAR(omega, omega_out, 5.0);
    }
  }
}

TEST(OmegaToThrust, StaticRecovery) {
  SimpleRotorModelParams params = {
      {0.046, -0.125, -0.34}, 0.66, 0.95, 2.32, pow(2.32, 4.0)};
  double air_density = 1.2;
  for (double thrust = 0.0; thrust < 1000.0; thrust += 10.0) {
    double thrust_out =
        OmegaToThrust(ThrustToOmega(thrust, 0.0, air_density, &params), 0.0,
                      air_density, &params);
    EXPECT_NEAR(thrust, thrust_out, 1e-9);
  }
}

TEST(OmegaToThrust, NonStaticRecovery) {
  SimpleRotorModelParams params = {
      {0.046, -0.125, -0.34}, 0.66, 0.95, 2.32, pow(2.32, 4.0)};
  double air_density = 1.2;
  for (double v_inf = 0.0; v_inf < 100.0; v_inf += 1.0) {
    for (double thrust = 0.0; thrust < 1000.0; thrust += 10.0) {
      double thrust_out =
          OmegaToThrust(ThrustToOmega(thrust, v_inf, air_density, &params),
                        v_inf, air_density, &params);
      EXPECT_NEAR(thrust, thrust_out, 5.0);
    }
  }
}

TEST(CalcThrustCoeff, CompareToOld) {
  SimpleRotorModelParams params_neg0 = {
      {0.0017748, -0.00045675, -0.0028623}, 0.64, 1.0, 0.78, pow(0.78, 4.0)};
  double k_omega_sq_to_thrust = CalcThrustCoeff(0.0, &params_neg0);
  EXPECT_NEAR(k_omega_sq_to_thrust, 1.2e-3, 2e-4);

  SimpleRotorModelParams params_neg2 = {
      {0.0018696, -0.0005084, -0.0026568}, 0.56, 0.9, 0.78, pow(0.78, 4.0)};
  k_omega_sq_to_thrust = CalcThrustCoeff(0.0, &params_neg2);
  EXPECT_NEAR(k_omega_sq_to_thrust, 1e-3, 2e-5);
}

TEST(ThrustToOmega, VInfNearZero) {
  SimpleRotorModelParams params = {
      {0.046, -0.125, -0.34}, 0.66, 0.95, 2.32, pow(2.32, 4.0)};
  double air_density = 1.2;
  for (int32_t i = 0; i < 222; ++i) {
    // At thrust = 0, even a small thrust tolerance can result in a
    // large omega difference, so we go to thrust = 10 N.
    double thrust = Rand(10.0, 100.0);
    double omega_0 = ThrustToOmega(thrust, 0.0, air_density, &params);
    double omega_0p = ThrustToOmega(thrust, 1e-6, air_density, &params);
    EXPECT_NEAR(omega_0, omega_0p, 1.0);
  }
}

// Checks that the ThrustToOmega function defaults to returning the
// highest omega at which the thrust is equal to the thrust at the
// minimum omega.
TEST(ThrustToOmega, ThrustBelowMinimumThrust) {
  double air_density = 1.2;
  for (int32_t i = 0; i < kNumPropVersions; ++i) {
    const SimpleRotorModelParams *params =
        &g_cont.rotor_control->simple_models[i];
    for (double v_freestream = 10.0; v_freestream < 100.0;
         v_freestream += 10.0) {
      double omega_min =
          2.0 * M_PI * v_freestream / (params->D * params->J_max);
      double thrust_min =
          OmegaToThrust(omega_min, v_freestream, air_density, params);

      // Find omega at the minimum thrust by brute force.
      const double omega_tol = 0.1;
      double omega_ans = omega_min;
      double omega_max =
          2.0 * M_PI * v_freestream / (params->D * params->J_neutral);
      for (double omega = omega_max; omega >= omega_min; omega -= omega_tol) {
        double thrust = OmegaToThrust(omega, v_freestream, air_density, params);
        if (thrust <= thrust_min) {
          omega_ans = omega;
          break;
        }
      }

      // Check that ThrustToOmega gets the same result as the brute force
      // method.
      for (double thrust = thrust_min - 1000.0; thrust < thrust_min;
           thrust += 100.0) {
        double omega = ThrustToOmega(thrust, v_freestream, air_density, params);
        EXPECT_NEAR(omega, omega_ans, 2.0 * omega_tol);
      }
    }
  }
}

TEST(ThrustToOmega, Continuous) {
  for (int32_t i = 0; i < kNumPropVersions; ++i) {
    const SimpleRotorModelParams *params =
        &g_cont.rotor_control->simple_models[i];
    double air_density = 1.2;
    for (double v_freestream = 0.0; v_freestream < 100.0; v_freestream += 1.0) {
      double omega_min = JToOmega(params->J_max, v_freestream, params->D);
      double thrust_min =
          OmegaToThrust(omega_min, v_freestream, air_density, params);
      for (double thrust = thrust_min - 1000.0; thrust < 1000.0;
           thrust += 1.0) {
        double omega0 =
            ThrustToOmega(thrust, v_freestream, air_density, params);
        double omega_perturb_thrust =
            ThrustToOmega(thrust + 1.0, v_freestream, air_density, params);
        double omega_perturb_vel =
            ThrustToOmega(thrust, v_freestream + 1.0, air_density, params);
        EXPECT_NEAR(omega0, omega_perturb_thrust, 10.0);
        EXPECT_NEAR(omega0, omega_perturb_vel, 10.0);
      }
    }
  }
}

TEST(CalcMinThrust, Simple) {
  const RotorControlParams &params = *g_cont.rotor_control;
  double air_density = 1.2;
  for (int32_t r = 0; r < kNumMotors; ++r) {
    const SimpleRotorModelParams &model = params.simple_models[r];
    for (double v_app_local = -1.0; v_app_local < 100.0; v_app_local += 1.0) {
      double min_omega = JToOmega(model.J_max, v_app_local, model.D);
      for (double omega = min_omega; omega < 200.0; omega += 1.0) {
        if (omega >= params.idle_speed) {
          EXPECT_GE(OmegaToThrust(omega, v_app_local, air_density, &model),
                    CalcMinThrust(v_app_local, params.idle_speed, air_density,
                                  &model));
        }
      }
    }
  }
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

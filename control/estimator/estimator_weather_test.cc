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

#include "control/estimator/estimator_weather.h"

#include <gtest/gtest.h>

#include "control/control_params.h"
#include "control/fault_detection/fault_detection_types.h"
#include "control/fault_detection/fault_detection_util.h"
#include "control/system_params.h"

TEST(EstimatorWindInit, Sanity) {
  GsWeather weather;
  weather.temperature = 18.0;    // [deg C]
  weather.pressure = 102404.39;  // [Pa]
  weather.humidity = 0.6;        // [#]

  const double rho_true = 1.220;  // [kg/m^3]
  double rho_estimate;

  const EstimatorWeatherParams params = GetControlParams()->estimator.weather;
  EstimatorWeatherState state = EstimatorWeatherState();
  EstimatorWeatherInit(&params, &state);
  rho_estimate = Lpf2(state.last_valid_rho, params.fc_rho, params.zeta_rho,
                      *g_sys.ts, state.rho_zs);
  EXPECT_NEAR(rho_estimate, g_sys.phys->rho, g_sys.phys->rho * 0.01);

  FaultMask fault;
  ClearAllFaults(&fault);

  EstimatorWeatherStep(true, &weather, &fault, &params, &state, &rho_estimate);
  EXPECT_NEAR(rho_estimate, rho_true, rho_true * 0.1);
  EXPECT_NEAR(state.last_valid_rho, rho_true, rho_true * 0.1);

  EstimatorWeatherStep(false, &weather, &fault, &params, &state, &rho_estimate);
  EXPECT_NEAR(rho_estimate, rho_true, rho_true * 0.1);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

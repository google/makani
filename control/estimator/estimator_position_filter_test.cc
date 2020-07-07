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

#include "control/estimator/estimator_position_kite.h"

#include <gtest/gtest.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "common/c_math/util.h"
#include "control/control_params.h"
#include "control/control_types.h"
#include "control/estimator/estimator_position_filter_kite.h"
#include "control/estimator/estimator_types.h"
#include "lib/util/test_util.h"

TEST(EstimatorPositionFilter, Zero0) {
  EstimatorPositionBaroEstimate baro = EstimatorPositionBaroEstimate();
  baro.valid = true;
  baro.sigma_Xg_z = 5.0;
  EstimatorPositionGpsEstimate gps_center = EstimatorPositionGpsEstimate();
  gps_center.wing_pos_valid = true;
  gps_center.wing_vel_valid = true;
  gps_center.sigma_Xg = {0.22, 0.22, 0.22};
  gps_center.sigma_Vg = {0.22, 0.22, 0.22};
  EstimatorPositionGpsEstimate gps_port = EstimatorPositionGpsEstimate();
  gps_port.wing_pos_valid = true;
  gps_port.wing_vel_valid = true;
  gps_port.sigma_Xg = {0.22, 0.22, 0.22};
  gps_port.sigma_Vg = {0.22, 0.22, 0.22};
  EstimatorPositionGpsEstimate gps_star = EstimatorPositionGpsEstimate();
  gps_star.wing_pos_valid = true;
  gps_star.wing_vel_valid = true;
  gps_star.sigma_Xg = {0.22, 0.22, 0.22};
  gps_star.sigma_Vg = {0.22, 0.22, 0.22};
  EstimatorPositionGlasEstimate glas = EstimatorPositionGlasEstimate();
  glas.wing_pos_valid = true;
  glas.sigma_Xg = {5.0, 5.0, 5.0};
  const Vec3 acc = {0.0, 0.0, -9.81};
  const FlightMode fm = kFlightModeCrosswindNormal;

  EstimatorPositionFilterParams params =
      GetControlParams()->estimator.nav.position.filter;
  params.sigma_vel_g_0 = 0.1;
  params.min_gps_sigma_vel_g = 0.25;
  params.min_gps_sigma_pos_g = 1.0;
  params.sigma_wing_accel_hover = 2.0;
  params.sigma_wing_accel_dynamic = 10.0;

  Vec3 Xg_out, Vg_out;
  EstimatorVelocitySolutionType vel_type;
  EstimatorPositionFilterState state;
  EstimatorPositionFilterInit(&params, &state);
  EstimatorPositionCorrections correct;
  memset(&correct, 0, sizeof(correct));
  EstimatorPositionFilterKiteStep(
      &kMat3Identity, &acc, &baro, &glas, &gps_center, &gps_port, &gps_star, fm,
      &params, &state, &correct, &Xg_out, &Vg_out, &vel_type);

  EXPECT_NEAR_VEC3(kVec3Zero, Xg_out, 1e-9);
  EXPECT_NEAR_VEC3(kVec3Zero, Vg_out, 1e-9);
  EXPECT_EQ(vel_type, kEstimatorVelocitySolutionTypeGps);
}

TEST(EstimatorPositionFilterKiteStep, Zero1) {
  EstimatorPositionBaroEstimate baro = EstimatorPositionBaroEstimate();
  baro.valid = true;
  baro.sigma_Xg_z = 5.0;
  EstimatorPositionGpsEstimate gps_center = EstimatorPositionGpsEstimate();
  gps_center.wing_pos_valid = true;
  gps_center.wing_vel_valid = true;
  gps_center.sigma_Xg = {0.22, 0.22, 0.22};
  gps_center.sigma_Vg = {0.22, 0.22, 0.22};
  EstimatorPositionGpsEstimate gps_port = EstimatorPositionGpsEstimate();
  gps_port.wing_pos_valid = true;
  gps_port.wing_vel_valid = true;
  gps_port.sigma_Xg = {0.22, 0.22, 0.22};
  gps_port.sigma_Vg = {0.22, 0.22, 0.22};
  EstimatorPositionGpsEstimate gps_star = EstimatorPositionGpsEstimate();
  gps_star.wing_pos_valid = true;
  gps_star.wing_vel_valid = true;
  gps_star.sigma_Xg = {0.22, 0.22, 0.22};
  gps_star.sigma_Vg = {0.22, 0.22, 0.22};
  EstimatorPositionGlasEstimate glas = EstimatorPositionGlasEstimate();
  glas.wing_pos_valid = true;
  glas.sigma_Xg = {5.0, 5.0, 5.0};
  const Vec3 acc = {0.0, 0.0, -9.81};
  const FlightMode fm = kFlightModeHoverFullLength;

  EstimatorPositionFilterParams params =
      GetControlParams()->estimator.nav.position.filter;
  params.sigma_vel_g_0 = 0.1;
  params.min_gps_sigma_vel_g = 0.25;
  params.min_gps_sigma_pos_g = 1.0;
  params.sigma_wing_accel_hover = 2.0;
  params.sigma_wing_accel_dynamic = 10.0;

  Vec3 Xg_out, Vg_out;
  EstimatorVelocitySolutionType vel_type;
  EstimatorPositionFilterState state;
  EstimatorPositionFilterInit(&params, &state);
  EstimatorPositionCorrections correct;
  memset(&correct, 0, sizeof(correct));
  EstimatorPositionFilterKiteStep(
      &kMat3Identity, &acc, &baro, &glas, &gps_center, &gps_port, &gps_star, fm,
      &params, &state, &correct, &Xg_out, &Vg_out, &vel_type);

  EXPECT_NEAR_VEC3(kVec3Zero, Xg_out, 1e-9);
  EXPECT_NEAR_VEC3(kVec3Zero, Vg_out, 1e-9);
  EXPECT_EQ(vel_type, kEstimatorVelocitySolutionTypeGps);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

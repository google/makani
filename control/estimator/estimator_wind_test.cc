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

#include "control/estimator/estimator_wind.h"

#include <gtest/gtest.h>

#include <math.h>
#include <stdint.h>
#include <stdlib.h>

#include "common/c_math/util.h"
#include "common/c_math/vec3.h"
#include "control/control_params.h"
#include "control/control_types.h"
#include "control/fault_detection/fault_detection_types.h"
#include "control/fault_detection/fault_detection_util.h"
#include "control/sensor_util.h"
#include "control/system_params.h"
#include "lib/util/test_util.h"

using ::test_util::Rand;
using ::test_util::LpfIterationTolerance;

TEST(FilterEstimates, Consistency0) {
  const SystemParams *system_params = GetSystemParams();
  const EstimatorWindParams *params = &GetControlParams()->estimator.wind;
  for (int32_t j = 0; j < test_util::kNumTests; j++) {
    const Vec3 vector = {Rand(-1e2, 1e2), Rand(-1e2, 1e2), Rand(-1e2, 1e2)};

    FaultMask fault;
    ClearAllFaults(&fault);
    EstimatorWindState state;
    EstimatorWindInit(&state);

    VesselEstimate vessel = VesselEstimate();
    vessel.pos_g = kVec3Zero;
    vessel.vel_g = kVec3Zero;
    vessel.dcm_g2v = kMat3Identity;
    vessel.dcm_g2p = kMat3Identity;
    vessel.pqr = kVec3Zero;
    vessel.position_valid = true;
    vessel.attitude_valid = true;

    int32_t N = 1000;
    WindEstimate wind_g;
    for (int32_t i = 0; i < N; i++) {
      EstimatorWindStep(i < 10, &vector, &fault, &vessel,
                        &system_params->wind_sensor, params, &state, &wind_g);
    }

    EXPECT_NEAR_RELATIVE_VEC3(
        state.last_valid_wind_g, wind_g.vector_f,
        LpfIterationTolerance(params->fc_vector, *g_sys.ts, N));
    EXPECT_NEAR_RELATIVE_VEC3(
        state.last_valid_wind_g, wind_g.vector_f_slow,
        LpfIterationTolerance(params->fc_vector_slow, *g_sys.ts, N));
    EXPECT_NEAR_RELATIVE(Vec3Norm(&state.last_valid_wind_g), wind_g.speed_f,
                         LpfIterationTolerance(params->fc_speed, *g_sys.ts, N));
    // We omit wind direction as it uses a variable filter coefficient.
  }
}

TEST(EstimateWind, HoldOnFault) {
  FaultMask fault;
  ClearAllFaults(&fault);

  VesselEstimate vessel = VesselEstimate();
  vessel.pos_g = kVec3Zero;
  vessel.vel_g = kVec3Zero;
  vessel.dcm_g2v = kMat3Identity;
  vessel.dcm_g2p = kMat3Identity;
  vessel.pqr = kVec3Zero;
  vessel.position_valid = true;
  vessel.attitude_valid = true;

  EstimatorWindState state;
  EstimatorWindInit(&state);
  WindSensorParams wind_sensor_params = GetSystemParams()->wind_sensor;
  wind_sensor_params.on_perch = true;

  for (bool on_perch : {true, false}) {
    wind_sensor_params.on_perch = on_perch;
    for (int32_t i = 0; i < 10; ++i) {
      Vec3 wind_ws = {Rand(-1.0, 1.0), Rand(-1.0, 1), Rand(-1.0, 1.0)};
      Vec3 wind_ws_2 = {Rand(-1.0, 1.0), Rand(-1.0, 1.0), Rand(-1.0, 1.0)};

      const Mat3 *dcm_parent2ws = &wind_sensor_params.dcm_parent2ws;
      Vec3 wind_p;
      Mat3TransVec3Mult(dcm_parent2ws, &wind_ws, &wind_p);
      Vec3 wind_p_2;
      Mat3TransVec3Mult(dcm_parent2ws, &wind_ws_2, &wind_p_2);

      WindEstimate wind_g;
      vessel.position_valid = true;
      vessel.attitude_valid = true;
      EstimatorWindStep(false, &wind_ws, &fault, &vessel, &wind_sensor_params,
                        &GetControlParams()->estimator.wind, &state, &wind_g);
      EXPECT_NEAR_VEC3(wind_p, wind_g.vector, 0.0);
      EXPECT_TRUE(wind_g.valid);

      // Either position or attitude fault in VesselEstimate should fault the
      // wind sensor. Test out the three possible combinations that fail (only a
      // single one valid, or both not valid).
      vessel.position_valid = ((i % 3) == 1);
      vessel.attitude_valid = ((i % 3) == 2);
      EstimatorWindStep(false, &wind_ws_2, &fault, &vessel, &wind_sensor_params,
                        &GetControlParams()->estimator.wind, &state, &wind_g);
      if (on_perch) {
        EXPECT_NEAR_VEC3(wind_p, wind_g.vector, 0.0);
        EXPECT_FALSE(wind_g.valid);
      } else {
        EXPECT_NEAR_VEC3(wind_p_2, wind_g.vector, 0.0);
        EXPECT_TRUE(wind_g.valid);
      }

      vessel.position_valid = true;
      vessel.attitude_valid = true;
      EstimatorWindStep(false, &wind_ws_2, &fault, &vessel, &wind_sensor_params,
                        &GetControlParams()->estimator.wind, &state, &wind_g);
      EXPECT_NEAR_VEC3(wind_p_2, wind_g.vector, 0.0);
      EXPECT_TRUE(wind_g.valid);

      SetFault(kFaultTypeNoUpdate, true, &fault);
      EstimatorWindStep(false, &wind_ws, &fault, &vessel, &wind_sensor_params,
                        &GetControlParams()->estimator.wind, &state, &wind_g);
      EXPECT_NEAR_VEC3(wind_p_2, wind_g.vector, 0.0);
      EXPECT_FALSE(wind_g.valid);

      ClearAllFaults(&fault);
      EstimatorWindStep(false, &wind_ws, &fault, &vessel, &wind_sensor_params,
                        &GetControlParams()->estimator.wind, &state, &wind_g);
      EXPECT_NEAR_VEC3(wind_p, wind_g.vector, 0.0);
      EXPECT_TRUE(wind_g.valid);
    }
  }
}

TEST(EstimateWindAloft, Recovery) {
  EstimatorWindParams params = GetControlParams()->estimator.wind;
  EstimatorWindState state;
  EstimatorWindInit(&state);

  Vec3 wind_aloft_g_true = {-15.1, 2.7, 5.5};
  Vec3 wing_velocity_g = {1.22, 50.66, 3.33};

  Vec3 apparent_wind_g;
  Vec3Sub(&wind_aloft_g_true, &wing_velocity_g, &apparent_wind_g);

  Mat3 dcm_g2b;
  AngleToDcm(0.1, 0.2, 0.3, kRotationOrderZyx, &dcm_g2b);

  ApparentWindEstimate apparent_wind = ApparentWindEstimate();
  Mat3Vec3Mult(&dcm_g2b, &apparent_wind_g, &apparent_wind.vector);
  ApparentWindCartToSph(&apparent_wind.vector, &apparent_wind.sph);
  apparent_wind.solution_type = kApparentWindSolutionTypePitot;

  WindEstimate gs_wind_estimate = WindEstimate();
  gs_wind_estimate.vector = {-10.1, 1.6, 2.5};
  gs_wind_estimate.valid = true;
  gs_wind_estimate.solution_type = kWindSolutionTypeGroundStationSensor;

  // Estimate the winds aloft using the Pitot and inertial velocity.
  WindEstimate wind_aloft_estimate;
  EstimatorWindAloftStep(false, &apparent_wind, &apparent_wind.sph,
                         &wing_velocity_g, &dcm_g2b, &gs_wind_estimate, &params,
                         &state, &wind_aloft_estimate);
  EXPECT_NEAR_VEC3(wind_aloft_estimate.vector, wind_aloft_g_true, 1e-3);
  EXPECT_EQ(wind_aloft_estimate.solution_type,
            kWindSolutionTypePitotAndInertial);

  // When the Pitot is not available, we expect to simply return the
  // wind as seen by the ground station.
  apparent_wind.solution_type = kApparentWindSolutionTypeMixed;
  EstimatorWindAloftStep(false, &apparent_wind, &apparent_wind.sph,
                         &wing_velocity_g, &dcm_g2b, &gs_wind_estimate, &params,
                         &state, &wind_aloft_estimate);
  EXPECT_NEAR_VEC3(wind_aloft_estimate.vector, gs_wind_estimate.vector, 1e-3);
  EXPECT_EQ(wind_aloft_estimate.solution_type,
            kWindSolutionTypeGroundStationSensor);

  // Now disable the ground station wind too.
  gs_wind_estimate.solution_type = kWindSolutionTypeNone;
  EstimatorWindAloftStep(false, &apparent_wind, &apparent_wind.sph,
                         &wing_velocity_g, &dcm_g2b, &gs_wind_estimate, &params,
                         &state, &wind_aloft_estimate);
  EXPECT_FALSE(wind_aloft_estimate.valid);
  EXPECT_EQ(wind_aloft_estimate.solution_type, kWindSolutionTypeNone);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

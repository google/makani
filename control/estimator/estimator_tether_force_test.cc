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

#include "control/estimator/estimator_tether_force.h"

#include <gtest/gtest.h>

#include <math.h>
#include <stdint.h>
#include <stdlib.h>

#include "common/c_math/util.h"
#include "control/control_params.h"
#include "control/control_types.h"
#include "control/fault_detection/fault_detection_types.h"
#include "control/fault_detection/fault_detection_util.h"
#include "control/sensor_util.h"
#include "control/system_params.h"
#include "lib/util/test_util.h"

using ::test_util::LpfIterationTolerance;
using ::test_util::Rand;

TEST(FilterTetherTension, FilterConsistency) {
  const EstimatorTetherForceParams &params =
      GetControlParams()->estimator.tether_force;
  for (int32_t j = 0; j < test_util::kNumTests; j++) {
    double loadcells[kNumLoadcellSensors];
    FaultMask faults[kNumLoadcellSensors];
    for (int32_t i = 0; i < kNumLoadcellSensors; ++i) {
      loadcells[i] = Rand(0, 100.0);
      ClearAllFaults(&faults[i]);
    }

    EstimatorTetherForceState state;
    EstimatorTetherForceInit(&state);
    int32_t N = 1000;
    TetherForceEstimate tether_force_b;
    for (int32_t i = 1; i < N; i++) {
      EstimatorTetherForceStep(loadcells, faults, &GetSystemParams()->wing,
                               GetSystemParams()->loadcells, &params, &state,
                               &tether_force_b);
    }

    EXPECT_NEAR_RELATIVE(
        tether_force_b.sph.tension, tether_force_b.tension_f,
        LpfIterationTolerance(params.fc_tension, *g_sys.ts, N));
  }
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

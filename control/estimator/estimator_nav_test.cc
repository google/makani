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

#include "control/estimator/estimator_nav.h"

#include <gtest/gtest.h>

#include <math.h>
#include <stdlib.h>

#include "control/control_params.h"
#include "control/control_types.h"
#include "control/fault_detection/fault_detection_types.h"
#include "control/fault_detection/fault_detection_util.h"
#include "control/ground_frame.h"
#include "control/sensor_types.h"
#include "control/system_params.h"
#include "lib/util/test_util.h"

using ::test_util::LpfIterationTolerance;
using ::test_util::Rand;

namespace {

static const int32_t kOneSecond = static_cast<int32_t>(1.0 / (*g_sys.ts));

}  // namespace

TEST(FilterInertialEstimates, Consistency) {
  const EstimatorNavParams *params = &GetControlParams()->estimator.nav;
  for (int32_t j = 0; j < test_util::kNumTests; j++) {
    const Vec3 pqr = {Rand(-1e5, 1e5), Rand(-1e5, 1e5), Rand(-1e5, 1e5)};
    const Vec3 acc_b = {Rand(-1e5, 1e5), Rand(-1e5, 1e5), Rand(-1e5, 1e5)};
    const double g_heading = GetSystemParams()->ground_frame.heading;
    EstimatorNavState state = EstimatorNavState();
    EstimatorNavInit(params, g_heading, &state);
    Vec3 pqr_f, acc_b_f;
    double acc_norm_f;
    int32_t N = 1000;
    for (int32_t i = 1; i < N; i++) {
      FilterInertialMeasurements(&pqr, &acc_b, params, &state, &pqr_f, &acc_b_f,
                                 &acc_norm_f);
    }
    EXPECT_NEAR_RELATIVE(
        Vec3Norm(&acc_b), acc_norm_f,
        LpfIterationTolerance(params->fc_acc_norm, *g_sys.ts, N));
    // TODO: Add a test for pqr_f, acc_b_f, and Vg_f.
  }
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

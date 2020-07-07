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

#include "control/estimator/estimator_apparent_wind.h"

#include <gtest/gtest.h>

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "common/c_math/util.h"
#include "control/control_params.h"
#include "control/control_types.h"
#include "control/fault_detection/fault_detection_types.h"
#include "control/fault_detection/fault_detection_util.h"
#include "control/sensor_util.h"
#include "control/system_params.h"
#include "lib/util/test_util.h"

extern "C" {

void FilterApparentWindSph(const ApparentWindSph *sph,
                           const EstimatorApparentWindParams *params,
                           ApparentWindSph *sph_f_z1, ApparentWindSph *sph_f);

}  // extern "C"

using ::test_util::Rand;
using ::test_util::LpfIterationTolerance;

namespace {

const ApparentWindSph kApparentWindSphZero = {0.0, 0.0, 0.0};

}  // namespace

TEST(EstimateApparentWind, FilterConsistency) {
  const EstimatorApparentWindParams *params =
      &GetControlParams()->estimator.apparent_wind;
  for (int32_t j = 0; j < test_util::kNumTests; j++) {
    const ApparentWindSph sph = {Rand(-1e5, 1e5), Rand(-1e5, 1e5),
                                 Rand(-1e5, 1e5)};
    ApparentWindSph sph_f_z1 = kApparentWindSphZero;
    ApparentWindSph sph_f;
    int32_t N = 1000;
    for (int32_t i = 1; i < N; i++) {
      FilterApparentWindSph(&sph, params, &sph_f_z1, &sph_f);
    }
    EXPECT_NEAR_RELATIVE(sph.v, sph_f.v,
                         LpfIterationTolerance(params->fc_v, *g_sys.ts, N));
    EXPECT_NEAR_RELATIVE(sph.alpha, sph_f.alpha,
                         LpfIterationTolerance(params->fc_alpha, *g_sys.ts, N));
    EXPECT_NEAR_RELATIVE(sph.beta, sph_f.beta,
                         LpfIterationTolerance(params->fc_beta, *g_sys.ts, N));
  }
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

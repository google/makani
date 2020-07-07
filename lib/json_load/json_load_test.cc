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
#include <jansson.h>
#include <stdint.h>

#include <string>

#include "common/macros.h"
#include "lib/json_load/json_load.h"
#include "lib/util/test_util.h"
#include "sim/sim_params.h"
#include "sim/sim_types.h"

class JSONLoadTest : public ::testing::Test {
 protected:
  int32_t status_;
  SimParams json_params_;

  JSONLoadTest() : status_(0), json_params_() {
    std::string path = test_util::TestRunfilesDir()
        + "/lib/json_load/test_data/sim_params.json";

    json_error_t err;
    json_t *obj = json_load_file(path.c_str(), 0, &err);
    status_ = JSONLoadStruct_SimParams(obj, &json_params_);
    json_decref(obj);
  }
};

TEST_F(JSONLoadTest, TestAeroSimParamsEndToEnd) {
  EXPECT_EQ(0, status_);
  const AeroSimParams &json_aero_params = json_params_.aero_sim;
  const AeroSimParams &aero_params = GetSimParams()->aero_sim;

  EXPECT_EQ(aero_params.low_alpha_stall_angle,
            json_aero_params.low_alpha_stall_angle);
  EXPECT_EQ(aero_params.high_alpha_stall_angle,
            json_aero_params.high_alpha_stall_angle);
  EXPECT_EQ(aero_params.low_beta_stall_angle,
            json_aero_params.low_beta_stall_angle);
  EXPECT_EQ(aero_params.high_beta_stall_angle,
            json_aero_params.high_beta_stall_angle);
  EXPECT_EQ(aero_params.linear_to_stalled_blending_angle,
            json_aero_params.linear_to_stalled_blending_angle);

  for (int32_t i = 0; i < ARRAYSIZE(aero_params.small_deflection_databases);
       ++i) {
    EXPECT_STREQ(aero_params.small_deflection_databases[i].name,
                 json_aero_params.small_deflection_databases[i].name);
  }
  for (int32_t i = 0; i < ARRAYSIZE(aero_params.large_deflection_databases);
       ++i) {
    EXPECT_STREQ(aero_params.large_deflection_databases[i].name,
                 json_aero_params.large_deflection_databases[i].name);
  }

  for (int32_t i = 0; i < kNumFlaps; ++i) {
    const HingeMomentCoeffs &a = aero_params.hinge_moment_coeffs[i];
    const HingeMomentCoeffs &b = json_aero_params.hinge_moment_coeffs[i];
    EXPECT_EQ(a.c, b.c);
    EXPECT_EQ(a.c_deltad, b.c_deltad);
    EXPECT_EQ(a.c_alphad, b.c_alphad);
    EXPECT_EQ(a.c_alphad_deltad, b.c_alphad_deltad);
    EXPECT_EQ(aero_params.min_avl_flap_angles[i],
              json_aero_params.min_avl_flap_angles[i]);
    EXPECT_EQ(aero_params.max_avl_flap_angles[i],
              json_aero_params.max_avl_flap_angles[i]);
  }

  EXPECT_EQ_VEC3(aero_params.moment_coeff_b_scale_factors.coeff,
                 json_aero_params.moment_coeff_b_scale_factors.coeff);
}

TEST_F(JSONLoadTest, TestPerchSimParamsAEndToEnd) {
  EXPECT_EQ(0, status_);
  const Mat2 &A = GetSimParams()->perch_sim.A;
  const Mat2 &json_A = json_params_.perch_sim.A;
  for (int32_t i = 0; i < 2; ++i) {
    for (int32_t j = 0; j < 2; ++j) {
      EXPECT_EQ(A.d[i][j], json_A.d[i][j]);
    }
  }
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

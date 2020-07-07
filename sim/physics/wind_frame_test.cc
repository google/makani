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

#include "lib/util/test_util.h"
#include "sim/physics/wind_frame.h"

static constexpr double kTolerance = 1e-15;

TEST(MwToG, Basic) {
  double ground_z = 22.0;
  Vec3 pos_g = {0.0, 0.0, ground_z - 1.0};
  Mat3 dcm_mw2g;
  CalcDcmMwToG(0.0, &dcm_mw2g);
  Vec3 pos_mw;
  TransformGToMw(&pos_g, &dcm_mw2g, ground_z, &pos_mw);
  EXPECT_NEAR(0.0, pos_mw.x, kTolerance);
  EXPECT_NEAR(0.0, pos_mw.y, kTolerance);
  EXPECT_NEAR(1.0, pos_mw.z, kTolerance);
}

TEST(CalcDcmMwToG, AlternateFormulation) {
  for (double wind_dir : {-0.3, -0.2, -0.1, 0.0, 0.1, 0.2, 0.3}) {
    Mat3 m1;
    CalcDcmMwToG(wind_dir, &m1);

    Mat3 m2 = kMat3Zero;
    m2.d[0][0] = -cos(wind_dir);
    m2.d[0][1] = -sin(wind_dir);
    m2.d[1][0] = -sin(wind_dir);
    m2.d[1][1] = cos(wind_dir);
    m2.d[2][2] = -1.0;

    EXPECT_NEAR_MAT3(m1, m2, 1e-15);
  }
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

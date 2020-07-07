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

#include <math.h>
#include <stdint.h>
#include <stdlib.h>

#include "common/c_math/util.h"
#include "common/c_math/vec3.h"
#include "lib/util/test_util.h"
#include "sim/physics/aero_frame.h"

using ::test_util::Rand;

TEST(BToW, StevensAndLewis) {
  for (double alpha = -M_PI; alpha < M_PI; alpha += 0.1) {
    for (double beta = -M_PI / 2.0; beta < M_PI / 2.0; beta += 0.1) {
      // See Stevens and Lewis, Aircraft Control and Simulation, pg. 73
      // equation (2.3-2).
      Mat3 dcm_b2w = {{
          {cos(alpha) * cos(beta), sin(beta), sin(alpha) * cos(beta)},
          {-cos(alpha) * sin(beta), cos(beta), -sin(alpha) * sin(beta)},
          {-sin(alpha), 0.0, cos(alpha)},
      }};
      Mat3 dcm_w2b;
      CalcDcmWToB(alpha, beta, &dcm_w2b);
      for (int32_t i = 0; i < 3; ++i) {
        for (int32_t j = 0; j < 3; ++j) {
          EXPECT_EQ(dcm_b2w.d[j][i], dcm_w2b.d[i][j]);
        }
      }
    }
  }
}

TEST(BToW, Recovery) {
  for (int32_t i = 0; i < 222; ++i) {
    Vec3 Xb = {Rand(-10.0, 10.0), Rand(-10.0, 10.0), Rand(-10.0, 10.0)};
    Vec3 Xb_out;
    double alpha = Rand(-M_PI, M_PI);
    double beta = Rand(-M_PI / 2.0, M_PI / 2.0);
    RotWToB(RotBToW(&Xb, alpha, beta, &Xb_out), alpha, beta, &Xb_out);
    EXPECT_NEAR_VEC3(Xb, Xb_out, 1e-9);
  }
}

TEST(RotSToB, CompareToCalcDcmSToB) {
  for (double alpha = -M_PI; alpha < M_PI; alpha += 0.1) {
    Vec3 Xb = {Rand(-10.0, 10.0), Rand(-10.0, 10.0), Rand(-10.0, 10.0)};

    Mat3 dcm_s2b;
    CalcDcmSToB(alpha, &dcm_s2b);
    Vec3 Xb_out;
    Mat3Vec3Mult(&dcm_s2b, RotBToS(&Xb, alpha, &Xb_out), &Xb_out);

    EXPECT_NEAR_VEC3(Xb, Xb_out, 1e-9);
  }
}

TEST(BToS, Recovery) {
  for (int32_t i = 0; i < 222; ++i) {
    Vec3 Xb = {Rand(-10.0, 10.0), Rand(-10.0, 10.0), Rand(-10.0, 10.0)};
    Vec3 Xb_out;
    double alpha = Rand(-M_PI, M_PI);
    RotSToB(RotBToS(&Xb, alpha, &Xb_out), alpha, &Xb_out);
    EXPECT_NEAR_VEC3(Xb, Xb_out, 1e-9);
  }
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

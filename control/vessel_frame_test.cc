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

#include "common/c_math/vec3.h"
#include "control/vessel_frame.h"

TEST(VecGToV, TrivialSanityCheck) {
  for (double azi_g = -PI; azi_g < PI; azi_g += 0.1) {
    double azi_v = AziGToV(azi_g, &kMat3Identity);
    EXPECT_NEAR(azi_v, azi_g, 1e-9);
  }
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

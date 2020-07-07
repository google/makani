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
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "common/c_math/vec3.h"
#include "control/system_params.h"

extern "C" {

void CalcXcat(double ele, double weight_over_tension_ratio, double length,
              Vec3 *X_cat);

}  // extern "C"

TEST(CalcXcat, FullFormula) {
  double tether_weight =
      g_sys.tether->length * g_sys.tether->linear_density * g_sys.phys->g;
  double length = g_sys.tether->length + g_sys.wing->bridle_rad;

  for (double tension = 1.001 * tether_weight;
       tension <= 10000.0 * tether_weight; tension *= 1.1) {
    for (double gamma = 1e-9 + asin(tether_weight / tension); gamma <= PI / 2.0;
         gamma += 0.05) {
      double vertical_tension = tension * sin(gamma);
      double horizontal_tension = tension * cos(gamma);
      double vertical_tension_at_gs = tether_weight - vertical_tension;
      double ele = atan2(-vertical_tension_at_gs, horizontal_tension);
      Vec3 X_cat;
      CalcXcat(ele, tether_weight / tension, length, &X_cat);

      EXPECT_NEAR((length / tether_weight) * horizontal_tension *
                      (asinh(tan(gamma)) - asinh(tan(ele))),
                  X_cat.x, 1e-9);

      EXPECT_EQ(0.0, X_cat.y);

      EXPECT_NEAR(-(length / tether_weight) *
                      (hypot(vertical_tension, horizontal_tension) -
                       hypot(vertical_tension_at_gs, horizontal_tension)),
                  X_cat.z, 1e-9);
    }
  }
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

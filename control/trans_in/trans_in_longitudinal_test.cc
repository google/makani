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

#include "control/control_params.h"
#include "control/trans_in/trans_in_types.h"

extern "C" {

double CalcAeroClimbAngleCmd(double tangent_climb_angle,
                             double radial_vel_ti_err, double airspeed,
                             double aero_climb_angle,
                             const TransInLongitudinalParams *params);
}  // extern "C"

TEST(CalcAeroClimbAngleCmd, Identities) {
  const TransInLongitudinalParams &params =
      GetControlParams()->trans_in.longitudinal;
  double airspeed = params.min_airspeed * 1.3;

  for (double tangent_climb_angle = PI / 2.0; tangent_climb_angle > -PI / 4.0;
       tangent_climb_angle -= 0.1) {
    for (double aero_climb_angle_cmd = PI / 2.0;
         aero_climb_angle_cmd > -PI / 2.0; aero_climb_angle_cmd -= 0.1) {
      for (double aero_climb_angle = PI / 2.0; aero_climb_angle > -PI / 2.0;
           aero_climb_angle -= 0.1) {
        double radial_vel_ti_err =
            airspeed * (sin(aero_climb_angle_cmd - tangent_climb_angle) -
                        sin(aero_climb_angle - tangent_climb_angle));
        double aero_climb_angle_cmd_output =
            CalcAeroClimbAngleCmd(tangent_climb_angle, radial_vel_ti_err,
                                  airspeed, aero_climb_angle, &params);
        EXPECT_GE(params.max_aero_climb_angle_cmd, aero_climb_angle_cmd_output);
        EXPECT_LE(params.min_aero_climb_angle_cmd, aero_climb_angle_cmd_output);
        if (aero_climb_angle_cmd_output < params.max_aero_climb_angle_cmd &&
            aero_climb_angle_cmd_output > params.min_aero_climb_angle_cmd) {
          EXPECT_NEAR(radial_vel_ti_err,
                      airspeed * (sin(aero_climb_angle_cmd_output -
                                      tangent_climb_angle) -
                                  sin(aero_climb_angle - tangent_climb_angle)),
                      1e-9);
        }
      }
    }
  }
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

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
#include <string.h>

#include "common/macros.h"
#include "control/control_params.h"
#include "control/control_types.h"
#include "control/estimator/estimator_types.h"
#include "control/hover/hover_altitude.h"
#include "control/hover/hover_types.h"
#include "control/system_params.h"

extern "C" {

double ConvertThrottleToThrust(double joystick_throttle, double thrust_ff,
                               const HoverAltitudeParams *params);

double CalcFeedForwardThrust(double payout);

}  // extern "C"

TEST(HoverAltitudeInit, SameInitialization) {
  HoverAltitudeState state0, state1;
  memset(&state0, 0, sizeof(state0));
  memset(&state1, 0xFF, sizeof(state1));
  HoverAltitudeInit(0.0, &GetControlParams()->hover.altitude, &state0);
  HoverAltitudeInit(0.0, &GetControlParams()->hover.altitude, &state1);
  EXPECT_TRUE(!memcmp(&state0, &state1, sizeof(state0)));
}

TEST(HoverAltitudeValidateParams, Default) {
  EXPECT_TRUE(HoverAltitudeValidateParams(&GetControlParams()->hover.altitude));
}

TEST(ConvertThrottleToThrust, Normal) {
  const HoverAltitudeParams &params = GetControlParams()->hover.altitude;
  GetControlParamsUnsafe()->flight_plan = kFlightPlanTurnKey;

  // Half throttle is defined as unity thrust-to-weight.
  EXPECT_NEAR(ConvertThrottleToThrust(0.5, 1.0, &params), 1.0, 1e-9);

  // Full throttle is defined as the maximum allowed pilot
  // thrust-to-weight.
  EXPECT_NEAR(ConvertThrottleToThrust(1.0, 1.0, &params),
              params.max_pilot_thrust_to_weight_ratio, 1e-9);

  // Zero throttle should be as much below one as full throttle is
  // above it.
  EXPECT_NEAR(ConvertThrottleToThrust(0.0, 1.0, &params),
              2.0 - params.max_pilot_thrust_to_weight_ratio, 1e-9);
}

#if defined(NDEBUG)

// The values should be saturated outside the normal range of throttle.
TEST(ConvertThrottleToThrust, OutsideRange) {
  const HoverAltitudeParams &params = GetControlParams()->hover.altitude;
  EXPECT_NEAR(ConvertThrottleToThrust(1.1, 1.0, &params),
              ConvertThrottleToThrust(1.0, 1.0, &params), 1e-9);
  EXPECT_NEAR(ConvertThrottleToThrust(-0.1, 1.0, &params),
              ConvertThrottleToThrust(0.0, 1.0, &params), 1e-9);
}

#endif  // defined(NDEBUG)

TEST(CalcFeedForwardThrust, ZeroPayout) {
  EXPECT_NEAR(CalcFeedForwardThrust(0.0), g_sys.wing->m * g_sys.phys->g, 1e-9);
}

TEST(CalcFeedForwardThrust, FullPayout) {
  double tether_mass = g_sys.tether->length * g_sys.tether->linear_density;
  EXPECT_NEAR(CalcFeedForwardThrust(g_sys.tether->length),
              (g_sys.wing->m + tether_mass) * g_sys.phys->g, 1e-9);
}

#if defined(NDEBUG)

// The values should be saturated outside the normal range of payout.
TEST(CalcFeedForwardThrust, OutsideRange) {
  EXPECT_NEAR(CalcFeedForwardThrust(-100.0), CalcFeedForwardThrust(0.0), 1e-9);
  EXPECT_NEAR(CalcFeedForwardThrust(2.0 * g_sys.tether->length),
              CalcFeedForwardThrust(g_sys.tether->length), 1e-9);
}

#endif  // defined(NDEBUG)

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

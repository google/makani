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

#include "control/crosswind/crosswind_power.h"
#include "control/crosswind/crosswind_types.h"

extern "C" {
double GetAirspeedDerivative(double loop_angle, double dloop_angle,
                             CrosswindPathType path_type,
                             const FlightStatus *flight_status,
                             const PlaybookEntry *playbook_entry,
                             const CrosswindPowerParams *params,
                             CrosswindPowerState *state);
}

TEST(GetAirspeedDerivative, AnalyticAgreement) {
  CrosswindPowerParams params;
  PlaybookEntry playbook_entry;
  CrosswindPowerState state = CrosswindPowerState();
  FlightStatus flight_status;
  const double airspeed_variation_loop_angle_offset = 2.0;
  const double airspeed_variation = 15.0;
  const double mean_airspeed_cmd = 42.0;

  flight_status.flight_mode = kFlightModeCrosswindNormal;
  for (int i = 0; i < CROSSWIND_SCHEDULE_TABLE_LENGTH; i++) {
    double loop_angle = (2.0 * PI / (CROSSWIND_SCHEDULE_TABLE_LENGTH - 1)) * i;
    playbook_entry.lookup_loop_angle[i] = loop_angle;
    playbook_entry.airspeed_lookup[i] =
        mean_airspeed_cmd +
        airspeed_variation *
            cos(loop_angle - airspeed_variation_loop_angle_offset);
  }

  params.min_airspeed = 10.0;
  params.max_airspeed = 100.0;

  double dloop_angle =
      playbook_entry.lookup_loop_angle[1] - playbook_entry.lookup_loop_angle[0];
  for (double loop_angle = 0.0; loop_angle < 2.0 * PI; loop_angle += 0.1) {
    EXPECT_NEAR(
        GetAirspeedDerivative(loop_angle, dloop_angle, kCrosswindPathNormal,
                              &flight_status, &playbook_entry, &params, &state),
        -airspeed_variation *
            sin(loop_angle - airspeed_variation_loop_angle_offset),
        airspeed_variation * 1e-2);
  }
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

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

#include "common/c_math/util.h"
#include "control/crosswind/crosswind_playbook.h"
#include "control/crosswind/crosswind_playbook_types.h"
#include "control/system_params.h"
#include "control/system_types.h"
#include "lib/util/test_util.h"

class AziAllowStartOverrider : ::test_util::Overrider<double> {
 public:
  explicit AziAllowStartOverrider(double value)
      : Overrider(&GetSystemParamsUnsafe()->test_site_params.azi_allow_start,
                  value) {}
};
class AziAllowEndOverrider : ::test_util::Overrider<double> {
 public:
  explicit AziAllowEndOverrider(double value)
      : Overrider(&GetSystemParamsUnsafe()->test_site_params.azi_allow_end,
                  value) {}
};

TEST(GetPlaybookEntryAzimuthWithLimits, EdgeCases) {
  double half_angle = DegToRad(25.0);

  PlaybookEntry entry = PlaybookEntry();
  entry.path_radius_target = g_sys.tether->length * sin(half_angle);
  entry.elevation = 0.0;
  entry.azi_offset = 0.0;

  {
    AziAllowStartOverrider azi_start_override(-PI);
    AziAllowEndOverrider azi_end_override(PI);

    // Normal case.
    EXPECT_NEAR(GetPlaybookEntryAzimuthWithLimits(0.0, &entry), 0.0, DBL_TOL);
    // No azi limits, loop spans wrap.
    EXPECT_NEAR(GetPlaybookEntryAzimuthWithLimits(PI, &entry),
                Wrap(PI, -PI, PI), DBL_TOL);
    // No azi limits, loop spans wrap on other side.
    EXPECT_NEAR(GetPlaybookEntryAzimuthWithLimits(-PI, &entry), -PI, DBL_TOL);
    // No azi limits, edge on wrap.
    EXPECT_NEAR(GetPlaybookEntryAzimuthWithLimits(PI - half_angle, &entry),
                PI - half_angle, DBL_TOL);
    // No azi limits, edge over wrap.
    EXPECT_NEAR(
        GetPlaybookEntryAzimuthWithLimits(PI - half_angle + 0.1, &entry),
        PI - half_angle + 0.1, DBL_TOL);
  }

  {
    AziAllowStartOverrider azi_start_override(DegToRad(-90.0));
    AziAllowEndOverrider azi_end_override(DegToRad(90.0));

    // No go zone covers wrap, kick to nearest side.
    EXPECT_NEAR(GetPlaybookEntryAzimuthWithLimits(DegToRad(175.0), &entry),
                DegToRad(90.0) - half_angle, DBL_TOL);
    // No go zone covers wrap, kick to nearest side.
    EXPECT_NEAR(GetPlaybookEntryAzimuthWithLimits(DegToRad(-175.0), &entry),
                DegToRad(-90.0) + half_angle, DBL_TOL);
  }

  {
    // Min no-go size is 0.35 rad, just over 20 deg, hence the 10.5 deg.
    double azi_allow = DegToRad(10.5);
    AziAllowStartOverrider azi_start_override(azi_allow);
    AziAllowEndOverrider azi_end_override(-azi_allow);

    // Half angle spans no-go zone, center is right of min size no-go.
    EXPECT_NEAR(GetPlaybookEntryAzimuthWithLimits(azi_allow, &entry),
                azi_allow + half_angle, DBL_TOL);
    // Half angle spans no-go zone, center is left of min size no-go.
    EXPECT_NEAR(GetPlaybookEntryAzimuthWithLimits(-azi_allow, &entry),
                -azi_allow - half_angle, DBL_TOL);
  }

  {
    AziAllowStartOverrider azi_start_override(DegToRad(170.0));
    AziAllowEndOverrider azi_end_override(DegToRad(90.0));

    // Create a situation where azi_left_clip will be < azi_left due to being
    // kicked over the wrap.
    EXPECT_NEAR(GetPlaybookEntryAzimuthWithLimits(DegToRad(171.0), &entry),
                Wrap(DegToRad(170.0) + half_angle, -PI, PI), DBL_TOL);
  }
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

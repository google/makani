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
#include "control/ground_frame.h"
#include "control/perch_frame.h"
#include "control/system_params.h"
#include "control/system_types.h"
#include "lib/util/test_util.h"

using ::test_util::Rand;

TEST(NedToG, Recovery) {
  for (int32_t i = 0; i < 222; ++i) {
    Vec3 Xg = {Rand(-10.0, 10.0), Rand(-10.0, 10.0), Rand(-10.0, 10.0)};
    Vec3 Xg_out;
    double g_heading = Rand(-6.3, 6.3);
    NedToG(GToNed(&Xg, g_heading, &Xg_out), g_heading, &Xg_out);
    EXPECT_NEAR_VEC3(Xg, Xg_out, 1e-9);
  }
}

TEST(NedToG, ZeroOrigin) {
  for (int32_t i = 0; i < 222; ++i) {
    Vec3 X = {Rand(-10.0, 10.0), Rand(-10.0, 10.0), Rand(-10.0, 10.0)};
    Vec3 X_out, X_out2;
    double g_heading = Rand(-6.3, 6.3);
    NedToG(&X, g_heading, &X_out);
    RotNedToG(&X, g_heading, &X_out2);
    EXPECT_NEAR_VEC3(X_out, X_out2, 1e-9);

    GToNed(&X, g_heading, &X_out);
    RotGToNed(&X, g_heading, &X_out2);
    EXPECT_NEAR_VEC3(X_out, X_out2, 1e-9);
  }
}

// Conversions between ECEF and Ground frame.

TEST(EcefToG, Recovery) {
  for (int32_t i = 0; i < 222; ++i) {
    Vec3 X_ecef = {Rand(-10000.0, 10000.0), Rand(-10000.0, 10000.0),
                   Rand(-10000.0, 10000.0)};
    Vec3 X_ecef_0 = {Rand(-10000.0, 10000.0), Rand(-10000.0, 10000.0),
                     Rand(-10000.0, 10000.0)};
    Vec3 X_ecef_out;
    double g_heading = Rand(-6.3, 6.3);
    GToEcef(EcefToG(&X_ecef, &X_ecef_0, g_heading, &X_ecef_out), &X_ecef_0,
            g_heading, &X_ecef_out);
    EXPECT_NEAR_VEC3(X_ecef, X_ecef_out, 1e-9);
  }
}

// For the Cartesian/cylindrical/spherical conversions, we're primarily
// interested in how thes differ from the standard conversion functions. Namely:
//  - Azimuth angles should update corresponding to the azi_offset
//    parameter, which differs between the top hat and GS02.
//  - Positive z_g should correspond to a negative elevation angle.

class AziOffsetOverrider : ::test_util::Overrider<double> {
 public:
  explicit AziOffsetOverrider(double value)
      : Overrider(&GetSystemParamsUnsafe()->ground_station.azi_ref_offset,
                  value) {}
};

TEST(VecGToAzimuth, AziOffsets) {
  const Vec3 pos_g = {0.0, -4.0, 3.0};
  {
    AziOffsetOverrider overrider(0.0);
    EXPECT_NEAR(VecGToAzimuth(&pos_g), -M_PI / 2.0, 1e-9);
  }
  {
    AziOffsetOverrider overrider(M_PI);
    EXPECT_NEAR(VecGToAzimuth(&pos_g), M_PI / 2.0, 1e-9);
  }
}

TEST(CylToVecG, AziOffsets) {
  const Vec3 expected = {0.0, -4.0, 3.0};
  {
    AziOffsetOverrider overrider(0.0);
    Vec3 actual;
    CylToVecG(-M_PI / 2.0, 4.0, 3.0, &actual);
    EXPECT_NEAR_VEC3(expected, actual, 1e-9);

    double azi, r, z;
    VecGToCyl(&actual, &azi, &r, &z);
    EXPECT_NEAR(azi, -M_PI / 2.0, 1e-9);
    EXPECT_NEAR(r, 4.0, 1e-9);
    EXPECT_NEAR(z, 3.0, 1e-9);
  }
  {
    AziOffsetOverrider overrider(M_PI);
    Vec3 actual;
    CylToVecG(M_PI / 2.0, 4.0, 3.0, &actual);
    EXPECT_NEAR_VEC3(expected, actual, 1e-9);

    double azi, r, z;
    VecGToCyl(&actual, &azi, &r, &z);
    EXPECT_NEAR(azi, M_PI / 2.0, 1e-9);
    EXPECT_NEAR(r, 4.0, 1e-9);
    EXPECT_NEAR(z, 3.0, 1e-9);
  }
}

TEST(SphToVecG, AziOffsets) {
  const Vec3 expected = {0.0, -4.0, 3.0};
  const double ele = asin(-0.6);
  {
    AziOffsetOverrider overrider(0.0);
    Vec3 actual;
    SphToVecG(-M_PI / 2.0, ele, 5.0, &actual);
    EXPECT_NEAR_VEC3(expected, actual, 1e-9);

    // Test VecGToElevation.
    double ele_out = VecGToElevation(&actual);
    EXPECT_NEAR(ele, ele_out, 1e-9);
  }
  {
    AziOffsetOverrider overrider(M_PI);
    Vec3 actual;
    SphToVecG(M_PI / 2.0, ele, 5.0, &actual);
    EXPECT_NEAR_VEC3(expected, actual, 1e-9);

    // Test VecGToElevation.
    double ele_out = VecGToElevation(&actual);
    EXPECT_NEAR(ele, ele_out, 1e-9);
  }
}

TEST(VecGToSph, AziOffsets) {
  const Vec3 pos_g = {0.0, -4.0, 3.0};
  const double expected_ele = asin(-0.6);
  const double expected_r = 5.0;
  {
    AziOffsetOverrider overrider(0.0);
    double actual_azi, actual_ele, actual_r;
    const double expected_azi = -M_PI / 2.0;
    VecGToSph(&pos_g, &actual_azi, &actual_ele, &actual_r);
    EXPECT_NEAR(expected_azi, actual_azi, 1e-9);
    EXPECT_NEAR(expected_ele, actual_ele, 1e-9);
    EXPECT_NEAR(expected_r, actual_r, 1e-9);

    // Test VecGToElevation.
    actual_ele = VecGToElevation(&pos_g);
    EXPECT_NEAR(expected_ele, actual_ele, 1e-9);
  }
  {
    AziOffsetOverrider overrider(M_PI);
    double actual_azi, actual_ele, actual_r;
    const double expected_azi = M_PI / 2.0;
    VecGToSph(&pos_g, &actual_azi, &actual_ele, &actual_r);
    EXPECT_NEAR(expected_azi, actual_azi, 1e-9);
    EXPECT_NEAR(expected_ele, actual_ele, 1e-9);
    EXPECT_NEAR(expected_r, actual_r, 1e-9);

    // Test VecGToElevation.
    actual_ele = VecGToElevation(&pos_g);
    EXPECT_NEAR(expected_ele, actual_ele, 1e-9);
  }
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

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

#include "common/c_math/coord_trans.h"
#include "common/c_math/geometry.h"
#include "common/c_math/mat3.h"
#include "common/c_math/util.h"
#include "common/c_math/vec3.h"
#include "lib/util/test_util.h"

using ::test_util::Rand;
using ::test_util::RandNormal;

TEST(RotateCov, Normal0) {
  Mat3 dcm_a2b;
  AngleToDcm(Rand(-M_PI, M_PI), Rand(-M_PI, M_PI), Rand(-M_PI, M_PI),
             kRotationOrderZyx, &dcm_a2b);

  Vec3 Xa[22], Xb[22];
  Vec3 Xa_mean = {0.0, 0.0, 0.0}, Xb_mean = {0.0, 0.0, 0.0};
  for (int32_t i = 0; i < 22; ++i) {
    Xa[i].x = Rand();
    Xa[i].y = Rand();
    Xa[i].z = Rand();
    Mat3Vec3Mult(&dcm_a2b, &Xa[i], &Xb[i]);
    Vec3Add(&Xa_mean, &Xa[i], &Xa_mean);
    Vec3Add(&Xb_mean, &Xb[i], &Xb_mean);
  }
  Vec3Scale(&Xa_mean, 1.0 / 22.0, &Xa_mean);
  Vec3Scale(&Xb_mean, 1.0 / 22.0, &Xb_mean);

  Mat3 cov_a = kMat3Zero, cov_b = kMat3Zero, cov_b_test;
  for (int32_t i = 0; i < 22; ++i) {
    cov_a.d[0][0] += (Xa[i].x - Xa_mean.x) * (Xa[i].x - Xa_mean.x) / 22.0;
    cov_a.d[0][1] += (Xa[i].x - Xa_mean.x) * (Xa[i].y - Xa_mean.y) / 22.0;
    cov_a.d[0][2] += (Xa[i].x - Xa_mean.x) * (Xa[i].z - Xa_mean.z) / 22.0;
    cov_a.d[1][0] += (Xa[i].y - Xa_mean.y) * (Xa[i].x - Xa_mean.x) / 22.0;
    cov_a.d[1][1] += (Xa[i].y - Xa_mean.y) * (Xa[i].y - Xa_mean.y) / 22.0;
    cov_a.d[1][2] += (Xa[i].y - Xa_mean.y) * (Xa[i].z - Xa_mean.z) / 22.0;
    cov_a.d[2][0] += (Xa[i].z - Xa_mean.z) * (Xa[i].x - Xa_mean.x) / 22.0;
    cov_a.d[2][1] += (Xa[i].z - Xa_mean.z) * (Xa[i].y - Xa_mean.y) / 22.0;
    cov_a.d[2][2] += (Xa[i].z - Xa_mean.z) * (Xa[i].z - Xa_mean.z) / 22.0;

    cov_b.d[0][0] += (Xb[i].x - Xb_mean.x) * (Xb[i].x - Xb_mean.x) / 22.0;
    cov_b.d[0][1] += (Xb[i].x - Xb_mean.x) * (Xb[i].y - Xb_mean.y) / 22.0;
    cov_b.d[0][2] += (Xb[i].x - Xb_mean.x) * (Xb[i].z - Xb_mean.z) / 22.0;
    cov_b.d[1][0] += (Xb[i].y - Xb_mean.y) * (Xb[i].x - Xb_mean.x) / 22.0;
    cov_b.d[1][1] += (Xb[i].y - Xb_mean.y) * (Xb[i].y - Xb_mean.y) / 22.0;
    cov_b.d[1][2] += (Xb[i].y - Xb_mean.y) * (Xb[i].z - Xb_mean.z) / 22.0;
    cov_b.d[2][0] += (Xb[i].z - Xb_mean.z) * (Xb[i].x - Xb_mean.x) / 22.0;
    cov_b.d[2][1] += (Xb[i].z - Xb_mean.z) * (Xb[i].y - Xb_mean.y) / 22.0;
    cov_b.d[2][2] += (Xb[i].z - Xb_mean.z) * (Xb[i].z - Xb_mean.z) / 22.0;
  }

  RotateCov(&cov_a, &dcm_a2b, &cov_b_test);
  EXPECT_NEAR_MAT3(cov_b, cov_b_test, 1e-9);
}

TEST(HtvToNed, Normal0) {
  const Vec3 htv = {0.0, 0.0, 0.0};
  Vec3 ned;
  HtvToNed(&htv, &ned);
  EXPECT_NEAR_VEC3(ned, kVec3Zero, 1e-9);
}

TEST(HtvToNed, Normal1) {
  const Vec3 htv = {0.0, 0.0, 22.0};
  const Vec3 ned_out = {0.0, 0.0, -22.0};
  Vec3 ned;
  HtvToNed(&htv, &ned);
  EXPECT_NEAR_VEC3(ned, ned_out, 1e-9);
}

TEST(HtvToNed, Consistency0) {
  for (int32_t i = 0; i < test_util::kNumTests; i++) {
    double a = RandNormal();
    const Vec3 htv = {a, 0.0, 22.0};
    const Vec3 ned_out = {a, 0.0, -22.0};
    Vec3 ned;
    HtvToNed(&htv, &ned);
    EXPECT_NEAR_VEC3(ned, ned_out, 1e-9);
  }
}

TEST(HtvToNed, Consistency1) {
  for (int32_t i = 0; i < test_util::kNumTests; i++) {
    double a = RandNormal();
    const Vec3 htv = {0.0, a, 22.0};
    const Vec3 ned_out = {0.0, 0.0, -22.0};
    Vec3 ned;
    HtvToNed(&htv, &ned);
    EXPECT_NEAR_VEC3(ned, ned_out, 1e-9);
  }
}

TEST(NedToHtvToNed, Normal0) {
  for (int32_t i = 0; i < test_util::kNumTests; ++i) {
    const Vec3 V_ned_const = {RandNormal(), RandNormal(), RandNormal()};
    Vec3 V_ned = V_ned_const;
    HtvToNed(NedToHtv(&V_ned, &V_ned), &V_ned);
    EXPECT_NEAR_VEC3(V_ned, V_ned_const, 1e-7);
  }
}

TEST(CalcDcmNedToEcef, Normal0) {
  const Vec3 ecef = {0, 0, -22};
  Mat3 dcm;
  CalcDcmNedToEcef(&ecef, &dcm);
  EXPECT_NEAR_MAT3(dcm, kMat3Identity, 1e-9);
}

TEST(CalcDcmNedToEcef, Normal1) {
  const Vec3 ecef = {0, 0, 22};
  const Mat3 dcm_out = {{{-1, 0, 0}, {0, 1, 0}, {0, 0, -1}}};
  Mat3 dcm;
  CalcDcmNedToEcef(&ecef, &dcm);
  EXPECT_NEAR_MAT3(dcm, dcm_out, 1e-9);
}

TEST(CalcDcmNedToEcef, Normal2) {
  const Vec3 ecef = {22, 0, 0};
  const Mat3 dcm_out = {{{0, 0, -1}, {0, 1, 0}, {1, 0, 0}}};
  Mat3 dcm;
  CalcDcmNedToEcef(&ecef, &dcm);
  EXPECT_NEAR_MAT3(dcm, dcm_out, 1e-9);
}

TEST(CalcDcmNedToEcef, Normal3) {
  const Vec3 ecef = {-22, 0, 0};
  const Mat3 dcm_out = {{{0, 0, 1}, {0, -1, 0}, {1, 0, 0}}};
  Mat3 dcm;
  CalcDcmNedToEcef(&ecef, &dcm);
  EXPECT_NEAR_MAT3(dcm, dcm_out, 1e-9);
}

TEST(CalcDcmNedToEcef, Normal4) {
  const Vec3 ecef = {0, 22, 0};
  const Mat3 dcm_out = {{{0, -1, 0}, {0, 0, -1}, {1, 0, 0}}};
  Mat3 dcm;
  CalcDcmNedToEcef(&ecef, &dcm);
  EXPECT_NEAR_MAT3(dcm, dcm_out, 1e-9);
}

TEST(CalcDcmNedToEcef, Normal5) {
  const Vec3 ecef = {0, -22, 0};
  const Mat3 dcm_out = {{{0, 1, 0}, {0, 0, 1}, {1, 0, 0}}};
  Mat3 dcm;
  CalcDcmNedToEcef(&ecef, &dcm);
  EXPECT_NEAR_MAT3(dcm, dcm_out, 1e-9);
}

TEST(CalcDcmEcefToNed, Normal0) {
  const Vec3 ned = {0, 0, -22};
  Mat3 dcm;
  CalcDcmEcefToNed(&ned, &dcm);
  EXPECT_NEAR_MAT3(dcm, kMat3Identity, 1e-9);
}

TEST(CalcDcmEcefToNed, Normal1) {
  const Vec3 ned = {0, 0, 22};
  const Mat3 dcm_out = {{{-1, 0, 0}, {0, 1, 0}, {0, 0, -1}}};
  Mat3 dcm;
  CalcDcmEcefToNed(&ned, &dcm);
  EXPECT_NEAR_MAT3(dcm, dcm_out, 1e-9);
}

TEST(CalcDcmEcefToNed, Normal2) {
  const Vec3 ned = {22, 0, 0};
  const Mat3 dcm_out = {{{0, 0, 1}, {0, 1, 0}, {-1, 0, 0}}};
  Mat3 dcm;
  CalcDcmEcefToNed(&ned, &dcm);
  EXPECT_NEAR_MAT3(dcm, dcm_out, 1e-9);
}

TEST(CalcDcmEcefToNed, Normal3) {
  const Vec3 ned = {-22, 0, 0};
  const Mat3 dcm_out = {{{0, 0, 1}, {0, -1, 0}, {1, 0, 0}}};
  Mat3 dcm;
  CalcDcmEcefToNed(&ned, &dcm);
  EXPECT_NEAR_MAT3(dcm, dcm_out, 1e-9);
}

TEST(CalcDcmEcefToNed, Normal4) {
  const Vec3 ned = {0, 22, 0};
  const Mat3 dcm_out = {{{0, 0, 1}, {-1, 0, 0}, {0, -1, 0}}};
  Mat3 dcm;
  CalcDcmEcefToNed(&ned, &dcm);
  EXPECT_NEAR_MAT3(dcm, dcm_out, 1e-9);
}

TEST(CalcDcmEcefToNed, Normal5) {
  const Vec3 ned = {0, -22, 0};
  const Mat3 dcm_out = {{{0, 0, 1}, {1, 0, 0}, {0, 1, 0}}};
  Mat3 dcm;
  CalcDcmEcefToNed(&ned, &dcm);
  EXPECT_NEAR_MAT3(dcm, dcm_out, 1e-9);
}

TEST(NedEcef, Consistency2) {
  for (int32_t i = 0; i < test_util::kNumTests; i++) {
    const Vec3 ecef = {kEarthA * RandNormal(), kEarthA * RandNormal(),
                       kEarthA * RandNormal()};
    const Vec3 ecef_ref = {kEarthA * RandNormal(), kEarthA * RandNormal(),
                           kEarthA * RandNormal()};
    Vec3 ned, ecef_out;
    EcefToNed(&ecef, &ecef_ref, &ned);
    NedToEcef(&ned, &ecef_ref, &ecef_out);
    EXPECT_NEAR_VEC3(ecef, ecef_out, 1e-6);
  }
}

TEST(NedEcef, Consistency3) {
  for (int32_t i = 0; i < test_util::kNumTests; i++) {
    const Vec3 ned = {RandNormal(), RandNormal(), RandNormal()};
    const Vec3 ecef_ref = {kEarthA * RandNormal(), kEarthA * RandNormal(),
                           kEarthA * RandNormal()};
    Vec3 ecef, ned_out;
    NedToEcef(&ned, &ecef_ref, &ecef);
    EcefToNed(&ecef, &ecef_ref, &ned_out);
    EXPECT_NEAR_VEC3(ned, ned_out, 1e-6);
  }
}

TEST(LlhToEcef, Normal0) {
  const Vec3 llh = {0.5635242858140013, 0.1916299296818766, 0.0318925687320692};
  const Vec3 ecef_out = {6377794.9356938154, 21331.0816715813,
                         62310.3056995760};
  Vec3 ecef;
  LlhToEcef(&llh, &ecef);
  EXPECT_NEAR_VEC3(ecef, ecef_out, 1e-9);
}

TEST(EcefToLlh, Normal0) {
  const Vec3 llh_out = {0.5635242858140013, 0.1916299296818766,
                        0.0318925687320692};
  const Vec3 ecef = {6377794.9356938154, 21331.0816715813, 62310.3056995760};
  Vec3 llh;
  EcefToLlh(&ecef, &llh);
  EXPECT_NEAR_VEC3(llh, llh_out, 1e-9);
}

TEST(EcefToLlh, Normal1) {
  const Vec3 X_ecef = {-2649130.232947433, -4274820.382902395,
                       3909650.492863685};
  const Vec3 ans = {38.0482569109154, -121.7867279607761, -21.032391723245382};
  Vec3 X_llh;
  EcefToLlh(&X_ecef, &X_llh);
  EXPECT_NEAR_VEC3(X_llh, ans, 1e-9);
}

TEST(LlhAndEcef, Consistency0) {
  for (int32_t i = 0; i < test_util::kNumTests; i++) {
    const Vec3 ecef = {kEarthA * RandNormal(), kEarthA * RandNormal(),
                       kEarthA * RandNormal()};
    Vec3 llh, ecef_out;
    EcefToLlh(&ecef, &llh);
    LlhToEcef(&llh, &ecef_out);
    EXPECT_NEAR_VEC3(ecef, ecef_out, 1e-7);
  }
}

TEST(LlhAndEcef, Consistency1) {
  for (int32_t i = 0; i < test_util::kNumTests; i++) {
    const Vec3 llh = {Rand(-90.0, 90.0), Rand(-180.0, 180.0), 10000.0 * Rand()};
    Vec3 ecef, llh_out;
    LlhToEcef(&llh, &ecef);
    EcefToLlh(&ecef, &llh_out);
    while (llh_out.x < -90.0) llh_out.x += 180.0;
    while (llh_out.x > 90.0) llh_out.x -= 180.0;
    while (llh_out.y < -180.0) llh_out.y += 360.0;
    while (llh_out.y > 180.0) llh_out.y -= 360.0;
    EXPECT_NEAR_VEC3(llh, llh_out, 1e-6);
  }
}

TEST(NedToLlh, Normal0) {
  const Vec3 X_ned = {91.337585613901936, 63.235924622540949,
                      9.754040499940952};
  const Vec3 X_llh_0 = {-22.0, 122.22, 22.2};
  const Vec3 X_llh_ans = {-21.9991751349011, 122.2206123776812,
                          12.4469303032383};
  Vec3 X_llh;
  NedToLlh(&X_ned, &X_llh_0, &X_llh);
  EXPECT_NEAR_VEC3(X_llh, X_llh_ans, 1e-9);
}

TEST(NedToLlh, Reuse0) {
  Vec3 X_ned = {91.337585613901936, 63.235924622540949, 9.754040499940952};
  const Vec3 X_llh_0 = {-22.0, 122.22, 22.2};
  const Vec3 X_llh_ans = {-21.9991751349011, 122.2206123776812,
                          12.4469303032383};
  NedToLlh(&X_ned, &X_llh_0, &X_ned);
  EXPECT_NEAR_VEC3(X_ned, X_llh_ans, 1e-9);
}

TEST(NedToLlh, Reuse1) {
  const Vec3 X_ned = {91.337585613901936, 63.235924622540949,
                      9.754040499940952};
  Vec3 X_llh_0 = {-22.0, 122.22, 22.2};
  const Vec3 X_llh_ans = {-21.9991751349011, 122.2206123776812,
                          12.4469303032383};
  NedToLlh(&X_ned, &X_llh_0, &X_llh_0);
  EXPECT_NEAR_VEC3(X_llh_0, X_llh_ans, 1e-9);
}

TEST(LlhToNed, Normal0) {
  const Vec3 X_llh_0 = {-22.0, 122.22, 22.2};
  const Vec3 X_llh = {-21.9991751349011, 122.2206123776812, 12.4469303032383};
  // This NED is slightly different from the NED above due to
  // approximations in the algorithm.
  const Vec3 X_ned_ans = {91.337585613948974, 63.235924628742836,
                          9.754040500902654};
  Vec3 X_ned;
  LlhToNed(&X_llh, &X_llh_0, &X_ned);
  EXPECT_NEAR_VEC3(X_ned, X_ned_ans, 1e-9);
}

TEST(LlhToNed, Reuse0) {
  Vec3 X_llh_0 = {-22.0, 122.22, 22.2};
  const Vec3 X_llh = {-21.9991751349011, 122.2206123776812, 12.4469303032383};
  // This NED is slightly different from the NED above due to
  // approximations in the algorithm.
  const Vec3 X_ned_ans = {91.337585613948974, 63.235924628742836,
                          9.754040500902654};
  LlhToNed(&X_llh, &X_llh_0, &X_llh_0);
  EXPECT_NEAR_VEC3(X_llh_0, X_ned_ans, 1e-9);
}

TEST(LlhToNed, Reuse1) {
  const Vec3 X_llh_0 = {-22.0, 122.22, 22.2};
  Vec3 X_llh = {-21.9991751349011, 122.2206123776812, 12.4469303032383};
  // This NED is slightly different from the NED above due to
  // approximations in the algorithm.
  const Vec3 X_ned_ans = {91.337585613948974, 63.235924628742836,
                          9.754040500902654};
  LlhToNed(&X_llh, &X_llh_0, &X_llh);
  EXPECT_NEAR_VEC3(X_llh, X_ned_ans, 1e-9);
}

TEST(NedToLlhToNed, Normal0) {
  for (int32_t i = 0; i < test_util::kNumTests; ++i) {
    const Vec3 X_ned_const = {RandNormal(), RandNormal(), RandNormal()};
    const Vec3 X_llh_0 = {Rand(-90.0, 90.0), Rand(-180.0, 180.0),
                          Rand(-250.0, 250.0)};
    Vec3 X_ned = X_ned_const;
    LlhToNed(NedToLlh(&X_ned, &X_llh_0, &X_ned), &X_llh_0, &X_ned);
    EXPECT_NEAR_VEC3(X_ned, X_ned_const, 1e-7);
  }
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

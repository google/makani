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

#include "common/c_math/force_moment.h"
#include "common/c_math/geometry.h"
#include "common/c_math/mat3.h"
#include "common/c_math/vec3.h"
#include "lib/util/test_util.h"

using ::test_util::Rand;
using ::test_util::RandNormal;

TEST(ForceMomentRef, Normal) {
  const ForceMoment fm = {{1.0, 0.0, 0.0}, {1.0, 1.0, 1.0}};
  const Vec3 ref_disp = {0.0, 1.0, 0.0};
  ForceMoment fm_out;
  ForceMomentRef(&fm, &ref_disp, &fm_out);
  EXPECT_NEAR(fm.force.x, fm_out.force.x, 1e-9);
  EXPECT_NEAR(fm.force.y, fm_out.force.y, 1e-9);
  EXPECT_NEAR(fm.force.z, fm_out.force.z, 1e-9);
  EXPECT_NEAR(fm.moment.x, fm_out.moment.x, 1e-9);
  EXPECT_NEAR(fm.moment.y, fm_out.moment.y, 1e-9);
  EXPECT_NEAR(fm.moment.z + 1.0, fm_out.moment.z, 1e-9);
}

TEST(ForceMomentAdd, Normal) {
  const ForceMoment fm1 = {{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}};
  const ForceMoment fm2 = {{1.0, 1.0, 1.0}, {1.0, 1.0, 1.0}};
  ForceMoment fm_out;
  ForceMomentAdd(&fm1, &fm2, &fm_out);
  EXPECT_NEAR(fm_out.force.x, 2.0, 1e-9);
  EXPECT_NEAR(fm_out.force.y, 3.0, 1e-9);
  EXPECT_NEAR(fm_out.force.z, 4.0, 1e-9);
  EXPECT_NEAR(fm_out.moment.x, 5.0, 1e-9);
  EXPECT_NEAR(fm_out.moment.y, 6.0, 1e-9);
  EXPECT_NEAR(fm_out.moment.z, 7.0, 1e-9);
}

TEST(ForceMomentLinComb, Normal) {
  const ForceMoment fm1 = {{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}};
  const ForceMoment fm2 = {{1.0, 1.0, 1.0}, {1.0, 1.0, 1.0}};
  ForceMoment fm_out, fm_out2;
  ForceMomentAdd(&fm1, &fm2, &fm_out);
  ForceMomentLinComb(1.0, &fm1, 1.0, &fm2, &fm_out2);
  EXPECT_NEAR(fm_out.force.x, fm_out2.force.x, 1e-9);
  EXPECT_NEAR(fm_out.force.y, fm_out2.force.y, 1e-9);
  EXPECT_NEAR(fm_out.force.z, fm_out2.force.z, 1e-9);
  EXPECT_NEAR(fm_out.moment.x, fm_out2.moment.x, 1e-9);
  EXPECT_NEAR(fm_out.moment.y, fm_out2.moment.y, 1e-9);
  EXPECT_NEAR(fm_out.moment.z, fm_out2.moment.z, 1e-9);
}

TEST(ForceMomentScale, Normal) {
  const ForceMoment fm1 = {{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}};
  ForceMoment fm_out, fm_out2;
  ForceMomentAdd(&fm1, &fm1, &fm_out);
  ForceMomentScale(&fm1, 2.0, &fm_out2);
  EXPECT_NEAR(fm_out.force.x, fm_out2.force.x, 1e-9);
  EXPECT_NEAR(fm_out.force.y, fm_out2.force.y, 1e-9);
  EXPECT_NEAR(fm_out.force.z, fm_out2.force.z, 1e-9);
  EXPECT_NEAR(fm_out.moment.x, fm_out2.moment.x, 1e-9);
  EXPECT_NEAR(fm_out.moment.y, fm_out2.moment.y, 1e-9);
  EXPECT_NEAR(fm_out.moment.z, fm_out2.moment.z, 1e-9);
}

TEST(ForceMomentPosAdd, Normal0) {
  const ForceMomentPos fm1 = {
      {0.442366659813661, 0.286445873377231, 0.253376384380172},
      {0.080942199884872, 0.753420061021129, 0.905088575796938},
      {0.808889320611676, 0.452563772553406, 0.033907599125428}};

  const ForceMomentPos fm2 = {
      {0.650679998306341, 0.181111393623197, 0.070266343478723},
      {0.067789878542121, 0.239248027986402, 0.616031081932806},
      {0.407850150109576, 0.064820915389913, 0.505671250159965}};

  ForceMomentPos fm_out = {
      {0.540538197630115, 0.194015563119582, 0.709839248803049},
      {0.498657379805283, 0.619498304529191, 0.243800939026136},
      {0.750909916893278, 0.507515846300335, 0.897946048158161}};

  const ForceMomentPos fm_out_correct = {
      {1.093046658120002, 0.467557267000427, 0.323642727858895},
      {0.422247650108562, 0.364615865185127, 1.787957288415788},
      {0.750909916893278, 0.507515846300335, 0.897946048158161}};

  ForceMomentPosAdd(&fm1, &fm2, &fm_out);

  EXPECT_NEAR(fm_out.force.x, fm_out_correct.force.x, 1e-9);
  EXPECT_NEAR(fm_out.force.y, fm_out_correct.force.y, 1e-9);
  EXPECT_NEAR(fm_out.force.z, fm_out_correct.force.z, 1e-9);
  EXPECT_NEAR(fm_out.moment.x, fm_out_correct.moment.x, 1e-9);
  EXPECT_NEAR(fm_out.moment.y, fm_out_correct.moment.y, 1e-9);
  EXPECT_NEAR(fm_out.moment.z, fm_out_correct.moment.z, 1e-9);
  EXPECT_NEAR(fm_out.pos.x, fm_out_correct.pos.x, 1e-9);
  EXPECT_NEAR(fm_out.pos.y, fm_out_correct.pos.y, 1e-9);
  EXPECT_NEAR(fm_out.pos.z, fm_out_correct.pos.z, 1e-9);
}

TEST(ForceMomentPosAdd3, CompareAdd) {
  ForceMomentPos fmx1 = {{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}, {7.0, 8.0, 9.0}};
  ForceMomentPos fmx2 = {{6.0, 3.0, 3.0}, {8.0, 5.0, 6.0}, {0.0, 0.0, 0.0}};
  ForceMomentPos fmx3 = {{8.0, 4.0, 3.0}, {4.0, 5.0, 7.0}, {7.0, 4.0, 0.0}};
  ForceMomentPos fm_out = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
  ForceMomentPos fm_out2 = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
  ForceMomentPos fm_out3 = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
  ForceMomentPosAdd(ForceMomentPosAdd(&fmx1, &fmx2, &fm_out), &fmx3, &fm_out2);
  ForceMomentPosAdd3(&fmx1, &fmx2, &fmx3, &fm_out3);
  EXPECT_NEAR(fm_out2.force.x, fm_out3.force.x, 1e-9);
  EXPECT_NEAR(fm_out2.force.y, fm_out3.force.y, 1e-9);
  EXPECT_NEAR(fm_out2.force.z, fm_out3.force.z, 1e-9);
  EXPECT_NEAR(fm_out2.moment.x, fm_out3.moment.x, 1e-9);
  EXPECT_NEAR(fm_out2.moment.y, fm_out3.moment.y, 1e-9);
  EXPECT_NEAR(fm_out2.moment.z, fm_out3.moment.z, 1e-9);
  EXPECT_NEAR(fm_out2.pos.x, fm_out3.pos.x, 1e-9);
  EXPECT_NEAR(fm_out2.pos.y, fm_out3.pos.y, 1e-9);
  EXPECT_NEAR(fm_out2.pos.z, fm_out3.pos.z, 1e-9);
}

TEST(ForceMomentPosPoseTransform, ReuseInput) {
  ForceMomentPos fmx_a = {{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}, {7.0, 9.0, 10.0}};
  ForceMomentPos fmx_b_ans = {
      {-2.0, 1.0, 3.0}, {-5.0, 4.0, 6.0}, {-9.0, -15.0, 10.0}};
  Mat3 dcm_a2b = {{{0.0, -1.0, 0.0}, {1.0, 0.0, 0.0}, {0.0, 0.0, 1.0}}};
  Vec3 R_a_b_origin = {22.0, 0.0, 0.0};

  ForceMomentPosPoseTransform(&dcm_a2b, &R_a_b_origin, &fmx_a, &fmx_a);
  EXPECT_NEAR_VEC3(fmx_a.force, fmx_b_ans.force, 1e-9);
  EXPECT_NEAR_VEC3(fmx_a.moment, fmx_b_ans.moment, 1e-9);
  EXPECT_NEAR_VEC3(fmx_a.pos, fmx_b_ans.pos, 1e-9);
}

TEST(ForceMomentPosPoseTransform, Recovery) {
  for (int32_t i = 0; i < test_util::kNumTests; ++i) {
    ForceMomentPos fmx_a = kForceMomentPosZero;
    ForceMomentPos fmx_b = kForceMomentPosZero;
    ForceMomentPos fmx_a_out = kForceMomentPosZero;
    Mat3 dcm_a2b;
    AngleToDcm(Rand(-M_PI, M_PI), Rand(-M_PI, M_PI), Rand(-M_PI, M_PI),
               kRotationOrderZyx, &dcm_a2b);
    Vec3 R_a_b_origin = {RandNormal(), RandNormal(), RandNormal()};
    ForceMomentPosPoseTransform(&dcm_a2b, &R_a_b_origin, &fmx_a, &fmx_b);
    ForceMomentPosInversePoseTransform(&dcm_a2b, &R_a_b_origin, &fmx_b,
                                       &fmx_a_out);
    EXPECT_NEAR_VEC3(fmx_a.force, fmx_a_out.force, 1e-9);
    EXPECT_NEAR_VEC3(fmx_a.moment, fmx_a_out.moment, 1e-9);
    EXPECT_NEAR_VEC3(fmx_a.pos, fmx_a_out.pos, 1e-9);
  }
}

TEST(ForceMomentPosToForceMoment, NormalXY) {
  ForceMomentPos fmx = {{1.0, 0.0, 0.0}, {1.0, 1.0, 1.0}, {0.0, 1.0, 0.0}};
  ForceMoment fm;
  ForceMomentPosToForceMoment(&fmx, &fm);
  EXPECT_NEAR(fm.force.x, fmx.force.x, 1e-9);
  EXPECT_NEAR(fm.force.y, fmx.force.y, 1e-9);
  EXPECT_NEAR(fm.force.z, fmx.force.z, 1e-9);
  EXPECT_NEAR(fm.moment.x, fmx.moment.x, 1e-9);
  EXPECT_NEAR(fm.moment.y, fmx.moment.y, 1e-9);
  EXPECT_NEAR(fm.moment.z, fmx.moment.z - 1.0, 1e-9);
}

TEST(ForceMomentPosToForceMoment, NormalYZ) {
  ForceMomentPos fmx = {{0.0, 1.0, 0.0}, {1.0, 1.0, 1.0}, {0.0, 0.0, 1.0}};
  ForceMoment fm;
  ForceMomentPosToForceMoment(&fmx, &fm);
  EXPECT_NEAR(fm.force.x, fmx.force.x, 1e-9);
  EXPECT_NEAR(fm.force.y, fmx.force.y, 1e-9);
  EXPECT_NEAR(fm.force.z, fmx.force.z, 1e-9);
  EXPECT_NEAR(fm.moment.x, fmx.moment.x - 1.0, 1e-9);
  EXPECT_NEAR(fm.moment.y, fmx.moment.y, 1e-9);
  EXPECT_NEAR(fm.moment.z, fmx.moment.z, 1e-9);
}

TEST(ForceMomentPosToForceMoment, NormalXZ) {
  ForceMomentPos fmx = {{1.0, 0.0, 0.0}, {1.0, 1.0, 1.0}, {0.0, 0.0, 1.0}};
  ForceMoment fm;
  ForceMomentPosToForceMoment(&fmx, &fm);
  EXPECT_NEAR(fm.force.x, fmx.force.x, 1e-9);
  EXPECT_NEAR(fm.force.y, fmx.force.y, 1e-9);
  EXPECT_NEAR(fm.force.z, fmx.force.z, 1e-9);
  EXPECT_NEAR(fm.moment.x, fmx.moment.x, 1e-9);
  EXPECT_NEAR(fm.moment.y, fmx.moment.y + 1.0, 1e-9);
  EXPECT_NEAR(fm.moment.z, fmx.moment.z, 1e-9);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

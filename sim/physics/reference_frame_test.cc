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

#include "common/c_math/geometry.h"
#include "common/c_math/mat3.h"
#include "common/c_math/vec3.h"
#include "common/macros.h"
#include "lib/util/test_util.h"
#include "sim/physics/reference_frame.h"

static const Vec3 kTestVec3s[] = {
    {0.0, 0.0, 0.0}, {1.0, 1.0, 1.0}, {1.0, -1.0, 0.0}, {1.0, 2.0, 3.0}};

static const Mat3 kTestDcms[] = {
    {{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}}},
    {{{0.0, 1.0, 0.0}, {-1.0, 0.0, 0.0}, {0.0, 0.0, 1.0}}},
    {{{0.0, -1.0, 0.0}, {1.0, 0.0, 0.0}, {0.0, 0.0, 1.0}}},
    {{{0.5, 0.5, -0.7071}, {-0.1464, 0.8536, 0.5}, {0.8536, -0.1464, 0.5}}}};

TEST(Rotate, CompareToTransform) {
  ReferenceFrame root;

  for (int32_t i = 0; i < ARRAYSIZE(kTestDcms); ++i) {
    for (int32_t j = 0; j < ARRAYSIZE(kTestVec3s); ++j) {
      ReferenceFrame frame(root, kTestVec3s[j], kTestVec3s[j], kTestDcms[i],
                           kTestVec3s[j]);
      for (int32_t k = 0; k < ARRAYSIZE(kTestVec3s); ++k) {
        Vec3 vec_rotate, vec_transform;
        frame.RotateFrom(root, kTestVec3s[k], &vec_rotate);
        frame.TransformFrom(root, ReferenceFrame::kVector, kTestVec3s[k],
                            &vec_transform);
        EXPECT_NEAR_VEC3(vec_rotate, vec_transform, 1e-9);

        frame.RotateTo(root, kTestVec3s[k], &vec_rotate);
        frame.TransformTo(root, ReferenceFrame::kVector, kTestVec3s[k],
                          &vec_transform);
        EXPECT_NEAR_VEC3(vec_rotate, vec_transform, 1e-9);
      }
    }
  }
}

TEST(Rotate, ChainFrames) {
  ReferenceFrame root;
  for (int32_t i = 0; i < ARRAYSIZE(kTestDcms); ++i) {
    ReferenceFrame frame1(root, kVec3Ones, kVec3Ones, kTestDcms[i], kVec3Ones);
    for (int32_t j = 0; j < ARRAYSIZE(kTestDcms); ++j) {
      ReferenceFrame frame2(frame1, kVec3Ones, kVec3Ones, kTestDcms[j],
                            kVec3Ones);
      Mat3 dcm;
      Mat3Mat3Mult(&kTestDcms[j], &kTestDcms[i], &dcm);
      ReferenceFrame frame3(root, kVec3Ones, kVec3Ones, dcm, kVec3Ones);

      for (int32_t k = 0; k < ARRAYSIZE(kTestVec3s); ++k) {
        Vec3 frame2_ans, frame3_ans;
        frame2.RotateFrom(root, kTestVec3s[k], &frame2_ans);
        frame3.RotateFrom(root, kTestVec3s[k], &frame3_ans);
        EXPECT_NEAR_VEC3(frame2_ans, frame3_ans, 1e-9);

        frame2.RotateTo(root, kTestVec3s[k], &frame2_ans);
        frame3.RotateTo(root, kTestVec3s[k], &frame3_ans);
        EXPECT_NEAR_VEC3(frame2_ans, frame3_ans, 1e-9);

        frame2.RotateFrom(frame3, kTestVec3s[k], &frame2_ans);
        frame3.RotateFrom(frame2, kTestVec3s[k], &frame3_ans);
        EXPECT_NEAR_VEC3(frame2_ans, kTestVec3s[k], 1e-3);
        EXPECT_NEAR_VEC3(frame2_ans, frame3_ans, 1e-9);

        frame2.RotateTo(frame3, kTestVec3s[k], &frame2_ans);
        frame3.RotateTo(frame2, kTestVec3s[k], &frame3_ans);
        EXPECT_NEAR_VEC3(frame2_ans, kTestVec3s[k], 1e-3);
        EXPECT_NEAR_VEC3(frame2_ans, frame3_ans, 1e-9);
      }
    }
  }
}

TEST(TransformPosition, ChainFrames) {
  ReferenceFrame root;
  for (int32_t i = 0; i < ARRAYSIZE(kTestVec3s); ++i) {
    ReferenceFrame frame1(root, kTestVec3s[i], kVec3Ones, kMat3Identity,
                          kVec3Ones);
    for (int32_t j = 0; j < ARRAYSIZE(kTestVec3s); ++j) {
      ReferenceFrame frame2(frame1, kTestVec3s[j], kVec3Ones, kMat3Identity,
                            kVec3Ones);
      Vec3 offset;
      Vec3Add(&kTestVec3s[i], &kTestVec3s[j], &offset);
      ReferenceFrame frame3(root, offset, kVec3Ones, kMat3Identity, kVec3Ones);

      for (int32_t k = 0; k < ARRAYSIZE(kTestVec3s); ++k) {
        Vec3 frame2_ans, frame3_ans;
        frame2.TransformFrom(root, ReferenceFrame::kPosition, kTestVec3s[k],
                             &frame2_ans);
        frame3.TransformFrom(root, ReferenceFrame::kPosition, kTestVec3s[k],
                             &frame3_ans);
        EXPECT_NEAR_VEC3(frame2_ans, frame3_ans, 1e-9);

        frame2.TransformTo(root, ReferenceFrame::kPosition, kTestVec3s[k],
                           &frame2_ans);
        frame3.TransformTo(root, ReferenceFrame::kPosition, kTestVec3s[k],
                           &frame3_ans);
        EXPECT_NEAR_VEC3(frame2_ans, frame3_ans, 1e-9);

        frame2.TransformFrom(frame3, ReferenceFrame::kPosition, kTestVec3s[k],
                             &frame2_ans);
        frame3.TransformFrom(frame2, ReferenceFrame::kPosition, kTestVec3s[k],
                             &frame3_ans);
        EXPECT_NEAR_VEC3(frame2_ans, kTestVec3s[k], 1e-3);
        EXPECT_NEAR_VEC3(frame2_ans, frame3_ans, 1e-9);

        frame2.TransformTo(frame3, ReferenceFrame::kPosition, kTestVec3s[k],
                           &frame2_ans);
        frame3.TransformTo(frame2, ReferenceFrame::kPosition, kTestVec3s[k],
                           &frame3_ans);
        EXPECT_NEAR_VEC3(frame2_ans, kTestVec3s[k], 1e-3);
        EXPECT_NEAR_VEC3(frame2_ans, frame3_ans, 1e-9);
      }
    }
  }
}

TEST(TransformVelocity, Simple) {
  ReferenceFrame root;
  ReferenceFrame frame1(root, kVec3Zero, kVec3Zero, kMat3Identity, kVec3Z);
  ReferenceFrame frame2(frame1, kVec3Y, kVec3Zero, kMat3Identity, kVec3Zero);

  Vec3 vel_root;
  frame2.TransformTo(root, ReferenceFrame::kVelocity, kVec3X, &vel_root);
  EXPECT_NEAR_VEC3(vel_root, kVec3Zero, 1e-9);

  Vec3 ans1 = {-1.0, 1.0, 0.0};
  frame2.TransformTo(root, ReferenceFrame::kVelocity, kVec3Y, &vel_root);
  EXPECT_NEAR_VEC3(vel_root, ans1, 1e-9);

  Vec3 ans2 = {-1.0, 0.0, 1.0};
  frame2.TransformTo(root, ReferenceFrame::kVelocity, kVec3Z, &vel_root);
  EXPECT_NEAR_VEC3(vel_root, ans2, 1e-9);
}

TEST(TransformVelocity, ChainFrames) {
  ReferenceFrame root;
  for (int32_t i = 0; i < ARRAYSIZE(kTestVec3s); ++i) {
    for (int32_t j = 0; j < ARRAYSIZE(kTestVec3s); ++j) {
      for (int32_t k = 0; k < ARRAYSIZE(kTestVec3s); ++k) {
        ReferenceFrame frame1(root, kVec3Ones, kTestVec3s[i], kMat3Identity,
                              kTestVec3s[k]);
        ReferenceFrame frame2(frame1, kVec3Ones, kTestVec3s[j], kMat3Identity,
                              kVec3Ones);

        Vec3 omega_cross_pos;
        Vec3Cross(&kTestVec3s[k], &kVec3Ones, &omega_cross_pos);
        Vec3 vel_offset;
        Vec3Add3(&kTestVec3s[i], &kTestVec3s[j], &omega_cross_pos, &vel_offset);
        ReferenceFrame frame3(root, kVec3Zero, vel_offset, kMat3Identity,
                              kVec3Zero);

        Vec3 frame2_ans, frame3_ans;
        frame2.TransformFrom(root, ReferenceFrame::kVelocity, kVec3Ones,
                             &frame2_ans);
        frame3.TransformFrom(root, ReferenceFrame::kVelocity, kVec3Ones,
                             &frame3_ans);
        EXPECT_NEAR_VEC3(frame2_ans, frame3_ans, 1e-9);

        frame2.TransformTo(root, ReferenceFrame::kVelocity, kVec3Ones,
                           &frame2_ans);
        frame3.TransformTo(root, ReferenceFrame::kVelocity, kVec3Ones,
                           &frame3_ans);
        EXPECT_NEAR_VEC3(frame2_ans, frame3_ans, 1e-9);

        frame2.TransformFrom(frame3, ReferenceFrame::kVelocity, kVec3Ones,
                             &frame2_ans);
        frame3.TransformFrom(frame2, ReferenceFrame::kVelocity, kVec3Ones,
                             &frame3_ans);
        EXPECT_NEAR_VEC3(frame2_ans, kVec3Ones, 1e-3);
        EXPECT_NEAR_VEC3(frame2_ans, frame3_ans, 1e-9);

        frame2.TransformTo(frame3, ReferenceFrame::kVelocity, kVec3Ones,
                           &frame2_ans);
        frame3.TransformTo(frame2, ReferenceFrame::kVelocity, kVec3Ones,
                           &frame3_ans);
        EXPECT_NEAR_VEC3(frame2_ans, kVec3Ones, 1e-3);
        EXPECT_NEAR_VEC3(frame2_ans, frame3_ans, 1e-9);
      }
    }
  }
}

TEST(TransformAcceleration, Centripetal) {
  ReferenceFrame root;

  Vec3 omega = {0.0, 0.0, 2.0};
  ReferenceFrame frame1(root, kVec3Zero, kVec3Zero, kMat3Identity, omega);
  ReferenceFrame frame2(frame1, kVec3X, kVec3Zero, kMat3Identity, kVec3Zero);

  Vec3 acc_root;
  frame2.TransformOriginTo(root, ReferenceFrame::kAcceleration, &acc_root);
  Vec3 ans1 = {-4.0, 0.0, 0.0};
  EXPECT_NEAR_VEC3(acc_root, ans1, 1e-9);
}

TEST(TransformAcceleration, Euler) {
  ReferenceFrame root;

  Vec3 domega = {0.0, 0.0, 2.0};
  ReferenceFrame frame1(root, kVec3Zero, kVec3Zero, kVec3Zero, kMat3Identity,
                        kVec3Zero, domega);
  ReferenceFrame frame2(frame1, kVec3X, kVec3Zero, kMat3Identity, kVec3Zero);

  Vec3 acc_root;
  frame2.TransformOriginTo(root, ReferenceFrame::kAcceleration, &acc_root);
  Vec3 ans1 = {0.0, 2.0, 0.0};
  EXPECT_NEAR_VEC3(acc_root, ans1, 1e-9);
}

TEST(TransformTo_TransformFrom, IdentityChecks) {
  // For simplicity this test treats the Earth as a sphere.
  ReferenceFrame sci;
  for (double t = 0.0; t < 100.0; ++t) {
    Mat3 dcm_i2s;
    double phi = t * t / 100.0;
    double dphi = 2.0 * t / 100.0;
    double ddphi = 2.0 / 100.0;
    AngleToDcm(0.0, 0.0, phi, kRotationOrderZyx, &dcm_i2s);
    ReferenceFrame scsf(sci, kVec3Zero, kVec3Zero, kVec3Zero, dcm_i2s,
                        {0.0, 0.0, dphi}, {0.0, 0.0, ddphi});
    for (double lat = -M_PI / 6.0; lat < M_PI / 6.0; lat += M_PI / 60.0) {
      Mat3 dcm_s2l;
      Vec3 x_sl = {1.0, 0.0, 0.0};
      AngleToDcm(0.0, lat, 0.0, kRotationOrderZyx, &dcm_s2l);
      ReferenceFrame l(scsf, x_sl, kVec3Zero, dcm_s2l, kVec3Zero);
      Vec3 rand_vec = {::test_util::RandNormal(), ::test_util::RandNormal(),
                       ::test_util::RandNormal()};
      Vec3 tmp;
      ReferenceFrame::VectorType types[] = {
          ReferenceFrame::kVector,
          ReferenceFrame::kPosition,
          ReferenceFrame::kVelocity,
          ReferenceFrame::kAcceleration,
          ReferenceFrame::kAngularVelocity,
          ReferenceFrame::kAngularAcceleration};
      for (ReferenceFrame::VectorType type : types) {
        l.TransformTo(sci, type, rand_vec, &tmp);
        l.TransformFrom(sci, type, tmp, &tmp);
        EXPECT_NEAR_VEC3(rand_vec, tmp, 1e-9);
      }

      Vec3 ans_one, ans_two;
      l.TransformTo(sci, ReferenceFrame::kVector, rand_vec, &ans_one);
      l.RotateTo(sci, rand_vec, &ans_two);
      EXPECT_NEAR_VEC3(ans_one, ans_two, 1e-9);

      ReferenceFrame pos(l, rand_vec);
      l.TransformTo(sci, ReferenceFrame::kPosition, rand_vec, &ans_one);
      pos.TransformOriginTo(sci, ReferenceFrame::kPosition, &ans_two);
      EXPECT_NEAR_VEC3(ans_one, ans_two, 1e-9);

      ReferenceFrame vel(l, kVec3Zero, rand_vec, kMat3Identity, kVec3Zero);
      l.TransformTo(sci, ReferenceFrame::kVelocity, rand_vec, &ans_one);
      vel.TransformOriginTo(sci, ReferenceFrame::kVelocity, &ans_two);
      EXPECT_NEAR_VEC3(ans_one, ans_two, 1e-9);

      ReferenceFrame rot(l, kVec3Zero, kVec3Zero, kMat3Identity, rand_vec);
      l.TransformTo(sci, ReferenceFrame::kAngularVelocity, rand_vec, &ans_one);
      rot.TransformOriginTo(sci, ReferenceFrame::kAngularVelocity, &ans_two);
      EXPECT_NEAR_VEC3(ans_one, ans_two, 1e-9);

      ReferenceFrame acc(l, kVec3Zero, kVec3Zero, rand_vec, kMat3Identity,
                         kVec3Zero, kVec3Zero);
      l.TransformTo(sci, ReferenceFrame::kAcceleration, rand_vec, &ans_one);
      acc.TransformOriginTo(sci, ReferenceFrame::kAcceleration, &ans_two);
      EXPECT_NEAR_VEC3(ans_one, ans_two, 1e-9);

      ReferenceFrame alp(l, kVec3Zero, kVec3Zero, kVec3Zero, kMat3Identity,
                         kVec3Zero, rand_vec);
      l.TransformTo(sci, ReferenceFrame::kAngularAcceleration, rand_vec,
                    &ans_one);
      alp.TransformOriginTo(sci, ReferenceFrame::kAngularAcceleration,
                            &ans_two);
      EXPECT_NEAR_VEC3(ans_one, ans_two, 1e-9);
    }
  }
}

static void EulerZyxRateToOmegaB(const Vec3 &eulers_rpy,
                                 const Vec3 &deulers_rpy, Vec3 *omega) {
  Mat3 tmp = {
      {{1.0, 0.0, -sin(eulers_rpy.y)},
       {0.0, cos(eulers_rpy.x), sin(eulers_rpy.x) * cos(eulers_rpy.y)},
       {0.0, -sin(eulers_rpy.x), cos(eulers_rpy.x) * cos(eulers_rpy.y)}}};
  Mat3Vec3Mult(&tmp, &deulers_rpy, omega);
}

static void CalcRate(double t, const ReferenceFrame &root, Vec3 *Xg, Vec3 *Vg) {
  Vec3 deulers_rpy = {0.1, 0.1, -0.1};
  Vec3 eulers_rpy;
  Vec3Scale(&deulers_rpy, t, &eulers_rpy);

  Mat3 dcm_g2b;
  AngleToDcm(eulers_rpy.z, eulers_rpy.y, eulers_rpy.x, kRotationOrderZyx,
             &dcm_g2b);
  Vec3 omega;
  EulerZyxRateToOmegaB(eulers_rpy, deulers_rpy, &omega);
  Mat3TransVec3Mult(&dcm_g2b, &omega, &omega);

  ReferenceFrame rotate(root, kVec3Zero, kVec3Zero, dcm_g2b, omega);
  Vec3 p_pos = {1.0, 0.0, 0.0};
  ReferenceFrame point(rotate, p_pos);
  point.TransformOriginTo(root, ReferenceFrame::kVelocity, Vg);
  if (Xg != nullptr) Mat3TransVec3Mult(&dcm_g2b, &p_pos, Xg);
}

TEST(TransformTo, Precession) {
  ReferenceFrame root;
  double dt = 0.001;
  Vec3 Xg = {1.0, 0.0, 0.0};
  for (double t = dt; t < 2000.0 * dt; t += dt) {
    Vec3 V_a, V_h, V_b, Xg_actual;
    CalcRate(t - dt, root, nullptr, &V_a);
    CalcRate(t - dt / 2.0, root, nullptr, &V_h);
    CalcRate(t, root, &Xg_actual, &V_h);
    Vec3 dX;
    Vec3LinComb3(dt / 6.0, &V_a, dt * 2.0 / 3.0, &V_h, dt / 6.0, &V_b, &dX);
    Vec3Add(&dX, &Xg, &Xg);
    EXPECT_NEAR_VEC3(Xg, Xg_actual, 1e-1);
  }
}

TEST(TransformAcceleration, Coriolis) {
  ReferenceFrame root;

  Vec3 omega = {0.0, 0.0, 2.0};
  ReferenceFrame frame1(root, kVec3Zero, kVec3Zero, kMat3Identity, omega);
  ReferenceFrame frame2(frame1, kVec3Zero, kVec3X, kMat3Identity, kVec3Zero);

  Vec3 acc_root;
  frame2.TransformOriginTo(root, ReferenceFrame::kAcceleration, &acc_root);
  Vec3 ans1 = {0.0, 4.0, 0.0};
  EXPECT_NEAR_VEC3(acc_root, ans1, 1e-9);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

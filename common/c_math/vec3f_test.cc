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

#include <float.h>
#include <math.h>

#include "common/c_math/util.h"
#include "common/c_math/vec3f.h"
#include "lib/util/test_util.h"

using ::test_util::RandNormalf;

TEST(Vec3fMin, PermuteArgs) {
  for (int32_t i = 0; i < 22; ++i) {
    Vec3f a = {RandNormalf(), RandNormalf(), RandNormalf()};
    Vec3f b = {RandNormalf(), RandNormalf(), RandNormalf()};
    Vec3f c1, c2;
    Vec3fMin(&a, &b, &c1);
    Vec3fMin(&b, &a, &c2);
    EXPECT_EQ_VEC3(c1, c2);
  }
}

TEST(Vec3fMin, PermuteInd) {
  for (int32_t i = 0; i < 22; ++i) {
    Vec3f a = {RandNormalf(), RandNormalf(), RandNormalf()};
    Vec3f b = {RandNormalf(), RandNormalf(), RandNormalf()};
    Vec3f c;
    Vec3fMin(&a, &b, &c);

    Vec3f c_permute;
    SwapInPlacef(&a.x, &a.y);
    SwapInPlacef(&b.x, &b.y);
    SwapInPlacef(&c.x, &c.y);
    Vec3fMin(&a, &b, &c_permute);
    EXPECT_EQ_VEC3(c, c_permute);

    SwapInPlacef(&a.y, &a.z);
    SwapInPlacef(&b.y, &b.z);
    SwapInPlacef(&c.y, &c.z);
    Vec3fMin(&a, &b, &c_permute);
    EXPECT_EQ_VEC3(c, c_permute);
  }
}

TEST(Vec3fNorm, Zero) {
  float v_norm = Vec3fNorm(&kVec3fZero);
  EXPECT_EQ(v_norm, 0.0f);
}

TEST(Vec3fNorm, Infinity) {
  Vec3f vx = {INFINITY, 0.0f, 0.0f};
  Vec3f vy = {0.0f, INFINITY, 0.0f};
  Vec3f vz = {0.0f, 0.0f, INFINITY};

  double vx_norm = Vec3fNorm(&vx);
  double vy_norm = Vec3fNorm(&vy);
  double vz_norm = Vec3fNorm(&vz);

  EXPECT_EQ(vx_norm, INFINITY);
  EXPECT_EQ(vy_norm, INFINITY);
  EXPECT_EQ(vz_norm, INFINITY);
}

TEST(Vec3fNorm, InfNaN) {
  // Expect the conventions of hypot(): if any vector component is
  // infinity, we expect positive infinity.  If any component is NaN
  // (and none is infinite), we expect NaN.
  Vec3f vw = {NAN, NAN, NAN};
  Vec3f vx = {NAN, 0.0f, 0.0f};
  Vec3f vy = {NAN, -INFINITY, 0.0f};
  Vec3f vz = {INFINITY, NAN, NAN};

  float vw_norm = Vec3fNorm(&vw);
  float vx_norm = Vec3fNorm(&vx);
  float vy_norm = Vec3fNorm(&vy);
  float vz_norm = Vec3fNorm(&vz);

  EXPECT_TRUE(isnan(vw_norm));
  EXPECT_TRUE(isnan(vx_norm));
  EXPECT_EQ(vy_norm, INFINITY);
  EXPECT_EQ(vz_norm, INFINITY);
}

TEST(Vec3fNorm, Underflow) {
  Vec3f vx = {FLT_MIN, 0.0f, 0.0f};
  Vec3f vy = {0.0f, FLT_MIN, 0.0f};
  Vec3f vz = {0.0f, 0.0f, FLT_MIN};

  float vx_norm = Vec3fNorm(&vx);
  float vy_norm = Vec3fNorm(&vy);
  float vz_norm = Vec3fNorm(&vz);

  EXPECT_EQ(vx_norm, FLT_MIN);
  EXPECT_EQ(vy_norm, FLT_MIN);
  EXPECT_EQ(vz_norm, FLT_MIN);
}

TEST(Vec3Norm, Underflow2) {
  // This is a very small number whose reciprocal is infinite.
  float very_small = nextafterf(0.0f, 1.0f);  // Smallest representable float.

  Vec3f vx = {very_small, 0.0f, 0.0f};
  Vec3f vy = {0.0f, very_small, 0.0f};
  Vec3f vz = {0.0f, 0.0f, very_small};

  double vx_norm = Vec3fNorm(&vx);
  double vy_norm = Vec3fNorm(&vy);
  double vz_norm = Vec3fNorm(&vz);

  EXPECT_EQ(vx_norm, very_small);
  EXPECT_EQ(vy_norm, very_small);
  EXPECT_EQ(vz_norm, very_small);
}

TEST(Vec3fNorm, Overflow) {
  Vec3f vx = {FLT_MAX, 0.0f, 0.0f};
  Vec3f vy = {0.0f, FLT_MAX, 0.0f};
  Vec3f vz = {0.0f, 0.0f, FLT_MAX};

  float vx_norm = Vec3fNorm(&vx);
  float vy_norm = Vec3fNorm(&vy);
  float vz_norm = Vec3fNorm(&vz);

  EXPECT_NEAR(vx_norm, FLT_MAX, FLT_MAX * FLT_EPSILON);
  EXPECT_NEAR(vy_norm, FLT_MAX, FLT_MAX * FLT_EPSILON);
  EXPECT_NEAR(vz_norm, FLT_MAX, FLT_MAX * FLT_EPSILON);
}

TEST(Vec3fNormalize, BigVector) {
  Vec3f v = {FLT_MAX, FLT_MAX, FLT_MAX};
  Vec3fNormalize(&v, &v);

  EXPECT_EQ(1.0f / sqrtf(3.0f), v.x);
  EXPECT_EQ(1.0f / sqrtf(3.0f), v.y);
  EXPECT_EQ(1.0f / sqrtf(3.0f), v.z);

  EXPECT_NEAR(Vec3fNorm(&v), 1.0f, FLT_EPSILON);
}

#if defined(NDEBUG)
TEST(Vec3fNormalize, Inf) {
  Vec3f v = {INFINITY, -INFINITY, 22.0f};
  Vec3fNormalize(&v, &v);

  EXPECT_NEAR(1.0f / sqrt(2.0f), v.x, FLT_EPSILON);
  EXPECT_NEAR(-1.0f / sqrt(2.0f), v.y, FLT_EPSILON);
  EXPECT_NEAR(22.0f / sqrt(2.0f) / FLT_MAX, v.z, FLT_MIN);

  EXPECT_NEAR(Vec3fNorm(&v), 1.0f, FLT_EPSILON);
}

TEST(Vec3fNormalize, Inf2) {
  Vec3f vx = {INFINITY, 0.0f, 0.0f};
  Vec3f vy = {0.0f, INFINITY, 0.0f};
  Vec3f vz = {0.0f, 0.0f, INFINITY};

  Vec3fNormalize(&vx, &vx);
  Vec3fNormalize(&vy, &vy);
  Vec3fNormalize(&vz, &vz);

  EXPECT_EQ_VEC3(vx, kVec3fX);
  EXPECT_EQ_VEC3(vy, kVec3fY);
  EXPECT_EQ_VEC3(vz, kVec3fZ);
}

TEST(Vec3fNormalize, SingleNaN) {
  Vec3f v = {1.0f, NAN, 0.0f};
  Vec3fNormalize(&v, &v);

  EXPECT_EQ(1.0f, v.x);
  EXPECT_EQ(0.0f, v.y);
  EXPECT_EQ(0.0f, v.z);

  EXPECT_NEAR(Vec3fNorm(&v), 1.0f, FLT_EPSILON);
}

TEST(Vec3fNormalize, AllNaN) {
  Vec3f v = {NAN, NAN, NAN};
  Vec3fNormalize(&v, &v);

  EXPECT_EQ(0.0f, v.x);
  EXPECT_EQ(0.0f, v.y);
  EXPECT_EQ(0.0f, v.z);
}

TEST(Vec3fNormalize, Zero) {
  Vec3f v = kVec3fZero;
  Vec3fNormalize(&v, &v);

  EXPECT_EQ(0.0f, v.x);
  EXPECT_EQ(0.0f, v.y);
  EXPECT_EQ(0.0f, v.z);
}
#endif  // defined(NDEBUG)

TEST(Vec3fNormalize, NoClobber) {
  Vec3f v_in = {-10.0f, 20.0f, 3.0f};
  Vec3f v_out = {123.0f, 99.8f, -56.2f};
  Vec3fNormalize(&v_in, &v_out);

  // Verify that the input was not clobbered.
  EXPECT_EQ(-10.0f, v_in.x);
  EXPECT_EQ(20.0f, v_in.y);
  EXPECT_EQ(3.0f, v_in.z);

  // Verify that the output is normalized and parallel to the input.
  EXPECT_NEAR(Vec3fNorm(&v_out), 1.0f, FLT_EPSILON);
  EXPECT_NEAR(Vec3fDot(&v_in, &v_out), Vec3fNorm(&v_in), FLT_EPSILON);
}

TEST(Vec3fXyNorm, Normal) {
  Vec3f v = {3.0f, 4.0f, 22.0f};
  EXPECT_NEAR(Vec3fXyNorm(&v), 5.0f, FLT_EPSILON);
}

TEST(Vec3fXzNorm, Normal) {
  Vec3f v = {4.0f, 22.0f, 3.0f};
  EXPECT_NEAR(Vec3fXzNorm(&v), 5.0f, FLT_EPSILON);
}

TEST(Vec3fYzNorm, Normal) {
  Vec3f v = {22.0f, 4.0f, 3.0f};
  EXPECT_NEAR(Vec3fYzNorm(&v), 5.0f, FLT_EPSILON);
}

TEST(Vec3fAxpyTest, Normal0) {
  const Vec3f x = {0.603510315099689f, 0.265730454435706f, 0.188188919436278f};
  Vec3f y = {0.3493157678185849f, 0.0773055504347321f, 0.0797215727300547f};
  const Vec3f zout = {0.687106507771254f, 0.226037534936056f,
                      0.185052787055320f};
  const float a = 0.559709969326492f;

  Vec3fAxpy(a, &x, &y);
  EXPECT_NEAR_VEC3(zout, y, FLT_EPSILON);
}

TEST(Vec3fAxpyTest, Consistency0) {
  for (int32_t i = 0; i < test_util::kNumTests; ++i) {
    const Vec3f x = {RandNormalf(), RandNormalf(), RandNormalf()};
    Vec3f y = {RandNormalf(), RandNormalf(), RandNormalf()};
    float a = RandNormalf();
    Vec3f z = kVec3fZero;
    Vec3f w = kVec3fZero;

    // w = -y
    // y = a*x + y
    // w = y + w
    // z = w/a
    // => z == x
    Vec3fAxpy(-1.0f, &y, &w);
    Vec3fAxpy(a, &x, &y);
    Vec3fAxpy(1.0f, &y, &w);
    Vec3fAxpy(1.0f / a, &w, &z);
    // TODO: Investigate why we can't use FLT_EPSILON here.
    EXPECT_NEAR_VEC3(x, z, 1e-3);
  }
}

// The remaining tests are generated code comparing the results to
// MATLAB.

TEST(Vec3fNormTest, Normal1) {
  const Vec3f v = {0.5367702526124893f, -0.4260820531771103f,
                   0.0239257162803643f};
  double v_norm = Vec3fNorm(&v);

  EXPECT_NEAR(0.6857409569427992f, v_norm, FLT_EPSILON);
}

TEST(Vec3fNormBoundTest, Unbounded1) {
  const Vec3f v = {-0.2303068939695418f, 0.4617951054964475f,
                   -0.4544887764605945f};
  double v_norm = Vec3fNormBound(&v, 0.5774509303531977f);

  EXPECT_NEAR(0.6876452812308256f, v_norm, FLT_EPSILON);
}

TEST(Vec3fNormBoundTest, Bounded1) {
  const Vec3f v = {0.9265032594833635f, 0.5911502601488472f,
                   -0.5127920011272906f};
  double v_norm = Vec3fNormBound(&v, 2.0619801096902344f);

  EXPECT_NEAR(2.0619801096902344f, v_norm, FLT_EPSILON);
}

TEST(Vec3fNormSquaredTest, Normal1) {
  const Vec3f v = {-0.6687971816684477f, 0.0130227044755817f,
                   0.8264218437770381f};
  double v_norm_sqrd = Vec3fNormSquared(&v);

  EXPECT_NEAR(1.1304323249113561f, v_norm_sqrd, FLT_EPSILON);
}

TEST(Vec3fNormalizeTest, Normal1) {
  Vec3f v = {0.8420114420131575f, -0.6763637083427534f, -0.0863678034960516f};
  const Vec3f *v_out_ptr = Vec3fNormalize(&v, &v);

  EXPECT_NEAR(0.7771422021234553f, v.x, FLT_EPSILON);
  EXPECT_NEAR(-0.6242561033151141f, v.y, FLT_EPSILON);
  EXPECT_NEAR(-0.0797139583293674f, v.z, FLT_EPSILON);

  EXPECT_NEAR(0.7771422021234553f, v_out_ptr->x, FLT_EPSILON);
  EXPECT_NEAR(-0.6242561033151141f, v_out_ptr->y, FLT_EPSILON);
  EXPECT_NEAR(-0.0797139583293674f, v_out_ptr->z, FLT_EPSILON);
}

#if defined(NDEBUG)
TEST(Vec3fNormalizeTest, Zero_1) {
  Vec3f v = {0.0000000000000000f, 0.0000000000000000f, 0.0000000000000000f};
  const Vec3f *v_out_ptr = Vec3fNormalize(&v, &v);

  EXPECT_NEAR(0.0000000000000000f, v.x, FLT_EPSILON);
  EXPECT_NEAR(0.0000000000000000f, v.y, FLT_EPSILON);
  EXPECT_NEAR(0.0000000000000000f, v.z, FLT_EPSILON);

  EXPECT_NEAR(0.0000000000000000f, v_out_ptr->x, FLT_EPSILON);
  EXPECT_NEAR(0.0000000000000000f, v_out_ptr->y, FLT_EPSILON);
  EXPECT_NEAR(0.0000000000000000f, v_out_ptr->z, FLT_EPSILON);
}
#endif  // defined(NDEBUG)

TEST(Vec3fScaleTest, Normal1) {
  const Vec3f v = {-0.1644879149932854f, 0.6427201538828351f,
                   -0.9340521997382087f};
  Vec3f v_out = {0.5517522819833633f, 0.2388025183107527f,
                 -0.7807407593672366f};
  const Vec3f *v_out_ptr = Vec3fScale(&v, -0.8023639014165720f, &v_out);

  EXPECT_NEAR(0.1319791652098899f, v_out.x, FLT_EPSILON);
  EXPECT_NEAR(-0.5156954501884911f, v_out.y, FLT_EPSILON);
  EXPECT_NEAR(0.7494497671086803f, v_out.z, FLT_EPSILON);

  EXPECT_NEAR(0.1319791652098899f, v_out_ptr->x, FLT_EPSILON);
  EXPECT_NEAR(-0.5156954501884911f, v_out_ptr->y, FLT_EPSILON);
  EXPECT_NEAR(0.7494497671086803f, v_out_ptr->z, FLT_EPSILON);
}

TEST(Vec3fAddTest, Normal1) {
  const Vec3f v0 = {-0.1478831571559869f, -0.2567374627753023f,
                    -0.0123440884023902f};
  const Vec3f v1 = {-0.9739330432167523f, 0.4058302368576159f,
                    0.1015556273908707f};
  Vec3f v_out = {-0.4500020725584011f, 0.9195757878627335f,
                 0.8028451634961025f};
  const Vec3f *v_out_ptr = Vec3fAdd(&v0, &v1, &v_out);

  EXPECT_NEAR(-1.1218162003727392f, v_out.x, FLT_EPSILON);
  EXPECT_NEAR(0.1490927740823136f, v_out.y, FLT_EPSILON);
  EXPECT_NEAR(0.0892115389884804f, v_out.z, FLT_EPSILON);

  EXPECT_NEAR(-1.1218162003727392f, v_out_ptr->x, FLT_EPSILON);
  EXPECT_NEAR(0.1490927740823136f, v_out_ptr->y, FLT_EPSILON);
  EXPECT_NEAR(0.0892115389884804f, v_out_ptr->z, FLT_EPSILON);
}

TEST(Vec3fAdd3Test, Normal1) {
  const Vec3f v0 = {0.6599076801606711f, 0.3503162626705241f,
                    0.6140936231935474f};
  const Vec3f v1 = {0.2605656957992768f, 0.4856778791661713f,
                    -0.3567832603321217f};
  const Vec3f v2 = {0.5566181827999006f, 0.0839612311831071f,
                    0.1145621134624220f};
  Vec3f v0c = {0.6599076801606711f, 0.3503162626705241f, 0.6140936231935474f};
  Vec3f v1c = {0.2605656957992768f, 0.4856778791661713f, -0.3567832603321217f};
  Vec3f v2c = {0.5566181827999006f, 0.0839612311831071f, 0.1145621134624220f};
  Vec3f v_out = {-0.6951257011121486f, -0.4643466868093897f,
                 0.7954443333909311f};
  const Vec3f *v_out_ptr = Vec3fAdd3(&v0, &v1, &v2, &v_out);
  Vec3fAdd3(&v0c, &v1, &v2, &v0c);
  Vec3fAdd3(&v0, &v1c, &v2, &v1c);
  Vec3fAdd3(&v0, &v1, &v2c, &v2c);

  EXPECT_NEAR(1.4770915587598485f, v_out.x, FLT_EPSILON);
  EXPECT_NEAR(0.9199553730198025f, v_out.y, FLT_EPSILON);
  EXPECT_NEAR(0.3718724763238477f, v_out.z, FLT_EPSILON);

  EXPECT_NEAR(1.4770915587598485f, v0c.x, FLT_EPSILON);
  EXPECT_NEAR(0.9199553730198025f, v0c.y, FLT_EPSILON);
  EXPECT_NEAR(0.3718724763238477f, v0c.z, FLT_EPSILON);

  EXPECT_NEAR(1.4770915587598485f, v1c.x, FLT_EPSILON);
  EXPECT_NEAR(0.9199553730198025f, v1c.y, FLT_EPSILON);
  EXPECT_NEAR(0.3718724763238477f, v1c.z, FLT_EPSILON);

  EXPECT_NEAR(1.4770915587598485f, v2c.x, FLT_EPSILON);
  EXPECT_NEAR(0.9199553730198025f, v2c.y, FLT_EPSILON);
  EXPECT_NEAR(0.3718724763238477f, v2c.z, FLT_EPSILON);

  EXPECT_NEAR(1.4770915587598485f, v_out_ptr->x, FLT_EPSILON);
  EXPECT_NEAR(0.9199553730198025f, v_out_ptr->y, FLT_EPSILON);
  EXPECT_NEAR(0.3718724763238477f, v_out_ptr->z, FLT_EPSILON);
}

TEST(Vec3fSubTest, Normal1) {
  const Vec3f v0 = {0.6766932755174186f, -0.9962267335320423f,
                    -0.2110364564189453f};
  const Vec3f v1 = {0.4697670300417833f, -0.8089695215392609f,
                    -0.3245848530679498f};
  Vec3f v_out = {0.5257322073508937f, 0.7635724211233310f,
                 -0.4459327109112239f};
  const Vec3f *v_out_ptr = Vec3fSub(&v0, &v1, &v_out);

  EXPECT_NEAR(0.2069262454756353f, v_out.x, FLT_EPSILON);
  EXPECT_NEAR(-0.1872572119927813f, v_out.y, FLT_EPSILON);
  EXPECT_NEAR(0.1135483966490045f, v_out.z, FLT_EPSILON);

  EXPECT_NEAR(0.2069262454756353f, v_out_ptr->x, FLT_EPSILON);
  EXPECT_NEAR(-0.1872572119927813f, v_out_ptr->y, FLT_EPSILON);
  EXPECT_NEAR(0.1135483966490045f, v_out_ptr->z, FLT_EPSILON);
}

TEST(Vec3fLinCombTest, Normal1) {
  const Vec3f v0 = {0.2674433189118164f, 0.7857832103283664f,
                    -0.9309107927791354f};
  const Vec3f v1 = {0.7982576750757842f, -0.3323133234393580f,
                    0.4954047249089635f};
  Vec3f v_out = {-0.5062570639231658f, 0.3430758576211665f,
                 -0.2031135335265573f};
  const Vec3f *v_out_ptr =
      Vec3fLinComb(-0.6641621523381429f, &v0, 0.1712432676141122f, &v1, &v_out);

  EXPECT_NEAR(-0.0409294776389068f, v_out.x, FLT_EPSILON);
  EXPECT_NEAR(-0.5787938876203245f, v_out.y, FLT_EPSILON);
  EXPECT_NEAR(0.7031104396518788f, v_out.z, FLT_EPSILON);

  EXPECT_NEAR(-0.0409294776389068f, v_out_ptr->x, FLT_EPSILON);
  EXPECT_NEAR(-0.5787938876203245f, v_out_ptr->y, FLT_EPSILON);
  EXPECT_NEAR(0.7031104396518788f, v_out_ptr->z, FLT_EPSILON);
}

TEST(Vec3fLinComb3Test, Normal1) {
  const Vec3f v0 = {-0.7546510030033908f, 0.3704451278835053f,
                    0.6640463277605138f};
  const Vec3f v1 = {0.3433385306612882f, 0.2126343590682989f,
                    -0.7918223874050583f};
  const Vec3f v2 = {-0.6046805279105418f, 0.7919650295736314f,
                    -0.5489419844219976f};
  Vec3f v_out = {0.0451612961133447f, -0.2061715785301237f,
                 0.8606211659267344f};
  const Vec3f *v_out_ptr =
      Vec3fLinComb3(0.6224819598251061f, &v0, -0.2799285848080595f, &v1,
                    -0.7336872774962144f, &v2, &v_out);

  EXPECT_NEAR(-0.1222204940539666f, v_out.x, FLT_EPSILON);
  EXPECT_NEAR(-0.4099816924230578f, v_out.y, FLT_EPSILON);
  EXPECT_NEAR(1.0377623298986112f, v_out.z, FLT_EPSILON);

  EXPECT_NEAR(-0.1222204940539666f, v_out_ptr->x, FLT_EPSILON);
  EXPECT_NEAR(-0.4099816924230578f, v_out_ptr->y, FLT_EPSILON);
  EXPECT_NEAR(1.0377623298986112f, v_out_ptr->z, FLT_EPSILON);
}

TEST(Vec3fDotTest, Normal1) {
  const Vec3f v0 = {0.4984800705693784f, 0.5270289236303241f,
                    0.2673143832374629f};
  const Vec3f v1 = {0.5548888677474180f, -0.2117005486838590f,
                    -0.2188018092068686f};
  double v_dot = Vec3fDot(&v0, &v1);

  EXPECT_NEAR(0.1065398589687170f, v_dot, FLT_EPSILON);
}

TEST(Vec3fMultTest, Normal1) {
  const Vec3f v0 = {-0.4019846845889379f, -0.9445145506261914f,
                    -0.2196442368237426f};
  const Vec3f v1 = {0.9126289942107253f, 0.3503899871409848f,
                    0.7621759362910447f};
  Vec3f v_out = {0.3283086149111243f, 0.3823580496403809f,
                 -0.0468320656530856f};
  const Vec3f *v_out_ptr = Vec3fMult(&v0, &v1, &v_out);

  EXPECT_NEAR(-0.3668628783845180f, v_out.x, FLT_EPSILON);
  EXPECT_NEAR(-0.3309484412483842f, v_out.y, FLT_EPSILON);
  EXPECT_NEAR(-0.1674075518520680f, v_out.z, FLT_EPSILON);

  EXPECT_NEAR(-0.3668628783845180f, v_out_ptr->x, FLT_EPSILON);
  EXPECT_NEAR(-0.3309484412483842f, v_out_ptr->y, FLT_EPSILON);
  EXPECT_NEAR(-0.1674075518520680f, v_out_ptr->z, FLT_EPSILON);
}

TEST(Vec3fCrossTest, Normal0) {
  const Vec3f v0 = {-0.2083752660290958f, -0.5824329527967755f,
                    -0.4510204716959005f};
  const Vec3f v1 = {0.1083629892868154f, -0.2094894230836941f,
                    -0.2308901701131918f};
  Vec3f v_out = {0.9077912846400851f, 0.8193497634327238f, 0.7946423012944603f};
  const Vec3f *v_out_ptr = Vec3fCross(&v0, &v1, &v_out);

  EXPECT_NEAR(0.0399940251362663f, v_out.x, FLT_EPSILON);
  EXPECT_NEAR(-0.0969857271633568f, v_out.y, FLT_EPSILON);
  EXPECT_NEAR(0.1067665900895518f, v_out.z, FLT_EPSILON);

  EXPECT_NEAR(0.0399940251362663f, v_out_ptr->x, FLT_EPSILON);
  EXPECT_NEAR(-0.0969857271633568f, v_out_ptr->y, FLT_EPSILON);
  EXPECT_NEAR(0.1067665900895518f, v_out_ptr->z, FLT_EPSILON);
}

TEST(Vec3fNormTest, Normal5) {
  const Vec3f v = {0.3280127653486116f, 0.2421450280486086f,
                   -0.8173297668101331f};
  double v_norm = Vec3fNorm(&v);

  EXPECT_NEAR(0.9133753535946826f, v_norm, FLT_EPSILON);
}

TEST(Vec3fNormBoundTest, Unbounded5) {
  const Vec3f v = {-0.3000657279204451f, 0.6160988521300801f,
                   -0.8841593808159371f};
  double v_norm = Vec3fNormBound(&v, 0.8752355704095247f);

  EXPECT_NEAR(1.1186398202072239f, v_norm, FLT_EPSILON);
}

TEST(Vec3fNormBoundTest, Bounded5) {
  const Vec3f v = {0.7593995167965277f, -0.9956466234278525f,
                   -0.9162006409318304f};
  double v_norm = Vec3fNormBound(&v, 2.0353017006257286f);

  EXPECT_NEAR(2.0353017006257286f, v_norm, FLT_EPSILON);
}

TEST(Vec3fNormSquaredTest, Normal5) {
  const Vec3f v = {-0.3451725884774099f, -0.6745536096950111f,
                   0.0326845696949909f};
  double v_norm_sqrd = Vec3fNormSquared(&v);

  EXPECT_NEAR(0.5752349692849116f, v_norm_sqrd, FLT_EPSILON);
}

TEST(Vec3fNormalizeTest, Normal5) {
  Vec3f v = {-0.5994888872635027f, -0.3502395159589060f, -0.6569427657571214f};
  const Vec3f *v_out_ptr = Vec3fNormalize(&v, &v);

  EXPECT_NEAR(-0.6271859652959697f, v.x, FLT_EPSILON);
  EXPECT_NEAR(-0.3664209855568628f, v.y, FLT_EPSILON);
  EXPECT_NEAR(-0.6872942792423781f, v.z, FLT_EPSILON);

  EXPECT_NEAR(-0.6271859652959697f, v_out_ptr->x, FLT_EPSILON);
  EXPECT_NEAR(-0.3664209855568628f, v_out_ptr->y, FLT_EPSILON);
  EXPECT_NEAR(-0.6872942792423781f, v_out_ptr->z, FLT_EPSILON);
}

#if defined(NDEBUG)
TEST(Vec3fNormalizeTest, Zero_5) {
  Vec3f v = {0.0000000000000000f, 0.0000000000000000f, 0.0000000000000000f};
  const Vec3f *v_out_ptr = Vec3fNormalize(&v, &v);

  EXPECT_NEAR(0.0000000000000000f, v.x, FLT_EPSILON);
  EXPECT_NEAR(0.0000000000000000f, v.y, FLT_EPSILON);
  EXPECT_NEAR(0.0000000000000000f, v.z, FLT_EPSILON);

  EXPECT_NEAR(0.0000000000000000f, v_out_ptr->x, FLT_EPSILON);
  EXPECT_NEAR(0.0000000000000000f, v_out_ptr->y, FLT_EPSILON);
  EXPECT_NEAR(0.0000000000000000f, v_out_ptr->z, FLT_EPSILON);
}
#endif  // defined(NDEBUG)

TEST(Vec3fScaleTest, Normal5) {
  const Vec3f v = {0.6247576085731952f, 0.7847415230573362f,
                   -0.0912514967997951f};
  Vec3f v_out = {0.4264724364750425f, 0.8409634250436200f, 0.2920849893201807f};
  const Vec3f *v_out_ptr = Vec3fScale(&v, -0.1418968624644088f, &v_out);

  EXPECT_NEAR(-0.0886511444573036f, v_out.x, FLT_EPSILON);
  EXPECT_NEAR(-0.1113523599673775f, v_out.y, FLT_EPSILON);
  EXPECT_NEAR(0.0129483010910720f, v_out.z, FLT_EPSILON);

  EXPECT_NEAR(-0.0886511444573036f, v_out_ptr->x, FLT_EPSILON);
  EXPECT_NEAR(-0.1113523599673775f, v_out_ptr->y, FLT_EPSILON);
  EXPECT_NEAR(0.0129483010910720f, v_out_ptr->z, FLT_EPSILON);
}

TEST(Vec3fAddTest, Normal5) {
  const Vec3f v0 = {-0.8499887665574646f, -0.5202824297132036f,
                    -0.2807972122668778f};
  const Vec3f v1 = {0.8958137842465088f, 0.5257801379118276f,
                    -0.8548859642364537f};
  Vec3f v_out = {-0.0744746064139561f, 0.0417691802188671f,
                 0.0052946333988333f};
  const Vec3f *v_out_ptr = Vec3fAdd(&v0, &v1, &v_out);

  EXPECT_NEAR(0.0458250176890442f, v_out.x, FLT_EPSILON);
  EXPECT_NEAR(0.0054977081986241f, v_out.y, FLT_EPSILON);
  EXPECT_NEAR(-1.1356831765033315f, v_out.z, FLT_EPSILON);

  EXPECT_NEAR(0.0458250176890442f, v_out_ptr->x, FLT_EPSILON);
  EXPECT_NEAR(0.0054977081986241f, v_out_ptr->y, FLT_EPSILON);
  EXPECT_NEAR(-1.1356831765033315f, v_out_ptr->z, FLT_EPSILON);
}

TEST(Vec3fAdd3Test, Normal5) {
  const Vec3f v0 = {-0.7930446418895178f, -0.9922740282210696f,
                    -0.7247845538551627f};
  const Vec3f v1 = {0.1395517240820978f, 0.8402973453306137f,
                    -0.9645953801617726f};
  const Vec3f v2 = {0.4949342401878045f, -0.1998357191849807f,
                    -0.4699342534129838f};
  Vec3f v0c = {-0.7930446418895178f, -0.9922740282210696f,
               -0.7247845538551627f};
  Vec3f v1c = {0.1395517240820978f, 0.8402973453306137f, -0.9645953801617726f};
  Vec3f v2c = {0.4949342401878045f, -0.1998357191849807f, -0.4699342534129838f};
  Vec3f v_out = {0.1206977849541890f, 0.7472340796806078f, 0.9513878870945605f};
  const Vec3f *v_out_ptr = Vec3fAdd3(&v0, &v1, &v2, &v_out);
  Vec3fAdd3(&v0c, &v1, &v2, &v0c);
  Vec3fAdd3(&v0, &v1c, &v2, &v1c);
  Vec3fAdd3(&v0, &v1, &v2c, &v2c);

  EXPECT_NEAR(-0.1585586776196155f, v_out.x, FLT_EPSILON);
  EXPECT_NEAR(-0.3518124020754365f, v_out.y, FLT_EPSILON);
  EXPECT_NEAR(-2.1593141874299189f, v_out.z, FLT_EPSILON);

  EXPECT_NEAR(-0.1585586776196155f, v0c.x, FLT_EPSILON);
  EXPECT_NEAR(-0.3518124020754365f, v0c.y, FLT_EPSILON);
  EXPECT_NEAR(-2.1593141874299189f, v0c.z, FLT_EPSILON);

  EXPECT_NEAR(-0.1585586776196155f, v1c.x, FLT_EPSILON);
  EXPECT_NEAR(-0.3518124020754365f, v1c.y, FLT_EPSILON);
  EXPECT_NEAR(-2.1593141874299189f, v1c.z, FLT_EPSILON);

  EXPECT_NEAR(-0.1585586776196155f, v2c.x, FLT_EPSILON);
  EXPECT_NEAR(-0.3518124020754365f, v2c.y, FLT_EPSILON);
  EXPECT_NEAR(-2.1593141874299189f, v2c.z, FLT_EPSILON);

  EXPECT_NEAR(-0.1585586776196155f, v_out_ptr->x, FLT_EPSILON);
  EXPECT_NEAR(-0.3518124020754365f, v_out_ptr->y, FLT_EPSILON);
  EXPECT_NEAR(-2.1593141874299189f, v_out_ptr->z, FLT_EPSILON);
}

TEST(Vec3fSubTest, Normal5) {
  const Vec3f v0 = {-0.6650133458925320f, -0.7138954511649391f,
                    -0.6193888143977264f};
  const Vec3f v1 = {-0.9240789349889980f, 0.4744553131060507f,
                    -0.1040192768909076f};
  Vec3f v_out = {0.1807143485814426f, 0.9831779885001866f,
                 -0.5293606931514121f};
  const Vec3f *v_out_ptr = Vec3fSub(&v0, &v1, &v_out);

  EXPECT_NEAR(0.2590655890964659f, v_out.x, FLT_EPSILON);
  EXPECT_NEAR(-1.1883507642709898f, v_out.y, FLT_EPSILON);
  EXPECT_NEAR(-0.5153695375068188f, v_out.z, FLT_EPSILON);

  EXPECT_NEAR(0.2590655890964659f, v_out_ptr->x, FLT_EPSILON);
  EXPECT_NEAR(-1.1883507642709898f, v_out_ptr->y, FLT_EPSILON);
  EXPECT_NEAR(-0.5153695375068188f, v_out_ptr->z, FLT_EPSILON);
}

TEST(Vec3fLinCombTest, Normal5) {
  const Vec3f v0 = {-0.0685294097150932f, -0.1751244364420363f,
                    -0.2245181368994498f};
  const Vec3f v1 = {0.4695557000508626f, -0.0868949598138307f,
                    0.2741851546438758f};
  Vec3f v_out = {-0.9395537125857720f, -0.6254790565049011f,
                 -0.6925528939466816f};
  const Vec3f *v_out_ptr =
      Vec3fLinComb(0.5355639112504176f, &v0, 0.6849658495554161f, &v1, &v_out);

  EXPECT_NEAR(0.2849277402962295f, v_out.x, FLT_EPSILON);
  EXPECT_NEAR(-0.1533104081073864f, v_out.y, FLT_EPSILON);
  EXPECT_NEAR(0.0675636558415995f, v_out.z, FLT_EPSILON);

  EXPECT_NEAR(0.2849277402962295f, v_out_ptr->x, FLT_EPSILON);
  EXPECT_NEAR(-0.1533104081073864f, v_out_ptr->y, FLT_EPSILON);
  EXPECT_NEAR(0.0675636558415995f, v_out_ptr->z, FLT_EPSILON);
}

TEST(Vec3fLinComb3Test, Normal5) {
  const Vec3f v0 = {-0.0939619143279544f, 0.2362686642867331f,
                    0.7220377453598541f};
  const Vec3f v1 = {0.6693452098786703f, -0.9853276874000878f,
                    0.7622255225662449f};
  const Vec3f v2 = {0.0281595156522720f, 0.9990653638489897f,
                    -0.5046935873378480f};
  Vec3f v_out = {-0.3570600089555287f, -0.9197891732707526f,
                 -0.2926810630752030f};
  const Vec3f *v_out_ptr =
      Vec3fLinComb3(0.3598048816818096f, &v0, 0.8834105783461450f, &v1,
                    -0.3419938976526125f, &v2, &v_out);

  EXPECT_NEAR(0.5478683009908418f, v_out.x, FLT_EPSILON);
  EXPECT_NEAR(-1.1271125411802152f, v_out.y, FLT_EPSILON);
  EXPECT_NEAR(1.1057529223133915f, v_out.z, FLT_EPSILON);

  EXPECT_NEAR(0.5478683009908418f, v_out_ptr->x, FLT_EPSILON);
  EXPECT_NEAR(-1.1271125411802152f, v_out_ptr->y, FLT_EPSILON);
  EXPECT_NEAR(1.1057529223133915f, v_out_ptr->z, FLT_EPSILON);
}

TEST(Vec3fDotTest, Normal5) {
  const Vec3f v0 = {-0.9620382927787867f, 0.9205105525834723f,
                    0.2463128629761429f};
  const Vec3f v1 = {-0.2704520951296421f, 0.7260433533652026f,
                    0.5195421882627813f};
  double v_dot = Vec3fDot(&v0, &v1);

  EXPECT_NEAR(1.0564857641106227f, v_dot, FLT_EPSILON);
}

TEST(Vec3fMultTest, Normal5) {
  const Vec3f v0 = {-0.1838588164901567f, -0.1075940672017186f,
                    -0.0282362842439785f};
  const Vec3f v1 = {0.2920674378173924f, 0.9058862428745307f,
                    0.2552559461597399f};
  Vec3f v_out = {-0.0444558939862263f, 0.7298459177794947f,
                 0.1010724084258454f};
  const Vec3f *v_out_ptr = Vec3fMult(&v0, &v1, &v_out);

  EXPECT_NEAR(-0.0536991734524182f, v_out.x, FLT_EPSILON);
  EXPECT_NEAR(-0.0974679852929546f, v_out.y, FLT_EPSILON);
  EXPECT_NEAR(-0.0072074794507321f, v_out.z, FLT_EPSILON);

  EXPECT_NEAR(-0.0536991734524182f, v_out_ptr->x, FLT_EPSILON);
  EXPECT_NEAR(-0.0974679852929546f, v_out_ptr->y, FLT_EPSILON);
  EXPECT_NEAR(-0.0072074794507321f, v_out_ptr->z, FLT_EPSILON);
}

TEST(Vec3fCrossTest, Normal1) {
  const Vec3f v0 = {-0.6764992137955061f, -0.9146635028029626f,
                    0.4607022747071750f};
  const Vec3f v1 = {0.2214359324988560f, -0.1805957517930052f,
                    0.4800794112013538f};
  Vec3f v_out = {0.4401456638848431f, 0.9495624845262263f, 0.1087389732398125f};
  const Vec3f *v_out_ptr = Vec3fCross(&v0, &v1, &v_out);

  EXPECT_NEAR(-0.3559102422195242f, v_out.x, FLT_EPSILON);
  EXPECT_NEAR(0.4267893820412527f, v_out.y, FLT_EPSILON);
  EXPECT_NEAR(0.3247122497686204f, v_out.z, FLT_EPSILON);

  EXPECT_NEAR(-0.3559102422195242f, v_out_ptr->x, FLT_EPSILON);
  EXPECT_NEAR(0.4267893820412527f, v_out_ptr->y, FLT_EPSILON);
  EXPECT_NEAR(0.3247122497686204f, v_out_ptr->z, FLT_EPSILON);
}

TEST(Vec3fNormTest, Normal9) {
  const Vec3f v = {0.5808260832296135f, 0.5922695614036191f,
                   -0.1694815147788191f};
  double v_norm = Vec3fNorm(&v);

  EXPECT_NEAR(0.8466794884587746f, v_norm, FLT_EPSILON);
}

TEST(Vec3fNormBoundTest, Unbounded9) {
  const Vec3f v = {0.9316956534433496f, 0.1787929162952937f,
                   0.0599260293110035f};
  double v_norm = Vec3fNormBound(&v, 0.3313565627667132f);

  EXPECT_NEAR(0.9505865697302847f, v_norm, FLT_EPSILON);
}

TEST(Vec3fNormBoundTest, Bounded9) {
  const Vec3f v = {0.1467027610475362f, -0.4714413318389643f,
                   -0.7572528793440667f};
  double v_norm = Vec3fNormBound(&v, 1.3554532671751538f);

  EXPECT_NEAR(1.3554532671751538f, v_norm, FLT_EPSILON);
}

TEST(Vec3fNormSquaredTest, Normal9) {
  const Vec3f v = {0.4009023052419520f, 0.3314888769839504f,
                   -0.1469763079001782f};
  double v_norm_sqrd = Vec3fNormSquared(&v);

  EXPECT_NEAR(0.2922095689963599f, v_norm_sqrd, FLT_EPSILON);
}

TEST(Vec3fNormalizeTest, Normal9) {
  Vec3f v = {0.2766904485844199f, 0.6403886563647754f, 0.1847774110244784f};
  const Vec3f *v_out_ptr = Vec3fNormalize(&v, &v);

  EXPECT_NEAR(0.3834065641149065f, v.x, FLT_EPSILON);
  EXPECT_NEAR(0.8873787139785118f, v.y, FLT_EPSILON);
  EXPECT_NEAR(0.2560437942451343f, v.z, FLT_EPSILON);

  EXPECT_NEAR(0.3834065641149065f, v_out_ptr->x, FLT_EPSILON);
  EXPECT_NEAR(0.8873787139785118f, v_out_ptr->y, FLT_EPSILON);
  EXPECT_NEAR(0.2560437942451343f, v_out_ptr->z, FLT_EPSILON);
}

#if defined(NDEBUG)
TEST(Vec3fNormalizeTest, Zero_9) {
  Vec3f v = {0.0000000000000000f, 0.0000000000000000f, 0.0000000000000000f};
  const Vec3f *v_out_ptr = Vec3fNormalize(&v, &v);

  EXPECT_NEAR(0.0000000000000000f, v.x, FLT_EPSILON);
  EXPECT_NEAR(0.0000000000000000f, v.y, FLT_EPSILON);
  EXPECT_NEAR(0.0000000000000000f, v.z, FLT_EPSILON);

  EXPECT_NEAR(0.0000000000000000f, v_out_ptr->x, FLT_EPSILON);
  EXPECT_NEAR(0.0000000000000000f, v_out_ptr->y, FLT_EPSILON);
  EXPECT_NEAR(0.0000000000000000f, v_out_ptr->z, FLT_EPSILON);
}
#endif  // defined(NDEBUG)

TEST(Vec3fScaleTest, Normal9) {
  const Vec3f v = {0.1564175171363360f, -0.5928003064979888f,
                   -0.7251831432396201f};
  Vec3f v_out = {0.0161593596423752f, 0.5658506405701715f,
                 -0.7104761965558826f};
  const Vec3f *v_out_ptr = Vec3fScale(&v, -0.9720439053818959f, &v_out);

  EXPECT_NEAR(-0.1520446942273437f, v_out.x, FLT_EPSILON);
  EXPECT_NEAR(0.5762279250398898f, v_out.y, FLT_EPSILON);
  EXPECT_NEAR(0.7049098546717592f, v_out.z, FLT_EPSILON);

  EXPECT_NEAR(-0.1520446942273437f, v_out_ptr->x, FLT_EPSILON);
  EXPECT_NEAR(0.5762279250398898f, v_out_ptr->y, FLT_EPSILON);
  EXPECT_NEAR(0.7049098546717592f, v_out_ptr->z, FLT_EPSILON);
}

TEST(Vec3fAddTest, Normal9) {
  const Vec3f v0 = {-0.1874458060054163f, 0.2609798151088987f,
                    -0.8520759883002875f};
  const Vec3f v1 = {0.0970432591201233f, 0.8304779456499265f,
                    -0.6511301867613077f};
  Vec3f v_out = {0.7865124802218066f, 0.5713707031423259f,
                 -0.5767817461730829f};
  const Vec3f *v_out_ptr = Vec3fAdd(&v0, &v1, &v_out);

  EXPECT_NEAR(-0.0904025468852929f, v_out.x, FLT_EPSILON);
  EXPECT_NEAR(1.0914577607588252f, v_out.y, FLT_EPSILON);
  EXPECT_NEAR(-1.5032061750615953f, v_out.z, FLT_EPSILON);

  EXPECT_NEAR(-0.0904025468852929f, v_out_ptr->x, FLT_EPSILON);
  EXPECT_NEAR(1.0914577607588252f, v_out_ptr->y, FLT_EPSILON);
  EXPECT_NEAR(-1.5032061750615953f, v_out_ptr->z, FLT_EPSILON);
}

TEST(Vec3fAdd3Test, Normal9) {
  const Vec3f v0 = {0.8693297264443576f, 0.9610149363578462f,
                    0.6551867685579851f};
  const Vec3f v1 = {-0.4887371184993261f, 0.4517688907210851f,
                    0.5059698789610241f};
  const Vec3f v2 = {-0.1800884707233683f, 0.4341860076266801f,
                    0.7087599202489370f};
  Vec3f v0c = {0.8693297264443576f, 0.9610149363578462f, 0.6551867685579851f};
  Vec3f v1c = {-0.4887371184993261f, 0.4517688907210851f, 0.5059698789610241f};
  Vec3f v2c = {-0.1800884707233683f, 0.4341860076266801f, 0.7087599202489370f};
  Vec3f v_out = {0.5754014248614572f, -0.6465311596291174f,
                 -0.4995093173608245f};
  const Vec3f *v_out_ptr = Vec3fAdd3(&v0, &v1, &v2, &v_out);
  Vec3fAdd3(&v0c, &v1, &v2, &v0c);
  Vec3fAdd3(&v0, &v1c, &v2, &v1c);
  Vec3fAdd3(&v0, &v1, &v2c, &v2c);

  EXPECT_NEAR(0.2005041372216632f, v_out.x, FLT_EPSILON);
  EXPECT_NEAR(1.8469698347056114f, v_out.y, FLT_EPSILON);
  EXPECT_NEAR(1.8699165677679461f, v_out.z, FLT_EPSILON);

  EXPECT_NEAR(0.2005041372216632f, v0c.x, FLT_EPSILON);
  EXPECT_NEAR(1.8469698347056114f, v0c.y, FLT_EPSILON);
  EXPECT_NEAR(1.8699165677679461f, v0c.z, FLT_EPSILON);

  EXPECT_NEAR(0.2005041372216632f, v1c.x, FLT_EPSILON);
  EXPECT_NEAR(1.8469698347056114f, v1c.y, FLT_EPSILON);
  EXPECT_NEAR(1.8699165677679461f, v1c.z, FLT_EPSILON);

  EXPECT_NEAR(0.2005041372216632f, v2c.x, FLT_EPSILON);
  EXPECT_NEAR(1.8469698347056114f, v2c.y, FLT_EPSILON);
  EXPECT_NEAR(1.8699165677679461f, v2c.z, FLT_EPSILON);

  EXPECT_NEAR(0.2005041372216632f, v_out_ptr->x, FLT_EPSILON);
  EXPECT_NEAR(1.8469698347056114f, v_out_ptr->y, FLT_EPSILON);
  EXPECT_NEAR(1.8699165677679461f, v_out_ptr->z, FLT_EPSILON);
}

TEST(Vec3fSubTest, Normal9) {
  const Vec3f v0 = {-0.2283674493043861f, -0.9014328921117525f,
                    -0.6363353672373084f};
  const Vec3f v1 = {-0.0143925514381493f, 0.3383132223497405f,
                    -0.6742920168150697f};
  Vec3f v_out = {0.9402371365747340f, 0.2139969018476338f, 0.2552918238895079f};
  const Vec3f *v_out_ptr = Vec3fSub(&v0, &v1, &v_out);

  EXPECT_NEAR(-0.2139748978662368f, v_out.x, FLT_EPSILON);
  EXPECT_NEAR(-1.2397461144614930f, v_out.y, FLT_EPSILON);
  EXPECT_NEAR(0.0379566495777612f, v_out.z, FLT_EPSILON);

  EXPECT_NEAR(-0.2139748978662368f, v_out_ptr->x, FLT_EPSILON);
  EXPECT_NEAR(-1.2397461144614930f, v_out_ptr->y, FLT_EPSILON);
  EXPECT_NEAR(0.0379566495777612f, v_out_ptr->z, FLT_EPSILON);
}

TEST(Vec3fLinCombTest, Normal9) {
  const Vec3f v0 = {0.6550471116645695f, -0.2153879652204482f,
                    -0.8835693971435579f};
  const Vec3f v1 = {0.9972032503337704f, -0.0601914771464085f,
                    -0.4320440353170660f};
  Vec3f v_out = {-0.9138865362858917f, -0.7334554478756057f,
                 0.7317667274237456f};
  const Vec3f *v_out_ptr =
      Vec3fLinComb(0.6365223438594392f, &v0, 0.1843987218984677f, &v1, &v_out);

  EXPECT_NEAR(0.6008351276896327f, v_out.x, FLT_EPSILON);
  EXPECT_NEAR(-0.1481984839162137f, v_out.y, FLT_EPSILON);
  EXPECT_NEAR(-0.6420800315486126f, v_out.z, FLT_EPSILON);

  EXPECT_NEAR(0.6008351276896327f, v_out_ptr->x, FLT_EPSILON);
  EXPECT_NEAR(-0.1481984839162137f, v_out_ptr->y, FLT_EPSILON);
  EXPECT_NEAR(-0.6420800315486126f, v_out_ptr->z, FLT_EPSILON);
}

TEST(Vec3fLinComb3Test, Normal9) {
  const Vec3f v0 = {0.2721656980963250f, 0.7439043767825189f,
                    -0.6606313034653339f};
  const Vec3f v1 = {0.3030186261976140f, -0.6539574477087946f,
                    -0.8155949803008553f};
  const Vec3f v2 = {-0.9834181291663000f, 0.2409230887886793f,
                    -0.4666921731161049f};
  Vec3f v_out = {0.5403502815697727f, 0.4690281493074104f, 0.8982080003084418f};
  const Vec3f *v_out_ptr =
      Vec3fLinComb3(0.6719751290823228f, &v0, -0.7093069530347784f, &v1,
                    -0.7307597956787206f, &v2, &v_out);

  EXPECT_NEAR(0.6865977927853595f, v_out.x, FLT_EPSILON);
  EXPECT_NEAR(0.7876848971245620f, v_out.y, FLT_EPSILON);
  EXPECT_NEAR(0.4756192620369019f, v_out.z, FLT_EPSILON);

  EXPECT_NEAR(0.6865977927853595f, v_out_ptr->x, FLT_EPSILON);
  EXPECT_NEAR(0.7876848971245620f, v_out_ptr->y, FLT_EPSILON);
  EXPECT_NEAR(0.4756192620369019f, v_out_ptr->z, FLT_EPSILON);
}

TEST(Vec3fDotTest, Normal9) {
  const Vec3f v0 = {-0.8134148512138326f, -0.2085083380971571f,
                    0.2476157835112773f};
  const Vec3f v1 = {-0.3982153847348187f, -0.9322421131796628f,
                    0.6479840620617590f};
  double v_dot = Vec3fDot(&v0, &v1);

  EXPECT_NEAR(0.6787456428786476f, v_dot, FLT_EPSILON);
}

TEST(Vec3fMultTest, Normal9) {
  const Vec3f v0 = {0.6176037319138981f, -0.9824204181640221f,
                    0.6700402073758693f};
  const Vec3f v1 = {0.9038076294910025f, -0.6015116040562272f,
                    -0.4133818335289658f};
  Vec3f v_out = {0.1602431564312929f, 0.8822212476536690f, 0.6521965665060454f};
  const Vec3f *v_out_ptr = Vec3fMult(&v0, &v1, &v_out);

  EXPECT_NEAR(0.5581949649058968f, v_out.x, FLT_EPSILON);
  EXPECT_NEAR(0.5909372815874304f, v_out.y, FLT_EPSILON);
  EXPECT_NEAR(-0.2769824494631654f, v_out.z, FLT_EPSILON);

  EXPECT_NEAR(0.5581949649058968f, v_out_ptr->x, FLT_EPSILON);
  EXPECT_NEAR(0.5909372815874304f, v_out_ptr->y, FLT_EPSILON);
  EXPECT_NEAR(-0.2769824494631654f, v_out_ptr->z, FLT_EPSILON);
}

TEST(Vec3fCrossTest, Normal2) {
  const Vec3f v0 = {-0.8682328113783639f, -0.6632068278920162f,
                    0.4973950204357509f};
  const Vec3f v1 = {0.5811618559965335f, 0.5025464754438578f,
                    0.2329020791955703f};
  Vec3f v_out = {0.8614810073566896f, -0.7688544844599099f,
                 -0.8415814650412219f};
  const Vec3f *v_out_ptr = Vec3fCross(&v0, &v1, &v_out);

  EXPECT_NEAR(-0.4044263635760615f, v_out.x, FLT_EPSILON);
  EXPECT_NEAR(0.4912802402357110f, v_out.y, FLT_EPSILON);
  EXPECT_NEAR(-0.0508968282156109f, v_out.z, FLT_EPSILON);

  EXPECT_NEAR(-0.4044263635760615f, v_out_ptr->x, FLT_EPSILON);
  EXPECT_NEAR(0.4912802402357110f, v_out_ptr->y, FLT_EPSILON);
  EXPECT_NEAR(-0.0508968282156109f, v_out_ptr->z, FLT_EPSILON);
}

TEST(Vec3fNormTest, Normal13) {
  const Vec3f v = {0.1089781761285467f, -0.9136381466688563f,
                   -0.8993518947221482f};
  double v_norm = Vec3fNorm(&v);

  EXPECT_NEAR(1.2866408731503616f, v_norm, FLT_EPSILON);
}

TEST(Vec3fNormBoundTest, Unbounded13) {
  const Vec3f v = {-0.1477720297813589f, 0.3805663847744476f,
                   -0.9572246478522421f};
  double v_norm = Vec3fNormBound(&v, 0.9604937775004271f);

  EXPECT_NEAR(1.0406470931405347f, v_norm, FLT_EPSILON);
}

TEST(Vec3fNormBoundTest, Bounded13) {
  const Vec3f v = {0.8501942861670015f, -0.7559542213691750f,
                   -0.5031235706295509f};
  double v_norm = Vec3fNormBound(&v, 1.5509570924415381f);

  EXPECT_NEAR(1.5509570924415381f, v_norm, FLT_EPSILON);
}

TEST(Vec3fNormSquaredTest, Normal13) {
  const Vec3f v = {0.7884754816377799f, -0.8385943227016530f,
                   -0.8342250138569103f};
  double v_norm_sqrd = Vec3fNormSquared(&v);

  EXPECT_NEAR(2.0208653969559354f, v_norm_sqrd, FLT_EPSILON);
}

TEST(Vec3fNormalizeTest, Normal13) {
  Vec3f v = {-0.9427571104185044f, -0.4860656308265348f, -0.2323228040170791f};
  const Vec3f *v_out_ptr = Vec3fNormalize(&v, &v);

  EXPECT_NEAR(-0.8682371685863967f, v.x, FLT_EPSILON);
  EXPECT_NEAR(-0.4476447245978872f, v.y, FLT_EPSILON);
  EXPECT_NEAR(-0.2139589204140803f, v.z, FLT_EPSILON);

  EXPECT_NEAR(-0.8682371685863967f, v_out_ptr->x, FLT_EPSILON);
  EXPECT_NEAR(-0.4476447245978872f, v_out_ptr->y, FLT_EPSILON);
  EXPECT_NEAR(-0.2139589204140803f, v_out_ptr->z, FLT_EPSILON);
}

#if defined(NDEBUG)
TEST(Vec3fNormalizeTest, Zero_13) {
  Vec3f v = {0.0000000000000000f, 0.0000000000000000f, 0.0000000000000000f};
  const Vec3f *v_out_ptr = Vec3fNormalize(&v, &v);

  EXPECT_NEAR(0.0000000000000000f, v.x, FLT_EPSILON);
  EXPECT_NEAR(0.0000000000000000f, v.y, FLT_EPSILON);
  EXPECT_NEAR(0.0000000000000000f, v.z, FLT_EPSILON);

  EXPECT_NEAR(0.0000000000000000f, v_out_ptr->x, FLT_EPSILON);
  EXPECT_NEAR(0.0000000000000000f, v_out_ptr->y, FLT_EPSILON);
  EXPECT_NEAR(0.0000000000000000f, v_out_ptr->z, FLT_EPSILON);
}
#endif  // defined(NDEBUG)

TEST(Vec3fScaleTest, Normal13) {
  const Vec3f v = {0.0596802821329565f, 0.9066534966906223f,
                   -0.1090525658216739f};
  Vec3f v_out = {0.6059729313853439f, 0.8399363862432556f,
                 -0.8703496009694609f};
  const Vec3f *v_out_ptr = Vec3fScale(&v, -0.6228805491712335f, &v_out);

  EXPECT_NEAR(-0.0371736869096701f, v_out.x, FLT_EPSILON);
  EXPECT_NEAR(-0.5647368279266740f, v_out.y, FLT_EPSILON);
  EXPECT_NEAR(0.0679267220875363f, v_out.z, FLT_EPSILON);

  EXPECT_NEAR(-0.0371736869096701f, v_out_ptr->x, FLT_EPSILON);
  EXPECT_NEAR(-0.5647368279266740f, v_out_ptr->y, FLT_EPSILON);
  EXPECT_NEAR(0.0679267220875363f, v_out_ptr->z, FLT_EPSILON);
}

TEST(Vec3fAddTest, Normal13) {
  const Vec3f v0 = {0.2836526783851223f, 0.8315315747116210f,
                    0.7841935311427746f};
  const Vec3f v1 = {0.7399198885757061f, -0.1245570166161289f,
                    0.2591704104448207f};
  Vec3f v_out = {-0.9902615957976295f, 0.5403371702743311f,
                 0.0783200347887563f};
  const Vec3f *v_out_ptr = Vec3fAdd(&v0, &v1, &v_out);

  EXPECT_NEAR(1.0235725669608284f, v_out.x, FLT_EPSILON);
  EXPECT_NEAR(0.7069745580954920f, v_out.y, FLT_EPSILON);
  EXPECT_NEAR(1.0433639415875953f, v_out.z, FLT_EPSILON);

  EXPECT_NEAR(1.0235725669608284f, v_out_ptr->x, FLT_EPSILON);
  EXPECT_NEAR(0.7069745580954920f, v_out_ptr->y, FLT_EPSILON);
  EXPECT_NEAR(1.0433639415875953f, v_out_ptr->z, FLT_EPSILON);
}

TEST(Vec3fAdd3Test, Normal13) {
  const Vec3f v0 = {0.9238706464398476f, 0.3438554757791441f,
                    -0.6071685139577589f};
  const Vec3f v1 = {-0.8941590063985037f, 0.7140891202658142f,
                    0.6435182537637201f};
  const Vec3f v2 = {0.8175935979005813f, -0.7410745640869674f,
                    0.3949477137278348f};
  Vec3f v0c = {0.9238706464398476f, 0.3438554757791441f, -0.6071685139577589f};
  Vec3f v1c = {-0.8941590063985037f, 0.7140891202658142f, 0.6435182537637201f};
  Vec3f v2c = {0.8175935979005813f, -0.7410745640869674f, 0.3949477137278348f};
  Vec3f v_out = {-0.4365920509928007f, -0.1954229708614825f,
                 -0.4937690311824776f};
  const Vec3f *v_out_ptr = Vec3fAdd3(&v0, &v1, &v2, &v_out);
  Vec3fAdd3(&v0c, &v1, &v2, &v0c);
  Vec3fAdd3(&v0, &v1c, &v2, &v1c);
  Vec3fAdd3(&v0, &v1, &v2c, &v2c);

  EXPECT_NEAR(0.8473052379419252f, v_out.x, FLT_EPSILON);
  EXPECT_NEAR(0.3168700319579909f, v_out.y, FLT_EPSILON);
  EXPECT_NEAR(0.4312974535337959f, v_out.z, FLT_EPSILON);

  EXPECT_NEAR(0.8473052379419252f, v0c.x, FLT_EPSILON);
  EXPECT_NEAR(0.3168700319579909f, v0c.y, FLT_EPSILON);
  EXPECT_NEAR(0.4312974535337959f, v0c.z, FLT_EPSILON);

  EXPECT_NEAR(0.8473052379419252f, v1c.x, FLT_EPSILON);
  EXPECT_NEAR(0.3168700319579909f, v1c.y, FLT_EPSILON);
  EXPECT_NEAR(0.4312974535337959f, v1c.z, FLT_EPSILON);

  EXPECT_NEAR(0.8473052379419252f, v2c.x, FLT_EPSILON);
  EXPECT_NEAR(0.3168700319579909f, v2c.y, FLT_EPSILON);
  EXPECT_NEAR(0.4312974535337959f, v2c.z, FLT_EPSILON);

  EXPECT_NEAR(0.8473052379419252f, v_out_ptr->x, FLT_EPSILON);
  EXPECT_NEAR(0.3168700319579909f, v_out_ptr->y, FLT_EPSILON);
  EXPECT_NEAR(0.4312974535337959f, v_out_ptr->z, FLT_EPSILON);
}

TEST(Vec3fSubTest, Normal13) {
  const Vec3f v0 = {0.8928540403091885f, 0.2899643959846412f,
                    0.6938761626236207f};
  const Vec3f v1 = {-0.5282575035720876f, -0.2511046824900267f,
                    0.8416000815962716f};
  Vec3f v_out = {0.0278196325456206f, -0.7299743947211050f,
                 -0.5716453655670475f};
  const Vec3f *v_out_ptr = Vec3fSub(&v0, &v1, &v_out);

  EXPECT_NEAR(1.4211115438812760f, v_out.x, FLT_EPSILON);
  EXPECT_NEAR(0.5410690784746679f, v_out.y, FLT_EPSILON);
  EXPECT_NEAR(-0.1477239189726509f, v_out.z, FLT_EPSILON);

  EXPECT_NEAR(1.4211115438812760f, v_out_ptr->x, FLT_EPSILON);
  EXPECT_NEAR(0.5410690784746679f, v_out_ptr->y, FLT_EPSILON);
  EXPECT_NEAR(-0.1477239189726509f, v_out_ptr->z, FLT_EPSILON);
}

TEST(Vec3fLinCombTest, Normal13) {
  const Vec3f v0 = {-0.4656397750996888f, -0.0899998838278602f,
                    0.3610148213185371f};
  const Vec3f v1 = {0.6614405432895130f, -0.5761754167998201f,
                    0.5731031455888651f};
  Vec3f v_out = {0.4545933240274511f, -0.6682646711062756f,
                 0.0236357642928082f};
  const Vec3f *v_out_ptr = Vec3fLinComb(-0.1337137710719969f, &v0,
                                        -0.8846217415542821f, &v1, &v_out);

  EXPECT_NEAR(-0.5228622350496835f, v_out.x, FLT_EPSILON);
  EXPECT_NEAR(0.5217315245128861f, v_out.y, FLT_EPSILON);
  EXPECT_NEAR(-0.5552521559124438f, v_out.z, FLT_EPSILON);

  EXPECT_NEAR(-0.5228622350496835f, v_out_ptr->x, FLT_EPSILON);
  EXPECT_NEAR(0.5217315245128861f, v_out_ptr->y, FLT_EPSILON);
  EXPECT_NEAR(-0.5552521559124438f, v_out_ptr->z, FLT_EPSILON);
}

TEST(Vec3fLinComb3Test, Normal13) {
  const Vec3f v0 = {0.3698081393521835f, 0.6078463556261542f,
                    -0.1225246868668557f};
  const Vec3f v1 = {0.1181868381870310f, -0.8306047589279235f,
                    0.6201096518544660f};
  const Vec3f v2 = {-0.5178480679588717f, -0.2474484523805625f,
                    0.4203620298524120f};
  Vec3f v_out = {-0.4925549380582988f, -0.4646545685677774f,
                 0.5583185003984297f};
  const Vec3f *v_out_ptr =
      Vec3fLinComb3(-0.1197758224415517f, &v0, 0.1739711626689577f, &v1,
                    0.5229317255640360f, &v2, &v_out);

  EXPECT_NEAR(-0.2945321561426570f, v_out.x, FLT_EPSILON);
  EXPECT_NEAR(-0.3467052189838005f, v_out.y, FLT_EPSILON);
  EXPECT_NEAR(0.3423773338865579f, v_out.z, FLT_EPSILON);

  EXPECT_NEAR(-0.2945321561426570f, v_out_ptr->x, FLT_EPSILON);
  EXPECT_NEAR(-0.3467052189838005f, v_out_ptr->y, FLT_EPSILON);
  EXPECT_NEAR(0.3423773338865579f, v_out_ptr->z, FLT_EPSILON);
}

TEST(Vec3fDotTest, Normal13) {
  const Vec3f v0 = {0.1474130676718000f, 0.8087665856059540f,
                    0.8014935200996285f};
  const Vec3f v1 = {-0.4542791519426888f, -0.9806813673402233f,
                    0.7290508449970545f};
  double v_dot = Vec3fDot(&v0, &v1);

  EXPECT_NEAR(-0.2757794763100483f, v_dot, FLT_EPSILON);
}

TEST(Vec3fMultTest, Normal13) {
  const Vec3f v0 = {-0.6613701546518296f, 0.0575800694754700f,
                    0.3037824685912978f};
  const Vec3f v1 = {-0.6405349916024616f, -0.0634890376584154f,
                    0.0881922309829326f};
  Vec3f v_out = {0.3993123419317641f, 0.3408205787925083f, 0.2448795173715939f};
  const Vec3f *v_out_ptr = Vec3fMult(&v0, &v1, &v_out);

  EXPECT_NEAR(0.4236307264560284f, v_out.x, FLT_EPSILON);
  EXPECT_NEAR(-0.0036557031993023f, v_out.y, FLT_EPSILON);
  EXPECT_NEAR(0.0267912536385692f, v_out.z, FLT_EPSILON);

  EXPECT_NEAR(0.4236307264560284f, v_out_ptr->x, FLT_EPSILON);
  EXPECT_NEAR(-0.0036557031993023f, v_out_ptr->y, FLT_EPSILON);
  EXPECT_NEAR(0.0267912536385692f, v_out_ptr->z, FLT_EPSILON);
}

TEST(Vec3fCrossTest, Normal3) {
  const Vec3f v0 = {0.3951306207190080f, -0.8786709319178831f,
                    0.0098176906855341f};
  const Vec3f v1 = {0.5927897212175395f, 0.0057579721354335f,
                    0.7165869499815805f};
  Vec3f v_out = {-0.5944968273433429f, -0.0007433188229025f,
                 0.3673608024588699f};
  const Vec3f *v_out_ptr = Vec3fCross(&v0, &v1, &v_out);

  EXPECT_NEAR(-0.6297006531299104f, v_out.x, FLT_EPSILON);
  EXPECT_NEAR(-0.2773256202208848f, v_out.y, FLT_EPSILON);
  EXPECT_NEAR(0.5231422478775142f, v_out.z, FLT_EPSILON);

  EXPECT_NEAR(-0.6297006531299104f, v_out_ptr->x, FLT_EPSILON);
  EXPECT_NEAR(-0.2773256202208848f, v_out_ptr->y, FLT_EPSILON);
  EXPECT_NEAR(0.5231422478775142f, v_out_ptr->z, FLT_EPSILON);
}

TEST(Vec3fNormTest, Normal17) {
  const Vec3f v = {0.0309435440394410f, -0.2110076214913605f,
                   -0.1052672490121829f};
  double v_norm = Vec3fNorm(&v);

  EXPECT_NEAR(0.2378295880662350f, v_norm, FLT_EPSILON);
}

TEST(Vec3fNormBoundTest, Unbounded17) {
  const Vec3f v = {0.1039707708752737f, -0.3727048007412765f,
                   -0.6046493500070114f};
  double v_norm = Vec3fNormBound(&v, 0.5085926469314221f);

  EXPECT_NEAR(0.7178576642732838f, v_norm, FLT_EPSILON);
}

TEST(Vec3fNormBoundTest, Bounded17) {
  const Vec3f v = {0.0427401484746950f, 0.5888597611281718f,
                   0.5448131624545518f};
  double v_norm = Vec3fNormBound(&v, 1.5280693955873637f);

  EXPECT_NEAR(1.5280693955873637f, v_norm, FLT_EPSILON);
}

TEST(Vec3fNormSquaredTest, Normal17) {
  const Vec3f v = {0.1819191820018540f, -0.7811024063100254f,
                   0.5492200909941696f};
  double v_norm_sqrd = Vec3fNormSquared(&v);

  EXPECT_NEAR(0.9448582662751795f, v_norm_sqrd, FLT_EPSILON);
}

TEST(Vec3fNormalizeTest, Normal17) {
  Vec3f v = {0.9397100769790467f, 0.3482193003095639f, -0.9878428268501911f};
  const Vec3f *v_out_ptr = Vec3fNormalize(&v, &v);

  EXPECT_NEAR(0.6677983931066236f, v.x, FLT_EPSILON);
  EXPECT_NEAR(0.2474596100352606f, v.y, FLT_EPSILON);
  EXPECT_NEAR(-0.7020035951228514f, v.z, FLT_EPSILON);

  EXPECT_NEAR(0.6677983931066236f, v_out_ptr->x, FLT_EPSILON);
  EXPECT_NEAR(0.2474596100352606f, v_out_ptr->y, FLT_EPSILON);
  EXPECT_NEAR(-0.7020035951228514f, v_out_ptr->z, FLT_EPSILON);
}

#if defined(NDEBUG)
TEST(Vec3fNormalizeTest, Zero_17) {
  Vec3f v = {0.0000000000000000f, 0.0000000000000000f, 0.0000000000000000f};
  const Vec3f *v_out_ptr = Vec3fNormalize(&v, &v);

  EXPECT_NEAR(0.0000000000000000f, v.x, FLT_EPSILON);
  EXPECT_NEAR(0.0000000000000000f, v.y, FLT_EPSILON);
  EXPECT_NEAR(0.0000000000000000f, v.z, FLT_EPSILON);

  EXPECT_NEAR(0.0000000000000000f, v_out_ptr->x, FLT_EPSILON);
  EXPECT_NEAR(0.0000000000000000f, v_out_ptr->y, FLT_EPSILON);
  EXPECT_NEAR(0.0000000000000000f, v_out_ptr->z, FLT_EPSILON);
}
#endif  // defined(NDEBUG)

TEST(Vec3fScaleTest, Normal17) {
  const Vec3f v = {-0.8603480877491050f, -0.6407724885131456f,
                   -0.0569471541483630f};
  Vec3f v_out = {0.1115498484481505f, -0.1650318402491182f,
                 0.9371708403419812f};
  const Vec3f *v_out_ptr = Vec3fScale(&v, -0.2315908607175066f, &v_out);

  EXPECT_NEAR(0.1992487541584761f, v_out.x, FLT_EPSILON);
  EXPECT_NEAR(0.1483970521388580f, v_out.y, FLT_EPSILON);
  EXPECT_NEAR(0.0131884404446319f, v_out.z, FLT_EPSILON);

  EXPECT_NEAR(0.1992487541584761f, v_out_ptr->x, FLT_EPSILON);
  EXPECT_NEAR(0.1483970521388580f, v_out_ptr->y, FLT_EPSILON);
  EXPECT_NEAR(0.0131884404446319f, v_out_ptr->z, FLT_EPSILON);
}

TEST(Vec3fAddTest, Normal17) {
  const Vec3f v0 = {-0.8069170174246405f, 0.2941291925394900f,
                    -0.5409928871337641f};
  const Vec3f v1 = {-0.8152866757495973f, 0.1163964590176354f,
                    -0.4375021385193112f};
  Vec3f v_out = {0.8564669317248157f, 0.5529849810952754f, 0.8891287272438448f};
  const Vec3f *v_out_ptr = Vec3fAdd(&v0, &v1, &v_out);

  EXPECT_NEAR(-1.6222036931742378f, v_out.x, FLT_EPSILON);
  EXPECT_NEAR(0.4105256515571254f, v_out.y, FLT_EPSILON);
  EXPECT_NEAR(-0.9784950256530753f, v_out.z, FLT_EPSILON);

  EXPECT_NEAR(-1.6222036931742378f, v_out_ptr->x, FLT_EPSILON);
  EXPECT_NEAR(0.4105256515571254f, v_out_ptr->y, FLT_EPSILON);
  EXPECT_NEAR(-0.9784950256530753f, v_out_ptr->z, FLT_EPSILON);
}

TEST(Vec3fAdd3Test, Normal17) {
  const Vec3f v0 = {-0.0302465641833307f, -0.5608734613643123f,
                    0.6668843113394458f};
  const Vec3f v1 = {-0.8956481819163189f, 0.8634911748805276f,
                    -0.6032576582887736f};
  const Vec3f v2 = {-0.5941898145417317f, 0.4488695440429267f,
                    -0.5631905320421389f};
  Vec3f v0c = {-0.0302465641833307f, -0.5608734613643123f, 0.6668843113394458f};
  Vec3f v1c = {-0.8956481819163189f, 0.8634911748805276f, -0.6032576582887736f};
  Vec3f v2c = {-0.5941898145417317f, 0.4488695440429267f, -0.5631905320421389f};
  Vec3f v_out = {0.6762373626794087f, 0.9911057948837927f, 0.0418949124070727f};
  const Vec3f *v_out_ptr = Vec3fAdd3(&v0, &v1, &v2, &v_out);
  Vec3fAdd3(&v0c, &v1, &v2, &v0c);
  Vec3fAdd3(&v0, &v1c, &v2, &v1c);
  Vec3fAdd3(&v0, &v1, &v2c, &v2c);

  EXPECT_NEAR(-1.5200845606413813f, v_out.x, FLT_EPSILON);
  EXPECT_NEAR(0.7514872575591420f, v_out.y, FLT_EPSILON);
  EXPECT_NEAR(-0.4995638789914667f, v_out.z, FLT_EPSILON);

  EXPECT_NEAR(-1.5200845606413813f, v0c.x, FLT_EPSILON);
  EXPECT_NEAR(0.7514872575591420f, v0c.y, FLT_EPSILON);
  EXPECT_NEAR(-0.4995638789914667f, v0c.z, FLT_EPSILON);

  EXPECT_NEAR(-1.5200845606413813f, v1c.x, FLT_EPSILON);
  EXPECT_NEAR(0.7514872575591420f, v1c.y, FLT_EPSILON);
  EXPECT_NEAR(-0.4995638789914667f, v1c.z, FLT_EPSILON);

  EXPECT_NEAR(-1.5200845606413813f, v2c.x, FLT_EPSILON);
  EXPECT_NEAR(0.7514872575591420f, v2c.y, FLT_EPSILON);
  EXPECT_NEAR(-0.4995638789914667f, v2c.z, FLT_EPSILON);

  EXPECT_NEAR(-1.5200845606413813f, v_out_ptr->x, FLT_EPSILON);
  EXPECT_NEAR(0.7514872575591420f, v_out_ptr->y, FLT_EPSILON);
  EXPECT_NEAR(-0.4995638789914667f, v_out_ptr->z, FLT_EPSILON);
}

TEST(Vec3fSubTest, Normal17) {
  const Vec3f v0 = {-0.6497500786402031f, 0.7189437409525017f,
                    -0.3740454112339746f};
  const Vec3f v1 = {-0.8655121625683944f, -0.4477580823105713f,
                    -0.0087234797128810f};
  Vec3f v_out = {0.8028428864869437f, -0.9826278353110933f,
                 -0.3066070681119892f};
  const Vec3f *v_out_ptr = Vec3fSub(&v0, &v1, &v_out);

  EXPECT_NEAR(0.2157620839281913f, v_out.x, FLT_EPSILON);
  EXPECT_NEAR(1.1667018232630730f, v_out.y, FLT_EPSILON);
  EXPECT_NEAR(-0.3653219315210936f, v_out.z, FLT_EPSILON);

  EXPECT_NEAR(0.2157620839281913f, v_out_ptr->x, FLT_EPSILON);
  EXPECT_NEAR(1.1667018232630730f, v_out_ptr->y, FLT_EPSILON);
  EXPECT_NEAR(-0.3653219315210936f, v_out_ptr->z, FLT_EPSILON);
}

TEST(Vec3fLinCombTest, Normal17) {
  const Vec3f v0 = {0.5786257987678165f, 0.5000289322718239f,
                    -0.8226467756795237f};
  const Vec3f v1 = {-0.4915666842744630f, 0.9228925531430792f,
                    0.8799817098997516f};
  Vec3f v_out = {0.2392122368391820f, -0.3550752785665461f,
                 -0.3693708116872825f};
  const Vec3f *v_out_ptr = Vec3fLinComb(-0.3179878737786455f, &v0,
                                        -0.0311796144941490f, &v1, &v_out);

  EXPECT_NEAR(-0.1686691277498036f, v_out.x, FLT_EPSILON);
  EXPECT_NEAR(-0.1877785710274457f, v_out.y, FLT_EPSILON);
  EXPECT_NEAR(0.2341542085926138f, v_out.z, FLT_EPSILON);

  EXPECT_NEAR(-0.1686691277498036f, v_out_ptr->x, FLT_EPSILON);
  EXPECT_NEAR(-0.1877785710274457f, v_out_ptr->y, FLT_EPSILON);
  EXPECT_NEAR(0.2341542085926138f, v_out_ptr->z, FLT_EPSILON);
}

TEST(Vec3fLinComb3Test, Normal17) {
  const Vec3f v0 = {-0.1250558191215512f, 0.4076601903139478f,
                    -0.5807133583327919f};
  const Vec3f v1 = {0.9699194636215120f, 0.9807730160422994f,
                    -0.3923627011240849f};
  const Vec3f v2 = {-0.3697439037174279f, 0.0373739300195339f,
                    0.9386290150708447f};
  Vec3f v_out = {-0.1140355255576238f, -0.4060251145772744f,
                 -0.4960198508449611f};
  const Vec3f *v_out_ptr =
      Vec3fLinComb3(-0.2074790665746116f, &v0, -0.6850652355451436f, &v1,
                    0.7837499767211191f, &v2, &v_out);

  EXPECT_NEAR(-0.9282984171159375f, v_out.x, FLT_EPSILON);
  EXPECT_NEAR(-0.7271826362345186f, v_out.y, FLT_EPSILON);
  EXPECT_NEAR(1.1249303805105371f, v_out.z, FLT_EPSILON);

  EXPECT_NEAR(-0.9282984171159375f, v_out_ptr->x, FLT_EPSILON);
  EXPECT_NEAR(-0.7271826362345186f, v_out_ptr->y, FLT_EPSILON);
  EXPECT_NEAR(1.1249303805105371f, v_out_ptr->z, FLT_EPSILON);
}

TEST(Vec3fDotTest, Normal17) {
  const Vec3f v0 = {-0.8565262904855220f, 0.7497424879534840f,
                    -0.7101905810352720f};
  const Vec3f v1 = {0.9697066138351822f, -0.5142678127417943f,
                    -0.7518027067631843f};
  double v_dot = Vec3fDot(&v0, &v1);

  EXPECT_NEAR(-0.6822244370669184f, v_dot, FLT_EPSILON);
}

TEST(Vec3fMultTest, Normal17) {
  const Vec3f v0 = {0.7865443408781227f, 0.2669686488304712f,
                    -0.2730416494805599f};
  const Vec3f v1 = {0.3091909459938575f, -0.6862745140716260f,
                    -0.9655222039349101f};
  Vec3f v_out = {0.1943731433920355f, -0.3315137926381417f,
                 0.9055899763016606f};
  const Vec3f *v_out_ptr = Vec3fMult(&v0, &v1, &v_out);

  EXPECT_NEAR(0.2431923888222219f, v_out.x, FLT_EPSILON);
  EXPECT_NEAR(-0.1832137797484902f, v_out.y, FLT_EPSILON);
  EXPECT_NEAR(0.2636277751724934f, v_out.z, FLT_EPSILON);

  EXPECT_NEAR(0.2431923888222219f, v_out_ptr->x, FLT_EPSILON);
  EXPECT_NEAR(-0.1832137797484902f, v_out_ptr->y, FLT_EPSILON);
  EXPECT_NEAR(0.2636277751724934f, v_out_ptr->z, FLT_EPSILON);
}

TEST(Vec3fCrossTest, Normal4) {
  const Vec3f v0 = {0.4158606774397273f, -0.0594934954178941f,
                    0.9418075672427990f};
  const Vec3f v1 = {-0.4306611688693243f, 0.3561415195483060f,
                    0.2272569686579682f};
  Vec3f v_out = {-0.7952570746219685f, 0.9728699361678961f,
                 -0.5860936866479094f};
  const Vec3f *v_out_ptr = Vec3fCross(&v0, &v1, &v_out);

  EXPECT_NEAR(-0.3489370895434811f, v_out.x, FLT_EPSILON);
  EXPECT_NEAR(-0.5001071846977601f, v_out.y, FLT_EPSILON);
  EXPECT_NEAR(0.1224837153069804f, v_out.z, FLT_EPSILON);

  EXPECT_NEAR(-0.3489370895434811f, v_out_ptr->x, FLT_EPSILON);
  EXPECT_NEAR(-0.5001071846977601f, v_out_ptr->y, FLT_EPSILON);
  EXPECT_NEAR(0.1224837153069804f, v_out_ptr->z, FLT_EPSILON);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

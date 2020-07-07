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
#include <stdint.h>

#include "common/c_math/vec2f.h"

TEST(Vec2NormTest, VerySmall) {
  // Test with a maximum component whose reciprocal is infinite.
  float very_small = nextafterf(0.0f, 1.0f);  // Smallest representable float.

  Vec2f vx = {very_small, 0.0f};
  Vec2f vy = {0.0f, very_small};

  EXPECT_EQ(Vec2fNorm(&vx), vx.x);
  EXPECT_EQ(Vec2fNorm(&vy), vy.y);
}

TEST(Vec2NormalizeTest, BigVector) {
  Vec2f v = {FLT_MAX, FLT_MAX};
  Vec2fNormalize(&v, &v);

  EXPECT_EQ(1.0f / sqrtf(2.0f), v.x);
  EXPECT_EQ(1.0f / sqrtf(2.0f), v.y);

  EXPECT_NEAR(Vec2fNorm(&v), 1.0, FLT_EPSILON);
}

#if defined(NDEBUG)
TEST(Vec2fNormalizeTest, Inf) {
  Vec2f v = {-INFINITY, INFINITY};
  Vec2fNormalize(&v, &v);

  EXPECT_EQ(-1.0f / sqrtf(2.0f), v.x);
  EXPECT_EQ(1.0f / sqrtf(2.0f), v.y);

  EXPECT_NEAR(Vec2fNorm(&v), 1.0f, FLT_EPSILON);
}

TEST(Vec2fNormalize, SingleNaN) {
  Vec2f v = {1.0f, NAN};
  Vec2fNormalize(&v, &v);

  EXPECT_EQ(1.0f, v.x);
  EXPECT_EQ(0.0f, v.y);

  EXPECT_NEAR(Vec2fNorm(&v), 1.0f, FLT_EPSILON);
}

TEST(Vec2fNormalize, AllNaN) {
  Vec2f v = {NAN, NAN};
  Vec2fNormalize(&v, &v);

  EXPECT_EQ(0.0f, v.x);
  EXPECT_EQ(0.0f, v.y);
}
#endif  // defined(NDEBUG)

TEST(Vec2fNormalize, NoClobber) {
  Vec2f v_in = {-16.0f, 32.0f};
  Vec2f v_out = {123.0f, 99.8f};
  Vec2fNormalize(&v_in, &v_out);

  // Verify that the input was not clobbered.
  EXPECT_EQ(-16.0f, v_in.x);
  EXPECT_EQ(32.0f, v_in.y);

  // Verify that the output is normalized and parallel to the input.
  EXPECT_NEAR(Vec2fNorm(&v_out), 1.0f, FLT_EPSILON);
  EXPECT_NEAR(Vec2fDot(&v_in, &v_out), Vec2fNorm(&v_in), FLT_EPSILON);
}

TEST(Vec2fNormTest, InfNaN) {
  // Expect the conventions of hypot(): if any vector component is
  // infinity, we expect positive infinity.  If any component is NaN
  // (and none is infinite), we expect NaN.
  Vec2f vw = {NAN, NAN};
  Vec2f vx = {NAN, 0.0f};
  Vec2f vy = {NAN, -INFINITY};
  Vec2f vz = {INFINITY, 0.0f};

  double vw_norm = Vec2fNorm(&vw);
  double vx_norm = Vec2fNorm(&vx);
  double vy_norm = Vec2fNorm(&vy);
  double vz_norm = Vec2fNorm(&vz);

  EXPECT_TRUE(isnan(vw_norm));
  EXPECT_TRUE(isnan(vx_norm));
  EXPECT_EQ(vy_norm, INFINITY);
  EXPECT_EQ(vz_norm, INFINITY);
}

TEST(Vec2fNormTest, Normal0) {
  const Vec2f v = {0.4544003592855579f, -0.4220290290438373f};
  float v_norm = Vec2fNorm(&v);
  EXPECT_NEAR(0.6201517458449409f, v_norm, FLT_EPSILON);
}

TEST(Vec2fNormBoundTest, Unbounded0) {
  const Vec2f v = {0.6991766186347612f, 0.4541216144867326f};
  float v_norm = Vec2fNormBound(&v, 0.6242717653548731f);
  EXPECT_NEAR(0.8337112118651008f, v_norm, FLT_EPSILON);
}

TEST(Vec2fNormBoundTest, Bounded0) {
  const Vec2f v = {0.2606303531880529f, 0.7570697687169641f};
  float v_norm = Vec2fNormBound(&v, 1.6580121131521610f);
  EXPECT_NEAR(1.6580121131521610f, v_norm, FLT_EPSILON);
}

TEST(Vec2fNormSquaredTest, Normal0) {
  const Vec2f v = {0.9839916079109012f, -0.4830996972170225f};
  float v_norm_sqrd = Vec2fNormSquared(&v);
  EXPECT_NEAR(1.2016248018902596f, v_norm_sqrd, FLT_EPSILON);
}

TEST(Vec2fNormalizeTest, Normal0) {
  Vec2f v = {-0.9231553602812748f, -0.2278026720360145f};
  const Vec2f *v_out_ptr = Vec2fNormalize(&v, &v);

  EXPECT_NEAR(-0.9708769467300692f, v.x, FLT_EPSILON);
  EXPECT_NEAR(-0.2395787016996680f, v.y, FLT_EPSILON);

  EXPECT_NEAR(-0.9708769467300692f, v_out_ptr->x, FLT_EPSILON);
  EXPECT_NEAR(-0.2395787016996680f, v_out_ptr->y, FLT_EPSILON);
}

#if defined(NDEBUG)
TEST(Vec2fNormalizeTest, Zero0) {
  Vec2f v = {0.0f, 0.0f};
  const Vec2f *v_out_ptr = Vec2fNormalize(&v, &v);

  EXPECT_NEAR(0.0f, v.x, FLT_EPSILON);
  EXPECT_NEAR(0.0f, v.y, FLT_EPSILON);

  EXPECT_NEAR(0.0f, v_out_ptr->x, FLT_EPSILON);
  EXPECT_NEAR(0.0f, v_out_ptr->y, FLT_EPSILON);
}
#endif  // defined(NDEBUG)

TEST(Vec2fScaleTest, Normal0) {
  const Vec2f v = {0.5811859629015408f, -0.2007402933214233f};
  Vec2f v_out = {0.4446003518376411f, -0.9271500639711621f};
  const Vec2f *v_out_ptr = Vec2fScale(&v, 0.3121464586449545f, &v_out);

  EXPECT_NEAR(0.1814151401338739f, v_out.x, FLT_EPSILON);
  EXPECT_NEAR(-0.0626603716676317f, v_out.y, FLT_EPSILON);

  EXPECT_NEAR(0.1814151401338739f, v_out_ptr->x, FLT_EPSILON);
  EXPECT_NEAR(-0.0626603716676317f, v_out_ptr->y, FLT_EPSILON);
}

TEST(Vec2fAddTest, Normal0) {
  const Vec2f v0 = {-0.0598214738954255f, 0.9061217491173328f};
  const Vec2f v1 = {0.8104901193982330f, -0.2607395966399593f};
  Vec2f v_out = {0.6686579438575668f, -0.4214636220747352f};
  const Vec2f *v_out_ptr = Vec2fAdd(&v0, &v1, &v_out);

  EXPECT_NEAR(0.7506686455028075f, v_out.x, FLT_EPSILON);
  EXPECT_NEAR(0.6453821524773735f, v_out.y, FLT_EPSILON);

  EXPECT_NEAR(0.7506686455028075f, v_out_ptr->x, FLT_EPSILON);
  EXPECT_NEAR(0.6453821524773735f, v_out_ptr->y, FLT_EPSILON);
}

TEST(Vec2fAdd3Test, Normal0) {
  const Vec2f v0 = {0.0901187511011177f, 0.2103952313755282f};
  const Vec2f v1 = {-0.1409219919528146f, 0.3516587125870898f};
  const Vec2f v2 = {0.9983854809458934f, -0.6479190339275105f};
  Vec2f v0c = {0.0901187511011177f, 0.2103952313755282f};
  Vec2f v1c = {-0.1409219919528146f, 0.3516587125870898f};
  Vec2f v2c = {0.9983854809458934f, -0.6479190339275105f};
  Vec2f v_out = {0.5350987369498024f, 0.3974766927161388f};
  const Vec2f *v_out_ptr = Vec2fAdd3(&v0, &v1, &v2, &v_out);
  Vec2fAdd3(&v0c, &v1, &v2, &v0c);
  Vec2fAdd3(&v0, &v1c, &v2, &v1c);
  Vec2fAdd3(&v0, &v1, &v2c, &v2c);

  EXPECT_NEAR(0.9475822400941964f, v_out.x, FLT_EPSILON);
  EXPECT_NEAR(-0.0858650899648925f, v_out.y, FLT_EPSILON);

  EXPECT_NEAR(0.9475822400941964f, v0c.x, FLT_EPSILON);
  EXPECT_NEAR(-0.0858650899648925f, v0c.y, FLT_EPSILON);

  EXPECT_NEAR(0.9475822400941964f, v1c.x, FLT_EPSILON);
  EXPECT_NEAR(-0.0858650899648925f, v1c.y, FLT_EPSILON);

  EXPECT_NEAR(0.9475822400941964f, v2c.x, FLT_EPSILON);
  EXPECT_NEAR(-0.0858650899648925f, v2c.y, FLT_EPSILON);

  EXPECT_NEAR(0.9475822400941964f, v_out_ptr->x, FLT_EPSILON);
  EXPECT_NEAR(-0.0858650899648925f, v_out_ptr->y, FLT_EPSILON);
}

TEST(Vec2fSubTest, Normal0) {
  const Vec2f v0 = {0.6916068001931850f, 0.4589259711470723f};
  const Vec2f v1 = {0.5418188329099831f, 0.9178754496307771f};
  Vec2f v_out = {0.0413039433257278f, 0.0205208349033210f};
  const Vec2f *v_out_ptr = Vec2fSub(&v0, &v1, &v_out);

  EXPECT_NEAR(0.1497879672832019f, v_out.x, FLT_EPSILON);
  EXPECT_NEAR(-0.4589494784837047f, v_out.y, FLT_EPSILON);

  EXPECT_NEAR(0.1497879672832019f, v_out_ptr->x, FLT_EPSILON);
  EXPECT_NEAR(-0.4589494784837047f, v_out_ptr->y, FLT_EPSILON);
}

TEST(Vec2fLinCombTest, Normal0) {
  const Vec2f v0 = {-0.1151833118969112f, -0.9490748614833537f};
  const Vec2f v1 = {0.4843592617604149f, -0.1587031961706715f};
  Vec2f v_out = {-0.3384193231294899f, -0.0791940929438708f};
  const Vec2f *v_out_ptr =
      Vec2fLinComb(0.4509901260680738f, &v0, -0.7249455547194936f, &v1, &v_out);

  EXPECT_NEAR(-0.4030806300537546f, v_out.x, FLT_EPSILON);
  EXPECT_NEAR(-0.3129722148447133f, v_out.y, FLT_EPSILON);

  EXPECT_NEAR(-0.4030806300537546f, v_out_ptr->x, FLT_EPSILON);
  EXPECT_NEAR(-0.3129722148447133f, v_out_ptr->y, FLT_EPSILON);
}

TEST(Vec2fLinComb3Test, Normal0) {
  const Vec2f v0 = {-0.7715167570048538f, 0.3949570576166892f};
  const Vec2f v1 = {-0.0983151348130065f, -0.1104057043353737f};
  const Vec2f v2 = {-0.5062591752055867f, 0.5653648337140147f};
  Vec2f v_out = {0.2314725479683408f, 0.2004573385825896f};
  const Vec2f *v_out_ptr =
      Vec2fLinComb3(0.4092843827945352f, &v0, -0.2678061115585173f, &v1,
                    -0.7358553512643784f, &v2, &v_out);

  EXPECT_NEAR(0.0830931574569712f, v_out.x, FLT_EPSILON);
  EXPECT_NEAR(-0.2248096603762251f, v_out.y, FLT_EPSILON);

  EXPECT_NEAR(0.0830931574569712f, v_out_ptr->x, FLT_EPSILON);
  EXPECT_NEAR(-0.2248096603762251f, v_out_ptr->y, FLT_EPSILON);
}

TEST(Vec2fDotTest, Normal0) {
  const Vec2f v0 = {-0.1446997933300362f, 0.1601897034955511f};
  const Vec2f v1 = {-0.1438686871495198f, 0.3426734946299088f};
  float v_dot = Vec2fDot(&v0, &v1);
  EXPECT_NEAR(0.0757105347977486f, v_dot, FLT_EPSILON);
}

TEST(Vec2fMultTest, Normal0) {
  const Vec2f v0 = {-0.5334189101655196f, 0.9913539847317210f};
  const Vec2f v1 = {0.1490047721834795f, -0.3474220477191863f};
  Vec2f v_out = {-0.9547117549742801f, -0.0675251687898835f};
  const Vec2f *v_out_ptr = Vec2fMult(&v0, &v1, &v_out);

  EXPECT_NEAR(-0.0794819631875732f, v_out.x, FLT_EPSILON);
  EXPECT_NEAR(-0.3444182313900694f, v_out.y, FLT_EPSILON);

  EXPECT_NEAR(-0.0794819631875732f, v_out_ptr->x, FLT_EPSILON);
  EXPECT_NEAR(-0.3444182313900694f, v_out_ptr->y, FLT_EPSILON);
}

// The remaining tests are generated code comparing the results to
// MATLAB.

TEST(Vec2fNormTest, Normal4) {
  const Vec2f v = {0.7350408409402285f, 0.1908152857978966f};
  double v_norm = Vec2fNorm(&v);

  EXPECT_NEAR(0.7594047084027404f, v_norm, FLT_EPSILON);
}

TEST(Vec2fNormBoundTest, Unbounded4) {
  const Vec2f v = {-0.6815783701826705f, 0.6211585663669401f};
  double v_norm = Vec2fNormBound(&v, 0.6754155610923913f);

  EXPECT_NEAR(0.9221643233566877f, v_norm, FLT_EPSILON);
}

TEST(Vec2fNormBoundTest, Bounded4) {
  const Vec2f v = {0.0186501675608723f, -0.8085448428771753f};
  double v_norm = Vec2fNormBound(&v, 1.0147768628418206f);

  EXPECT_NEAR(1.0147768628418206f, v_norm, FLT_EPSILON);
}

TEST(Vec2fNormSquaredTest, Normal4) {
  const Vec2f v = {0.6020241750251341f, -0.9400481152064184f};
  double v_norm_sqrd = Vec2fNormSquared(&v);

  EXPECT_NEAR(1.2461235662178327f, v_norm_sqrd, FLT_EPSILON);
}

TEST(Vec2fNormalizeTest, Normal4) {
  Vec2f v = {0.9683818264601043f, -0.8941390028992506f};
  const Vec2f *v_out_ptr = Vec2fNormalize(&v, &v);

  EXPECT_NEAR(0.7347096270004123f, v.x, FLT_EPSILON);
  EXPECT_NEAR(-0.6783817243948388f, v.y, FLT_EPSILON);

  EXPECT_NEAR(0.7347096270004123f, v_out_ptr->x, FLT_EPSILON);
  EXPECT_NEAR(-0.6783817243948388f, v_out_ptr->y, FLT_EPSILON);
}

#if defined(NDEBUG)
TEST(Vec2fNormalizeTest, Zero_4) {
  Vec2f v = {0.0000000000000000f, 0.0000000000000000f};
  const Vec2f *v_out_ptr = Vec2fNormalize(&v, &v);

  EXPECT_NEAR(0.0000000000000000f, v.x, FLT_EPSILON);
  EXPECT_NEAR(0.0000000000000000f, v.y, FLT_EPSILON);

  EXPECT_NEAR(0.0000000000000000f, v_out_ptr->x, FLT_EPSILON);
  EXPECT_NEAR(0.0000000000000000f, v_out_ptr->y, FLT_EPSILON);
}
#endif  // defined(NDEBUG)

TEST(Vec2fScaleTest, Normal4) {
  const Vec2f v = {-0.0886311813159784f, -0.8558224344378991f};
  Vec2f v_out = {-0.4045072202786570f, 0.6302616468340054f};
  const Vec2f *v_out_ptr = Vec2fScale(&v, 0.6026974605041078f, &v_out);

  EXPECT_NEAR(-0.0534177879006193f, v_out.x, FLT_EPSILON);
  EXPECT_NEAR(-0.5158020078781650f, v_out.y, FLT_EPSILON);

  EXPECT_NEAR(-0.0534177879006193f, v_out_ptr->x, FLT_EPSILON);
  EXPECT_NEAR(-0.5158020078781650f, v_out_ptr->y, FLT_EPSILON);
}

TEST(Vec2fAddTest, Normal4) {
  const Vec2f v0 = {-0.5527839849755161f, -0.1058442899014300f};
  const Vec2f v1 = {0.7880978384787809f, -0.0726788147470605f};
  Vec2f v_out = {-0.2725636913010485f, 0.0440953231773156f};
  const Vec2f *v_out_ptr = Vec2fAdd(&v0, &v1, &v_out);

  EXPECT_NEAR(0.2353138535032648f, v_out.x, FLT_EPSILON);
  EXPECT_NEAR(-0.1785231046484905f, v_out.y, FLT_EPSILON);

  EXPECT_NEAR(0.2353138535032648f, v_out_ptr->x, FLT_EPSILON);
  EXPECT_NEAR(-0.1785231046484905f, v_out_ptr->y, FLT_EPSILON);
}

TEST(Vec2fAdd3Test, Normal4) {
  const Vec2f v0 = {-0.2243251092754037f, -0.4227932154939156f};
  const Vec2f v1 = {0.8979025632574826f, 0.8165461704772250f};
  const Vec2f v2 = {-0.5563299208222701f, -0.6009012329192769f};
  Vec2f v0c = {-0.2243251092754037f, -0.4227932154939156f};
  Vec2f v1c = {0.8979025632574826f, 0.8165461704772250f};
  Vec2f v2c = {-0.5563299208222701f, -0.6009012329192769f};
  Vec2f v_out = {0.7023997505476991f, -0.9946903808255914f};
  const Vec2f *v_out_ptr = Vec2fAdd3(&v0, &v1, &v2, &v_out);
  Vec2fAdd3(&v0c, &v1, &v2, &v0c);
  Vec2fAdd3(&v0, &v1c, &v2, &v1c);
  Vec2fAdd3(&v0, &v1, &v2c, &v2c);

  EXPECT_NEAR(0.1172475331598088f, v_out.x, FLT_EPSILON);
  EXPECT_NEAR(-0.2071482779359675f, v_out.y, FLT_EPSILON);

  EXPECT_NEAR(0.1172475331598088f, v0c.x, FLT_EPSILON);
  EXPECT_NEAR(-0.2071482779359675f, v0c.y, FLT_EPSILON);

  EXPECT_NEAR(0.1172475331598088f, v1c.x, FLT_EPSILON);
  EXPECT_NEAR(-0.2071482779359675f, v1c.y, FLT_EPSILON);

  EXPECT_NEAR(0.1172475331598088f, v2c.x, FLT_EPSILON);
  EXPECT_NEAR(-0.2071482779359675f, v2c.y, FLT_EPSILON);

  EXPECT_NEAR(0.1172475331598088f, v_out_ptr->x, FLT_EPSILON);
  EXPECT_NEAR(-0.2071482779359675f, v_out_ptr->y, FLT_EPSILON);
}

TEST(Vec2fSubTest, Normal4) {
  const Vec2f v0 = {-0.2439007407481864f, -0.6250084101335307f};
  const Vec2f v1 = {-0.0382060350276217f, 0.9492573867191427f};
  Vec2f v_out = {0.6969820736843015f, 0.4221164241528790f};
  const Vec2f *v_out_ptr = Vec2fSub(&v0, &v1, &v_out);

  EXPECT_NEAR(-0.2056947057205647f, v_out.x, FLT_EPSILON);
  EXPECT_NEAR(-1.5742657968526734f, v_out.y, FLT_EPSILON);

  EXPECT_NEAR(-0.2056947057205647f, v_out_ptr->x, FLT_EPSILON);
  EXPECT_NEAR(-1.5742657968526734f, v_out_ptr->y, FLT_EPSILON);
}

TEST(Vec2fLinCombTest, Normal4) {
  const Vec2f v0 = {-0.5796149327379121f, -0.1455795501769679f};
  const Vec2f v1 = {0.4971147117023527f, -0.3552901440847234f};
  Vec2f v_out = {0.8210612658046434f, 0.8580331720039456f};
  const Vec2f *v_out_ptr = Vec2fLinComb(-0.3596960081234288f, &v0,
                                        -0.7288262288568024f, &v1, &v_out);

  EXPECT_NEAR(-0.1538250630847055f, v_out.x, FLT_EPSILON);
  EXPECT_NEAR(0.3113091589263186f, v_out.y, FLT_EPSILON);

  EXPECT_NEAR(-0.1538250630847055f, v_out_ptr->x, FLT_EPSILON);
  EXPECT_NEAR(0.3113091589263186f, v_out_ptr->y, FLT_EPSILON);
}

TEST(Vec2fLinComb3Test, Normal4) {
  const Vec2f v0 = {-0.4963406088219333f, 0.6352168929518587f};
  const Vec2f v1 = {0.4862322449983985f, 0.0437031662216629f};
  const Vec2f v2 = {-0.9463009675522265f, -0.8765677868163460f};
  Vec2f v_out = {0.0131108885415783f, -0.7438913434305576f};
  const Vec2f *v_out_ptr =
      Vec2fLinComb3(-0.0904799865426431f, &v0, 0.3246026678927694f, &v1,
                    -0.1174410057544337f, &v2, &v_out);

  EXPECT_NEAR(0.3138757129244738f, v_out.x, FLT_EPSILON);
  EXPECT_NEAR(0.0596567509206189f, v_out.y, FLT_EPSILON);

  EXPECT_NEAR(0.3138757129244738f, v_out_ptr->x, FLT_EPSILON);
  EXPECT_NEAR(0.0596567509206189f, v_out_ptr->y, FLT_EPSILON);
}

TEST(Vec2fDotTest, Normal4) {
  const Vec2f v0 = {0.3434885331813626f, 0.4102278528744021f};
  const Vec2f v1 = {0.8805471439706187f, -0.5521916442019388f};
  double v_dot = Vec2fDot(&v0, &v1);

  EXPECT_NEAR(0.0759334543033588f, v_dot, FLT_EPSILON);
}

TEST(Vec2fMultTest, Normal4) {
  const Vec2f v0 = {-0.2843156435845902f, -0.9847514846798333f};
  const Vec2f v1 = {-0.1756165518372965f, 0.0293362035399500f};
  Vec2f v_out = {-0.0457463783745555f, 0.4119740221135340f};
  const Vec2f *v_out_ptr = Vec2fMult(&v0, &v1, &v_out);

  EXPECT_NEAR(0.0499305329597275f, v_out.x, FLT_EPSILON);
  EXPECT_NEAR(-0.0288888699908356f, v_out.y, FLT_EPSILON);

  EXPECT_NEAR(0.0499305329597275f, v_out_ptr->x, FLT_EPSILON);
  EXPECT_NEAR(-0.0288888699908356f, v_out_ptr->y, FLT_EPSILON);
}

TEST(Vec2fNormTest, Normal8) {
  const Vec2f v = {-0.5858208792333235f, -0.4859082502971162f};
  double v_norm = Vec2fNorm(&v);

  EXPECT_NEAR(0.7611129549892769f, v_norm, FLT_EPSILON);
}

TEST(Vec2fNormBoundTest, Unbounded8) {
  const Vec2f v = {0.6973811113183586f, -0.1629617765078992f};
  double v_norm = Vec2fNormBound(&v, -0.2250914166242485f);

  EXPECT_NEAR(0.7161682449161226f, v_norm, FLT_EPSILON);
}

TEST(Vec2fNormBoundTest, Bounded8) {
  const Vec2f v = {-0.2536083531115996f, 0.2161294620563727f};
  double v_norm = Vec2fNormBound(&v, 0.7972541423870023f);

  EXPECT_NEAR(0.7972541423870023f, v_norm, FLT_EPSILON);
}

TEST(Vec2fNormSquaredTest, Normal8) {
  const Vec2f v = {0.9129689758988406f, -0.3231470888542989f};
  double v_norm_sqrd = Vec2fNormSquared(&v);

  EXPECT_NEAR(0.9379363919887860f, v_norm_sqrd, FLT_EPSILON);
}

TEST(Vec2fNormalizeTest, Normal8) {
  Vec2f v = {-0.9994021608463159f, 0.5243770748557992f};
  const Vec2f *v_out_ptr = Vec2fNormalize(&v, &v);

  EXPECT_NEAR(-0.8855105789046385f, v.x, FLT_EPSILON);
  EXPECT_NEAR(0.4646192146779681f, v.y, FLT_EPSILON);

  EXPECT_NEAR(-0.8855105789046385f, v_out_ptr->x, FLT_EPSILON);
  EXPECT_NEAR(0.4646192146779681f, v_out_ptr->y, FLT_EPSILON);
}

#if defined(NDEBUG)
TEST(Vec2fNormalizeTest, Zero_8) {
  Vec2f v = {0.0000000000000000f, 0.0000000000000000f};
  const Vec2f *v_out_ptr = Vec2fNormalize(&v, &v);

  EXPECT_NEAR(0.0000000000000000f, v.x, FLT_EPSILON);
  EXPECT_NEAR(0.0000000000000000f, v.y, FLT_EPSILON);

  EXPECT_NEAR(0.0000000000000000f, v_out_ptr->x, FLT_EPSILON);
  EXPECT_NEAR(0.0000000000000000f, v_out_ptr->y, FLT_EPSILON);
}
#endif  // defined(NDEBUG)

TEST(Vec2fScaleTest, Normal8) {
  const Vec2f v = {-0.1660853778134723f, 0.1326087411917054f};
  Vec2f v_out = {0.9528885184560159f, -0.1671134987213712f};
  const Vec2f *v_out_ptr = Vec2fScale(&v, -0.1191911181865262f, &v_out);

  EXPECT_NEAR(0.0197959018960194f, v_out.x, FLT_EPSILON);
  EXPECT_NEAR(-0.0158057841439470f, v_out.y, FLT_EPSILON);

  EXPECT_NEAR(0.0197959018960194f, v_out_ptr->x, FLT_EPSILON);
  EXPECT_NEAR(-0.0158057841439470f, v_out_ptr->y, FLT_EPSILON);
}

TEST(Vec2fAddTest, Normal8) {
  const Vec2f v0 = {0.8203263225908590f, 0.8556764110504829f};
  const Vec2f v1 = {-0.0387675597435506f, 0.1815056288682759f};
  Vec2f v_out = {-0.3127245447356586f, 0.7849231166076256f};
  const Vec2f *v_out_ptr = Vec2fAdd(&v0, &v1, &v_out);

  EXPECT_NEAR(0.7815587628473084f, v_out.x, FLT_EPSILON);
  EXPECT_NEAR(1.0371820399187588f, v_out.y, FLT_EPSILON);

  EXPECT_NEAR(0.7815587628473084f, v_out_ptr->x, FLT_EPSILON);
  EXPECT_NEAR(1.0371820399187588f, v_out_ptr->y, FLT_EPSILON);
}

TEST(Vec2fAdd3Test, Normal8) {
  const Vec2f v0 = {-0.1104272622036997f, 0.3972781445116349f};
  const Vec2f v1 = {-0.9622025463633064f, -0.6758940582973296f};
  const Vec2f v2 = {0.1013962848691037f, 0.6435544678024805f};
  Vec2f v0c = {-0.1104272622036997f, 0.3972781445116349f};
  Vec2f v1c = {-0.9622025463633064f, -0.6758940582973296f};
  Vec2f v2c = {0.1013962848691037f, 0.6435544678024805f};
  Vec2f v_out = {-0.7077476652723209f, 0.8355935383734168f};
  const Vec2f *v_out_ptr = Vec2fAdd3(&v0, &v1, &v2, &v_out);
  Vec2fAdd3(&v0c, &v1, &v2, &v0c);
  Vec2fAdd3(&v0, &v1c, &v2, &v1c);
  Vec2fAdd3(&v0, &v1, &v2c, &v2c);

  EXPECT_NEAR(-0.9712335236979024f, v_out.x, FLT_EPSILON);
  EXPECT_NEAR(0.3649385540167858f, v_out.y, FLT_EPSILON);

  EXPECT_NEAR(-0.9712335236979024f, v0c.x, FLT_EPSILON);
  EXPECT_NEAR(0.3649385540167858f, v0c.y, FLT_EPSILON);

  EXPECT_NEAR(-0.9712335236979024f, v1c.x, FLT_EPSILON);
  EXPECT_NEAR(0.3649385540167858f, v1c.y, FLT_EPSILON);

  EXPECT_NEAR(-0.9712335236979024f, v2c.x, FLT_EPSILON);
  EXPECT_NEAR(0.3649385540167858f, v2c.y, FLT_EPSILON);

  EXPECT_NEAR(-0.9712335236979024f, v_out_ptr->x, FLT_EPSILON);
  EXPECT_NEAR(0.3649385540167858f, v_out_ptr->y, FLT_EPSILON);
}

TEST(Vec2fSubTest, Normal8) {
  const Vec2f v0 = {0.9520144195793818f, 0.8931676763221625f};
  const Vec2f v1 = {0.6457964869564676f, 0.9256736792993325f};
  Vec2f v_out = {0.8320473179125483f, -0.1240629056284164f};
  const Vec2f *v_out_ptr = Vec2fSub(&v0, &v1, &v_out);

  EXPECT_NEAR(0.3062179326229142f, v_out.x, FLT_EPSILON);
  EXPECT_NEAR(-0.0325060029771700f, v_out.y, FLT_EPSILON);

  EXPECT_NEAR(0.3062179326229142f, v_out_ptr->x, FLT_EPSILON);
  EXPECT_NEAR(-0.0325060029771700f, v_out_ptr->y, FLT_EPSILON);
}

TEST(Vec2fLinCombTest, Normal8) {
  const Vec2f v0 = {-0.1654631931194057f, -0.7461873823031120f};
  const Vec2f v1 = {0.3563245745554680f, 0.8175642451252616f};
  Vec2f v_out = {0.0722429035169696f, -0.6081394045295416f};
  const Vec2f *v_out_ptr = Vec2fLinComb(-0.9643926468648234f, &v0,
                                        -0.3129730767588224f, &v1, &v_out);

  EXPECT_NEAR(0.0480514883477259f, v_out.x, FLT_EPSILON);
  EXPECT_NEAR(0.4637420274315749f, v_out.y, FLT_EPSILON);

  EXPECT_NEAR(0.0480514883477259f, v_out_ptr->x, FLT_EPSILON);
  EXPECT_NEAR(0.4637420274315749f, v_out_ptr->y, FLT_EPSILON);
}

TEST(Vec2fLinComb3Test, Normal8) {
  const Vec2f v0 = {0.3579304671439529f, -0.5964756641971727f};
  const Vec2f v1 = {-0.7504192593688770f, -0.2231799318082210f};
  const Vec2f v2 = {0.4288673217622898f, 0.3846501901780108f};
  Vec2f v_out = {-0.5453734705666557f, -0.4265788713568637f};
  const Vec2f *v_out_ptr =
      Vec2fLinComb3(0.8187346011155743f, &v0, 0.8865089251318923f, &v1,
                    -0.0359360079990867f, &v2, &v_out);

  EXPECT_NEAR(-0.3876150922825549f, v_out.x, FLT_EPSILON);
  EXPECT_NEAR(-0.7000290587710216f, v_out.y, FLT_EPSILON);

  EXPECT_NEAR(-0.3876150922825549f, v_out_ptr->x, FLT_EPSILON);
  EXPECT_NEAR(-0.7000290587710216f, v_out_ptr->y, FLT_EPSILON);
}

TEST(Vec2fDotTest, Normal8) {
  const Vec2f v0 = {-0.2377540209680176f, 0.2770566952391842f};
  const Vec2f v1 = {0.7060516279911360f, 0.1241191269260813f};
  double v_dot = Vec2fDot(&v0, &v1);

  EXPECT_NEAR(-0.1334785784437946f, v_dot, FLT_EPSILON);
}

TEST(Vec2fMultTest, Normal8) {
  const Vec2f v0 = {0.5698708964575960f, 0.3326456969003151f};
  const Vec2f v1 = {-0.8248991082727837f, -0.0942045642665317f};
  Vec2f v_out = {-0.2699596498230421f, 0.7038950808289186f};
  const Vec2f *v_out_ptr = Vec2fMult(&v0, &v1, &v_out);

  EXPECT_NEAR(-0.4700859943184828f, v_out.x, FLT_EPSILON);
  EXPECT_NEAR(-0.0313367429316309f, v_out.y, FLT_EPSILON);

  EXPECT_NEAR(-0.4700859943184828f, v_out_ptr->x, FLT_EPSILON);
  EXPECT_NEAR(-0.0313367429316309f, v_out_ptr->y, FLT_EPSILON);
}

TEST(Vec2fNormTest, Normal12) {
  const Vec2f v = {-0.8312632708567140f, -0.3272890707401832f};
  double v_norm = Vec2fNorm(&v);

  EXPECT_NEAR(0.8933738082691788f, v_norm, FLT_EPSILON);
}

TEST(Vec2fNormBoundTest, Unbounded12) {
  const Vec2f v = {0.9401577271193231f, 0.5399619149152564f};
  double v_norm = Vec2fNormBound(&v, 0.7627716069076602f);

  EXPECT_NEAR(1.0841842193193563f, v_norm, FLT_EPSILON);
}

TEST(Vec2fNormBoundTest, Bounded12) {
  const Vec2f v = {0.9964228821753756f, -0.6821988553485068f};
  double v_norm = Vec2fNormBound(&v, 2.1786554938227427f);

  EXPECT_NEAR(2.1786554938227427f, v_norm, FLT_EPSILON);
}

TEST(Vec2fNormSquaredTest, Normal12) {
  const Vec2f v = {-0.0652198338749084f, 0.8082109995538993f};
  double v_norm_sqrd = Vec2fNormSquared(&v);

  EXPECT_NEAR(0.6574586465305836f, v_norm_sqrd, FLT_EPSILON);
}

TEST(Vec2fNormalizeTest, Normal12) {
  Vec2f v = {-0.6506625365957746f, 0.0356668427870872f};
  const Vec2f *v_out_ptr = Vec2fNormalize(&v, &v);

  EXPECT_NEAR(-0.9985009699122906f, v.x, FLT_EPSILON);
  EXPECT_NEAR(0.0547340212684485f, v.y, FLT_EPSILON);

  EXPECT_NEAR(-0.9985009699122906f, v_out_ptr->x, FLT_EPSILON);
  EXPECT_NEAR(0.0547340212684485f, v_out_ptr->y, FLT_EPSILON);
}

#if defined(NDEBUG)
TEST(Vec2fNormalizeTest, Zero_12) {
  Vec2f v = {0.0000000000000000f, 0.0000000000000000f};
  const Vec2f *v_out_ptr = Vec2fNormalize(&v, &v);

  EXPECT_NEAR(0.0000000000000000f, v.x, FLT_EPSILON);
  EXPECT_NEAR(0.0000000000000000f, v.y, FLT_EPSILON);

  EXPECT_NEAR(0.0000000000000000f, v_out_ptr->x, FLT_EPSILON);
  EXPECT_NEAR(0.0000000000000000f, v_out_ptr->y, FLT_EPSILON);
}
#endif  // defined(NDEBUG)

TEST(Vec2fScaleTest, Normal12) {
  const Vec2f v = {-0.6459070429021103f, 0.4220480049165438f};
  Vec2f v_out = {0.5346535282495251f, -0.6777132716440679f};
  const Vec2f *v_out_ptr = Vec2fScale(&v, -0.4260802904807948f, &v_out);

  EXPECT_NEAR(0.2752082604633223f, v_out.x, FLT_EPSILON);
  EXPECT_NEAR(-0.1798263365316809f, v_out.y, FLT_EPSILON);

  EXPECT_NEAR(0.2752082604633223f, v_out_ptr->x, FLT_EPSILON);
  EXPECT_NEAR(-0.1798263365316809f, v_out_ptr->y, FLT_EPSILON);
}

TEST(Vec2fAddTest, Normal12) {
  const Vec2f v0 = {0.7370456481294776f, 0.8602849170316760f};
  const Vec2f v1 = {-0.0887559376837550f, 0.8655468061236307f};
  Vec2f v_out = {0.3864059477265509f, -0.5473029479473617f};
  const Vec2f *v_out_ptr = Vec2fAdd(&v0, &v1, &v_out);

  EXPECT_NEAR(0.6482897104457226f, v_out.x, FLT_EPSILON);
  EXPECT_NEAR(1.7258317231553066f, v_out.y, FLT_EPSILON);

  EXPECT_NEAR(0.6482897104457226f, v_out_ptr->x, FLT_EPSILON);
  EXPECT_NEAR(1.7258317231553066f, v_out_ptr->y, FLT_EPSILON);
}

TEST(Vec2fAdd3Test, Normal12) {
  const Vec2f v0 = {-0.9195526203577309f, -0.7333955485279866f};
  const Vec2f v1 = {-0.1121434949241429f, -0.0425770436177020f};
  const Vec2f v2 = {-0.5655182836842014f, -0.0161066781574575f};
  Vec2f v0c = {-0.9195526203577309f, -0.7333955485279866f};
  Vec2f v1c = {-0.1121434949241429f, -0.0425770436177020f};
  Vec2f v2c = {-0.5655182836842014f, -0.0161066781574575f};
  Vec2f v_out = {-0.8242131553859959f, -0.1551053920632923f};
  const Vec2f *v_out_ptr = Vec2fAdd3(&v0, &v1, &v2, &v_out);
  Vec2fAdd3(&v0c, &v1, &v2, &v0c);
  Vec2fAdd3(&v0, &v1c, &v2, &v1c);
  Vec2fAdd3(&v0, &v1, &v2c, &v2c);

  EXPECT_NEAR(-1.5972143989660752f, v_out.x, FLT_EPSILON);
  EXPECT_NEAR(-0.7920792703031461f, v_out.y, FLT_EPSILON);

  EXPECT_NEAR(-1.5972143989660752f, v0c.x, FLT_EPSILON);
  EXPECT_NEAR(-0.7920792703031461f, v0c.y, FLT_EPSILON);

  EXPECT_NEAR(-1.5972143989660752f, v1c.x, FLT_EPSILON);
  EXPECT_NEAR(-0.7920792703031461f, v1c.y, FLT_EPSILON);

  EXPECT_NEAR(-1.5972143989660752f, v2c.x, FLT_EPSILON);
  EXPECT_NEAR(-0.7920792703031461f, v2c.y, FLT_EPSILON);

  EXPECT_NEAR(-1.5972143989660752f, v_out_ptr->x, FLT_EPSILON);
  EXPECT_NEAR(-0.7920792703031461f, v_out_ptr->y, FLT_EPSILON);
}

TEST(Vec2fSubTest, Normal12) {
  const Vec2f v0 = {-0.1155558494048445f, 0.8468614390874263f};
  const Vec2f v1 = {0.1405183095254503f, 0.4027561993062709f};
  Vec2f v_out = {-0.9100712678775715f, -0.5901350858098380f};
  const Vec2f *v_out_ptr = Vec2fSub(&v0, &v1, &v_out);

  EXPECT_NEAR(-0.2560741589302948f, v_out.x, FLT_EPSILON);
  EXPECT_NEAR(0.4441052397811553f, v_out.y, FLT_EPSILON);

  EXPECT_NEAR(-0.2560741589302948f, v_out_ptr->x, FLT_EPSILON);
  EXPECT_NEAR(0.4441052397811553f, v_out_ptr->y, FLT_EPSILON);
}

TEST(Vec2fLinCombTest, Normal12) {
  const Vec2f v0 = {0.7101015383216975f, -0.5781548449055203f};
  const Vec2f v1 = {0.4822391035383315f, 0.0664841775777909f};
  Vec2f v_out = {0.9576055881439218f, -0.1003034024441662f};
  const Vec2f *v_out_ptr =
      Vec2fLinComb(-0.0815143756999865f, &v0, 0.1181029426860676f, &v1, &v_out);

  EXPECT_NEAR(-0.0009296263737251f, v_out.x, FLT_EPSILON);
  EXPECT_NEAR(0.0549799082543962f, v_out.y, FLT_EPSILON);

  EXPECT_NEAR(-0.0009296263737251f, v_out_ptr->x, FLT_EPSILON);
  EXPECT_NEAR(0.0549799082543962f, v_out_ptr->y, FLT_EPSILON);
}

TEST(Vec2fLinComb3Test, Normal12) {
  const Vec2f v0 = {-0.9035642426325352f, 0.6183054993301946f};
  const Vec2f v1 = {-0.8233049784502382f, -0.7052801417356040f};
  const Vec2f v2 = {0.3797036853008271f, -0.8125142395230183f};
  Vec2f v_out = {0.0526123284428266f, -0.2048396236786523f};
  const Vec2f *v_out_ptr =
      Vec2fLinComb3(-0.6323401902815846f, &v0, -0.0684352306709610f, &v1,
                    0.1314003798227337f, &v2, &v_out);

  EXPECT_NEAR(0.6775962596993063f, v_out.x, FLT_EPSILON);
  EXPECT_NEAR(-0.4494780875959855f, v_out.y, FLT_EPSILON);

  EXPECT_NEAR(0.6775962596993063f, v_out_ptr->x, FLT_EPSILON);
  EXPECT_NEAR(-0.4494780875959855f, v_out_ptr->y, FLT_EPSILON);
}

TEST(Vec2fDotTest, Normal12) {
  const Vec2f v0 = {0.2857091950929600f, 0.0487298121719939f};
  const Vec2f v1 = {-0.7092180701440558f, 0.2061640865838354f};
  double v_dot = Vec2fDot(&v0, &v1);

  EXPECT_NEAR(-0.1925837867503996f, v_dot, FLT_EPSILON);
}

TEST(Vec2fMultTest, Normal12) {
  const Vec2f v0 = {0.2189559597420998f, -0.9263213821779510f};
  const Vec2f v1 = {0.2791520924887865f, -0.6754568133305667f};
  Vec2f v_out = {-0.5188894216842403f, -0.6093642645503228f};
  const Vec2f *v_out_ptr = Vec2fMult(&v0, &v1, &v_out);

  EXPECT_NEAR(0.0611220143248977f, v_out.x, FLT_EPSILON);
  EXPECT_NEAR(0.6256900889258848f, v_out.y, FLT_EPSILON);

  EXPECT_NEAR(0.0611220143248977f, v_out_ptr->x, FLT_EPSILON);
  EXPECT_NEAR(0.6256900889258848f, v_out_ptr->y, FLT_EPSILON);
}

TEST(Vec2fNormTest, Normal16) {
  const Vec2f v = {-0.1710068065653074f, -0.7691480721168182f};
  double v_norm = Vec2fNorm(&v);

  EXPECT_NEAR(0.7879289845745507f, v_norm, FLT_EPSILON);
}

TEST(Vec2fNormBoundTest, Unbounded16) {
  const Vec2f v = {-0.2908298715549946f, -0.1960511606940207f};
  double v_norm = Vec2fNormBound(&v, 0.1318435421307123f);

  EXPECT_NEAR(0.3507393217165241f, v_norm, FLT_EPSILON);
}

TEST(Vec2fNormBoundTest, Bounded16) {
  const Vec2f v = {0.0252507642293547f, 0.6776535464619091f};
  double v_norm = Vec2fNormBound(&v, 0.9568848321300826f);

  EXPECT_NEAR(0.9568848321300826f, v_norm, FLT_EPSILON);
}

TEST(Vec2fNormSquaredTest, Normal16) {
  const Vec2f v = {-0.8086043428304153f, 0.8241492057304098f};
  double v_norm_sqrd = Vec2fNormSquared(&v);

  EXPECT_NEAR(1.3330628965502729f, v_norm_sqrd, FLT_EPSILON);
}

TEST(Vec2fNormalizeTest, Normal16) {
  Vec2f v = {-0.2881150657326004f, -0.8737822764542982f};
  const Vec2f *v_out_ptr = Vec2fNormalize(&v, &v);

  EXPECT_NEAR(-0.3131489863249859f, v.x, FLT_EPSILON);
  EXPECT_NEAR(-0.9497040130291299f, v.y, FLT_EPSILON);

  EXPECT_NEAR(-0.3131489863249859f, v_out_ptr->x, FLT_EPSILON);
  EXPECT_NEAR(-0.9497040130291299f, v_out_ptr->y, FLT_EPSILON);
}

#if defined(NDEBUG)
TEST(Vec2fNormalizeTest, Zero_16) {
  Vec2f v = {0.0000000000000000f, 0.0000000000000000f};
  const Vec2f *v_out_ptr = Vec2fNormalize(&v, &v);

  EXPECT_NEAR(0.0000000000000000f, v.x, FLT_EPSILON);
  EXPECT_NEAR(0.0000000000000000f, v.y, FLT_EPSILON);

  EXPECT_NEAR(0.0000000000000000f, v_out_ptr->x, FLT_EPSILON);
  EXPECT_NEAR(0.0000000000000000f, v_out_ptr->y, FLT_EPSILON);
}
#endif  // defined(NDEBUG)

TEST(Vec2fScaleTest, Normal16) {
  const Vec2f v = {0.9899232831265425f, -0.4395487112418694f};
  Vec2f v_out = {0.1429370240307217f, 0.1034551354974269f};
  const Vec2f *v_out_ptr = Vec2fScale(&v, -0.6402545085626576f, &v_out);

  EXPECT_NEAR(-0.6338028451529171f, v_out.x, FLT_EPSILON);
  EXPECT_NEAR(0.2814230441055126f, v_out.y, FLT_EPSILON);

  EXPECT_NEAR(-0.6338028451529171f, v_out_ptr->x, FLT_EPSILON);
  EXPECT_NEAR(0.2814230441055126f, v_out_ptr->y, FLT_EPSILON);
}

TEST(Vec2fAddTest, Normal16) {
  const Vec2f v0 = {-0.9119332250645995f, 0.3773891556349251f};
  const Vec2f v1 = {-0.8902579027620598f, 0.2887665479932624f};
  Vec2f v_out = {0.5465958902130417f, 0.8589725656846445f};
  const Vec2f *v_out_ptr = Vec2fAdd(&v0, &v1, &v_out);

  EXPECT_NEAR(-1.8021911278266594f, v_out.x, FLT_EPSILON);
  EXPECT_NEAR(0.6661557036281875f, v_out.y, FLT_EPSILON);

  EXPECT_NEAR(-1.8021911278266594f, v_out_ptr->x, FLT_EPSILON);
  EXPECT_NEAR(0.6661557036281875f, v_out_ptr->y, FLT_EPSILON);
}

TEST(Vec2fAdd3Test, Normal16) {
  const Vec2f v0 = {-0.8015777766883065f, 0.5815560228235377f};
  const Vec2f v1 = {0.8297001107787001f, -0.4168610969116791f};
  const Vec2f v2 = {0.0994054873567434f, -0.4635929843403468f};
  Vec2f v0c = {-0.8015777766883065f, 0.5815560228235377f};
  Vec2f v1c = {0.8297001107787001f, -0.4168610969116791f};
  Vec2f v2c = {0.0994054873567434f, -0.4635929843403468f};
  Vec2f v_out = {-0.2368007470592364f, -0.9580083543430833f};
  const Vec2f *v_out_ptr = Vec2fAdd3(&v0, &v1, &v2, &v_out);
  Vec2fAdd3(&v0c, &v1, &v2, &v0c);
  Vec2fAdd3(&v0, &v1c, &v2, &v1c);
  Vec2fAdd3(&v0, &v1, &v2c, &v2c);

  EXPECT_NEAR(0.1275278214471369f, v_out.x, FLT_EPSILON);
  EXPECT_NEAR(-0.2988980584284882f, v_out.y, FLT_EPSILON);

  EXPECT_NEAR(0.1275278214471369f, v0c.x, FLT_EPSILON);
  EXPECT_NEAR(-0.2988980584284882f, v0c.y, FLT_EPSILON);

  EXPECT_NEAR(0.1275278214471369f, v1c.x, FLT_EPSILON);
  EXPECT_NEAR(-0.2988980584284882f, v1c.y, FLT_EPSILON);

  EXPECT_NEAR(0.1275278214471369f, v2c.x, FLT_EPSILON);
  EXPECT_NEAR(-0.2988980584284882f, v2c.y, FLT_EPSILON);

  EXPECT_NEAR(0.1275278214471369f, v_out_ptr->x, FLT_EPSILON);
  EXPECT_NEAR(-0.2988980584284882f, v_out_ptr->y, FLT_EPSILON);
}

TEST(Vec2fSubTest, Normal16) {
  const Vec2f v0 = {-0.1853784116438575f, 0.9446665173392454f};
  const Vec2f v1 = {0.5991782778305039f, -0.7327342593353734f};
  Vec2f v_out = {-0.4216298718682872f, 0.4431406792549180f};
  const Vec2f *v_out_ptr = Vec2fSub(&v0, &v1, &v_out);

  EXPECT_NEAR(-0.7845566894743614f, v_out.x, FLT_EPSILON);
  EXPECT_NEAR(1.6774007766746188f, v_out.y, FLT_EPSILON);

  EXPECT_NEAR(-0.7845566894743614f, v_out_ptr->x, FLT_EPSILON);
  EXPECT_NEAR(1.6774007766746188f, v_out_ptr->y, FLT_EPSILON);
}

TEST(Vec2fLinCombTest, Normal16) {
  const Vec2f v0 = {-0.4901688671150555f, -0.7024596307856850f};
  const Vec2f v1 = {0.6612234070039194f, -0.2530545604837147f};
  Vec2f v_out = {0.0542253406892201f, 0.5008509289561713f};
  const Vec2f *v_out_ptr =
      Vec2fLinComb(0.1683422896967617f, &v0, -0.0884953560173947f, &v1, &v_out);

  EXPECT_NEAR(-0.1410313502380627f, v_out.x, FLT_EPSILON);
  EXPECT_NEAR(-0.0958595092441724f, v_out.y, FLT_EPSILON);

  EXPECT_NEAR(-0.1410313502380627f, v_out_ptr->x, FLT_EPSILON);
  EXPECT_NEAR(-0.0958595092441724f, v_out_ptr->y, FLT_EPSILON);
}

TEST(Vec2fLinComb3Test, Normal16) {
  const Vec2f v0 = {-0.3725807748776355f, -0.0200665056819946f};
  const Vec2f v1 = {0.9381997681473198f, 0.8686725455885040f};
  const Vec2f v2 = {0.8216334250435322f, 0.3319086289579189f};
  Vec2f v_out = {-0.4743294811130183f, 0.4374039918592605f};
  const Vec2f *v_out_ptr =
      Vec2fLinComb3(0.9831694296248310f, &v0, -0.8264583827004972f, &v1,
                    -0.3733219447029266f, &v2, &v_out);

  EXPECT_NEAR(-1.4484268790288155f, v_out.x, FLT_EPSILON);
  EXPECT_NEAR(-0.8615592568955814f, v_out.y, FLT_EPSILON);

  EXPECT_NEAR(-1.4484268790288155f, v_out_ptr->x, FLT_EPSILON);
  EXPECT_NEAR(-0.8615592568955814f, v_out_ptr->y, FLT_EPSILON);
}

TEST(Vec2fDotTest, Normal16) {
  const Vec2f v0 = {0.8023559404381342f, 0.5994963378868248f};
  const Vec2f v1 = {-0.6035516616368675f, 0.4882393605662985f};
  double v_dot = Vec2fDot(&v0, &v1);

  EXPECT_NEAR(-0.1915655524039464f, v_dot, FLT_EPSILON);
}

TEST(Vec2fMultTest, Normal16) {
  const Vec2f v0 = {0.0068347627991343f, 0.7632916385147215f};
  const Vec2f v1 = {0.6341114042945113f, 0.6508056931193578f};
  Vec2f v_out = {0.2720719884986933f, -0.8204497947114506f};
  const Vec2f *v_out_ptr = Vec2fMult(&v0, &v1, &v_out);

  EXPECT_NEAR(0.0043340010365789f, v_out.x, FLT_EPSILON);
  EXPECT_NEAR(0.4967545438557836f, v_out.y, FLT_EPSILON);

  EXPECT_NEAR(0.0043340010365789f, v_out_ptr->x, FLT_EPSILON);
  EXPECT_NEAR(0.4967545438557836f, v_out_ptr->y, FLT_EPSILON);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

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
#include "common/c_math/util.h"
#include "common/c_math/vec2.h"
#include "common/c_math/vec3.h"
#include "lib/util/test_util.h"

using ::test_util::Rand;

TEST(AngleToDcmTest, kRotationOrderXyz0) {
  const Mat3 dcm = {
      {{0.589756992877874, 0.771212109127361, 0.239621727076188},
       {-0.757879544555245, 0.426052106787664, 0.494062949680026},
       {0.278935987811510, -0.472981484877707, 0.835753091329309}}};
  Mat3 dcm_out;
  AngleToDcm(0.514994607366190, 0.282685942040181, 0.909510572151698,
             kRotationOrderXyz, &dcm_out);
  EXPECT_NEAR_MAT3(dcm, dcm_out, 1e-5);
}

TEST(AngleToDcmTest, kRotationOrderXyz1) {
  const Mat3 dcm = {
      {{0.781797643336233, 0.601336847116637, 0.164883113670671},
       {-0.521654343894280, 0.485922950191285, 0.701252901579470},
       {0.341568719835614, -0.634249858318645, 0.693583395744620}}};
  Mat3 dcm_out;
  AngleToDcm(0.740743389155394, 0.348585497527345, 0.588406222718084,
             kRotationOrderXyz, &dcm_out);
  EXPECT_NEAR_MAT3(dcm, dcm_out, 1e-5);
}

TEST(AngleToDcmTest, kRotationOrderXyz2) {
  const Mat3 dcm = {
      {{0.937800515915418, 0.339214686334105, -0.073916093809342},
       {-0.282488558471180, 0.869339257565658, 0.405523697936466},
       {0.201817756110932, -0.359419882352724, 0.911091072005375}}};
  Mat3 dcm_out;
  AngleToDcm(0.375750757154823, 0.203213512839378, 0.292579875490275,
             kRotationOrderXyz, &dcm_out);
  EXPECT_NEAR_MAT3(dcm, dcm_out, 1e-5);
}

TEST(AngleToDcmTest, kRotationOrderXyx0) {
  const Mat3 dcm = {
      {{0.974597844113083, 0.162849285800682, -0.153750292241471},
       {0.143434561560359, 0.073384267253498, 0.986935294672176},
       {0.172004560405001, -0.983918116224046, 0.048161932747839}}};
  Mat3 dcm_out;
  AngleToDcm(0.814130007200787, 0.225877999412891, 0.695072467403252,
             kRotationOrderXyx, &dcm_out);
  EXPECT_NEAR_MAT3(dcm, dcm_out, 1e-5);
}

TEST(AngleToDcmTest, kRotationOrderXzy0) {
  const Mat3 dcm = {
      {{0.668207704245513, 0.732572762842195, -0.129752114158859},
       {-0.724686467670882, 0.680363769393359, 0.109245891783451},
       {0.168309202248840, 0.021030654735739, 0.985509880214169}}};
  Mat3 dcm_out;
  AngleToDcm(0.159210814908665, 0.810579283183470, 0.246748762264296,
             kRotationOrderXzy, &dcm_out);
  EXPECT_NEAR_MAT3(dcm, dcm_out, 1e-5);
}

TEST(AngleToDcmTest, kRotationOrderXzx0) {
  const Mat3 dcm = {
      {{0.995421613227264, 0.083449650974738, 0.046604373960237},
       {-0.093560224527487, 0.750956210684713, 0.653690487937849},
       {0.019552398993407, -0.655057955746094, 0.755325610784747}}};
  Mat3 dcm_out;
  AngleToDcm(0.509325083780436, 0.095727578561412, 0.206016953045765,
             kRotationOrderXzx, &dcm_out);
  EXPECT_NEAR_MAT3(dcm, dcm_out, 1e-5);
}

TEST(AngleToDcmTest, kRotationOrderYxz0) {
  const Mat3 dcm = {
      {{0.403174435707920, 0.832450361326326, -0.380101000155576},
       {-0.567716615408908, 0.553288449100303, 0.609565202977364},
       {0.717738266348015, -0.029971453415718, 0.695667659877829}}};
  Mat3 dcm_out;
  AngleToDcm(0.801012087512567, 0.029975942396791, 0.984190913230052,
             kRotationOrderYxz, &dcm_out);
  EXPECT_NEAR_MAT3(dcm, dcm_out, 1e-5);
}

TEST(AngleToDcmTest, kRotationOrderYxy0) {
  const Mat3 dcm = {
      {{0.880503844076073, 0.316105148203653, -0.353257011036449},
       {0.038001967724135, 0.695738556099018, 0.717289141146266},
       {0.472513313057265, -0.645000307625043, 0.600587855477653}}};
  Mat3 dcm_out;
  AngleToDcm(0.052930498550375, 0.801348730252054, 0.455684409342011,
             kRotationOrderYxy, &dcm_out);
  EXPECT_NEAR_MAT3(dcm, dcm_out, 1e-5);
}

TEST(AngleToDcmTest, kRotationOrderYzx0) {
  const Mat3 dcm = {
      {{0.953517058520108, 0.293366502047779, -0.068857204324678},
       {-0.148238465009344, 0.655599198184908, 0.740415456909821},
       {0.262355820600256, -0.695791482281550, 0.668613218970139}}};
  Mat3 dcm_out;
  AngleToDcm(0.072088782781050, 0.297746389860671, 0.815130823417003,
             kRotationOrderYzx, &dcm_out);
  EXPECT_NEAR_MAT3(dcm, dcm_out, 1e-5);
}

TEST(AngleToDcmTest, kRotationOrderYzy0) {
  const Mat3 dcm = {
      {{0.377770210551601, 0.270752764960299, -0.885427923823364},
       {-0.421568976397449, 0.901714288457886, 0.095869390996786},
       {0.824359912998568, 0.337052343497625, 0.454781762589215}}};
  Mat3 dcm_out;
  AngleToDcm(0.223608007512242, 0.447077855049285, 0.894048919239390,
             kRotationOrderYzy, &dcm_out);
  EXPECT_NEAR_MAT3(dcm, dcm_out, 1e-5);
}

TEST(AngleToDcmTest, kRotationOrderZyx0) {
  const Mat3 dcm = {{{0.814598, 0.579891, -0.012546},
                     {-0.431413, 0.620199, 0.655161},
                     {0.387703, -0.528280, 0.755385}}};
  Mat3 dcm_out;
  AngleToDcm(0.618651, 0.012546, 0.714465, kRotationOrderZyx, &dcm_out);
  EXPECT_NEAR_MAT3(dcm, dcm_out, 1e-5);
}

TEST(AngleToDcmTest, kRotationOrderZyx1) {
  const Mat3 dcm = {{{-0.736547, -0.656573, 0.162513},
                     {0.671493, -0.680949, 0.292242},
                     {-0.081215, 0.324376, 0.942435}}};
  Mat3 dcm_out;
  AngleToDcm(-2.41354, -0.16324, 0.30069, kRotationOrderZyx, &dcm_out);
  EXPECT_NEAR_MAT3(dcm, dcm_out, 1e-5);
}

TEST(AngleToDcmTest, kRotationOrderZyx2) {
  const Mat3 dcm = {{{0.43752, 0.20482, -0.87557},
                     {-0.22415, 0.96782, 0.11440},
                     {0.87082, 0.14620, 0.46935}}};
  Mat3 dcm_out;
  AngleToDcm(0.43784, 1.06662, 0.23907, kRotationOrderZyx, &dcm_out);
  EXPECT_NEAR_MAT3(dcm, dcm_out, 1e-5);
}

TEST(AngleToDcmTest, kRotationOrderZyx3) {
  const Mat3 dcm = {{{0.77563, -0.62133, -0.11106},
                     {0.55282, 0.75365, -0.35554},
                     {0.30461, 0.21437, 0.92804}}};
  Mat3 dcm_out;
  AngleToDcm(-0.67539, 0.11129, -0.36586, kRotationOrderZyx, &dcm_out);
  EXPECT_NEAR_MAT3(dcm, dcm_out, 1e-5);
}

TEST(AngleToDcmTest, kRotationOrderZyx4) {
  const Mat3 dcm = {{{0.246801, 0.852726, -0.460377},
                     {-0.962786, 0.269762, -0.016472},
                     {0.110146, 0.447310, 0.887570}}};
  Mat3 dcm_out;
  AngleToDcm(1.289069, 0.478420, -0.018556, kRotationOrderZyx, &dcm_out);
  EXPECT_NEAR_MAT3(dcm, dcm_out, 1e-5);
}

TEST(AngleToDcmTest, kRotationOrderZyx5) {
  const Mat3 dcm = {{{-0.068573, 0.693429, -0.717254},
                     {-0.924035, -0.315185, -0.216374},
                     {-0.376108, 0.647931, 0.662366}}};
  Mat3 dcm_out;
  AngleToDcm(1.66937, 0.79985, -0.31574, kRotationOrderZyx, &dcm_out);
  EXPECT_NEAR_MAT3(dcm, dcm_out, 1e-5);
}

TEST(AngleToDcmTest, kRotationOrderZyx6) {
  const Mat3 dcm = {{{0.63686, -0.54062, -0.54967},
                     {0.70316, 0.69969, 0.12653},
                     {0.31619, -0.46709, 0.82574}}};
  Mat3 dcm_out;
  AngleToDcm(-0.70384, 0.58197, 0.15205, kRotationOrderZyx, &dcm_out);
  EXPECT_NEAR_MAT3(dcm, dcm_out, 1e-5);
}

TEST(AngleToDcmTest, kRotationOrderZyx7) {
  const Mat3 dcm = {{{0.32264, -0.91247, -0.25160},
                     {0.37654, 0.36761, -0.85034},
                     {0.86840, 0.17961, 0.46219}}};
  Mat3 dcm_out;
  AngleToDcm(-1.23093, 0.25434, -1.07293, kRotationOrderZyx, &dcm_out);
  EXPECT_NEAR_MAT3(dcm, dcm_out, 1e-5);
}

TEST(AngleToDcmTest, kRotationOrderZyx8) {
  const Mat3 dcm = {{{-0.304048, 0.745998, 0.592488},
                     {0.871280, -0.033760, 0.489623},
                     {0.385260, 0.665092, -0.639708}}};
  Mat3 dcm_out;
  AngleToDcm(-1.18378, -2.50745, -0.65328, kRotationOrderZyx, &dcm_out);
  EXPECT_NEAR_MAT3(dcm, dcm_out, 1e-5);
}

TEST(AngleToDcmTest, kRotationOrderZyx9) {
  const Mat3 dcm = {{{0.072763, 0.135104, -0.988156},
                     {-0.959568, -0.260634, -0.106293},
                     {-0.271908, 0.955938, 0.110677}}};
  Mat3 dcm_out;
  AngleToDcm(1.07677, 1.41674, -0.76519, kRotationOrderZyx, &dcm_out);
  EXPECT_NEAR_MAT3(dcm, dcm_out, 1e-5);
}

TEST(AngleToDcmTest, kRotationOrderZyz0) {
  const Mat3 dcm = {{{-0.032347, 0.952545, -0.302673},
                     {-0.966454, 0.047391, 0.252430},
                     {0.254794, 0.300685, 0.919058}}};
  Mat3 dcm_out;
  AngleToDcm(0.86783, 0.40511, 0.69513, kRotationOrderZyz, &dcm_out);
  EXPECT_NEAR_MAT3(dcm, dcm_out, 1e-5);
}

TEST(AngleToDcmTest, kRotationOrderZyz1) {
  const Mat3 dcm = {{{-0.542192, -0.834902, 0.094687},
                     {0.837542, -0.527952, 0.140679},
                     {-0.067463, 0.155579, 0.985517}}};
  Mat3 dcm_out;
  AngleToDcm(-1.16164, -0.17040, -0.97838, kRotationOrderZyz, &dcm_out);
  EXPECT_NEAR_MAT3(dcm, dcm_out, 1e-5);
}

TEST(AngleToDcmTest, kRotationOrderZyz2) {
  const Mat3 dcm = {{{0.6617482, -0.6163273, 0.4268840},
                     {0.5211315, 0.7874762, 0.3290945},
                     {-0.5389909, 0.0046850, 0.8422985}}};
  Mat3 dcm_out;
  AngleToDcm(-0.0086920, -0.5692629, -0.6567576, kRotationOrderZyz, &dcm_out);
  EXPECT_NEAR_MAT3(dcm, dcm_out, 1e-5);
}

TEST(AngleToDcmTest, kRotationOrderZyz3) {
  const Mat3 dcm = {{{0.983189, 0.151190, -0.102376},
                     {-0.064889, 0.813404, 0.578069},
                     {0.170671, -0.561708, 0.809540}}};
  Mat3 dcm_out;
  AngleToDcm(-1.27582, 0.62743, 1.39551, kRotationOrderZyz, &dcm_out);
  EXPECT_NEAR_MAT3(dcm, dcm_out, 1e-5);
}

TEST(AngleToDcmTest, kRotationOrderZyz4) {
  const Mat3 dcm = {{{0.9233244, 0.3836426, -0.0170413},
                     {-0.3834099, 0.9234495, 0.0154250},
                     {0.0216545, -0.0077084, 0.9997358}}};
  Mat3 dcm_out;
  AngleToDcm(-0.341987, 0.022988, 0.735653, kRotationOrderZyz, &dcm_out);
  EXPECT_NEAR_MAT3(dcm, dcm_out, 1e-5);
}

TEST(AngleToDcmTest, kRotationOrderZyz5) {
  const Mat3 dcm = {{{-0.017169, 0.881581, -0.471721},
                     {-0.774794, 0.286475, 0.563583},
                     {0.631980, 0.375163, 0.678125}}};
  Mat3 dcm_out;
  AngleToDcm(0.53572, 0.82559, 0.87390, kRotationOrderZyz, &dcm_out);
  EXPECT_NEAR_MAT3(dcm, dcm_out, 1e-5);
}

TEST(AngleToDcmTest, kRotationOrderZyz6) {
  const Mat3 dcm = {{{0.200334, 0.481341, 0.853333},
                     {-0.968968, 0.226072, 0.099961},
                     {-0.144800, -0.846878, 0.511695}}};
  Mat3 dcm_out;
  AngleToDcm(1.40145, -1.03364, -0.11661, kRotationOrderZyz, &dcm_out);
  EXPECT_NEAR_MAT3(dcm, dcm_out, 1e-5);
}

TEST(AngleToDcmTest, kRotationOrderZyz7) {
  const Mat3 dcm = {{{0.16965, -0.96716, 0.18924},
                     {0.91999, 0.22427, 0.32144},
                     {-0.35333, 0.11957, 0.92783}}};
  Mat3 dcm_out;
  AngleToDcm(-0.32630, -0.38225, -1.03871, kRotationOrderZyz, &dcm_out);
  EXPECT_NEAR_MAT3(dcm, dcm_out, 1e-5);
}

TEST(AngleToDcmTest, kRotationOrderZyz_8) {
  const Mat3 dcm = {{{-0.114501, 0.033221, 0.992867},
                     {0.594279, -0.798598, 0.095255},
                     {0.796067, 0.600947, 0.071698}}};
  Mat3 dcm_out;
  AngleToDcm(-2.494966, -1.499037, -0.095647, kRotationOrderZyz, &dcm_out);
  EXPECT_NEAR_MAT3(dcm, dcm_out, 1e-5);
}

TEST(AngleToDcmTest, kRotationOrderZyz_9) {
  const Mat3 dcm = {{{0.54573, 0.68325, -0.48513},
                     {-0.35310, 0.71253, 0.60631},
                     {0.75993, -0.15958, 0.63011}}};
  Mat3 dcm_out;
  AngleToDcm(-0.20699, 0.88910, 0.89598, kRotationOrderZyz, &dcm_out);
  EXPECT_NEAR_MAT3(dcm, dcm_out, 1e-5);
}

TEST(AngleToDcmTest, kRotationOrderZxy0) {
  const Mat3 dcm = {
      {{0.695947658559599, 0.546323073495880, -0.466034286197299},
       {-0.417967980093196, 0.835908827191996, 0.355751599067827},
       {0.583917480610253, -0.052797083144028, 0.810094342560950}}};
  Mat3 dcm_out;
  AngleToDcm(0.463660592626470, 0.363718154349476, 0.522047661588658,
             kRotationOrderZxy, &dcm_out);
  EXPECT_NEAR_MAT3(dcm, dcm_out, 1e-5);
}

TEST(AngleToDcmTest, kRotationOrderZxz0) {
  const Mat3 dcm = {
      {{0.244590796969728, 0.941625864581395, 0.231335412743171},
       {-0.963633517670812, 0.209574768801112, 0.165797647460204},
       {0.107637287494116, -0.263475136271528, 0.958642303942200}}};
  Mat3 dcm_out;
  AngleToDcm(0.387837445362298, 0.288603392253042, 0.948951281628741,
             kRotationOrderZxz, &dcm_out);
  EXPECT_NEAR_MAT3(dcm, dcm_out, 1e-5);
}

TEST(AngleToDcmToAngle, Normal0) {
  double r1_in, r2_in, r3_in, r1_out, r2_out, r3_out;
  Mat3 dcm;

  for (int32_t ord = 0; ord < 12; ++ord) {
    for (int32_t i = 0; i < 22; ++i) {
      r1_in = Rand();
      r2_in = Rand();
      r3_in = Rand();
      AngleToDcm(r1_in, r2_in, r3_in, (RotationOrder)ord, &dcm);
      DcmToAngle(&dcm, (RotationOrder)ord, &r1_out, &r2_out, &r3_out);

      EXPECT_NEAR(r1_in, r1_out, 1e-9);
      EXPECT_NEAR(r2_in, r2_out, 1e-9);
      EXPECT_NEAR(r3_in, r3_out, 1e-9);
    }
  }
}

TEST(SphToCart, Downwind) {
  Vec3 Xg;
  SphToCart(0.0, 0.0, -1.0, &Xg);

  EXPECT_NEAR(-1.0, Xg.x, 1e-9);
  EXPECT_NEAR(0.0, Xg.y, 1e-9);
  EXPECT_NEAR(0.0, Xg.z, 1e-9);
}

TEST(SphToCart, Upwind) {
  Vec3 Xg;
  SphToCart(0.0, 0.0, 1e7, &Xg);

  EXPECT_NEAR(1e7, Xg.x, 1e-9);
  EXPECT_NEAR(0.0, Xg.y, 1e-9);
  EXPECT_NEAR(0.0, Xg.z, 1e-9);
}

TEST(SphToCart, PositiveAzi) {
  Vec3 Xg;
  SphToCart(PI / 4.0, 0.0, -5.0, &Xg);

  EXPECT_NEAR(-5.0 / sqrt(2.0), Xg.x, 1e-9);
  EXPECT_NEAR(-5.0 / sqrt(2.0), Xg.y, 1e-9);
  EXPECT_NEAR(0.0, Xg.z, 1e-9);
}

TEST(SphToCart, NegativeAzi) {
  Vec3 Xg;
  SphToCart(-PI / 4.0, 0.0, -10.0, &Xg);

  EXPECT_NEAR(-10.0 / sqrt(2.0), Xg.x, 1e-9);
  EXPECT_NEAR(10.0 / sqrt(2.0), Xg.y, 1e-9);
  EXPECT_NEAR(0.0, Xg.z, 1e-9);
}

TEST(SphToCart, PositiveEle) {
  Vec3 Xg;
  SphToCart(0.0, PI / 6.0, -2.0, &Xg);

  EXPECT_NEAR(-sqrt(3.0), Xg.x, 1e-9);
  EXPECT_NEAR(0.0, Xg.y, 1e-9);
  EXPECT_NEAR(-1.0, Xg.z, 1e-9);
}

TEST(SphToCart, NegativeEle) {
  Vec3 Xg;
  SphToCart(0.0, -PI / 3.0, -2.0, &Xg);

  EXPECT_NEAR(-1.0, Xg.x, 1e-9);
  EXPECT_NEAR(0.0, Xg.y, 1e-9);
  EXPECT_NEAR(sqrt(3.0), Xg.z, 1e-9);
}

TEST(SphToCart, Recovery) {
  Vec3 Xg;
  double azi, ele, r;
  double azi2, ele2, r2;

  for (uint32_t i = 0U; i < test_util::kNumTests; ++i) {
    azi = Rand(-PI, PI);
    ele = Rand(-PI / 2.0, PI / 2.0);
    r = Rand(0.0, 200.0);

    SphToCart(azi, ele, r, &Xg);
    CartToSph(&Xg, &azi2, &ele2, &r2);

    EXPECT_NEAR(azi, azi2, 1e-9);
    EXPECT_NEAR(ele, ele2, 1e-9);
    EXPECT_NEAR(r, r2, 1e-9);

    for (int32_t j = -4; j <= 4; j += 2) {
      for (int32_t k = -4; k <= 4; k += 2) {
        // Check angles greater than the azi wrap value.
        SphToCart(azi + (double)j * PI, ele + (double)k * PI, r, &Xg);
        CartToSph(&Xg, &azi2, &ele2, &r2);

        EXPECT_NEAR(azi, azi2, 1e-9);
        EXPECT_NEAR(ele, ele2, 1e-9);
        EXPECT_NEAR(r, r2, 1e-9);
      }
    }

    // Another way to check angles greater than wrap values.
    SphToCart(azi + PI, PI - ele, r, &Xg);
    CartToSph(&Xg, &azi2, &ele2, &r2);

    EXPECT_NEAR(azi, azi2, 1e-9);
    EXPECT_NEAR(ele, ele2, 1e-9);
    EXPECT_NEAR(r, r2, 1e-9);
  }
}

TEST(SphToCart, RecoveryEdgeCases) {
  Vec3 Xg;
  double azi_v[2] = {-PI, PI};
  double ele_v[2] = {-PI / 2.0, PI / 2.0};
  double azi, ele, r;
  double azi2, ele2, r2;

  for (uint32_t i = 0; i < 2; ++i) {
    for (uint32_t j = 0; j < 2; ++j) {
      azi = azi_v[i];
      ele = ele_v[j];
      r = Rand(0.0, 200.0);

      SphToCart(azi, ele, r, &Xg);
      CartToSph(&Xg, &azi2, &ele2, &r2);

      EXPECT_NEAR(azi, azi2, 1e-9);
      EXPECT_NEAR(ele, ele2, 1e-9);
      EXPECT_NEAR(r, r2, 1e-9);
    }
  }
}

TEST(CartToSph, Recovery) {
  Vec3 Xg, Xg2;
  double azi, ele, r;

  for (uint32_t i = 0; i < test_util::kNumTests; ++i) {
    Xg.x = Rand(-200.0, 200.0);
    Xg.y = Rand(-200.0, 200.0);
    Xg.z = Rand(-200.0, 200.0);

    CartToSph(&Xg, &azi, &ele, &r);
    SphToCart(azi, ele, r, &Xg2);

    EXPECT_NEAR_VEC3(Xg, Xg2, 1e-9);
  }
}

TEST(CartToCyl, Recover) {
  Vec3 X, X2;
  double azi, r, z;

  for (uint32_t i = 0; i < test_util::kNumTests; ++i) {
    X.x = Rand(-200.0, 200.0);
    X.y = Rand(-200.0, 200.0);
    X.z = Rand(-200.0, 200.0);

    CartToCyl(&X, &azi, &r, &z);
    CylToCart(azi, r, z, &X2);

    EXPECT_NEAR_VEC3(X, X2, 1e-9);
  }
}

TEST(CylToCart, Recover) {
  Vec3 X;
  double azi, r, z, azi2, r2, z2;

  for (uint32_t i = 0; i < test_util::kNumTests; ++i) {
    azi = Rand(-3.0 * PI, 3.0 * PI);
    r = Rand(-200.0, 200.0);
    z = Rand(-200.0, 200.0);

    CylToCart(azi, r, z, &X);
    CartToCyl(&X, &azi2, &r2, &z2);

    if (r > 0) {
      EXPECT_NEAR(Wrap(azi, -PI, PI), azi2, 1e-9);
      EXPECT_NEAR(r, r2, 1e-9);
    } else {
      EXPECT_NEAR(Wrap(azi + PI, -PI, PI), azi2, 1e-9);
      EXPECT_NEAR(-r, r2, 1e-9);
    }
    EXPECT_NEAR(z, z2, 1e-9);
  }
}

TEST(CylToCart, Edge) {
  Vec3 X;
  double r, z, azi2, r2, z2;
  r = 123.0;
  z = Rand(-200.0, 200.0);
  double azi[2] = {-PI, PI};

  for (uint32_t i = 0; i < 2; ++i) {
    CylToCart(azi[i], r, z, &X);
    CartToCyl(&X, &azi2, &r2, &z2);

    EXPECT_NEAR(azi[i], azi2, 1e-9);
    EXPECT_NEAR(r, r2, 1e-9);
    EXPECT_NEAR(z, z2, 1e-9);
  }
}

TEST(PoseTransform, Recovery) {
  Mat3 dcm_a2b;
  Vec3 X_a_b_origin, X_b;
  Vec3 X_a;
  Vec3 X_b_recovered;

  for (int32_t i = 0; i < 222; ++i) {
    X_a_b_origin.x = Rand(-10.0, 10.0);
    X_a_b_origin.y = Rand(-10.0, 10.0);
    X_a_b_origin.z = Rand(-10.0, 10.0);
    X_b.x = Rand(-10.0, 10.0);
    X_b.y = Rand(-10.0, 10.0);
    X_b.z = Rand(-10.0, 10.0);
    AngleToDcm(Rand(-PI, PI), Rand(-PI, PI), Rand(-PI, PI), kRotationOrderZyx,
               &dcm_a2b);
    InversePoseTransform(&dcm_a2b, &X_a_b_origin, &X_b, &X_a);
    PoseTransform(&dcm_a2b, &X_a_b_origin, &X_a, &X_b_recovered);

    EXPECT_NEAR_VEC3(X_b, X_b_recovered, 1e-9);
  }
}

TEST(InversePoseTransform, ReuseInputs) {
  for (int32_t i = 0; i < 222; ++i) {
    Mat3 dcm_a2b;
    AngleToDcm(Rand(-PI, PI), Rand(-PI, PI), Rand(-PI, PI), kRotationOrderZyx,
               &dcm_a2b);
    Vec3 X_origin = {Rand(-10.0, 10.0), Rand(-10.0, 10.0), Rand(-10.0, 10.0)};
    Vec3 X_b = {Rand(-10.0, 10.0), Rand(-10.0, 10.0), Rand(-10.0, 10.0)};
    Vec3 X_b2 = X_b;
    Vec3 X_a;
    InversePoseTransform(&dcm_a2b, &X_origin, &X_b, &X_a);
    InversePoseTransform(&dcm_a2b, &X_origin, &X_b2, &X_b2);
    InversePoseTransform(&dcm_a2b, &X_origin, &X_b, &X_origin);
    EXPECT_NEAR_VEC3(X_a, X_b2, 1e-9);
    EXPECT_NEAR_VEC3(X_a, X_origin, 1e-9);
  }
}

TEST(PoseTransform, ReuseInputs) {
  for (int32_t i = 0; i < 222; ++i) {
    Mat3 dcm_a2b;
    AngleToDcm(Rand(-PI, PI), Rand(-PI, PI), Rand(-PI, PI), kRotationOrderZyx,
               &dcm_a2b);
    Vec3 X_origin = {Rand(-10.0, 10.0), Rand(-10.0, 10.0), Rand(-10.0, 10.0)};
    Vec3 X_a = {Rand(-10.0, 10.0), Rand(-10.0, 10.0), Rand(-10.0, 10.0)};
    Vec3 X_a2 = X_a;
    Vec3 X_b;
    PoseTransform(&dcm_a2b, &X_origin, &X_a, &X_b);
    PoseTransform(&dcm_a2b, &X_origin, &X_a2, &X_a2);
    PoseTransform(&dcm_a2b, &X_origin, &X_a, &X_origin);
    EXPECT_NEAR_VEC3(X_b, X_a2, 1e-9);
    EXPECT_NEAR_VEC3(X_b, X_origin, 1e-9);
  }
}

TEST(Vec3ToAxisAngle, CompareMATLAB1) {
  Vec3 v1 = {1.0, 2.0, 3.0}, v2 = {4.0, 5.0, 6.0};
  Vec3 axis_ans = {-0.408248290463863, 0.816496580927726, -0.408248290463863};
  Vec3 axis;
  double ang = Vec3ToAxisAngle(&v1, &v2, &axis);
  EXPECT_NEAR(ang, 0.225726128552735, 1e-9);
  EXPECT_NEAR_VEC3(axis, axis_ans, 1e-9);
}

TEST(Vec3ToAxisAngle, Equal) {
  Vec3 v1 = {1.0, 2.0, 3.0}, v2 = {1.0, 2.0, 3.0};
  Vec3 axis_ans = {0.0, 0.0, 0.0};
  Vec3 axis;
  double ang = Vec3ToAxisAngle(&v1, &v2, &axis);
  EXPECT_NEAR(ang, 0.0, 1e-9);
  EXPECT_NEAR_VEC3(axis, axis_ans, 1e-9);
}

TEST(Vec3ToAxisAngle, Parallel) {
  Vec3 v1 = {1.0, 2.0, 3.0}, v2 = {2.0, 4.0, 6.0};
  Vec3 axis_ans = {0.0, 0.0, 0.0};
  Vec3 axis;
  double ang = Vec3ToAxisAngle(&v1, &v2, &axis);
  EXPECT_NEAR(ang, 0.0, 1e-9);
  EXPECT_NEAR_VEC3(axis, axis_ans, 1e-9);
}

TEST(Vec3ToAxisAngle, Opposite) {
  Vec3 v1 = {1.0, 2.0, 3.0}, v2 = {-2.0, -4.0, -6.0};
  Vec3 axis_ans = {0.0, 0.0, 0.0};
  Vec3 axis;
  double ang = Vec3ToAxisAngle(&v1, &v2, &axis);
  EXPECT_NEAR(ang, 3.141592653589793, 1e-9);
  EXPECT_NEAR_VEC3(axis, axis_ans, 1e-9);
}

#if defined(NDEBUG)
// These tests will assert out when compiled without NDEBUG, as they
// try to normalize the zero vector.
TEST(Vec3ToAxisAngle, Zero0) {
  Vec3 v1 = {0.0, 0.0, 0.0}, v2 = {0.0, 0.0, 0.0};
  Vec3 axis_ans = {0.0, 0.0, 0.0};
  Vec3 axis;
  double ang = Vec3ToAxisAngle(&v1, &v2, &axis);
  EXPECT_NEAR(ang, 1.570796326794897, 1e-9);
  EXPECT_NEAR_VEC3(axis, axis_ans, 1e-9);
}

TEST(Vec3ToAxisAngle, Zero1) {
  Vec3 v1 = {1.0, 2.0, 3.0}, v2 = {0.0, 0.0, 0.0};
  Vec3 axis_ans = {0.0, 0.0, 0.0};
  Vec3 axis;
  double ang = Vec3ToAxisAngle(&v1, &v2, &axis);
  EXPECT_NEAR(ang, 1.570796326794897, 1e-9);
  EXPECT_NEAR_VEC3(axis, axis_ans, 1e-9);
}

TEST(Vec3ToAxisAngle, Zero2) {
  Vec3 v1 = {0.0, 0.0, 0.0}, v2 = {4.0, 5.0, 6.0};
  Vec3 axis_ans = {0.0, 0.0, 0.0};
  Vec3 axis;
  double ang = Vec3ToAxisAngle(&v1, &v2, &axis);
  EXPECT_NEAR(ang, 1.570796326794897, 1e-9);
  EXPECT_NEAR_VEC3(axis, axis_ans, 1e-9);
}
#endif  // defined(NDEBUG)

TEST(ProjectVec3ToPlane, InPlane1) {
  Vec3 V = {1.0, 1.0, 0.0};
  Vec3 N = {0.0, 0.0, 1.0};
  Vec3 W;
  ProjectVec3ToPlane(&V, &N, &W);

  EXPECT_NEAR(W.x, 1.0, 1e-9);
  EXPECT_NEAR(W.y, 1.0, 1e-9);
  EXPECT_NEAR(W.z, 0.0, 1e-9);
}

TEST(ProjectVec3ToPlane, InPlane2) {
  Vec3 V = {1.0, 1.0, 0.0};
  Vec3 N = {0.0, 0.0, 10.0};
  Vec3 W;
  ProjectVec3ToPlane(&V, &N, &W);

  EXPECT_NEAR(W.x, 1.0, 1e-9);
  EXPECT_NEAR(W.y, 1.0, 1e-9);
  EXPECT_NEAR(W.z, 0.0, 1e-9);
}

TEST(ProjectVec3ToPlane, InPlane3) {
  Vec3 V = {0.0, 1.0, 10.0};
  Vec3 N = {-1.0, 0.0, 0.0};
  Vec3 W;
  ProjectVec3ToPlane(&V, &N, &W);

  EXPECT_NEAR(W.x, 0.0, 1e-9);
  EXPECT_NEAR(W.y, 1.0, 1e-9);
  EXPECT_NEAR(W.z, 10.0, 1e-9);
}

TEST(ProjectVec3ToPlane, ToPlane1) {
  Vec3 V = {1.0, 1.0, 1.0};
  Vec3 N = {0.0, 1.0, 0.0};
  Vec3 W;
  ProjectVec3ToPlane(&V, &N, &W);

  EXPECT_NEAR(W.x, 1.0, 1e-9);
  EXPECT_NEAR(W.y, 0.0, 1e-9);
  EXPECT_NEAR(W.z, 1.0, 1e-9);
}

TEST(ProjectVec3ToPlane, ToPlane2) {
  Vec3 V = {1.0, 0.0, 0.0};
  Vec3 N = {1.0, 1.0, 0.0};
  Vec3 W;
  ProjectVec3ToPlane(&V, &N, &W);

  EXPECT_NEAR(W.x, 0.5, 1e-9);
  EXPECT_NEAR(W.y, -0.5, 1e-9);
  EXPECT_NEAR(W.z, 0.0, 1e-9);
}

TEST(ProjectVec3ToPlane, ToPlane3) {
  Vec3 V = {1.0, 0.0, 0.0};
  Vec3 N = {1.0, 1.0, 0.0};
  Vec3 W;
  ProjectVec3ToPlane(&V, &N, &W);

  EXPECT_NEAR(W.x, 0.5, 1e-9);
  EXPECT_NEAR(W.y, -0.5, 1e-9);
  EXPECT_NEAR(W.z, 0.0, 1e-9);
}

TEST(ProjectVec3ToPlane, V0) {
  Vec3 V = {0.0, 0.0, 0.0};
  Vec3 N = {1.0, 1.0, 1.0};
  Vec3 W;
  ProjectVec3ToPlane(&V, &N, &W);

  EXPECT_NEAR(W.x, 0.0, 1e-9);
  EXPECT_NEAR(W.y, 0.0, 1e-9);
  EXPECT_NEAR(W.z, 0.0, 1e-9);
}

TEST(Vec2ToAngle, SameDir1) {
  Vec2 v1 = {1, 0};
  Vec2 v2 = {1, 0};
  double c = Vec2ToAngle(&v1, &v2);
  EXPECT_NEAR(c, 0, 1e-9);
}

TEST(Vec2ToAngle, Angle90) {
  Vec2 v1 = {0, 1};
  Vec2 v2 = {1, 0};
  double c = Vec2ToAngle(&v1, &v2);
  EXPECT_NEAR(c, -PI / 2.0, 1e-9);
}

TEST(Vec2ToAngle, Angle90Again) {
  Vec2 v1 = {1, 0};
  Vec2 v2 = {0, -1};
  double c = Vec2ToAngle(&v1, &v2);
  EXPECT_NEAR(c, -PI / 2.0, 1e-9);
}

TEST(Vec2ToAngle, Angle135) {
  Vec2 v1 = {-1, 1};
  Vec2 v2 = {1, 0};
  double c = Vec2ToAngle(&v1, &v2);
  EXPECT_NEAR(c, -PI * 3 / 4, 1e-9);
}

TEST(Vec2ToAngle, Angle180Degrees0) {
  Vec2 v1 = {0, -1};
  Vec2 v2 = {0, 1};
  double c;
  c = Vec2ToAngle(&v1, &v2);
  EXPECT_NEAR(c, PI, 1e-9);
}

TEST(Vec2ToAngle, Angle180Degrees1) {
  Vec2 v1 = {1, 0};
  Vec2 v2 = {-1, 0};
  double c;
  c = Vec2ToAngle(&v1, &v2);
  EXPECT_NEAR(c, PI, 1e-9);
}

TEST(Vec2ToAngle, Negative180) {
  Vec2 v1 = {-1, 0.0000000001};
  Vec2 v2 = {1, 0};
  double c = Vec2ToAngle(&v1, &v2);
  EXPECT_NEAR(c, -PI, 1e-9);
}

TEST(Vec2ToAngle, Positive180) {
  Vec2 v1 = {-1, -0.0000000001};
  Vec2 v2 = {1, 0};
  double c = Vec2ToAngle(&v1, &v2);
  EXPECT_NEAR(c, PI, 1e-9);
}

TEST(Vec2ToAngle, SameDir2) {
  Vec2 v1 = {-1.0, -1.0};
  Vec2 v2 = {-1.0, -1.0};
  double c = Vec2ToAngle(&v1, &v2);
  EXPECT_NEAR(c, 0.0, 2e-8);  // I don't know why this is less precise.
}

TEST(Vec2ToAngle, Zero) {
  Vec2 v1 = {0.0, 1.0};
  Vec2 v2 = {0.0, 0.0};
  double c = Vec2ToAngle(&v1, &v2);
  EXPECT_NEAR(c, 0.0, 1e-9);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

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

#include "common/c_math/mat2.h"
#include "common/c_math/vec2.h"
#include "lib/util/test_util.h"

using ::test_util::Rand;
using ::test_util::RandNormal;

const double kConsistencyTol = 1e-4;

TEST(Mat2Vec2Axpby, Normal0) {
  const Vec2 x = {0.932268080463705, 0.182380305568693};
  const Vec2 y = {0.479243036938108, 0.159068875303512};
  const Vec2 zout = {0.133079102913542, 0.432877799721210};
  const Vec2 zout_trans = {0.191170700046199, 0.135932679572066};
  const Mat2 a = {{{0.0907478812993996, 0.0711614905230870},
                   {0.3896805156531172, 0.3169655639694510}}};

  const double b = 0.0740736801287379;

  Vec2 z;

  Mat2Vec2Axpby(&a, kNoTrans, &x, b, &y, &z);
  EXPECT_NEAR(zout.x, z.x, 1e-9);
  EXPECT_NEAR(zout.y, z.y, 1e-9);
  Mat2Vec2Axpby(&a, kTrans, &x, b, &y, &z);
  EXPECT_NEAR(zout_trans.x, z.x, 1e-9);
  EXPECT_NEAR(zout_trans.y, z.y, 1e-9);
}

TEST(Mat2Abpyc, Normal0) {
  const Mat2 a = {{{-0.21791932834625, 2.7874562066608504},
                   {-1.20153355179260, -0.0636516258042825}}};
  const Mat2 b = {{{-1.99211790907442, 0.0983102955720217},
                   {-0.39606794569933, 0.3231537765375129}}};
  const Mat2 c = {{{-0.07993577305536, 0.8615565232970752},
                   {0.18820511747443, 0.2976338700041187}}};
  const double y = -0.0779075336891960;
  const Mat2 zout = {{{-0.6636734578346, 0.812231542670893},
                      {2.4041442790224, -0.161880302632231}}};
  const Mat2 zout_atb = {{{0.9162375212116, -0.476825562342804},
                          {-5.5423936579071, 0.230278459555096}}};
  const Mat2 zout_abt = {{{0.7143842392352, 0.919966116957752},
                          {2.3726763002032, 0.432131741531526}}};
  const Mat2 zout_atbt = {{{0.3222254770479, -0.369090988055944},
                           {-5.5738616367263, -1.147779237514824}}};
  const Mat2 zout_abct = {{{-0.6636734578346, 0.864690690004688},
                           {2.3516851316886, -0.161880302632231}}};
  const Mat2 zout_atbct = {{{0.9162375212116, -0.424366415009009},
                            {-5.5948528052409, 0.230278459555096}}};
  const Mat2 zout_abtct = {{{0.7143842392352, 0.972425264291547},
                            {2.3202171528694, 0.432131741531526}}};
  const Mat2 zout_atbtct = {{{0.3222254770479, -0.316631840722149},
                             {-5.6263207840601, -1.147779237514824}}};

  Mat2 z;
  Mat2Abpyc(&a, kNoTrans, &b, kNoTrans, y, &c, kNoTrans, &z);
  EXPECT_NEAR_MAT2(zout, z, 1e-9);
  Mat2Abpyc(&a, kTrans, &b, kNoTrans, y, &c, kNoTrans, &z);
  EXPECT_NEAR_MAT2(zout_atb, z, 1e-9);
  Mat2Abpyc(&a, kNoTrans, &b, kTrans, y, &c, kNoTrans, &z);
  EXPECT_NEAR_MAT2(zout_abt, z, 1e-9);
  Mat2Abpyc(&a, kTrans, &b, kTrans, y, &c, kNoTrans, &z);
  EXPECT_NEAR_MAT2(zout_atbt, z, 1e-9);

  Mat2Abpyc(&a, kNoTrans, &b, kNoTrans, y, &c, kTrans, &z);
  EXPECT_NEAR_MAT2(zout_abct, z, 1e-9);
  Mat2Abpyc(&a, kTrans, &b, kNoTrans, y, &c, kTrans, &z);
  EXPECT_NEAR_MAT2(zout_atbct, z, 1e-9);
  Mat2Abpyc(&a, kNoTrans, &b, kTrans, y, &c, kTrans, &z);
  EXPECT_NEAR_MAT2(zout_abtct, z, 1e-9);
  Mat2Abpyc(&a, kTrans, &b, kTrans, y, &c, kTrans, &z);
  EXPECT_NEAR_MAT2(zout_atbtct, z, 1e-9);
}

TEST(Mat2Trans, Normal0) {
  const Mat2 a = {{{0.777067128052834, -0.808676720780742},
                   {1.961109517776196, 1.472997548264610}}};
  const Mat2 zout = {{{0.777067128052834, 1.961109517776196},
                      {-0.808676720780742, 1.472997548264610}}};
  Mat2 z;
  Mat2Trans(&a, &z);
  EXPECT_NEAR_MAT2(zout, z, 1e-9);
}

TEST(Mat2Add, Normal0) {
  const Mat2 a = {{{0.2488911716321847, 0.0425190632838913},
                   {-0.5133203122990397, 1.1403745559303446}}};
  const Mat2 b = {{{-0.171588609342766, 1.871142405355631},
                   {1.630537705276971, 0.710307064054608}}};
  const Mat2 zout = {{{0.0773025622894183, 1.9136614686395224},
                      {1.1172173929779314, 1.8506816199849530}}};

  const Mat2 zout_atb = {{{0.0773025622894183, 1.3578220930565914},
                          {1.6730567685608624, 1.8506816199849530}}};

  const Mat2 zout_abt = {{{0.0773025622894183, 1.6730567685608624},
                          {1.3578220930565914, 1.8506816199849530}}};
  const Mat2 zout_atbt = {{{0.0773025622894183, 1.1172173929779314},
                           {1.9136614686395224, 1.8506816199849530}}};

  Mat2 z;
  Mat2Add(&a, kNoTrans, &b, kNoTrans, &z);
  EXPECT_NEAR_MAT2(zout, z, 1e-9);
  Mat2Add(&a, kTrans, &b, kNoTrans, &z);
  EXPECT_NEAR_MAT2(zout_atb, z, 1e-9);
  Mat2Add(&a, kNoTrans, &b, kTrans, &z);
  EXPECT_NEAR_MAT2(zout_abt, z, 1e-9);
  Mat2Add(&a, kTrans, &b, kTrans, &z);
  EXPECT_NEAR_MAT2(zout_atbt, z, 1e-9);
}

TEST(Mat2Mult, Normal0) {
  const Mat2 a = {{{0.0467704881437452, 0.7051976082871888},
                   {0.6916047560366408, 0.7516314745929150}}};
  const Mat2 b = {{{0.861600502549458, 0.788232100005367},
                   {0.717137606603607, 0.887731645445828}}};
  const Mat2 zout = {{{0.546021201078797, 0.662892233257069},
                      {1.134910202104268, 1.212392114933704}}};
  const Mat2 zout_atb = {{{0.536273255548922, 0.650825428162388},
                          {1.146621810434421, 1.223106437408216}}};
  const Mat2 zout_abt = {{{0.596156867788108, 0.659567109096337},
                          {1.188347061015270, 1.163222825169031}}};
  const Mat2 zout_atbt = {{{0.585442545313596, 0.647500304001656},
                           {1.200058669345422, 1.172970770698905}}};

  Mat2 z;
  Mat2Mult(&a, kNoTrans, &b, kNoTrans, &z);
  EXPECT_NEAR_MAT2(zout, z, 1e-9);
  Mat2Mult(&a, kTrans, &b, kNoTrans, &z);
  EXPECT_NEAR_MAT2(zout_atb, z, 1e-9);
  Mat2Mult(&a, kNoTrans, &b, kTrans, &z);
  EXPECT_NEAR_MAT2(zout_abt, z, 1e-9);
  Mat2Mult(&a, kTrans, &b, kTrans, &z);
  EXPECT_NEAR_MAT2(zout_atbt, z, 1e-9);
}

TEST(Mat2Trans, Consistency0) {
  double x, y;
  Mat2 a, b, c, d;
  for (int32_t t = 0; t < test_util::kNumTests; t++) {
    for (int32_t i = 0; i < 2; i++)
      for (int32_t j = 0; j < 2; j++) {
        a.d[i][j] = RandNormal();
        b.d[i][j] = RandNormal();
      }
    c = a;
    Mat2Trans(&c, &c);
    Mat2Trans(&c, &c);
    EXPECT_NEAR_MAT2(c, a, kConsistencyTol);
    Mat2Add(&a, kTrans, &b, kTrans, &c);
    Mat2Add(&a, kNoTrans, &b, kNoTrans, &d);
    Mat2Trans(&d, &d);
    EXPECT_NEAR_MAT2(d, c, kConsistencyTol);
    Mat2Mult(&a, kTrans, &b, kTrans, &c);
    Mat2Mult(&b, kNoTrans, &a, kNoTrans, &d);
    Mat2Trans(&d, &d);
    EXPECT_NEAR_MAT2(d, c, kConsistencyTol);
    x = Mat2Det(&a);
    Mat2Trans(&a, &c);
    y = Mat2Det(&c);
    EXPECT_NEAR(x, y, 1e-9 * (fabs(x) + fabs(y)) / 2);
    Mat2Inv(&a, &c);
    Mat2Trans(&a, &d);
    Mat2Trans(&c, &c);
    Mat2Inv(&d, &d);
    EXPECT_NEAR_MAT2(d, c, kConsistencyTol);
    Mat2Mult(&a, kNoTrans, &a, kTrans, &c);
    Mat2Trans(&c, &d);
    EXPECT_NEAR_MAT2(d, c, kConsistencyTol);
    Mat2Mult(&a, kNoTrans, &b, kTrans, &c);
    Mat2Mult(&a, kTrans, &b, kNoTrans, &d);
    x = Mat2Trace(&c);
    y = Mat2Trace(&d);
    EXPECT_NEAR(x, y, 1e-9 * (fabs(x) + fabs(y)) / 2.0);
  }
}

TEST(Mat2Mult, Consistency0) {
  double x, y;
  Mat2 a, b, c;
  for (int32_t t = 0; t < test_util::kNumTests; ++t) {
    do {
      for (int32_t i = 0; i < 2; ++i) {
        for (int32_t j = 0; j < 2; ++j) {
          a.d[i][j] = RandNormal();
        }
      }
    } while (fabs(Mat2Det(&a)) < kConsistencyTol);
    Mat2Inv(&a, &b);
    Mat2Mult(&a, kNoTrans, &b, kNoTrans, &c);
    EXPECT_NEAR_MAT2(c, kMat2Identity, kConsistencyTol);
    Mat2Mult(&b, kNoTrans, &a, kNoTrans, &c);
    EXPECT_NEAR_MAT2(c, kMat2Identity, kConsistencyTol);
    x = Mat2Det(&a);
    y = Mat2Det(&b);
    EXPECT_NEAR(x * y, Mat2Det(&c), 1e-9 * (fabs(x) + fabs(y)) / 2.0);
  }
}

TEST(Mat2Vec2Axpby, Consistency0) {
  double b;
  Vec2 w, x, y, z;
  Mat2 A, Ainv;
  for (int32_t t = 0; t < test_util::kNumTests; t++) {
    b = RandNormal();
    x.x = RandNormal();
    x.y = RandNormal();
    y.x = RandNormal();
    y.y = RandNormal();
    do {
      for (int32_t i = 0; i < 2; i++) {
        for (int32_t j = 0; j < 2; j++) {
          A.d[i][j] = RandNormal();
        }
      }
    } while (fabs(Mat2Det(&A)) < kConsistencyTol);
    Mat2Inv(&A, &Ainv);
    // z := Ax + by
    Mat2Vec2Axpby(&A, kNoTrans, &x, b, &y, &z);
    // z := A^(-1) (Ax + by) = x + A^(-1) b y
    Mat2Vec2Axpby(&Ainv, kNoTrans, &z, 0, &y, &z);
    // w := A^(-1) y
    Mat2Vec2Axpby(&Ainv, kNoTrans, &y, 0, &y, &w);
    // z := x + A^(-1) b y - b A^(-1) y = x
    Mat2Vec2Axpby(&kMat2Identity, kNoTrans, &z, -b, &w, &z);
    EXPECT_NEAR(x.x, z.x, 1e-9 * (fabs(x.x) + fabs(z.x)) / 2);
    EXPECT_NEAR(x.y, z.y, 1e-9 * (fabs(x.y) + fabs(z.y)) / 2);
  }
}

TEST(Mat2Abpyc, Consistency0) {
  double z;
  Mat2 a, b, c, d, e, f;
  for (int32_t t = 0; t < test_util::kNumTests; t++) {
    z = RandNormal();
    for (int32_t i = 0; i < 2; i++) {
      for (int32_t j = 0; j < 2; j++) {
        c.d[i][j] = RandNormal();
      }
    }
    do {
      for (int32_t i = 0; i < 2; i++) {
        for (int32_t j = 0; j < 2; j++) {
          a.d[i][j] = RandNormal();
        }
      }
    } while (fabs(Mat2Det(&a)) < kConsistencyTol);
    do {
      for (int32_t i = 0; i < 2; i++) {
        for (int32_t j = 0; j < 2; j++) {
          b.d[i][j] = RandNormal();
        }
      }
    } while (fabs(Mat2Det(&b)) < kConsistencyTol);
    Mat2Inv(&a, &d);
    Mat2Inv(&b, &e);
    Mat2Abpyc(&a, kNoTrans, &b, kNoTrans, z, &c, kNoTrans, &f);
    Mat2Abpyc(&d, kNoTrans, &f, kNoTrans, 0, &kMat2Identity, kNoTrans, &f);
    Mat2Abpyc(&f, kNoTrans, &e, kNoTrans, 0, &kMat2Identity, kTrans, &f);
    Mat2Abpyc(&a, kNoTrans, &f, kNoTrans, -1, &a, kNoTrans, &f);
    Mat2Abpyc(&f, kNoTrans, &b, kNoTrans, 0, &kMat2Identity, kTrans, &f);
    Mat2Abpyc(&kMat2Zero, kTrans, &kMat2Zero, kNoTrans, 1.0 / z, &f, kNoTrans,
              &f);
    EXPECT_NEAR_MAT2(f, c,
                     1e-9 * (fabs(Mat2Trace(&f)) + fabs(Mat2Trace(&c))) / 2.0);
  }
}

TEST(Mat2Scale, Normal0) {
  const Mat2 m = {{{-0.6697314303043371, 0.7505048561176924},
                   {0.5294508026386400, -0.0273353126958942}}};
  Mat2 m_out = {{{-0.6743653225123754, 0.9282670835958706},
                 {0.5688424014751889, -0.2855714742966673}}};
  const Mat2 *m_out_ptr = Mat2Scale(&m, -0.6059724753538160, &m_out);

  EXPECT_NEAR(0.4058388126437709, m_out.d[0][0], 1e-9);
  EXPECT_NEAR(-0.4547852854266976, m_out.d[0][1], 1e-9);
  EXPECT_NEAR(-0.3208326134530014, m_out.d[1][0], 1e-9);
  EXPECT_NEAR(0.0165644470989016, m_out.d[1][1], 1e-9);

  EXPECT_NEAR(0.4058388126437709, m_out_ptr->d[0][0], 1e-9);
  EXPECT_NEAR(-0.4547852854266976, m_out_ptr->d[0][1], 1e-9);
  EXPECT_NEAR(-0.3208326134530014, m_out_ptr->d[1][0], 1e-9);
  EXPECT_NEAR(0.0165644470989016, m_out_ptr->d[1][1], 1e-9);
}

TEST(Mat2Vec2Mult, Normal0) {
  const Mat2 m = {{{0.4428799268866348, -0.6239045704111352},
                   {0.9784210854088435, -0.0624111365993389}}};
  const Vec2 v = {0.6788589197943256, -0.8517109755427212};
  Vec2 v_out = {-0.1781980856741117, 0.7127813455973886};
  const Vec2 *v_out_ptr = Mat2Vec2Mult(&m, &v, &v_out);

  EXPECT_NEAR(0.8320393590752811, v_out.x, 1e-9);
  EXPECT_NEAR(0.7173661311823921, v_out.y, 1e-9);

  EXPECT_NEAR(0.8320393590752811, v_out_ptr->x, 1e-9);
  EXPECT_NEAR(0.7173661311823921, v_out_ptr->y, 1e-9);
}

TEST(Mat2TransVec2Mult, Normal0) {
  const Mat2 m = {{{0.4776718904266422, -0.9776383180119288},
                   {-0.3491450805711762, 0.8123547838501166}}};
  const Vec2 v = {0.6385452707050261, 0.7589517598109956};
  Vec2 v_out = {-0.4776683990167558, -0.3137448994398406};
  const Vec2 *v_out_ptr = Mat2TransVec2Mult(&m, &v, &v_out);

  EXPECT_NEAR(0.0400308532518157, v_out.x, 1e-9);
  EXPECT_NEAR(-0.0077282316326066, v_out.y, 1e-9);

  EXPECT_NEAR(0.0400308532518157, v_out_ptr->x, 1e-9);
  EXPECT_NEAR(-0.0077282316326066, v_out_ptr->y, 1e-9);
}

TEST(Mat2Vec2LeftDivide, Normal0) {
  const Mat2 m = {{{0.5272088603376204, -0.3216560669296342},
                   {-0.2317801320548354, -0.6665030464928268}}};
  const Vec2 v = {0.7801410833280593, 0.7993279599342769};
  Vec2 v_out = {-0.1229447015805785, -0.3245529766435649};
  const Vec2 *v_out_ptr = Mat2Vec2LeftDivide(&m, &v, &v_out);

  EXPECT_NEAR(0.6171241162888512, v_out.x, 1e-9);
  EXPECT_NEAR(-1.4138946161772310, v_out.y, 1e-9);

  EXPECT_NEAR(0.6171241162888512, v_out_ptr->x, 1e-9);
  EXPECT_NEAR(-1.4138946161772310, v_out_ptr->y, 1e-9);
}

TEST(Mat2Vec2LeftDivide, Singular0) {
  const Mat2 m = {{{-0.0859699957804489, 0.0000000000000000},
                   {0.3156328447725847, 0.0000000000000000}}};
  const Vec2 v = {0.8749989430331648, 0.6797791469296055};
  Vec2 v_out = {0.6780880565931580, 0.5177502544499486};
  const Vec2 *v_out_ptr = Mat2Vec2LeftDivide(&m, &v, &v_out);

  EXPECT_DOUBLE_EQ(INFINITY, v_out.x);
  EXPECT_DOUBLE_EQ(INFINITY, v_out.y);

  EXPECT_DOUBLE_EQ(INFINITY, v_out_ptr->x);
  EXPECT_DOUBLE_EQ(INFINITY, v_out_ptr->y);
}

TEST(Mat2Det, Normal0) {
  const Mat2 m = {{{0.0921845356380242, -0.3154948622537805},
                   {0.9232168059097821, -0.2805449521912180}}};
  double m_det = Mat2Det(&m);

  EXPECT_NEAR(0.2654082528675428, m_det, 1e-9);
}

TEST(Mat2Inv, Normal0) {
  const Mat2 m = {{{-0.5482804377276447, -0.2137693106080421},
                   {0.5110861330953875, -0.9985179005315223}}};
  Mat2 m_out = {{{0.1447656309214067, 0.8267323506025690},
                 {-0.7832936459601016, -0.4630896978405428}}};
  const Mat2 *m_out_ptr = Mat2Inv(&m, &m_out);

  EXPECT_NEAR(-1.5204566776425497, m_out.d[0][0], 1e-9);
  EXPECT_NEAR(0.3255094131172073, m_out.d[0][1], 1e-9);
  EXPECT_NEAR(-0.7782377496705268, m_out.d[1][0], 1e-9);
  EXPECT_NEAR(-0.8348740191037369, m_out.d[1][1], 1e-9);

  EXPECT_NEAR(-1.5204566776425497, m_out_ptr->d[0][0], 1e-9);
  EXPECT_NEAR(0.3255094131172073, m_out_ptr->d[0][1], 1e-9);
  EXPECT_NEAR(-0.7782377496705268, m_out_ptr->d[1][0], 1e-9);
  EXPECT_NEAR(-0.8348740191037369, m_out_ptr->d[1][1], 1e-9);
}

TEST(Mat2Inv, Reuse0) {
  Mat2 m = {{{-0.6347025929098731, 0.6301585116722517},
             {0.2728499055224709, -0.3476322941084966}}};
  const Mat2 *m_out_ptr = Mat2Inv(&m, &m);

  EXPECT_NEAR(-7.1375911357924586, m.d[0][0], 1e-9);
  EXPECT_NEAR(-12.9384233953024435, m.d[0][1], 1e-9);
  EXPECT_NEAR(-5.6021580850344792, m.d[1][0], 1e-9);
  EXPECT_NEAR(-13.0317225349728343, m.d[1][1], 1e-9);

  EXPECT_NEAR(-7.1375911357924586, m_out_ptr->d[0][0], 1e-9);
  EXPECT_NEAR(-12.9384233953024435, m_out_ptr->d[0][1], 1e-9);
  EXPECT_NEAR(-5.6021580850344792, m_out_ptr->d[1][0], 1e-9);
  EXPECT_NEAR(-13.0317225349728343, m_out_ptr->d[1][1], 1e-9);
}

TEST(Mat2Inv, Singular0) {
  const Mat2 m = {{{0.0000000000000000, 0.9386916079676948},
                   {0.0000000000000000, -0.3787731303830468}}};
  Mat2 m_out = {{{0.7818112712584868, 0.2177322247418358},
                 {-0.3667268052856405, 0.0893780782819287}}};
  const Mat2 *m_out_ptr = Mat2Inv(&m, &m_out);

  EXPECT_DOUBLE_EQ(INFINITY, m_out.d[0][0]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out.d[0][1]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out.d[1][0]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out.d[1][1]);

  EXPECT_DOUBLE_EQ(INFINITY, m_out_ptr->d[0][0]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out_ptr->d[0][1]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out_ptr->d[1][0]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out_ptr->d[1][1]);
}

TEST(Mat2Trace, Normal0) {
  const Mat2 m = {{{-0.8809363795927085, -0.5311061206969749},
                   {0.6706151412986765, -0.1360765394398662}}};
  double m_trace = Mat2Trace(&m);

  EXPECT_NEAR(-1.0170129190325747, m_trace, 1e-9);
}

TEST(Mat2Diag, Normal0) {
  const Mat2 m = {{{0.2748859906535102, 0.4704754533938411},
                   {0.2815705100933237, -0.4820479424670043}}};
  Vec2 v_out = {0.1748472228561222, 0.9132668800889023};
  const Vec2 *v_out_ptr = Mat2Diag(&m, &v_out);

  EXPECT_NEAR(0.2748859906535102, v_out.x, 1e-9);
  EXPECT_NEAR(-0.4820479424670043, v_out.y, 1e-9);

  EXPECT_NEAR(0.2748859906535102, v_out_ptr->x, 1e-9);
  EXPECT_NEAR(-0.4820479424670043, v_out_ptr->y, 1e-9);
}

TEST(Mat2Scale, Normal2) {
  const Mat2 m = {{{-0.6322148417010760, 0.8544983065633378},
                   {0.1870680672946425, 0.4831245676797560}}};
  Mat2 m_out = {{{-0.4556864631471920, 0.5304621010796087},
                 {-0.3237730525048528, -0.6898094790158280}}};
  const Mat2 *m_out_ptr = Mat2Scale(&m, -0.0981289336214031, &m_out);

  EXPECT_NEAR(0.0620385682357507, m_out.d[0][0], 1e-9);
  EXPECT_NEAR(-0.0838510076043551, m_out.d[0][1], 1e-9);
  EXPECT_NEAR(-0.0183567899582401, m_out.d[1][0], 1e-9);
  EXPECT_NEAR(-0.0474084986327158, m_out.d[1][1], 1e-9);

  EXPECT_NEAR(0.0620385682357507, m_out_ptr->d[0][0], 1e-9);
  EXPECT_NEAR(-0.0838510076043551, m_out_ptr->d[0][1], 1e-9);
  EXPECT_NEAR(-0.0183567899582401, m_out_ptr->d[1][0], 1e-9);
  EXPECT_NEAR(-0.0474084986327158, m_out_ptr->d[1][1], 1e-9);
}

TEST(Mat2Vec2Mult, Normal2) {
  const Mat2 m = {{{-0.1091004394974653, 0.9651005224386344},
                   {0.1543456989547094, 0.7682479310023049}}};
  const Vec2 v = {-0.0814639196470872, 0.4292439453822356};
  Vec2 v_out = {-0.4418555420166654, 0.1636806507390034};
  const Vec2 *v_out_ptr = Mat2Vec2Mult(&m, &v, &v_out);

  EXPECT_NEAR(0.4231513053786997, v_out.x, 1e-9);
  EXPECT_NEAR(0.3171921673176489, v_out.y, 1e-9);

  EXPECT_NEAR(0.4231513053786997, v_out_ptr->x, 1e-9);
  EXPECT_NEAR(0.3171921673176489, v_out_ptr->y, 1e-9);
}

TEST(Mat2TransVec2Mult, Normal2) {
  const Mat2 m = {{{0.6493007138167206, -0.8032227535241343},
                   {-0.2953881073592957, 0.6011255512581961}}};
  const Vec2 v = {-0.6679643994577207, 0.4157278704695062};
  Vec2 v_out = {-0.2590513679987814, 0.8269445016631487};
  const Vec2 *v_out_ptr = Mat2TransVec2Mult(&m, &v, &v_out);

  EXPECT_NEAR(-0.5565108302065530, v_out.x, 1e-9);
  EXPECT_NEAR(0.7864288494979030, v_out.y, 1e-9);

  EXPECT_NEAR(-0.5565108302065530, v_out_ptr->x, 1e-9);
  EXPECT_NEAR(0.7864288494979030, v_out_ptr->y, 1e-9);
}

TEST(Mat2Vec2LeftDivide, Normal2) {
  const Mat2 m = {{{0.7050199291165613, -0.0484895277883166},
                   {-0.6104410604732968, 0.2726552218252978}}};
  const Vec2 v = {0.4776847284830608, 0.1368428019863432};
  Vec2 v_out = {0.8003333656251677, 0.2727795003995599};
  const Vec2 *v_out_ptr = Mat2Vec2LeftDivide(&m, &v, &v_out);

  EXPECT_NEAR(0.8416706332213315, v_out.x, 1e-9);
  EXPECT_NEAR(2.3862851829630221, v_out.y, 1e-9);

  EXPECT_NEAR(0.8416706332213315, v_out_ptr->x, 1e-9);
  EXPECT_NEAR(2.3862851829630221, v_out_ptr->y, 1e-9);
}

TEST(Mat2Vec2LeftDivide, Singular2) {
  const Mat2 m = {{{0.4313647161503320, 0.0000000000000000},
                   {-0.8879301337867000, 0.0000000000000000}}};
  const Vec2 v = {-0.5949464057546661, 0.9707482796488287};
  Vec2 v_out = {0.4591570221953765, -0.4663374206697792};
  const Vec2 *v_out_ptr = Mat2Vec2LeftDivide(&m, &v, &v_out);

  EXPECT_TRUE(isnan(v_out.x));
  EXPECT_TRUE(isnan(v_out.y));

  EXPECT_TRUE(isnan(v_out_ptr->x));
  EXPECT_TRUE(isnan(v_out_ptr->y));
}

TEST(Mat2Det, Normal2) {
  const Mat2 m = {{{-0.1355750206327893, -0.8928958585345581},
                   {-0.9629321167570475, -0.6794540886175122}}};
  double m_det = Mat2Det(&m);

  EXPECT_NEAR(-0.7676810970189311, m_det, 1e-9);
}

TEST(Mat2Inv, Normal2) {
  const Mat2 m = {{{0.9828146020906550, -0.4679328954475719},
                   {0.3274723835381594, -0.2852245695934001}}};
  Mat2 m_out = {{{0.8302727422429084, 0.8903321243997573},
                 {-0.8093607454787493, 0.3068984310716625}}};
  const Mat2 *m_out_ptr = Mat2Inv(&m, &m_out);

  EXPECT_NEAR(2.2443116812749961, m_out.d[0][0], 1e-9);
  EXPECT_NEAR(-3.6819663355190757, m_out.d[0][1], 1e-9);
  EXPECT_NEAR(2.5767418869887679, m_out.d[1][0], 1e-9);
  EXPECT_NEAR(-7.7333530387794935, m_out.d[1][1], 1e-9);

  EXPECT_NEAR(2.2443116812749961, m_out_ptr->d[0][0], 1e-9);
  EXPECT_NEAR(-3.6819663355190757, m_out_ptr->d[0][1], 1e-9);
  EXPECT_NEAR(2.5767418869887679, m_out_ptr->d[1][0], 1e-9);
  EXPECT_NEAR(-7.7333530387794935, m_out_ptr->d[1][1], 1e-9);
}

TEST(Mat2Inv, Reuse2) {
  Mat2 m = {{{0.5314649658963433, -0.1618634940117223},
             {0.4437311302257148, 0.9052970408499839}}};
  const Mat2 *m_out_ptr = Mat2Inv(&m, &m);

  EXPECT_NEAR(1.6371909022419278, m.d[0][0], 1e-9);
  EXPECT_NEAR(0.2927231923262143, m.d[0][1], 1e-9);
  EXPECT_NEAR(-0.8024687330966893, m.d[1][0], 1e-9);
  EXPECT_NEAR(0.9611316151092942, m.d[1][1], 1e-9);

  EXPECT_NEAR(1.6371909022419278, m_out_ptr->d[0][0], 1e-9);
  EXPECT_NEAR(0.2927231923262143, m_out_ptr->d[0][1], 1e-9);
  EXPECT_NEAR(-0.8024687330966893, m_out_ptr->d[1][0], 1e-9);
  EXPECT_NEAR(0.9611316151092942, m_out_ptr->d[1][1], 1e-9);
}

TEST(Mat2Inv, Singular2) {
  const Mat2 m = {{{0.0000000000000000, 0.8495616566237214},
                   {0.0000000000000000, -0.1780832035980637}}};
  Mat2 m_out = {{{-0.8767094338929864, -0.2595525432989230},
                 {0.0884388403215941, 0.1421659585783706}}};
  const Mat2 *m_out_ptr = Mat2Inv(&m, &m_out);

  EXPECT_DOUBLE_EQ(INFINITY, m_out.d[0][0]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out.d[0][1]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out.d[1][0]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out.d[1][1]);

  EXPECT_DOUBLE_EQ(INFINITY, m_out_ptr->d[0][0]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out_ptr->d[0][1]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out_ptr->d[1][0]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out_ptr->d[1][1]);
}

TEST(Mat2Trace, Normal2) {
  const Mat2 m = {{{0.6641351721479534, -0.4991300676262460},
                   {0.2627319306098281, -0.2635917657771114}}};
  double m_trace = Mat2Trace(&m);

  EXPECT_NEAR(0.4005434063708420, m_trace, 1e-9);
}

TEST(Mat2Diag, Normal2) {
  const Mat2 m = {{{-0.0719843056382403, -0.2714741516249053},
                   {-0.0478365182235365, 0.4586099851621306}}};
  Vec2 v_out = {0.4209657295622657, -0.2757647270550745};
  const Vec2 *v_out_ptr = Mat2Diag(&m, &v_out);

  EXPECT_NEAR(-0.0719843056382403, v_out.x, 1e-9);
  EXPECT_NEAR(0.4586099851621306, v_out.y, 1e-9);

  EXPECT_NEAR(-0.0719843056382403, v_out_ptr->x, 1e-9);
  EXPECT_NEAR(0.4586099851621306, v_out_ptr->y, 1e-9);
}

TEST(Mat2Scale, Normal4) {
  const Mat2 m = {{{0.8387934325445197, 0.5204831494003621},
                   {-0.4899503921253598, 0.2059818832780891}}};
  Mat2 m_out = {{{-0.6155754876344377, -0.8902710031078034},
                 {-0.1330807046440610, -0.9864714817284823}}};
  const Mat2 *m_out_ptr = Mat2Scale(&m, -0.2157640380145536, &m_out);

  EXPECT_NEAR(-0.1809814580658936, m_out.d[0][0], 1e-9);
  EXPECT_NEAR(-0.1123015460331543, m_out.d[0][1], 1e-9);
  EXPECT_NEAR(0.1057136750317816, m_out.d[1][0], 1e-9);
  EXPECT_NEAR(-0.0444434828939229, m_out.d[1][1], 1e-9);

  EXPECT_NEAR(-0.1809814580658936, m_out_ptr->d[0][0], 1e-9);
  EXPECT_NEAR(-0.1123015460331543, m_out_ptr->d[0][1], 1e-9);
  EXPECT_NEAR(0.1057136750317816, m_out_ptr->d[1][0], 1e-9);
  EXPECT_NEAR(-0.0444434828939229, m_out_ptr->d[1][1], 1e-9);
}

TEST(Mat2Vec2Mult, Normal4) {
  const Mat2 m = {{{-0.3273083590639492, -0.3145197134074813},
                   {-0.6285986614429353, -0.3539621136563056}}};
  const Vec2 v = {0.4072378519899020, 0.1769455916469476};
  Vec2 v_out = {-0.4577814333809722, 0.3903441963750189};
  const Vec2 *v_out_ptr = Mat2Vec2Mult(&m, &v, &v_out);

  EXPECT_NEAR(-0.1889452298570574, v_out.x, 1e-9);
  EXPECT_NEAR(-0.3186212042712677, v_out.y, 1e-9);

  EXPECT_NEAR(-0.1889452298570574, v_out_ptr->x, 1e-9);
  EXPECT_NEAR(-0.3186212042712677, v_out_ptr->y, 1e-9);
}

TEST(Mat2TransVec2Mult, Normal4) {
  const Mat2 m = {{{0.0957510953563705, -0.0380983883363539},
                   {0.6318237361782353, -0.7417700143514236}}};
  const Vec2 v = {0.3519158942178831, -0.0837912008222326};
  Vec2 v_out = {0.3706367969403259, 0.2792271766229395};
  const Vec2 *v_out_ptr = Mat2TransVec2Mult(&m, &v, &v_out);

  EXPECT_NEAR(-0.0192449372176849, v_out.x, 1e-9);
  EXPECT_NEAR(0.0487463718367823, v_out.y, 1e-9);

  EXPECT_NEAR(-0.0192449372176849, v_out_ptr->x, 1e-9);
  EXPECT_NEAR(0.0487463718367823, v_out_ptr->y, 1e-9);
}

TEST(Mat2Vec2LeftDivide, Normal4) {
  const Mat2 m = {{{-0.0012628852554923, 0.6175006234915170},
                   {0.7576399347598597, -0.4382801416955571}}};
  const Vec2 v = {0.0738847621804106, 0.4926905045777901};
  Vec2 v_out = {0.1624919266307088, -0.0245369493378524};
  const Vec2 *v_out_ptr = Mat2Vec2LeftDivide(&m, &v, &v_out);

  EXPECT_NEAR(0.7203646152224639, v_out.x, 1e-9);
  EXPECT_NEAR(0.1211245741075450, v_out.y, 1e-9);

  EXPECT_NEAR(0.7203646152224639, v_out_ptr->x, 1e-9);
  EXPECT_NEAR(0.1211245741075450, v_out_ptr->y, 1e-9);
}

TEST(Mat2Vec2LeftDivide, Singular4) {
  const Mat2 m = {{{0.8769432856718877, 0.0000000000000000},
                   {0.0604588500823193, 0.0000000000000000}}};
  const Vec2 v = {-0.6397077096296380, 0.6663107625472908};
  Vec2 v_out = {-0.0163795760313388, -0.4765838978587489};
  const Vec2 *v_out_ptr = Mat2Vec2LeftDivide(&m, &v, &v_out);

  EXPECT_TRUE(isnan(v_out.x));
  EXPECT_TRUE(isnan(v_out.y));

  EXPECT_TRUE(isnan(v_out_ptr->x));
  EXPECT_TRUE(isnan(v_out_ptr->y));
}

TEST(Mat2Det, Normal4) {
  const Mat2 m = {{{0.8746138791472107, 0.3699639609256624},
                   {0.1565346963768544, 0.2226722972850745}}};
  double m_det = Mat2Det(&m);

  EXPECT_NEAR(0.1368400854132430, m_det, 1e-9);
}

TEST(Mat2Inv, Normal4) {
  const Mat2 m = {{{0.3362270024434344, -0.8747057885706964},
                   {-0.4048279835986641, 0.8229336615951020}}};
  Mat2 m_out = {{{0.6101558740346182, 0.4417746820731403},
                 {-0.3730971693905272, 0.2824361006514127}}};
  const Mat2 *m_out_ptr = Mat2Inv(&m, &m_out);

  EXPECT_NEAR(-10.6304512749110831, m_out.d[0][0], 1e-9);
  EXPECT_NEAR(-11.2992306661268884, m_out.d[0][1], 1e-9);
  EXPECT_NEAR(-5.2294666693115568, m_out.d[1][0], 1e-9);
  EXPECT_NEAR(-4.3432963476744142, m_out.d[1][1], 1e-9);

  EXPECT_NEAR(-10.6304512749110831, m_out_ptr->d[0][0], 1e-9);
  EXPECT_NEAR(-11.2992306661268884, m_out_ptr->d[0][1], 1e-9);
  EXPECT_NEAR(-5.2294666693115568, m_out_ptr->d[1][0], 1e-9);
  EXPECT_NEAR(-4.3432963476744142, m_out_ptr->d[1][1], 1e-9);
}

TEST(Mat2Inv, Reuse4) {
  Mat2 m = {{{-0.7906228866953275, -0.4469089924325409},
             {-0.6469425944807701, 0.0902537697487182}}};
  const Mat2 *m_out_ptr = Mat2Inv(&m, &m);

  EXPECT_NEAR(-0.2503702828473340, m.d[0][0], 1e-9);
  EXPECT_NEAR(-1.2397568672630590, m.d[0][1], 1e-9);
  EXPECT_NEAR(-1.7946640989856153, m.d[1][0], 1e-9);
  EXPECT_NEAR(2.1932432996273397, m.d[1][1], 1e-9);

  EXPECT_NEAR(-0.2503702828473340, m_out_ptr->d[0][0], 1e-9);
  EXPECT_NEAR(-1.2397568672630590, m_out_ptr->d[0][1], 1e-9);
  EXPECT_NEAR(-1.7946640989856153, m_out_ptr->d[1][0], 1e-9);
  EXPECT_NEAR(2.1932432996273397, m_out_ptr->d[1][1], 1e-9);
}

TEST(Mat2Inv, Singular4) {
  const Mat2 m = {{{0.0000000000000000, -0.7231486437370183},
                   {0.0000000000000000, 0.1331393282066655}}};
  Mat2 m_out = {{{-0.7949035472981232, 0.2654583729276112},
                 {0.9653375347005009, -0.0791519640807870}}};
  const Mat2 *m_out_ptr = Mat2Inv(&m, &m_out);

  EXPECT_DOUBLE_EQ(INFINITY, m_out.d[0][0]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out.d[0][1]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out.d[1][0]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out.d[1][1]);

  EXPECT_DOUBLE_EQ(INFINITY, m_out_ptr->d[0][0]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out_ptr->d[0][1]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out_ptr->d[1][0]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out_ptr->d[1][1]);
}

TEST(Mat2Trace, Normal4) {
  const Mat2 m = {{{0.4612948137599537, -0.1041340740050318},
                   {0.1747501257580404, -0.8451005676262389}}};
  double m_trace = Mat2Trace(&m);

  EXPECT_NEAR(-0.3838057538662851, m_trace, 1e-9);
}

TEST(Mat2Diag, Normal4) {
  const Mat2 m = {{{0.5355962392473346, -0.9386620534935473},
                   {0.3593758791842752, 0.3549840332628260}}};
  Vec2 v_out = {0.4665822997822560, -0.0025929178770325};
  const Vec2 *v_out_ptr = Mat2Diag(&m, &v_out);

  EXPECT_NEAR(0.5355962392473346, v_out.x, 1e-9);
  EXPECT_NEAR(0.3549840332628260, v_out.y, 1e-9);

  EXPECT_NEAR(0.5355962392473346, v_out_ptr->x, 1e-9);
  EXPECT_NEAR(0.3549840332628260, v_out_ptr->y, 1e-9);
}

TEST(Mat2Scale, Normal6) {
  const Mat2 m = {{{0.8665393158738439, -0.6649556020867833},
                   {0.8204401174718983, 0.6202426090454871}}};
  Mat2 m_out = {{{0.9397862247231707, -0.0136289341424298},
                 {-0.3457222173952559, 0.6648475111443595}}};
  const Mat2 *m_out_ptr = Mat2Scale(&m, -0.7340358378126215, &m_out);

  EXPECT_NEAR(-0.6360709127250329, m_out.d[0][0], 1e-9);
  EXPECT_NEAR(0.4881012424859681, m_out.d[0][1], 1e-9);
  EXPECT_NEAR(-0.6022324490035704, m_out.d[1][0], 1e-9);
  EXPECT_NEAR(-0.4552803031777904, m_out.d[1][1], 1e-9);

  EXPECT_NEAR(-0.6360709127250329, m_out_ptr->d[0][0], 1e-9);
  EXPECT_NEAR(0.4881012424859681, m_out_ptr->d[0][1], 1e-9);
  EXPECT_NEAR(-0.6022324490035704, m_out_ptr->d[1][0], 1e-9);
  EXPECT_NEAR(-0.4552803031777904, m_out_ptr->d[1][1], 1e-9);
}

TEST(Mat2Vec2Mult, Normal6) {
  const Mat2 m = {{{0.4604454959856734, 0.0764106988239714},
                   {-0.1622816209213547, 0.4943194329272200}}};
  const Vec2 v = {-0.8906530497644567, -0.5787185834866699};
  Vec2 v_out = {-0.7999411826521785, 0.9768043223823968};
  const Vec2 *v_out_ptr = Mat2Vec2Mult(&m, &v, &v_out);

  EXPECT_NEAR(-0.4543174766365832, v_out.x, 1e-9);
  EXPECT_NEAR(-0.1415352214192507, v_out.y, 1e-9);

  EXPECT_NEAR(-0.4543174766365832, v_out_ptr->x, 1e-9);
  EXPECT_NEAR(-0.1415352214192507, v_out_ptr->y, 1e-9);
}

TEST(Mat2TransVec2Mult, Normal6) {
  const Mat2 m = {{{-0.2659646744350770, 0.2782082903181109},
                   {0.1483415497328027, 0.2372394156619753}}};
  const Vec2 v = {-0.4426037633686168, -0.2550889077091276};
  Vec2 v_out = {0.2000242630177747, 0.3726588029790090};
  const Vec2 *v_out_ptr = Mat2TransVec2Mult(&m, &v, &v_out);

  EXPECT_NEAR(0.0798766819388542, v_out.x, 1e-9);
  EXPECT_NEAR(-0.1836531797019096, v_out.y, 1e-9);

  EXPECT_NEAR(0.0798766819388542, v_out_ptr->x, 1e-9);
  EXPECT_NEAR(-0.1836531797019096, v_out_ptr->y, 1e-9);
}

TEST(Mat2Vec2LeftDivide, Normal6) {
  const Mat2 m = {{{-0.4578574766248014, 0.5433114340047727},
                   {-0.1419799707560441, 0.4443860024596427}}};
  const Vec2 v = {0.4127283419006080, -0.4081864335185321};
  Vec2 v_out = {-0.6695430937475382, 0.6896207328888930};
  const Vec2 *v_out_ptr = Mat2Vec2LeftDivide(&m, &v, &v_out);

  EXPECT_NEAR(-3.2074370640234569, v_out.x, 1e-9);
  EXPECT_NEAR(-1.9433066057224901, v_out.y, 1e-9);

  EXPECT_NEAR(-3.2074370640234569, v_out_ptr->x, 1e-9);
  EXPECT_NEAR(-1.9433066057224901, v_out_ptr->y, 1e-9);
}

TEST(Mat2Vec2LeftDivide, Singular6) {
  const Mat2 m = {{{0.0000000000000000, 0.0844258254739434},
                   {0.0000000000000000, -0.8729705838797879}}};
  const Vec2 v = {0.4561294593271890, -0.5646856751322842};
  Vec2 v_out = {-0.3152153998678353, -0.6196941881828104};
  const Vec2 *v_out_ptr = Mat2Vec2LeftDivide(&m, &v, &v_out);

  EXPECT_TRUE(isnan(v_out.x));
  EXPECT_TRUE(isnan(v_out.y));

  EXPECT_TRUE(isnan(v_out_ptr->x));
  EXPECT_TRUE(isnan(v_out_ptr->y));
}

TEST(Mat2Det, Normal6) {
  const Mat2 m = {{{-0.1729890343904745, 0.4005428011281380},
                   {0.4278627615590771, -0.1956276327251512}}};
  double m_det = Mat2Det(&m);

  EXPECT_NEAR(-0.1375359137280751, m_det, 1e-9);
}

TEST(Mat2Inv, Normal6) {
  const Mat2 m = {{{0.2548007914487267, 0.4312010847922654},
                   {-0.0149571392877252, -0.6580568367939685}}};
  Mat2 m_out = {{{0.6790044084101987, 0.8840605228128280},
                 {-0.8824666818619287, 0.0926917781470906}}};
  const Mat2 *m_out_ptr = Mat2Inv(&m, &m_out);

  EXPECT_NEAR(4.0816340927485388, m_out.d[0][0], 1e-9);
  EXPECT_NEAR(2.6745486865434773, m_out.d[0][1], 1e-9);
  EXPECT_NEAR(-0.0927724874711423, m_out.d[1][0], 1e-9);
  EXPECT_NEAR(-1.5804160660396707, m_out.d[1][1], 1e-9);

  EXPECT_NEAR(4.0816340927485388, m_out_ptr->d[0][0], 1e-9);
  EXPECT_NEAR(2.6745486865434773, m_out_ptr->d[0][1], 1e-9);
  EXPECT_NEAR(-0.0927724874711423, m_out_ptr->d[1][0], 1e-9);
  EXPECT_NEAR(-1.5804160660396707, m_out_ptr->d[1][1], 1e-9);
}

TEST(Mat2Inv, Reuse6) {
  Mat2 m = {{{-0.7100250404104207, 0.8110148361256642},
             {-0.9130481682821368, 0.3834566085376019}}};
  const Mat2 *m_out_ptr = Mat2Inv(&m, &m);

  EXPECT_NEAR(0.8189460753023806, m.d[0][0], 1e-9);
  EXPECT_NEAR(-1.7320797249788085, m.d[0][1], 1e-9);
  EXPECT_NEAR(1.9499917261260606, m.d[1][0], 1e-9);
  EXPECT_NEAR(-1.5163963986123579, m.d[1][1], 1e-9);

  EXPECT_NEAR(0.8189460753023806, m_out_ptr->d[0][0], 1e-9);
  EXPECT_NEAR(-1.7320797249788085, m_out_ptr->d[0][1], 1e-9);
  EXPECT_NEAR(1.9499917261260606, m_out_ptr->d[1][0], 1e-9);
  EXPECT_NEAR(-1.5163963986123579, m_out_ptr->d[1][1], 1e-9);
}

TEST(Mat2Inv, Singular6) {
  const Mat2 m = {{{0.0000000000000000, 0.8416184664545279},
                   {0.0000000000000000, 0.0615993527032781}}};
  Mat2 m_out = {{{-0.3858964383291306, -0.7984290028155576},
                 {0.2172432753645261, -0.9623695241086312}}};
  const Mat2 *m_out_ptr = Mat2Inv(&m, &m_out);

  EXPECT_DOUBLE_EQ(INFINITY, m_out.d[0][0]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out.d[0][1]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out.d[1][0]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out.d[1][1]);

  EXPECT_DOUBLE_EQ(INFINITY, m_out_ptr->d[0][0]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out_ptr->d[0][1]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out_ptr->d[1][0]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out_ptr->d[1][1]);
}

TEST(Mat2Trace, Normal6) {
  const Mat2 m = {{{-0.0823945703946420, 0.0525141421057953},
                   {0.6438351337235997, 0.6805308284039773}}};
  double m_trace = Mat2Trace(&m);

  EXPECT_NEAR(0.5981362580093352, m_trace, 1e-9);
}

TEST(Mat2Diag, Normal6) {
  const Mat2 m = {{{0.6500730567115489, -0.9231522111691379},
                   {-0.3382775499339110, -0.5698200860760736}}};
  Vec2 v_out = {0.5961943770511413, -0.8691392788804582};
  const Vec2 *v_out_ptr = Mat2Diag(&m, &v_out);

  EXPECT_NEAR(0.6500730567115489, v_out.x, 1e-9);
  EXPECT_NEAR(-0.5698200860760736, v_out.y, 1e-9);

  EXPECT_NEAR(0.6500730567115489, v_out_ptr->x, 1e-9);
  EXPECT_NEAR(-0.5698200860760736, v_out_ptr->y, 1e-9);
}

TEST(Mat2Scale, Normal8) {
  const Mat2 m = {{{0.0265131317811242, 0.2070927303098975},
                   {0.2604059592145875, -0.5932429332677649}}};
  Mat2 m_out = {{{-0.7117598028144037, 0.1771231397623334},
                 {0.4553167338688793, 0.8778756229556248}}};
  const Mat2 *m_out_ptr = Mat2Scale(&m, -0.7376681244106751, &m_out);

  EXPECT_NEAR(-0.0195578921932350, m_out.d[0][0], 1e-9);
  EXPECT_NEAR(-0.1527657059467878, m_out.d[0][1], 1e-9);
  EXPECT_NEAR(-0.1920931755191875, m_out.d[1][0], 1e-9);
  EXPECT_NEAR(0.4376164019035194, m_out.d[1][1], 1e-9);

  EXPECT_NEAR(-0.0195578921932350, m_out_ptr->d[0][0], 1e-9);
  EXPECT_NEAR(-0.1527657059467878, m_out_ptr->d[0][1], 1e-9);
  EXPECT_NEAR(-0.1920931755191875, m_out_ptr->d[1][0], 1e-9);
  EXPECT_NEAR(0.4376164019035194, m_out_ptr->d[1][1], 1e-9);
}

TEST(Mat2Vec2Mult, Normal8) {
  const Mat2 m = {{{-0.5050948357794418, -0.8143119860321315},
                   {0.7418381831810437, 0.4917556371702487}}};
  const Vec2 v = {-0.1820713115744095, -0.6444467999848495};
  Vec2 v_out = {0.0370479482270973, 0.8730697102907277};
  const Vec2 *v_out_ptr = Mat2Vec2Mult(&m, &v, &v_out);

  EXPECT_NEAR(0.6167440328075386, v_out.x, 1e-9);
  EXPECT_NEAR(-0.4519777977366272, v_out.y, 1e-9);

  EXPECT_NEAR(0.6167440328075386, v_out_ptr->x, 1e-9);
  EXPECT_NEAR(-0.4519777977366272, v_out_ptr->y, 1e-9);
}

TEST(Mat2TransVec2Mult, Normal8) {
  const Mat2 m = {{{-0.2171324788562543, -0.3203441865623340},
                   {-0.5341541796463292, 0.3685974251811117}}};
  const Vec2 v = {0.6709558643390057, -0.8959101866202022};
  Vec2 v_out = {-0.0324046123959361, -0.1939376593676281};
  const Vec2 *v_out_ptr = Mat2TransVec2Mult(&m, &v, &v_out);

  EXPECT_NEAR(0.3328678607438348, v_out.x, 1e-9);
  EXPECT_NEAR(-0.5451669985626423, v_out.y, 1e-9);

  EXPECT_NEAR(0.3328678607438348, v_out_ptr->x, 1e-9);
  EXPECT_NEAR(-0.5451669985626423, v_out_ptr->y, 1e-9);
}

TEST(Mat2Vec2LeftDivide, Normal8) {
  const Mat2 m = {{{0.3171697787694006, -0.5144432921912041},
                   {0.1642682882538689, 0.1359312338095209}}};
  const Vec2 v = {-0.6684078572137182, -0.1847437614659417};
  Vec2 v_out = {0.1440392072499419, -0.0849516954342062};
  const Vec2 *v_out_ptr = Mat2Vec2LeftDivide(&m, &v, &v_out);

  EXPECT_NEAR(-1.4566501798922862, v_out.x, 1e-9);
  EXPECT_NEAR(0.4012151485807696, v_out.y, 1e-9);

  EXPECT_NEAR(-1.4566501798922862, v_out_ptr->x, 1e-9);
  EXPECT_NEAR(0.4012151485807696, v_out_ptr->y, 1e-9);
}

TEST(Mat2Vec2LeftDivide, Singular8) {
  const Mat2 m = {{{0.7147758211728306, 0.0000000000000000},
                   {-0.5625180233219644, 0.0000000000000000}}};
  const Vec2 v = {-0.4367490700290706, -0.4450135099651709};
  Vec2 v_out = {-0.5611105017513240, -0.6619112083974450};
  const Vec2 *v_out_ptr = Mat2Vec2LeftDivide(&m, &v, &v_out);

  EXPECT_DOUBLE_EQ(-INFINITY, v_out.x);
  EXPECT_DOUBLE_EQ(-INFINITY, v_out.y);

  EXPECT_DOUBLE_EQ(-INFINITY, v_out_ptr->x);
  EXPECT_DOUBLE_EQ(-INFINITY, v_out_ptr->y);
}

TEST(Mat2Det, Normal8) {
  const Mat2 m = {{{-0.9454627050148714, -0.3062249274355027},
                   {-0.6958919614583861, -0.7129723113627859}}};
  double m_det = Mat2Det(&m);

  EXPECT_NEAR(0.4609892647012209, m_det, 1e-9);
}

TEST(Mat2Inv, Normal8) {
  const Mat2 m = {{{0.3730041764049932, -0.9310019906471316},
                   {-0.0923376217563134, -0.2791182008253985}}};
  Mat2 m_out = {{{-0.6606047458746718, 0.8361121798808238},
                 {0.3394473550474406, -0.5415886135755241}}};
  const Mat2 *m_out_ptr = Mat2Inv(&m, &m_out);

  EXPECT_NEAR(1.4684344244078567, m_out.d[0][0], 1e-9);
  EXPECT_NEAR(-4.8979800250062659, m_out.d[0][1], 1e-9);
  EXPECT_NEAR(-0.4857861008488711, m_out.d[1][0], 1e-9);
  EXPECT_NEAR(-1.9623663790510917, m_out.d[1][1], 1e-9);

  EXPECT_NEAR(1.4684344244078567, m_out_ptr->d[0][0], 1e-9);
  EXPECT_NEAR(-4.8979800250062659, m_out_ptr->d[0][1], 1e-9);
  EXPECT_NEAR(-0.4857861008488711, m_out_ptr->d[1][0], 1e-9);
  EXPECT_NEAR(-1.9623663790510917, m_out_ptr->d[1][1], 1e-9);
}

TEST(Mat2Inv, Reuse8) {
  Mat2 m = {{{0.7652067484846761, -0.9410012315887581},
             {-0.6742992829927796, -0.2777324828770862}}};
  const Mat2 *m_out_ptr = Mat2Inv(&m, &m);

  EXPECT_NEAR(0.3278862116134887, m.d[0][0], 1e-9);
  EXPECT_NEAR(-1.1109299342771290, m.d[0][1], 1e-9);
  EXPECT_NEAR(-0.7960661824783452, m.d[1][0], 1e-9);
  EXPECT_NEAR(-0.9033899789559559, m.d[1][1], 1e-9);

  EXPECT_NEAR(0.3278862116134887, m_out_ptr->d[0][0], 1e-9);
  EXPECT_NEAR(-1.1109299342771290, m_out_ptr->d[0][1], 1e-9);
  EXPECT_NEAR(-0.7960661824783452, m_out_ptr->d[1][0], 1e-9);
  EXPECT_NEAR(-0.9033899789559559, m_out_ptr->d[1][1], 1e-9);
}

TEST(Mat2Inv, Singular8) {
  const Mat2 m = {{{0.0000000000000000, -0.6273524376414306},
                   {0.0000000000000000, -0.9583382232439992}}};
  Mat2 m_out = {{{-0.6230508127061960, 0.6392466843447342},
                 {-0.4801479426715003, 0.1166105632460013}}};
  const Mat2 *m_out_ptr = Mat2Inv(&m, &m_out);

  EXPECT_DOUBLE_EQ(INFINITY, m_out.d[0][0]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out.d[0][1]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out.d[1][0]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out.d[1][1]);

  EXPECT_DOUBLE_EQ(INFINITY, m_out_ptr->d[0][0]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out_ptr->d[0][1]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out_ptr->d[1][0]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out_ptr->d[1][1]);
}

TEST(Mat2Trace, Normal8) {
  const Mat2 m = {{{-0.5832223896949085, 0.3984667885063089},
                   {0.3105996761308616, -0.2623937779638090}}};
  double m_trace = Mat2Trace(&m);

  EXPECT_NEAR(-0.8456161676587175, m_trace, 1e-9);
}

TEST(Mat2Diag, Normal8) {
  const Mat2 m = {{{0.1234620432803775, -0.7972984186617873},
                   {0.1873792168237958, -0.1878706704904893}}};
  Vec2 v_out = {-0.9015130698290086, 0.2986314906208609};
  const Vec2 *v_out_ptr = Mat2Diag(&m, &v_out);

  EXPECT_NEAR(0.1234620432803775, v_out.x, 1e-9);
  EXPECT_NEAR(-0.1878706704904893, v_out.y, 1e-9);

  EXPECT_NEAR(0.1234620432803775, v_out_ptr->x, 1e-9);
  EXPECT_NEAR(-0.1878706704904893, v_out_ptr->y, 1e-9);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

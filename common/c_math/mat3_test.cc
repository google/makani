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

#include "common/c_math/mat3.h"
#include "common/c_math/vec3.h"
#include "lib/util/test_util.h"

using ::test_util::RandNormal;

const double kConsistencyTol = 1e-4;

TEST(Mat3Vec3AxpbyTest, Normal0) {
  const Vec3 x = {0.825843077904185, 0.672115930455797, 0.644116443680800};
  const Vec3 y = {0.8274137142351754, 0.0960662714801508, 0.5076550506700549};
  const Vec3 zout = {1.58040207996670, 1.04161517599284, 1.37733227966570};

  const Vec3 zout_trans = {1.42763067601224, 1.49266839151783,
                           1.10254524866129};

  const Mat3 a = {{{0.160823390695128, 0.880874297750638, 0.700743221696630},
                   {0.604439679573222, 0.591496501959142, 0.152087273279646},
                   {0.752014551556382, 0.497927845734393, 0.269579054355709}}};

  const double b = 0.488482737115042;

  Vec3 z;
  Mat3Vec3Axpby(&a, kNoTrans, &x, b, &y, &z);
  EXPECT_NEAR(zout.x, z.x, 1e-9);
  EXPECT_NEAR(zout.y, z.y, 1e-9);
  EXPECT_NEAR(zout.z, z.z, 1e-9);
  Mat3Vec3Axpby(&a, kTrans, &x, b, &y, &z);
  EXPECT_NEAR(zout_trans.x, z.x, 1e-9);
  EXPECT_NEAR(zout_trans.y, z.y, 1e-9);
  EXPECT_NEAR(zout_trans.z, z.z, 1e-9);
}

TEST(Mat3AbpycTest, Normal0) {
  const Mat3 a = {
      {{1.206182293914872, 0.252383842187177, 0.493329872156661},
       {-0.514495002497536, -0.686912023319489, -0.151131603130403},
       {-0.363811936173529, -0.318977453380156, 0.145164984753677}}};

  const Mat3 b = {{{0.246571662686439, -0.789292120696153, 0.646228211917438},
                   {0.618282085210439, -0.911721386167771, 1.193276552412905},
                   {-0.921136695754912, 2.212407918029111, 1.066787284124506}}};

  const Mat3 c = {{{0.368400622147818, 0.615904877309113, 1.785569918352305},
                   {-0.265560867950290, 0.629463634960661, 1.251437525286518},
                   {-0.413495007488234, 1.018318269610553, 2.037406525489216}}};

  const double y = -1.93869201772888;
  const Mat3 zout = {
      {{-0.715184811905466, -1.284736881115239, -1.854759365178016},
       {0.102488314104288, -0.522341646293974, -3.739534409910320},
       {0.380998916974393, -1.075068878522861, -4.410777460853894}}};
  const Mat3 zout_atb = {
      {{-0.399787489982242, -2.481904361440057, -3.684245890899914},
       {0.446187877812484, -1.498976564033067, -3.423011484091909},
       {0.696121880098766, -1.904632805409344, -3.656581725174159}}};
  const Mat3 zout_abt = {
      {{-0.2972058686308998, -0.0897137428897088, -3.4880748751682384},
       {0.8324895886690490, -1.0925085837912976, -3.6331865862066670},
       {1.0575098555437776, -1.7351033648333252, -4.1656313267672713}}};
  const Mat3 zout_atbt = {
      {{-0.245823657149585, -0.413341121564235, -6.099111685135846},
       {0.913113456797354, -0.818647650118698, -4.518642649983390},
       {1.136377329288138, -1.358176591510101, -4.583832611785050}}};
  const Mat3 zout_abct = {
      {{-0.715184811905466, 0.424153723120608, 2.408550253016599},
       {-1.606402290131559, -0.522341646293974, -3.287587969752427},
       {-3.882310701220221, -1.527015318680753, -4.410777460853894}}};
  const Mat3 zout_atbct = {
      {{-0.399787489982242, -0.773013757204210, 0.579063727294701},
       {-1.262702726423363, -1.498976564033067, -2.971065043934017},
       {-3.567187738095849, -2.356579245567237, -3.656581725174159}}};
  const Mat3 zout_abtct = {
      {{-0.297205868630900, 1.619176861346138, 0.775234743026376},
       {-0.876401015566798, -1.092508583791298, -3.181240146048774},
       {-3.205799762650837, -2.187049804991218, -4.165631326767271}}};
  const Mat3 zout_atbtct = {
      {{-0.245823657149585, 1.295549482671612, -1.835802066941231},
       {-0.795777147438492, -0.818647650118698, -4.066696209825497},
       {-3.126932288906477, -1.810123031667994, -4.583832611785050}}};

  Mat3 z;
  Mat3Abpyc(&a, kNoTrans, &b, kNoTrans, y, &c, kNoTrans, &z);
  EXPECT_NEAR_MAT3(zout, z, 1e-9);
  Mat3Abpyc(&a, kTrans, &b, kNoTrans, y, &c, kNoTrans, &z);
  EXPECT_NEAR_MAT3(zout_atb, z, 1e-9);
  Mat3Abpyc(&a, kNoTrans, &b, kTrans, y, &c, kNoTrans, &z);
  EXPECT_NEAR_MAT3(zout_abt, z, 1e-9);
  Mat3Abpyc(&a, kTrans, &b, kTrans, y, &c, kNoTrans, &z);
  EXPECT_NEAR_MAT3(zout_atbt, z, 1e-9);
  Mat3Abpyc(&a, kNoTrans, &b, kNoTrans, y, &c, kTrans, &z);
  EXPECT_NEAR_MAT3(zout_abct, z, 1e-9);
  Mat3Abpyc(&a, kTrans, &b, kNoTrans, y, &c, kTrans, &z);
  EXPECT_NEAR_MAT3(zout_atbct, z, 1e-9);
  Mat3Abpyc(&a, kNoTrans, &b, kTrans, y, &c, kTrans, &z);
  EXPECT_NEAR_MAT3(zout_abtct, z, 1e-9);
  Mat3Abpyc(&a, kTrans, &b, kTrans, y, &c, kTrans, &z);
  EXPECT_NEAR_MAT3(zout_atbtct, z, 1e-9);
}

TEST(Mat3TransTest, Normal0) {
  const Mat3 a = {
      {{0.479688474788460, -0.911254996533266, -0.697436355059933},
       {1.236898575137608, -1.422094416475656, 1.229722812321566},
       {0.487297388504045, -0.466194965516502, -1.045464364814155}}};
  const Mat3 zout = {
      {{0.479688474788460, 1.236898575137608, 0.487297388504045},
       {-0.911254996533266, -1.422094416475656, -0.466194965516502},
       {-0.697436355059933, 1.229722812321566, -1.045464364814155}}};

  Mat3 z;
  Mat3Trans(&a, &z);
  EXPECT_NEAR_MAT3(zout, z, 1e-9);
}

TEST(Mat3AddTest, Normal0) {
  const Mat3 a = {
      {{0.4075024622931406, -1.3967361182409168, -1.9759734249384751},
       {-0.7375063047920832, -1.8670401925591973, 1.7150881058103851},
       {-0.1342418511269897, -0.0816261627595310, -0.9110734798654643}}};
  const Mat3 b = {
      {{0.0688386032819606, 1.0433184976656429, 0.6558261676732557},
       {2.6533446547494868, 0.8418835726594344, -0.1435362650699619},
       {-0.0839703971582201, 2.2798863038252453, 1.1337579496408676}}};
  const Mat3 zout = {
      {{0.476341065575101, -0.353417620575274, -1.320147257265219},
       {1.915838349957403, -1.025156619899763, 1.571551840740423},
       {-0.218212248285210, 2.198260141065714, 0.222684469775403}}};
  const Mat3 zout_atb = {
      {{0.476341065575101, 0.305812192873560, 0.521584316546266},
       {1.256608536508570, -1.025156619899763, -0.225162427829493},
       {-2.059943822096695, 3.994974409635630, 0.222684469775403}}};
  const Mat3 zout_abt = {
      {{0.476341065575101, 1.256608536508570, -2.059943822096695},
       {0.305812192873560, -1.025156619899763, 3.994974409635630},
       {0.521584316546266, -0.225162427829493, 0.222684469775403}}};
  const Mat3 zout_atbt = {
      {{0.476341065575101, 1.915838349957403, -0.218212248285210},
       {-0.353417620575274, -1.025156619899763, 2.198260141065714},
       {-1.320147257265219, 1.571551840740423, 0.222684469775403}}};

  Mat3 z;
  Mat3Add(&a, kNoTrans, &b, kNoTrans, &z);
  EXPECT_NEAR_MAT3(zout, z, 1e-9);
  Mat3Add(&a, kTrans, &b, kNoTrans, &z);
  EXPECT_NEAR_MAT3(zout_atb, z, 1e-9);
  Mat3Add(&a, kNoTrans, &b, kTrans, &z);
  EXPECT_NEAR_MAT3(zout_abt, z, 1e-9);
  Mat3Add(&a, kTrans, &b, kTrans, &z);
  EXPECT_NEAR_MAT3(zout_atbt, z, 1e-9);
}

TEST(Mat3MultTest, Normal0) {
  const Mat3 a = {
      {{0.11716169441353051, 0.00866189982756144, 2.06521012902134160},
       {-0.24309675984834830, 0.03985960426596999, 0.46993464315331823},
       {-0.15407422697499681, -2.21391325662147187, -0.24188291877964674}}};
  const Mat3 b = {
      {{-0.3372574099162383, 0.9043372876038493, 1.7760958886098350},
       {1.1825156385899531, 0.6774642591120553, -0.9855231742364294},
       {-0.7613153886582166, 0.2282249986257290, -0.0547412364556610}}};
  const Mat3 zout = {
      {{-1.6015470696300824, 0.5831543953439223, 0.0865017447340737},
       {-0.2286476865182288, -0.0855871738661461, -0.4967705228405930},
       {-2.3818751853915581, -1.6943859014537239, 1.9214531891301756}}};
  const Mat3 zout_atb = {
      {{-0.2096802898188392, -0.0938992676058282, 0.4561021078533301},
       {1.7296995469120728, -0.4704336136818205, 0.0972941500243139},
       {0.0433468339179804, 2.1308067224600347, 3.2181207081175716}}};
  const Mat3 zout_abt = {
      {{3.636330848666440, -1.890898778381460, -0.200272294845434},
       {0.952681697612272, -0.723593744142795, 0.168445458914759},
       {-2.379768892328310, -1.443661065127725, -0.374730299906733}}};
  const Mat3 zout_atbt = {
      {{-0.533005715096875, 0.125699890820917, -0.136243544910039},
       {-3.898996996318154, 2.219109109428240, 0.123694869568123},
       {-0.701135256075641, 2.998888421306352, -1.451784448697418}}};

  Mat3 z;
  Mat3Mult(&a, kNoTrans, &b, kNoTrans, &z);
  EXPECT_NEAR_MAT3(zout, z, 1e-9);
  Mat3Mult(&a, kTrans, &b, kNoTrans, &z);
  EXPECT_NEAR_MAT3(zout_atb, z, 1e-9);
  Mat3Mult(&a, kNoTrans, &b, kTrans, &z);
  EXPECT_NEAR_MAT3(zout_abt, z, 1e-9);
  Mat3Mult(&a, kTrans, &b, kTrans, &z);
  EXPECT_NEAR_MAT3(zout_atbt, z, 1e-9);
}

TEST(Mat3TransTest, Consistency0) {
  double x, y;
  Mat3 a, b, c, d;
  for (int32_t t = 0; t < test_util::kNumTests; ++t) {
    for (int32_t i = 0; i < 3; ++i)
      for (int32_t j = 0; j < 3; ++j) {
        a.d[i][j] = RandNormal();
        b.d[i][j] = RandNormal();
      }
    b = a;
    Mat3Trans(&a, &a);
    Mat3Trans(&a, &a);
    EXPECT_NEAR_MAT3(b, a, kConsistencyTol);
    Mat3Add(&a, kTrans, &b, kTrans, &c);
    Mat3Add(&a, kNoTrans, &b, kNoTrans, &d);
    Mat3Trans(&d, &d);
    EXPECT_NEAR_MAT3(d, c, kConsistencyTol);
    Mat3Mult(&a, kTrans, &b, kTrans, &c);
    Mat3Mult(&b, kNoTrans, &a, kNoTrans, &d);
    Mat3Trans(&d, &d);
    EXPECT_NEAR_MAT3(d, c, kConsistencyTol);
    x = Mat3Det(&a);
    Mat3Trans(&a, &c);
    y = Mat3Det(&c);
    EXPECT_NEAR(x, y, 1e-9 * (fabs(x) + fabs(y)) / 2);
    Mat3Inv(&a, &c);
    Mat3Trans(&a, &d);
    Mat3Trans(&c, &c);
    Mat3Inv(&d, &d);
    EXPECT_NEAR_MAT3(d, c, kConsistencyTol);
    Mat3Mult(&a, kNoTrans, &a, kTrans, &c);
    Mat3Trans(&c, &d);
    EXPECT_NEAR_MAT3(d, c, kConsistencyTol);
    Mat3Mult(&a, kNoTrans, &b, kTrans, &c);
    Mat3Mult(&a, kTrans, &b, kNoTrans, &d);
    x = Mat3Trace(&c);
    y = Mat3Trace(&d);
    EXPECT_NEAR(x, y, 1e-9 * (fabs(x) + fabs(y)) / 2.0);
  }
}

TEST(Mat3MultTest, Consistency0) {
  double x, y;
  Mat3 a, b, c;
  for (int32_t t = 0; t < test_util::kNumTests; ++t) {
    do {
      for (int32_t i = 0; i < 3; ++i)
        for (int32_t j = 0; j < 3; ++j) a.d[i][j] = RandNormal();
    } while (fabs(Mat3Det(&a)) < kConsistencyTol);
    Mat3Inv(&a, &b);
    Mat3Mult(&a, kNoTrans, &b, kNoTrans, &c);
    EXPECT_NEAR_MAT3(c, kMat3Identity, kConsistencyTol);
    Mat3Mult(&b, kNoTrans, &a, kNoTrans, &c);
    EXPECT_NEAR_MAT3(c, kMat3Identity, kConsistencyTol);
    x = Mat3Det(&a);
    y = Mat3Det(&b);
    EXPECT_NEAR(x * y, Mat3Det(&c), 1e-9 * (fabs(x) + fabs(y)) / 2.0);
  }
}

TEST(Mat3Vec3AxpbyTest, Consistency0) {
  double b;
  Vec3 w, x, y, z;
  Mat3 A, Ainv;
  for (int32_t t = 0; t < test_util::kNumTests; t++) {
    b = RandNormal();
    x.x = RandNormal();
    x.y = RandNormal();
    x.z = RandNormal();
    y.x = RandNormal();
    y.y = RandNormal();
    y.z = RandNormal();
    do {
      for (int32_t i = 0; i < 3; i++)
        for (int32_t j = 0; j < 3; j++) A.d[i][j] = RandNormal();
    } while (fabs(Mat3Det(&A)) < kConsistencyTol);
    Mat3Inv(&A, &Ainv);
    // z := Ax + by
    Mat3Vec3Axpby(&A, kNoTrans, &x, b, &y, &z);
    // z := A^(-1) (Ax + by) = x + A^(-1) b y
    Mat3Vec3Axpby(&Ainv, kNoTrans, &z, 0, &y, &z);
    // w := A^(-1) y
    Mat3Vec3Axpby(&Ainv, kNoTrans, &y, 0, &y, &w);
    // z := x + A^(-1) b y - b A^(-1) y = x
    Mat3Vec3Axpby(&kMat3Identity, kNoTrans, &z, -b, &w, &z);
    EXPECT_NEAR(x.x, z.x, 1e-9 * (fabs(x.x) + fabs(z.x)) / 2);
    EXPECT_NEAR(x.y, z.y, 1e-9 * (fabs(x.y) + fabs(z.y)) / 2);
    EXPECT_NEAR(x.z, z.z, 1e-9 * (fabs(x.z) + fabs(z.z)) / 2);
  }
}

TEST(Mat3AbpycTest, Consistency0) {
  double z;
  Mat3 a, b, c, d, e, f;
  for (int32_t t = 0; t < test_util::kNumTests; t++) {
    z = RandNormal();
    for (int32_t i = 0; i < 3; i++)
      for (int32_t j = 0; j < 3; j++) c.d[i][j] = RandNormal();
    do {
      for (int32_t i = 0; i < 3; i++)
        for (int32_t j = 0; j < 3; j++) a.d[i][j] = RandNormal();
    } while (fabs(Mat3Det(&a)) < kConsistencyTol);
    do {
      for (int32_t i = 0; i < 3; i++)
        for (int32_t j = 0; j < 3; j++) b.d[i][j] = RandNormal();
    } while (fabs(Mat3Det(&b)) < kConsistencyTol);
    Mat3Inv(&a, &d);
    Mat3Inv(&b, &e);
    Mat3Abpyc(&a, kNoTrans, &b, kNoTrans, z, &c, kNoTrans, &f);
    Mat3Abpyc(&d, kNoTrans, &f, kNoTrans, 0, &kMat3Identity, kNoTrans, &f);
    Mat3Abpyc(&f, kNoTrans, &e, kNoTrans, 0, &kMat3Identity, kTrans, &f);
    Mat3Abpyc(&a, kNoTrans, &f, kNoTrans, -1, &a, kNoTrans, &f);
    Mat3Abpyc(&f, kNoTrans, &b, kNoTrans, 0, &kMat3Identity, kTrans, &f);
    Mat3Abpyc(&kMat3Zero, kTrans, &kMat3Zero, kNoTrans, 1.0 / z, &f, kNoTrans,
              &f);
    EXPECT_NEAR_MAT3(f, c,
                     1e-9 * (fabs(Mat3Trace(&f)) + fabs(Mat3Trace(&c))) / 3.0);
  }
}

TEST(Mat3Cross, Random) {
  for (int32_t i = 0; i < 22; ++i) {
    Vec3 u = {test_util::RandNormal(), test_util::RandNormal(),
              test_util::RandNormal()};
    for (int32_t j = 0; j < 22; ++j) {
      Vec3 v = {test_util::RandNormal(), test_util::RandNormal(),
                test_util::RandNormal()};
      Vec3 w;
      Vec3Cross(&u, &v, &w);
      Mat3 m;
      Mat3Cross(&u, &m);
      Vec3 w_mat;
      Mat3Vec3Mult(&m, &v, &w_mat);
      EXPECT_EQ_VEC3(w, w_mat);
    }
  }
}

TEST(Mat3ContainsNaN, NoNaN) {
  Mat3 A = {{{-1.0, 0.0, 1.0},
             {INFINITY, -INFINITY, DBL_MAX},
             {DBL_MIN, DBL_EPSILON, 22.0}}};
  EXPECT_EQ(Mat3ContainsNaN(&A), 0);
}

TEST(Mat3ContainsNaN, YesNaN) {
  Mat3 A0 = {{{-1.0, 0.0, 1.0},
              {INFINITY, -INFINITY, DBL_MAX},
              {DBL_MIN, DBL_EPSILON, 22.0}}};
  for (int32_t i = 0; i < 3; ++i) {
    for (int32_t j = 0; j < 3; ++j) {
      Mat3 A = A0;
      A.d[i][j] = NAN;
      EXPECT_EQ(Mat3ContainsNaN(&A), 1);
    }
  }
}

TEST(Mat3IsOrthogonal, Normal) {
  Mat3 rot_x = {{{1.0, 0.0, 0.0}, {0.0, 0.0, 1.0}, {0.0, -1.0, 0.0}}};
  Mat3 rot_x2 = {{{1.0, 0.0, 0.0}, {0.0, 0.0, -1.0}, {0.0, 1.0, 0.0}}};
  Mat3 rot_y = {{{0.0, 0.0, 1.0}, {0.0, 1.0, 0.0}, {-1.0, 0.0, 0.0}}};
  Mat3 rot_y2 = {{{0.0, 0.0, -1.0}, {0.0, 1.0, 0.0}, {1.0, 0.0, 0.0}}};
  Mat3 rot_z = {{{0.0, 1.0, 0.0}, {-1.0, 0.0, 0.0}, {0.0, 0.0, 1.0}}};
  Mat3 rot_z2 = {{{0.0, -1.0, 0.0}, {1.0, 0.0, 0.0}, {0.0, 0.0, 1.0}}};

  Mat3 refl_x = {{{-1.0, 0.0, 0.0}, {0.0, 0.0, 1.0}, {0.0, -1.0, 0.0}}};
  Mat3 refl_x2 = {{{-1.0, 0.0, 0.0}, {0.0, 0.0, -1.0}, {0.0, 1.0, 0.0}}};
  Mat3 refl_y = {{{0.0, 0.0, 1.0}, {0.0, -1.0, 0.0}, {-1.0, 0.0, 0.0}}};
  Mat3 refl_y2 = {{{0.0, 0.0, -1.0}, {0.0, -1.0, 0.0}, {1.0, 0.0, 0.0}}};
  Mat3 refl_z = {{{0.0, 1.0, 0.0}, {-1.0, 0.0, 0.0}, {0.0, 0.0, -1.0}}};
  Mat3 refl_z2 = {{{0.0, -1.0, 0.0}, {1.0, 0.0, 0.0}, {0.0, 0.0, -1.0}}};

  EXPECT_EQ(Mat3IsOrthogonal(&rot_x, DBL_EPSILON), 1);
  EXPECT_EQ(Mat3IsOrthogonal(&rot_x2, DBL_EPSILON), 1);
  EXPECT_EQ(Mat3IsOrthogonal(&rot_y, DBL_EPSILON), 1);
  EXPECT_EQ(Mat3IsOrthogonal(&rot_y2, DBL_EPSILON), 1);
  EXPECT_EQ(Mat3IsOrthogonal(&rot_z, DBL_EPSILON), 1);
  EXPECT_EQ(Mat3IsOrthogonal(&rot_z2, DBL_EPSILON), 1);

  EXPECT_EQ(Mat3IsOrthogonal(&refl_x, DBL_EPSILON), 1);
  EXPECT_EQ(Mat3IsOrthogonal(&refl_x2, DBL_EPSILON), 1);
  EXPECT_EQ(Mat3IsOrthogonal(&refl_y, DBL_EPSILON), 1);
  EXPECT_EQ(Mat3IsOrthogonal(&refl_y2, DBL_EPSILON), 1);
  EXPECT_EQ(Mat3IsOrthogonal(&refl_z, DBL_EPSILON), 1);
  EXPECT_EQ(Mat3IsOrthogonal(&refl_z2, DBL_EPSILON), 1);
}

TEST(Mat3IsOrthogonal, DetectsNaN) {
  Mat3 rot_x_nan = {{{1.0, NAN, 0.0}, {0.0, 0.0, 1.0}, {0.0, -1.0, 0.0}}};
  Mat3 rot_x2_nan = {{{1.0, 0.0, 0.0}, {NAN, 0.0, -1.0}, {0.0, 1.0, 0.0}}};
  Mat3 rot_y_nan = {{{0.0, 0.0, 1.0}, {0.0, 1.0, NAN}, {-1.0, 0.0, 0.0}}};
  Mat3 rot_y2_nan = {{{0.0, 0.0, -1.0}, {0.0, 1.0, 0.0}, {1.0, NAN, 0.0}}};
  Mat3 rot_z_nan = {{{NAN, 1.0, 0.0}, {-1.0, 0.0, 0.0}, {0.0, 0.0, 1.0}}};
  Mat3 rot_z2_nan = {{{0.0, -1.0, 0.0}, {1.0, NAN, 0.0}, {0.0, 0.0, 1.0}}};

  EXPECT_EQ(Mat3IsOrthogonal(&rot_x_nan, DBL_EPSILON), 0);
  EXPECT_EQ(Mat3IsOrthogonal(&rot_x2_nan, DBL_EPSILON), 0);
  EXPECT_EQ(Mat3IsOrthogonal(&rot_y_nan, DBL_EPSILON), 0);
  EXPECT_EQ(Mat3IsOrthogonal(&rot_y2_nan, DBL_EPSILON), 0);
  EXPECT_EQ(Mat3IsOrthogonal(&rot_z_nan, DBL_EPSILON), 0);
  EXPECT_EQ(Mat3IsOrthogonal(&rot_z2_nan, DBL_EPSILON), 0);
}

TEST(Mat3SpecialOrthogonal, Normal) {
  Mat3 rot_x = {{{1.0, 0.0, 0.0}, {0.0, 0.0, 1.0}, {0.0, -1.0, 0.0}}};
  Mat3 rot_x2 = {{{1.0, 0.0, 0.0}, {0.0, 0.0, -1.0}, {0.0, 1.0, 0.0}}};
  Mat3 rot_y = {{{0.0, 0.0, 1.0}, {0.0, 1.0, 0.0}, {-1.0, 0.0, 0.0}}};
  Mat3 rot_y2 = {{{0.0, 0.0, -1.0}, {0.0, 1.0, 0.0}, {1.0, 0.0, 0.0}}};
  Mat3 rot_z = {{{0.0, 1.0, 0.0}, {-1.0, 0.0, 0.0}, {0.0, 0.0, 1.0}}};
  Mat3 rot_z2 = {{{0.0, -1.0, 0.0}, {1.0, 0.0, 0.0}, {0.0, 0.0, 1.0}}};

  Mat3 refl_x = {{{-1.0, 0.0, 0.0}, {0.0, 0.0, 1.0}, {0.0, -1.0, 0.0}}};
  Mat3 refl_x2 = {{{-1.0, 0.0, 0.0}, {0.0, 0.0, -1.0}, {0.0, 1.0, 0.0}}};
  Mat3 refl_y = {{{0.0, 0.0, 1.0}, {0.0, -1.0, 0.0}, {-1.0, 0.0, 0.0}}};
  Mat3 refl_y2 = {{{0.0, 0.0, -1.0}, {0.0, -1.0, 0.0}, {1.0, 0.0, 0.0}}};
  Mat3 refl_z = {{{0.0, 1.0, 0.0}, {-1.0, 0.0, 0.0}, {0.0, 0.0, -1.0}}};
  Mat3 refl_z2 = {{{0.0, -1.0, 0.0}, {1.0, 0.0, 0.0}, {0.0, 0.0, -1.0}}};

  EXPECT_EQ(Mat3IsSpecialOrthogonal(&rot_x, DBL_EPSILON), 1);
  EXPECT_EQ(Mat3IsSpecialOrthogonal(&rot_x2, DBL_EPSILON), 1);
  EXPECT_EQ(Mat3IsSpecialOrthogonal(&rot_y, DBL_EPSILON), 1);
  EXPECT_EQ(Mat3IsSpecialOrthogonal(&rot_y2, DBL_EPSILON), 1);
  EXPECT_EQ(Mat3IsSpecialOrthogonal(&rot_z, DBL_EPSILON), 1);
  EXPECT_EQ(Mat3IsSpecialOrthogonal(&rot_z2, DBL_EPSILON), 1);

  EXPECT_EQ(Mat3IsSpecialOrthogonal(&refl_x, DBL_EPSILON), 0);
  EXPECT_EQ(Mat3IsSpecialOrthogonal(&refl_x2, DBL_EPSILON), 0);
  EXPECT_EQ(Mat3IsSpecialOrthogonal(&refl_y, DBL_EPSILON), 0);
  EXPECT_EQ(Mat3IsSpecialOrthogonal(&refl_y2, DBL_EPSILON), 0);
  EXPECT_EQ(Mat3IsSpecialOrthogonal(&refl_z, DBL_EPSILON), 0);
  EXPECT_EQ(Mat3IsSpecialOrthogonal(&refl_z2, DBL_EPSILON), 0);
}

TEST(Mat3IsSpecialOrthogonal, DetectsNaN) {
  Mat3 rot_x_nan = {{{1.0, NAN, 0.0}, {0.0, 0.0, 1.0}, {0.0, -1.0, 0.0}}};
  Mat3 rot_x2_nan = {{{1.0, 0.0, 0.0}, {NAN, 0.0, -1.0}, {0.0, 1.0, 0.0}}};
  Mat3 rot_y_nan = {{{0.0, 0.0, 1.0}, {0.0, 1.0, NAN}, {-1.0, 0.0, 0.0}}};
  Mat3 rot_y2_nan = {{{0.0, 0.0, -1.0}, {0.0, 1.0, 0.0}, {1.0, NAN, 0.0}}};
  Mat3 rot_z_nan = {{{NAN, 1.0, 0.0}, {-1.0, 0.0, 0.0}, {0.0, 0.0, 1.0}}};
  Mat3 rot_z2_nan = {{{0.0, -1.0, 0.0}, {1.0, NAN, 0.0}, {0.0, 0.0, 1.0}}};

  EXPECT_EQ(Mat3IsSpecialOrthogonal(&rot_x_nan, DBL_EPSILON), 0);
  EXPECT_EQ(Mat3IsSpecialOrthogonal(&rot_x2_nan, DBL_EPSILON), 0);
  EXPECT_EQ(Mat3IsSpecialOrthogonal(&rot_y_nan, DBL_EPSILON), 0);
  EXPECT_EQ(Mat3IsSpecialOrthogonal(&rot_y2_nan, DBL_EPSILON), 0);
  EXPECT_EQ(Mat3IsSpecialOrthogonal(&rot_z_nan, DBL_EPSILON), 0);
  EXPECT_EQ(Mat3IsSpecialOrthogonal(&rot_z2_nan, DBL_EPSILON), 0);
}

TEST(Mat3ScaleTest, Normal1) {
  const Mat3 m = {
      {{0.8666725896344352, -0.7166455764772535, -0.4898961445874253},
       {-0.2020947093194583, 0.5247514617010127, 0.1048558450921258},
       {0.5845432352457445, 0.8197391588537519, 0.9242424981485318}}};
  Mat3 m_out = {
      {{0.9122715963502814, 0.3042959810847168, -0.1010222757463710},
       {0.5799280200999213, 0.9423250946662296, 0.1965072521055944},
       {-0.6605622487142981, -0.2563193552296719, 0.7691550488602148}}};
  const Mat3 *m_out_ptr = Mat3Scale(&m, 0.7910469636214725, &m_out);

  EXPECT_NEAR(0.6855787204842785, m_out.d[0][0], 1e-9);
  EXPECT_NEAR(-0.5669003072650911, m_out.d[0][1], 1e-9);
  EXPECT_NEAR(-0.3875308576657486, m_out.d[0][2], 1e-9);
  EXPECT_NEAR(-0.1598664061711216, m_out.d[1][0], 1e-9);
  EXPECT_NEAR(0.4151030504345156, m_out.d[1][1], 1e-9);
  EXPECT_NEAR(0.0829458978780896, m_out.d[1][2], 1e-9);
  EXPECT_NEAR(0.4624011513466183, m_out.d[2][0], 1e-9);
  EXPECT_NEAR(0.6484521725728803, m_out.d[2][1], 1e-9);
  EXPECT_NEAR(0.7311192218103205, m_out.d[2][2], 1e-9);

  EXPECT_NEAR(0.6855787204842785, m_out_ptr->d[0][0], 1e-9);
  EXPECT_NEAR(-0.5669003072650911, m_out_ptr->d[0][1], 1e-9);
  EXPECT_NEAR(-0.3875308576657486, m_out_ptr->d[0][2], 1e-9);
  EXPECT_NEAR(-0.1598664061711216, m_out_ptr->d[1][0], 1e-9);
  EXPECT_NEAR(0.4151030504345156, m_out_ptr->d[1][1], 1e-9);
  EXPECT_NEAR(0.0829458978780896, m_out_ptr->d[1][2], 1e-9);
  EXPECT_NEAR(0.4624011513466183, m_out_ptr->d[2][0], 1e-9);
  EXPECT_NEAR(0.6484521725728803, m_out_ptr->d[2][1], 1e-9);
  EXPECT_NEAR(0.7311192218103205, m_out_ptr->d[2][2], 1e-9);
}

TEST(Mat3Vec3MultTest, Normal1) {
  const Mat3 m = {
      {{0.8186290698989662, -0.4813176059499529, 0.1338970483118325},
       {0.0945126221080150, -0.5638090352309415, 0.7812620457902812},
       {0.1544112473752504, -0.0897011290521437, 0.6665749566616146}}};
  const Vec3 v = {-0.4731793730537315, 0.7556201971731584, -0.8363640031996620};
  Vec3 v_out = {-0.7639083724363585, -0.2324317564427187, -0.5888240094156796};
  const Vec3 *v_out_ptr = Mat3Vec3Mult(&m, &v, &v_out);

  EXPECT_NEAR(-0.8630383657118708, v_out.x, 1e-9);
  EXPECT_NEAR(-1.1241663698090643, v_out.y, 1e-9);
  EXPECT_NEAR(-0.6983435012326504, v_out.z, 1e-9);

  EXPECT_NEAR(-0.8630383657118708, v_out_ptr->x, 1e-9);
  EXPECT_NEAR(-1.1241663698090643, v_out_ptr->y, 1e-9);
  EXPECT_NEAR(-0.6983435012326504, v_out_ptr->z, 1e-9);
}

TEST(Mat3TransVec3MultTest, Normal1) {
  const Mat3 m = {
      {{-0.7630421892062711, -0.4082508668716549, -0.6995802064818673},
       {0.1582617339432160, 0.5489053577662562, 0.2556380245472623},
       {0.9148739261040650, -0.3385377428913545, -0.8860289541383828}}};
  const Vec3 v = {-0.1730429508923703, -0.4691783433651762,
                  -0.8054134231458367};
  Vec3 v_out = {0.5119173943463122, -0.5031435168630518, -0.9328065535037322};
  const Vec3 *v_out_ptr = Mat3TransVec3Mult(&m, &v, &v_out);

  EXPECT_NEAR(-0.6790656466442971, v_out.x, 1e-9);
  EXPECT_NEAR(0.0857732706529890, v_out.y, 1e-9);
  EXPECT_NEAR(0.7147372114162058, v_out.z, 1e-9);

  EXPECT_NEAR(-0.6790656466442971, v_out_ptr->x, 1e-9);
  EXPECT_NEAR(0.0857732706529890, v_out_ptr->y, 1e-9);
  EXPECT_NEAR(0.7147372114162058, v_out_ptr->z, 1e-9);
}

TEST(Mat3Vec3LeftDivideTest, Normal1) {
  const Mat3 m = {
      {{0.1089441084430545, -0.3848589707042960, -0.5311689300610885},
       {-0.4973924985486893, -0.3103686769944483, 0.2477228400490183},
       {-0.4944912534322534, -0.0862115176123155, 0.4098332525215012}}};
  const Vec3 v = {0.4852767024751019, 0.2361765103964635, -0.2220085757628856};
  Vec3 v_out = {0.7170382591937352, 0.8148290355957415, 0.5927498719656625};
  const Vec3 *v_out_ptr = Mat3Vec3LeftDivide(&m, &v, &v_out);

  EXPECT_NEAR(3.4248583693327990, v_out.x, 1e-9);
  EXPECT_NEAR(-4.0664624782297212, v_out.y, 1e-9);
  EXPECT_NEAR(2.7352051685847569, v_out.z, 1e-9);

  EXPECT_NEAR(3.4248583693327990, v_out_ptr->x, 1e-9);
  EXPECT_NEAR(-4.0664624782297212, v_out_ptr->y, 1e-9);
  EXPECT_NEAR(2.7352051685847569, v_out_ptr->z, 1e-9);
}

TEST(Mat3Vec3LeftDivideTest, Singular1) {
  const Mat3 m = {
      {{0.0000000000000000, 0.7853273115916355, 0.0594416425787185},
       {0.0000000000000000, 0.8933459186292685, 0.5369086755670800},
       {0.0000000000000000, 0.4807327628062048, 0.1684203886610240}}};
  const Vec3 v = {-0.1601894068841210, 0.5325360766948402, 0.3287823130086405};
  Vec3 v_out = {-0.6325717442288228, 0.9489300066597546, 0.2681581743469743};
  const Vec3 *v_out_ptr = Mat3Vec3LeftDivide(&m, &v, &v_out);

  EXPECT_TRUE(isnan(v_out.x));
  EXPECT_TRUE(isnan(v_out.y));
  EXPECT_TRUE(isnan(v_out.z));

  EXPECT_TRUE(isnan(v_out_ptr->x));
  EXPECT_TRUE(isnan(v_out_ptr->y));
  EXPECT_TRUE(isnan(v_out_ptr->z));
}

TEST(Mat3DetTest, Normal1) {
  const Mat3 m = {
      {{0.4184517339645673, 0.6126492519761340, 0.2103397631555637},
       {0.5329630372138685, 0.7344660770002478, 0.4087857410623481},
       {-0.3997239481874193, -0.0570706674728567, 0.2958549734350442}}};
  double m_det = Mat3Det(&m);

  EXPECT_NEAR(-0.0406656587024579, m_det, 1e-9);
}

TEST(Mat3InvTest, Normal1) {
  const Mat3 m = {
      {{0.7855741307482806, -0.2905599344933709, -0.0635515015243604},
       {0.9026295071103760, 0.4341414751935415, -0.7906789906537726},
       {0.2853792511025530, -0.9285019159963126, 0.6206193907597877}}};
  Mat3 m_out = {
      {{0.3927386094350362, 0.5601730577784074, 0.2124998804200655},
       {-0.0588678312470670, -0.3893951749709093, 0.8989315974093810},
       {0.0737020123861039, 0.1755916253803200, -0.5969369194744270}}};
  const Mat3 *m_out_ptr = Mat3Inv(&m, &m_out);

  EXPECT_NEAR(6.1471968509773429, m_out.d[0][0], 1e-9);
  EXPECT_NEAR(-3.1659253694960698, m_out.d[0][1], 1e-9);
  EXPECT_NEAR(-3.4039656463305201, m_out.d[0][2], 1e-9);
  EXPECT_NEAR(10.3950099871849275, m_out.d[1][0], 1e-9);
  EXPECT_NEAR(-6.6891286305051008, m_out.d[1][1], 1e-9);
  EXPECT_NEAR(-7.4576061428059850, m_out.d[1][2], 1e-9);
  EXPECT_NEAR(12.7251973975896480, m_out.d[2][0], 1e-9);
  EXPECT_NEAR(-8.5517459135121499, m_out.d[2][1], 1e-9);
  EXPECT_NEAR(-7.9807052424658602, m_out.d[2][2], 1e-9);

  EXPECT_NEAR(6.1471968509773429, m_out_ptr->d[0][0], 1e-9);
  EXPECT_NEAR(-3.1659253694960698, m_out_ptr->d[0][1], 1e-9);
  EXPECT_NEAR(-3.4039656463305201, m_out_ptr->d[0][2], 1e-9);
  EXPECT_NEAR(10.3950099871849275, m_out_ptr->d[1][0], 1e-9);
  EXPECT_NEAR(-6.6891286305051008, m_out_ptr->d[1][1], 1e-9);
  EXPECT_NEAR(-7.4576061428059850, m_out_ptr->d[1][2], 1e-9);
  EXPECT_NEAR(12.7251973975896480, m_out_ptr->d[2][0], 1e-9);
  EXPECT_NEAR(-8.5517459135121499, m_out_ptr->d[2][1], 1e-9);
  EXPECT_NEAR(-7.9807052424658602, m_out_ptr->d[2][2], 1e-9);
}

TEST(Mat3InvTest, Reuse1) {
  Mat3 m = {{{0.2252999626042302, 0.3567196864760862, -0.5923995690874002},
             {-0.0474426594209401, 0.7163078723652749, -0.5536787316366334},
             {-0.3846057556397970, 0.8230853372628459, -0.4480268960178087}}};
  const Mat3 *m_out_ptr = Mat3Inv(&m, &m);

  EXPECT_NEAR(-3.2623230118458761, m.d[0][0], 1e-9);
  EXPECT_NEAR(7.9325811220279281, m.d[0][1], 1e-9);
  EXPECT_NEAR(-5.4896318271703839, m.d[0][2], 1e-9);
  EXPECT_NEAR(-4.6392007926523879, m.d[1][0], 1e-9);
  EXPECT_NEAR(7.9569116669400897, m.d[1][1], 1e-9);
  EXPECT_NEAR(-3.6991355290264583, m.d[1][2], 1e-9);
  EXPECT_NEAR(-5.7223125770475276, m.d[2][0], 1e-9);
  EXPECT_NEAR(7.8082387406569058, m.d[2][1], 1e-9);
  EXPECT_NEAR(-4.3152771286742659, m.d[2][2], 1e-9);

  EXPECT_NEAR(-3.2623230118458761, m_out_ptr->d[0][0], 1e-9);
  EXPECT_NEAR(7.9325811220279281, m_out_ptr->d[0][1], 1e-9);
  EXPECT_NEAR(-5.4896318271703839, m_out_ptr->d[0][2], 1e-9);
  EXPECT_NEAR(-4.6392007926523879, m_out_ptr->d[1][0], 1e-9);
  EXPECT_NEAR(7.9569116669400897, m_out_ptr->d[1][1], 1e-9);
  EXPECT_NEAR(-3.6991355290264583, m_out_ptr->d[1][2], 1e-9);
  EXPECT_NEAR(-5.7223125770475276, m_out_ptr->d[2][0], 1e-9);
  EXPECT_NEAR(7.8082387406569058, m_out_ptr->d[2][1], 1e-9);
  EXPECT_NEAR(-4.3152771286742659, m_out_ptr->d[2][2], 1e-9);
}

TEST(Mat3InvTest, Singular1) {
  const Mat3 m = {
      {{0.0000000000000000, 0.1938630987194752, 0.9297943456250559},
       {0.0000000000000000, 0.4860077905986644, 0.5304219870191134},
       {0.0000000000000000, -0.7779406049185791, 0.4701851832453003}}};
  Mat3 m_out = {
      {{-0.9518640096829316, 0.6222725375454516, 0.9928313254924177},
       {-0.1206256025591379, 0.8994702470661189, -0.2234807869994511},
       {-0.0526301457362273, -0.0197808837977544, -0.9833946677443199}}};
  const Mat3 *m_out_ptr = Mat3Inv(&m, &m_out);

  EXPECT_DOUBLE_EQ(INFINITY, m_out.d[0][0]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out.d[0][1]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out.d[0][2]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out.d[1][0]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out.d[1][1]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out.d[1][2]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out.d[2][0]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out.d[2][1]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out.d[2][2]);

  EXPECT_DOUBLE_EQ(INFINITY, m_out_ptr->d[0][0]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out_ptr->d[0][1]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out_ptr->d[0][2]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out_ptr->d[1][0]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out_ptr->d[1][1]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out_ptr->d[1][2]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out_ptr->d[2][0]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out_ptr->d[2][1]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out_ptr->d[2][2]);
}

TEST(Mat3TraceTest, Normal1) {
  const Mat3 m = {
      {{0.7882508747285919, -0.2012012582669938, -0.4846781976055876},
       {-0.3472775035867355, 0.2080016761798777, -0.3756404934511617},
       {-0.4753712595155146, 0.7220227179618304, 0.7354474270284892}}};
  double m_trace = Mat3Trace(&m);

  EXPECT_NEAR(1.7316999779369588, m_trace, 1e-9);
}

TEST(Mat3DiagTest, Normal1) {
  const Mat3 m = {
      {{-0.0031025467443351, -0.8762790718792546, 0.6944782592260068},
       {0.9405147454254243, -0.6588609711944513, -0.3408758376317624},
       {-0.1128448015603778, -0.8987533030323547, 0.4050306445222767}}};
  Vec3 v_out = {-0.5203636045786271, -0.0511095676061033, 0.2735860791461300};
  const Vec3 *v_out_ptr = Mat3Diag(&m, &v_out);

  EXPECT_NEAR(-0.0031025467443351, v_out.x, 1e-9);
  EXPECT_NEAR(-0.6588609711944513, v_out.y, 1e-9);
  EXPECT_NEAR(0.4050306445222767, v_out.z, 1e-9);

  EXPECT_NEAR(-0.0031025467443351, v_out_ptr->x, 1e-9);
  EXPECT_NEAR(-0.6588609711944513, v_out_ptr->y, 1e-9);
  EXPECT_NEAR(0.4050306445222767, v_out_ptr->z, 1e-9);
}

TEST(Mat3ScaleTest, Normal3) {
  const Mat3 m = {
      {{-0.7269515151296981, -0.4596969475310975, 0.6788816014292600},
       {0.6519946521404743, 0.7927352192994752, 0.9509869963025714},
       {0.7442690216753509, 0.9784278391287566, -0.4036584611447411}}};
  Mat3 m_out = {
      {{-0.0876474888350269, -0.9486884566803144, -0.7214214090143352},
       {0.4206192831757583, 0.9709561106082412, -0.9051843135795661},
       {-0.7946017449385161, 0.4020680257994240, -0.2649455892029720}}};
  const Mat3 *m_out_ptr = Mat3Scale(&m, -0.8865752926350225, &m_out);

  EXPECT_NEAR(0.6444972522575851, m_out.d[0][0], 1e-9);
  EXPECT_NEAR(0.4075559557808093, m_out.d[0][1], 1e-9);
  EXPECT_NEAR(-0.6018796544516789, m_out.d[0][2], 1e-9);
  EXPECT_NEAR(-0.5780423495179107, m_out.d[1][0], 1e-9);
  EXPECT_NEAR(-0.7028194590325210, m_out.d[1][1], 1e-9);
  EXPECT_NEAR(-0.8431215745390533, m_out.d[1][2], 1e-9);
  EXPECT_NEAR(-0.6598505256910062, m_out.d[2][0], 1e-9);
  EXPECT_NEAR(-0.8674499477978301, m_out.d[2][1], 1e-9);
  EXPECT_NEAR(0.3578736183140018, m_out.d[2][2], 1e-9);

  EXPECT_NEAR(0.6444972522575851, m_out_ptr->d[0][0], 1e-9);
  EXPECT_NEAR(0.4075559557808093, m_out_ptr->d[0][1], 1e-9);
  EXPECT_NEAR(-0.6018796544516789, m_out_ptr->d[0][2], 1e-9);
  EXPECT_NEAR(-0.5780423495179107, m_out_ptr->d[1][0], 1e-9);
  EXPECT_NEAR(-0.7028194590325210, m_out_ptr->d[1][1], 1e-9);
  EXPECT_NEAR(-0.8431215745390533, m_out_ptr->d[1][2], 1e-9);
  EXPECT_NEAR(-0.6598505256910062, m_out_ptr->d[2][0], 1e-9);
  EXPECT_NEAR(-0.8674499477978301, m_out_ptr->d[2][1], 1e-9);
  EXPECT_NEAR(0.3578736183140018, m_out_ptr->d[2][2], 1e-9);
}

TEST(Mat3Vec3MultTest, Normal3) {
  const Mat3 m = {
      {{-0.6379837792069474, -0.6210889954216021, 0.8845973106395304},
       {-0.7199681842515504, 0.5590093578713682, 0.7652169079794515},
       {0.5237265799925432, 0.8467842172409885, -0.4414496175746889}}};
  const Vec3 v = {0.5252498864060990, 0.1813730087149017, -0.2416664577992902};
  Vec3 v_out = {-0.0021338112207490, -0.1419060720880159, 0.2884580038808138};
  const Vec3 *v_out_ptr = Mat3Vec3Mult(&m, &v, &v_out);

  EXPECT_NEAR(-0.6615271859777483, v_out.x, 1e-9);
  EXPECT_NEAR(-0.4617012574567365, v_out.y, 1e-9);
  EXPECT_NEAR(0.5353546932383555, v_out.z, 1e-9);

  EXPECT_NEAR(-0.6615271859777483, v_out_ptr->x, 1e-9);
  EXPECT_NEAR(-0.4617012574567365, v_out_ptr->y, 1e-9);
  EXPECT_NEAR(0.5353546932383555, v_out_ptr->z, 1e-9);
}

TEST(Mat3TransVec3MultTest, Normal3) {
  const Mat3 m = {
      {{0.7200028050460705, 0.2926818956528310, -0.3502375666359696},
       {-0.8119265808553060, 0.1249657708787550, 0.8936220709991969},
       {0.7025158788857171, -0.5383561011731115, -0.2002128136230463}}};
  const Vec3 v = {-0.8721666187289725, -0.4096513144499090, 0.9705905758065150};
  Vec3 v_out = {-0.3419050141575986, 0.5334135186363187, -0.0753005963576683};
  const Vec3 *v_out_ptr = Mat3TransVec3Mult(&m, &v, &v_out);

  EXPECT_NEAR(0.3864996705326976, v_out.x, 1e-9);
  EXPECT_NEAR(-0.8289831298230044, v_out.y, 1e-9);
  EXPECT_NEAR(-0.2549326118197491, v_out.z, 1e-9);

  EXPECT_NEAR(0.3864996705326976, v_out_ptr->x, 1e-9);
  EXPECT_NEAR(-0.8289831298230044, v_out_ptr->y, 1e-9);
  EXPECT_NEAR(-0.2549326118197491, v_out_ptr->z, 1e-9);
}

TEST(Mat3Vec3LeftDivideTest, Normal3) {
  const Mat3 m = {
      {{0.2212853286056835, -0.6780890637316073, -0.6784462266000408},
       {0.0652491269565605, 0.9221775310818925, 0.5520780854928780},
       {-0.0528596021373862, -0.6210657516115126, 0.8800666470102421}}};
  const Vec3 v = {-0.4217368466045208, 0.5289869631879753, 0.3931811183993161};
  Vec3 v_out = {0.1745774588574460, -0.4789189605836439, -0.3231288713728806};
  const Vec3 *v_out_ptr = Mat3Vec3LeftDivide(&m, &v, &v_out);

  EXPECT_NEAR(0.4876678020839232, v_out.x, 1e-9);
  EXPECT_NEAR(0.1786489189693809, v_out.y, 1e-9);
  EXPECT_NEAR(0.6021268631497424, v_out.z, 1e-9);

  EXPECT_NEAR(0.4876678020839232, v_out_ptr->x, 1e-9);
  EXPECT_NEAR(0.1786489189693809, v_out_ptr->y, 1e-9);
  EXPECT_NEAR(0.6021268631497424, v_out_ptr->z, 1e-9);
}

TEST(Mat3Vec3LeftDivideTest, Singular3) {
  const Mat3 m = {
      {{-0.3491798219194622, -0.5161647735102293, 0.0000000000000000},
       {-0.0699896296867637, 0.1052304855087800, 0.0000000000000000},
       {-0.3624443315549337, -0.5200003641686963, 0.0000000000000000}}};
  const Vec3 v = {-0.3907951450314395, 0.7885441971617546, 0.2792181735114465};
  Vec3 v_out = {-0.4297523624820963, 0.6808279376168589, 0.9057926468579076};
  const Vec3 *v_out_ptr = Mat3Vec3LeftDivide(&m, &v, &v_out);

  EXPECT_TRUE(isnan(v_out.x));
  EXPECT_TRUE(isnan(v_out.y));
  EXPECT_TRUE(isnan(v_out.z));

  EXPECT_TRUE(isnan(v_out_ptr->x));
  EXPECT_TRUE(isnan(v_out_ptr->y));
  EXPECT_TRUE(isnan(v_out_ptr->z));
}

TEST(Mat3DetTest, Normal3) {
  const Mat3 m = {
      {{0.9548642421706048, -0.4147939013030804, 0.2513739217417625},
       {-0.9311601231998947, -0.9196617312731379, 0.4494735047876171},
       {-0.1134732107762786, 0.8593215612702991, 0.0417421489328194}}};
  double m_det = Mat3Det(&m);

  EXPECT_NEAR(-0.6278050221757143, m_det, 1e-9);
}

TEST(Mat3InvTest, Normal3) {
  const Mat3 m = {
      {{-0.1557105750646943, -0.1269118552697157, 0.7941475380970044},
       {-0.4865300439716471, 0.0511995134301260, 0.0586500646040922},
       {0.4484487523837779, 0.2825938984759206, -0.5588922524551760}}};
  Mat3 m_out = {
      {{-0.4912872676341729, -0.4910582556577490, 0.0019649974538074},
       {-0.6024813993881919, -0.8625351633916969, -0.6291983831135386},
       {0.7677423474983396, 0.2444896010435003, 0.0086895249965568}}};
  const Mat3 *m_out_ptr = Mat3Inv(&m, &m_out);

  EXPECT_NEAR(0.5065280762013382, m_out.d[0][0], 1e-9);
  EXPECT_NEAR(-1.7204921976590486, m_out.d[0][1], 1e-9);
  EXPECT_NEAR(0.5391934578175848, m_out.d[0][2], 1e-9);
  EXPECT_NEAR(2.7531283849463861, m_out.d[1][0], 1e-9);
  EXPECT_NEAR(3.0164596606173353, m_out.d[1][1], 1e-9);
  EXPECT_NEAR(4.2285533080109037, m_out.d[1][2], 1e-9);
  EXPECT_NEAR(1.7985025963607819, m_out.d[2][0], 1e-9);
  EXPECT_NEAR(0.1447157572996904, m_out.d[2][1], 1e-9);
  EXPECT_NEAR(0.7814815749496755, m_out.d[2][2], 1e-9);

  EXPECT_NEAR(0.5065280762013382, m_out_ptr->d[0][0], 1e-9);
  EXPECT_NEAR(-1.7204921976590486, m_out_ptr->d[0][1], 1e-9);
  EXPECT_NEAR(0.5391934578175848, m_out_ptr->d[0][2], 1e-9);
  EXPECT_NEAR(2.7531283849463861, m_out_ptr->d[1][0], 1e-9);
  EXPECT_NEAR(3.0164596606173353, m_out_ptr->d[1][1], 1e-9);
  EXPECT_NEAR(4.2285533080109037, m_out_ptr->d[1][2], 1e-9);
  EXPECT_NEAR(1.7985025963607819, m_out_ptr->d[2][0], 1e-9);
  EXPECT_NEAR(0.1447157572996904, m_out_ptr->d[2][1], 1e-9);
  EXPECT_NEAR(0.7814815749496755, m_out_ptr->d[2][2], 1e-9);
}

TEST(Mat3InvTest, Reuse3) {
  Mat3 m = {{{0.9680593806063511, 0.0720012801180181, 0.2683183104619622},
             {-0.2326047169382162, -0.5772345762569813, 0.9927543959658145},
             {0.0949782938659669, 0.6568656482367596, 0.4175301656067067}}};
  const Mat3 *m_out_ptr = Mat3Inv(&m, &m);

  EXPECT_NEAR(1.0182678104648675, m.d[0][0], 1e-9);
  EXPECT_NEAR(-0.1666708036610718, m.d[0][1], 1e-9);
  EXPECT_NEAR(-0.2580812941538511, m.d[0][2], 1e-9);
  EXPECT_NEAR(-0.2182309586275004, m.d[1][0], 1e-9);
  EXPECT_NEAR(-0.4317764220629099, m.d[1][1], 1e-9);
  EXPECT_NEAR(1.1668697098300007, m.d[1][2], 1e-9);
  EXPECT_NEAR(0.1116927221290249, m.d[2][0], 1e-9);
  EXPECT_NEAR(0.7171917926111401, m.d[2][1], 1e-9);
  EXPECT_NEAR(0.6180044315293557, m.d[2][2], 1e-9);

  EXPECT_NEAR(1.0182678104648675, m_out_ptr->d[0][0], 1e-9);
  EXPECT_NEAR(-0.1666708036610718, m_out_ptr->d[0][1], 1e-9);
  EXPECT_NEAR(-0.2580812941538511, m_out_ptr->d[0][2], 1e-9);
  EXPECT_NEAR(-0.2182309586275004, m_out_ptr->d[1][0], 1e-9);
  EXPECT_NEAR(-0.4317764220629099, m_out_ptr->d[1][1], 1e-9);
  EXPECT_NEAR(1.1668697098300007, m_out_ptr->d[1][2], 1e-9);
  EXPECT_NEAR(0.1116927221290249, m_out_ptr->d[2][0], 1e-9);
  EXPECT_NEAR(0.7171917926111401, m_out_ptr->d[2][1], 1e-9);
  EXPECT_NEAR(0.6180044315293557, m_out_ptr->d[2][2], 1e-9);
}

TEST(Mat3InvTest, Singular3) {
  const Mat3 m = {
      {{0.7280465472989464, 0.3231821602788527, 0.0000000000000000},
       {0.6341454386265883, -0.7802682876983988, 0.0000000000000000},
       {0.0864771606648147, 0.4846744929682756, 0.0000000000000000}}};
  Mat3 m_out = {{{-0.8811043918369497, 0.4033009691862273, 0.9897963214492465},
                 {0.4282109249146440, 0.4205964123909083, -0.7616299074021029},
                 {0.7002984581165981, 0.5326506675096225, 0.3500997304694164}}};
  const Mat3 *m_out_ptr = Mat3Inv(&m, &m_out);

  EXPECT_DOUBLE_EQ(INFINITY, m_out.d[0][0]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out.d[0][1]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out.d[0][2]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out.d[1][0]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out.d[1][1]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out.d[1][2]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out.d[2][0]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out.d[2][1]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out.d[2][2]);

  EXPECT_DOUBLE_EQ(INFINITY, m_out_ptr->d[0][0]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out_ptr->d[0][1]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out_ptr->d[0][2]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out_ptr->d[1][0]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out_ptr->d[1][1]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out_ptr->d[1][2]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out_ptr->d[2][0]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out_ptr->d[2][1]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out_ptr->d[2][2]);
}

TEST(Mat3TraceTest, Normal3) {
  const Mat3 m = {
      {{-0.4431497269046669, -0.1704212186295817, -0.6894776238838209},
       {-0.3119298818553209, 0.7320931619265487, 0.0597238757186123},
       {-0.4027397124471952, 0.4047535072774728, 0.2588874509903027}}};
  double m_trace = Mat3Trace(&m);

  EXPECT_NEAR(0.5478308860121845, m_trace, 1e-9);
}

TEST(Mat3DiagTest, Normal3) {
  const Mat3 m = {
      {{-0.2212438603569138, 0.4935959450842062, -0.0265744980233060},
       {-0.1871148783256109, 0.0076737477850166, -0.5871009294935352},
       {-0.5403641291259373, 0.3721838632660948, -0.1772135073075627}}};
  Vec3 v_out = {0.6435492152831175, 0.0681844947080261, 0.1325187740547575};
  const Vec3 *v_out_ptr = Mat3Diag(&m, &v_out);

  EXPECT_NEAR(-0.2212438603569138, v_out.x, 1e-9);
  EXPECT_NEAR(0.0076737477850166, v_out.y, 1e-9);
  EXPECT_NEAR(-0.1772135073075627, v_out.z, 1e-9);

  EXPECT_NEAR(-0.2212438603569138, v_out_ptr->x, 1e-9);
  EXPECT_NEAR(0.0076737477850166, v_out_ptr->y, 1e-9);
  EXPECT_NEAR(-0.1772135073075627, v_out_ptr->z, 1e-9);
}

TEST(Mat3ScaleTest, Normal5) {
  const Mat3 m = {
      {{0.9567087370211806, -0.4518977415032355, 0.2320730700404439},
       {0.8048511582148818, -0.8393049929511236, -0.2047576585053448},
       {0.4505076867638331, 0.9718068640611317, 0.4542353166991555}}};
  Mat3 m_out = {
      {{-0.1309024571617696, -0.0357865371510355, -0.8295824047731857},
       {0.1317132655315476, -0.6834005738469899, 0.6433612410281790},
       {0.5572837559485564, 0.9836802073160258, -0.5542284732463700}}};
  const Mat3 *m_out_ptr = Mat3Scale(&m, 0.5552419293169546, &m_out);

  EXPECT_NEAR(0.5312048049380272, m_out.d[0][0], 1e-9);
  EXPECT_NEAR(-0.2509125738462309, m_out.d[0][1], 1e-9);
  EXPECT_NEAR(0.1288566991517648, m_out.d[0][2], 1e-9);
  EXPECT_NEAR(0.4468871099002165, m_out.d[1][0], 1e-9);
  EXPECT_NEAR(-0.4660173235715349, m_out.d[1][1], 1e-9);
  EXPECT_NEAR(-0.1136900373509298, m_out.d[1][2], 1e-9);
  EXPECT_NEAR(0.2501407571708690, m_out.d[2][0], 1e-9);
  EXPECT_NEAR(0.5395879181247621, m_out.d[2][1], 1e-9);
  EXPECT_NEAR(0.2522104936079370, m_out.d[2][2], 1e-9);

  EXPECT_NEAR(0.5312048049380272, m_out_ptr->d[0][0], 1e-9);
  EXPECT_NEAR(-0.2509125738462309, m_out_ptr->d[0][1], 1e-9);
  EXPECT_NEAR(0.1288566991517648, m_out_ptr->d[0][2], 1e-9);
  EXPECT_NEAR(0.4468871099002165, m_out_ptr->d[1][0], 1e-9);
  EXPECT_NEAR(-0.4660173235715349, m_out_ptr->d[1][1], 1e-9);
  EXPECT_NEAR(-0.1136900373509298, m_out_ptr->d[1][2], 1e-9);
  EXPECT_NEAR(0.2501407571708690, m_out_ptr->d[2][0], 1e-9);
  EXPECT_NEAR(0.5395879181247621, m_out_ptr->d[2][1], 1e-9);
  EXPECT_NEAR(0.2522104936079370, m_out_ptr->d[2][2], 1e-9);
}

TEST(Mat3Vec3MultTest, Normal5) {
  const Mat3 m = {
      {{-0.9198631136583766, -0.5874926049233722, 0.3224108054306700},
       {0.9135328380973273, -0.6080393102182684, -0.5611220497248337},
       {0.1594594162853433, -0.7548817012478106, 0.9680492067851796}}};
  const Vec3 v = {-0.6637480818341450, -0.1596436074953573,
                  -0.6282353806022787};
  Vec3 v_out = {0.8517955233596353, -0.1157507327077312, 0.1049531254936431};
  const Vec3 *v_out_ptr = Mat3Vec3Mult(&m, &v, &v_out);

  EXPECT_NEAR(0.5017969410075193, v_out.x, 1e-9);
  EXPECT_NEAR(-0.1567693555241589, v_out.y, 1e-9);
  EXPECT_NEAR(-0.5934916055367773, v_out.z, 1e-9);

  EXPECT_NEAR(0.5017969410075193, v_out_ptr->x, 1e-9);
  EXPECT_NEAR(-0.1567693555241589, v_out_ptr->y, 1e-9);
  EXPECT_NEAR(-0.5934916055367773, v_out_ptr->z, 1e-9);
}

TEST(Mat3TransVec3MultTest, Normal5) {
  const Mat3 m = {
      {{0.2306606949067151, 0.2590212235801532, -0.8164510230211655},
       {-0.9852129312573277, -0.3816237700908078, 0.8460024943897899},
       {-0.0663918619809738, -0.2486858552524076, -0.4272810588552305}}};
  const Vec3 v = {-0.4200211718871891, -0.2691115969435685,
                  -0.7457267483339496};
  Vec3 v_out = {0.1543711743187008, 0.4206120038984433, -0.4102095018076906};
  const Vec3 *v_out_ptr = Mat3TransVec3Mult(&m, &v, &v_out);

  EXPECT_NEAR(0.2177600372279896, v_out.x, 1e-9);
  EXPECT_NEAR(0.1793566785229981, v_out.y, 1e-9);
  EXPECT_NEAR(0.4338925478391636, v_out.z, 1e-9);

  EXPECT_NEAR(0.2177600372279896, v_out_ptr->x, 1e-9);
  EXPECT_NEAR(0.1793566785229981, v_out_ptr->y, 1e-9);
  EXPECT_NEAR(0.4338925478391636, v_out_ptr->z, 1e-9);
}

TEST(Mat3Vec3LeftDivideTest, Normal5) {
  const Mat3 m = {
      {{-0.6966181137392349, -0.5378601993533789, -0.1226733139514768},
       {-0.8940132145810902, -0.6170727742427959, 0.5047504106830840},
       {-0.2311607835131504, -0.6366471823033901, -0.0164666503123982}}};
  const Vec3 v = {0.9796697624161204, 0.4354176656168958, 0.3913274741455806};
  Vec3 v_out = {-0.5295815369425940, -0.4920371380893827, -0.0627419603262453};
  const Vec3 *v_out_ptr = Mat3Vec3LeftDivide(&m, &v, &v_out);

  EXPECT_NEAR(-1.0304466818078664, v_out.x, 1e-9);
  EXPECT_NEAR(-0.2090198733282164, v_out.y, 1e-9);
  EXPECT_NEAR(-1.2180193318021937, v_out.z, 1e-9);

  EXPECT_NEAR(-1.0304466818078664, v_out_ptr->x, 1e-9);
  EXPECT_NEAR(-0.2090198733282164, v_out_ptr->y, 1e-9);
  EXPECT_NEAR(-1.2180193318021937, v_out_ptr->z, 1e-9);
}

TEST(Mat3Vec3LeftDivideTest, Singular5) {
  const Mat3 m = {
      {{0.2788049090929665, 0.7870236107112332, 0.0000000000000000},
       {-0.9776727017995033, -0.7940538568839028, 0.0000000000000000},
       {-0.4989585634605143, -0.0720735978401168, 0.0000000000000000}}};
  const Vec3 v = {-0.9846452206557241, -0.3226323909709312, 0.2444856739666277};
  Vec3 v_out = {-0.7539424952222242, 0.4179075854322212, -0.6692373858775795};
  const Vec3 *v_out_ptr = Mat3Vec3LeftDivide(&m, &v, &v_out);

  EXPECT_TRUE(isnan(v_out.x));
  EXPECT_TRUE(isnan(v_out.y));
  EXPECT_TRUE(isnan(v_out.z));

  EXPECT_TRUE(isnan(v_out_ptr->x));
  EXPECT_TRUE(isnan(v_out_ptr->y));
  EXPECT_TRUE(isnan(v_out_ptr->z));
}

TEST(Mat3DetTest, Normal5) {
  const Mat3 m = {
      {{-0.3098739372876509, -0.8388504295953418, -0.4050504410604479},
       {-0.7514625391079959, -0.8209947931977846, -0.0017402536688160},
       {-0.5541895369867584, 0.9066901493484274, 0.7523503982050541}}};
  double m_det = Mat3Det(&m);

  EXPECT_NEAR(0.1761196845770239, m_det, 1e-9);
}

TEST(Mat3InvTest, Normal5) {
  const Mat3 m = {
      {{0.3767348962834072, -0.1207692462798846, -0.2157416544465238},
       {-0.2578913518305772, 0.4116189916911828, -0.4163996070500822},
       {0.7360269285459582, 0.4974755367786399, -0.0382132360443312}}};
  Mat3 m_out = {{{0.9798683933451384, 0.1725653961242413, -0.2051506033673494},
                 {0.0569272537666170, 0.9438904026752635, 0.6897847120092331},
                 {0.4640450384300230, 0.9462792065899346, 0.7491563519806304}}};
  const Mat3 *m_out_ptr = Mat3Inv(&m, &m_out);

  EXPECT_NEAR(0.9412914638068619, m_out.d[0][0], 1e-9);
  EXPECT_NEAR(-0.5504631084968483, m_out.d[0][1], 1e-9);
  EXPECT_NEAR(0.6839735929522004, m_out.d[0][2], 1e-9);
  EXPECT_NEAR(-1.5555616104621792, m_out.d[1][0], 1e-9);
  EXPECT_NEAR(0.7100545632390667, m_out.d[1][1], 1e-9);
  EXPECT_NEAR(1.0450042564157154, m_out.d[1][2], 1e-9);
  EXPECT_NEAR(-2.1206783453411879, m_out.d[2][0], 1e-9);
  EXPECT_NEAR(-1.3587149744419691, m_out.d[2][1], 1e-9);
  EXPECT_NEAR(0.6093971260771414, m_out.d[2][2], 1e-9);

  EXPECT_NEAR(0.9412914638068619, m_out_ptr->d[0][0], 1e-9);
  EXPECT_NEAR(-0.5504631084968483, m_out_ptr->d[0][1], 1e-9);
  EXPECT_NEAR(0.6839735929522004, m_out_ptr->d[0][2], 1e-9);
  EXPECT_NEAR(-1.5555616104621792, m_out_ptr->d[1][0], 1e-9);
  EXPECT_NEAR(0.7100545632390667, m_out_ptr->d[1][1], 1e-9);
  EXPECT_NEAR(1.0450042564157154, m_out_ptr->d[1][2], 1e-9);
  EXPECT_NEAR(-2.1206783453411879, m_out_ptr->d[2][0], 1e-9);
  EXPECT_NEAR(-1.3587149744419691, m_out_ptr->d[2][1], 1e-9);
  EXPECT_NEAR(0.6093971260771414, m_out_ptr->d[2][2], 1e-9);
}

TEST(Mat3InvTest, Reuse5) {
  Mat3 m = {{{-0.3346576090983631, -0.0135054532074002, -0.0201208864520055},
             {0.9678215382425266, 0.7937708336632248, -0.2401844172686300},
             {0.7025480492440626, 0.6646013493355412, 0.6289923916647122}}};
  const Mat3 *m_out_ptr = Mat3Inv(&m, &m);

  EXPECT_NEAR(-3.1120271869194629, m.d[0][0], 1e-9);
  EXPECT_NEAR(0.0230368460236385, m.d[0][1], 1e-9);
  EXPECT_NEAR(-0.0907541251406655, m.d[0][2], 1e-9);
  EXPECT_NEAR(3.6721367662729101, m.d[1][0], 1e-9);
  EXPECT_NEAR(0.9274228067911187, m.d[1][1], 1e-9);
  EXPECT_NEAR(0.4716100818581905, m.d[1][2], 1e-9);
  EXPECT_NEAR(-0.4040723287164191, m.d[2][0], 1e-9);
  EXPECT_NEAR(-1.0056575380163899, m.d[2][1], 1e-9);
  EXPECT_NEAR(1.1929022461295418, m.d[2][2], 1e-9);

  EXPECT_NEAR(-3.1120271869194629, m_out_ptr->d[0][0], 1e-9);
  EXPECT_NEAR(0.0230368460236385, m_out_ptr->d[0][1], 1e-9);
  EXPECT_NEAR(-0.0907541251406655, m_out_ptr->d[0][2], 1e-9);
  EXPECT_NEAR(3.6721367662729101, m_out_ptr->d[1][0], 1e-9);
  EXPECT_NEAR(0.9274228067911187, m_out_ptr->d[1][1], 1e-9);
  EXPECT_NEAR(0.4716100818581905, m_out_ptr->d[1][2], 1e-9);
  EXPECT_NEAR(-0.4040723287164191, m_out_ptr->d[2][0], 1e-9);
  EXPECT_NEAR(-1.0056575380163899, m_out_ptr->d[2][1], 1e-9);
  EXPECT_NEAR(1.1929022461295418, m_out_ptr->d[2][2], 1e-9);
}

TEST(Mat3InvTest, Singular5) {
  const Mat3 m = {
      {{0.5352533646649009, -0.2267657469102142, 0.0000000000000000},
       {-0.7053792262921386, -0.0845807829631626, 0.0000000000000000},
       {-0.2086547759031565, 0.8983466107023796, 0.0000000000000000}}};
  Mat3 m_out = {
      {{-0.7199199333463573, -0.0748577577583562, 0.1897001575309731},
       {0.1862021678480315, 0.3974489174298892, -0.1958782269440260},
       {-0.5374001941722577, -0.7743838335819353, -0.1376550440686313}}};
  const Mat3 *m_out_ptr = Mat3Inv(&m, &m_out);

  EXPECT_DOUBLE_EQ(INFINITY, m_out.d[0][0]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out.d[0][1]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out.d[0][2]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out.d[1][0]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out.d[1][1]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out.d[1][2]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out.d[2][0]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out.d[2][1]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out.d[2][2]);

  EXPECT_DOUBLE_EQ(INFINITY, m_out_ptr->d[0][0]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out_ptr->d[0][1]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out_ptr->d[0][2]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out_ptr->d[1][0]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out_ptr->d[1][1]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out_ptr->d[1][2]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out_ptr->d[2][0]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out_ptr->d[2][1]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out_ptr->d[2][2]);
}

TEST(Mat3TraceTest, Normal5) {
  const Mat3 m = {
      {{-0.7822405627817122, 0.7506465564151303, -0.0302721449645773},
       {-0.2988255039927086, -0.8103233865362955, 0.1541101706380110},
       {-0.1841589591544524, 0.4960214565717267, 0.4484023523270497}}};
  double m_trace = Mat3Trace(&m);

  EXPECT_NEAR(-1.1441615969909580, m_trace, 1e-9);
}

TEST(Mat3DiagTest, Normal5) {
  const Mat3 m = {
      {{-0.0997953040005766, -0.2818908959280282, 0.3682120370281066},
       {0.3403409986924937, -0.6555088849922419, 0.2930691481217080},
       {-0.7011425800744640, -0.0936775776208358, 0.7908748317157952}}};
  Vec3 v_out = {-0.1296460325435926, 0.1500602432857916, -0.3967939268894509};
  const Vec3 *v_out_ptr = Mat3Diag(&m, &v_out);

  EXPECT_NEAR(-0.0997953040005766, v_out.x, 1e-9);
  EXPECT_NEAR(-0.6555088849922419, v_out.y, 1e-9);
  EXPECT_NEAR(0.7908748317157952, v_out.z, 1e-9);

  EXPECT_NEAR(-0.0997953040005766, v_out_ptr->x, 1e-9);
  EXPECT_NEAR(-0.6555088849922419, v_out_ptr->y, 1e-9);
  EXPECT_NEAR(0.7908748317157952, v_out_ptr->z, 1e-9);
}

TEST(Mat3ScaleTest, Normal7) {
  const Mat3 m = {
      {{0.5450148781580848, 0.0839856539693258, 0.7177089424050220},
       {0.0581760955266737, -0.9772125760797081, 0.8183825423068547},
       {0.1051027891704066, -0.6598193141302786, -0.3910667763150397}}};
  Mat3 m_out = {
      {{0.3200281483743921, 0.3801582952815181, -0.4228469993008117},
       {0.8517813015613231, 0.5555229738247840, 0.6979654964390596},
       {0.2267399933612289, -0.0350768250204421, -0.5643187576543154}}};
  const Mat3 *m_out_ptr = Mat3Scale(&m, 0.7328896367022635, &m_out);

  EXPECT_NEAR(0.3994357560506072, m_out.d[0][0], 1e-9);
  EXPECT_NEAR(0.0615522154257812, m_out.d[0][1], 1e-9);
  EXPECT_NEAR(0.5260014460571824, m_out.d[0][2], 1e-9);
  EXPECT_NEAR(0.0426366575153001, m_out.d[1][0], 1e-9);
  EXPECT_NEAR(-0.7161889698639403, m_out.d[1][1], 1e-9);
  EXPECT_NEAR(0.5997840841147455, m_out.d[1][2], 1e-9);
  EXPECT_NEAR(0.0770287449714939, m_out.d[2][0], 1e-9);
  EXPECT_NEAR(-0.4835747374220766, m_out.d[2][1], 1e-9);
  EXPECT_NEAR(-0.2866087876198548, m_out.d[2][2], 1e-9);

  EXPECT_NEAR(0.3994357560506072, m_out_ptr->d[0][0], 1e-9);
  EXPECT_NEAR(0.0615522154257812, m_out_ptr->d[0][1], 1e-9);
  EXPECT_NEAR(0.5260014460571824, m_out_ptr->d[0][2], 1e-9);
  EXPECT_NEAR(0.0426366575153001, m_out_ptr->d[1][0], 1e-9);
  EXPECT_NEAR(-0.7161889698639403, m_out_ptr->d[1][1], 1e-9);
  EXPECT_NEAR(0.5997840841147455, m_out_ptr->d[1][2], 1e-9);
  EXPECT_NEAR(0.0770287449714939, m_out_ptr->d[2][0], 1e-9);
  EXPECT_NEAR(-0.4835747374220766, m_out_ptr->d[2][1], 1e-9);
  EXPECT_NEAR(-0.2866087876198548, m_out_ptr->d[2][2], 1e-9);
}

TEST(Mat3Vec3MultTest, Normal7) {
  const Mat3 m = {
      {{-0.4737065022781572, 0.8408824030032334, -0.3238841931336383},
       {-0.2851261594139618, 0.6939439218620256, -0.5220275545820696},
       {0.0023246493791933, -0.7813697381751934, -0.1514936103252185}}};
  const Vec3 v = {-0.7580689576684150, -0.1220888083311862,
                  -0.7379359191589794};
  Vec3 v_out = {0.9267063576107462, 0.4366528170324753, -0.5951656626580137};
  const Vec3 *v_out_ptr = Mat3Vec3Mult(&m, &v, &v_out);

  EXPECT_NEAR(0.4954456436545601, v_out.x, 1e-9);
  EXPECT_NEAR(0.5166453873189697, v_out.y, 1e-9);
  EXPECT_NEAR(0.2054268322500831, v_out.z, 1e-9);

  EXPECT_NEAR(0.4954456436545601, v_out_ptr->x, 1e-9);
  EXPECT_NEAR(0.5166453873189697, v_out_ptr->y, 1e-9);
  EXPECT_NEAR(0.2054268322500831, v_out_ptr->z, 1e-9);
}

TEST(Mat3TransVec3MultTest, Normal7) {
  const Mat3 m = {
      {{-0.9579487993667677, 0.8478470408346785, 0.4990409483745619},
       {-0.2567344721453433, -0.8751068789852319, 0.3176274655198157},
       {0.6530143756349704, 0.3521732101153252, -0.6363365846500786}}};
  const Vec3 v = {-0.4459923859071782, 0.8781899320688267, 0.9305874152487328};
  Vec3 v_out = {-0.0592151751694117, -0.1730134433552699, 0.6731903831120514};
  const Vec3 *v_out_ptr = Mat3TransVec3Mult(&m, &v, &v_out);

  EXPECT_NEAR(0.8094632018958685, v_out.x, 1e-9);
  EXPECT_NEAR(-0.8189154179141331, v_out.y, 1e-9);
  EXPECT_NEAR(-0.5357980384006343, v_out.z, 1e-9);

  EXPECT_NEAR(0.8094632018958685, v_out_ptr->x, 1e-9);
  EXPECT_NEAR(-0.8189154179141331, v_out_ptr->y, 1e-9);
  EXPECT_NEAR(-0.5357980384006343, v_out_ptr->z, 1e-9);
}

TEST(Mat3Vec3LeftDivideTest, Normal7) {
  const Mat3 m = {
      {{0.6230205294487241, -0.7243776430602298, 0.7328612035826967},
       {0.0369994448219033, -0.7904781117221364, 0.7596258627911638},
       {-0.3871875463247449, -0.1309368004404390, -0.2449967983091665}}};
  const Vec3 v = {0.7876980945954735, 0.1970458850739039, 0.6734136144843670};
  Vec3 v_out = {-0.6862189637994782, 0.9736744005541178, 0.2314404385551649};
  const Vec3 *v_out_ptr = Mat3Vec3LeftDivide(&m, &v, &v_out);

  EXPECT_NEAR(1.2190633129073365, v_out.x, 1e-9);
  EXPECT_NEAR(-3.0952923501739740, v_out.y, 1e-9);
  EXPECT_NEAR(-3.0209867058341668, v_out.z, 1e-9);

  EXPECT_NEAR(1.2190633129073365, v_out_ptr->x, 1e-9);
  EXPECT_NEAR(-3.0952923501739740, v_out_ptr->y, 1e-9);
  EXPECT_NEAR(-3.0209867058341668, v_out_ptr->z, 1e-9);
}

TEST(Mat3Vec3LeftDivideTest, Singular7) {
  const Mat3 m = {
      {{0.2724409358715103, -0.4650419539396202, 0.0000000000000000},
       {0.8577580277597132, -0.9980985173650938, 0.0000000000000000},
       {0.0320731138960892, -0.8821978827800596, 0.0000000000000000}}};
  const Vec3 v = {0.7720168414406252, 0.4457102631066128, 0.5256700153738489};
  Vec3 v_out = {0.7238730607358157, -0.9975577380231626, 0.9613431367581580};
  const Vec3 *v_out_ptr = Mat3Vec3LeftDivide(&m, &v, &v_out);

  EXPECT_DOUBLE_EQ(INFINITY, v_out.x);
  EXPECT_DOUBLE_EQ(INFINITY, v_out.y);
  EXPECT_DOUBLE_EQ(INFINITY, v_out.z);

  EXPECT_DOUBLE_EQ(INFINITY, v_out_ptr->x);
  EXPECT_DOUBLE_EQ(INFINITY, v_out_ptr->y);
  EXPECT_DOUBLE_EQ(INFINITY, v_out_ptr->z);
}

TEST(Mat3DetTest, Normal7) {
  const Mat3 m = {
      {{0.9407159670997201, -0.8826132501854635, 0.5324936310581776},
       {0.6479424379092811, 0.4893724825712273, 0.1534526177787321},
       {0.4028231085711240, 0.6776247117900245, -0.3656193317580176}}};
  double m_det = Mat3Det(&m);

  EXPECT_NEAR(-0.4009579945060640, m_det, 1e-9);
}

TEST(Mat3InvTest, Normal7) {
  const Mat3 m = {
      {{-0.8253600375590811, 0.5339279567310367, 0.1415516384535993},
       {0.4328340853006729, -0.8117735016786196, 0.7696823637690098},
       {0.9277876798173861, 0.2834178493215538, 0.4308185989561200}}};
  Mat3 m_out = {
      {{0.3615799019645827, 0.4074293775640057, -0.4550501338083646},
       {0.6208966678822534, 0.8088053039018588, 0.2936235643596197},
       {-0.5053316505800771, -0.5533154337994635, 0.4478398066235014}}};
  const Mat3 *m_out_ptr = Mat3Inv(&m, &m_out);

  EXPECT_NEAR(-0.6494481750655837, m_out.d[0][0], 1e-9);
  EXPECT_NEAR(-0.2171897566364980, m_out.d[0][1], 1e-9);
  EXPECT_NEAR(0.6014075974747293, m_out.d[0][2], 1e-9);
  EXPECT_NEAR(0.6034273437090543, m_out.d[1][0], 1e-9);
  EXPECT_NEAR(-0.5568592528331512, m_out.d[1][1], 1e-9);
  EXPECT_NEAR(0.7965965667431629, m_out.d[1][2], 1e-9);
  EXPECT_NEAR(1.0016464854941129, m_out.d[2][0], 1e-9);
  EXPECT_NEAR(0.8340629514912308, m_out.d[2][1], 1e-9);
  EXPECT_NEAR(0.5019554757270761, m_out.d[2][2], 1e-9);

  EXPECT_NEAR(-0.6494481750655837, m_out_ptr->d[0][0], 1e-9);
  EXPECT_NEAR(-0.2171897566364980, m_out_ptr->d[0][1], 1e-9);
  EXPECT_NEAR(0.6014075974747293, m_out_ptr->d[0][2], 1e-9);
  EXPECT_NEAR(0.6034273437090543, m_out_ptr->d[1][0], 1e-9);
  EXPECT_NEAR(-0.5568592528331512, m_out_ptr->d[1][1], 1e-9);
  EXPECT_NEAR(0.7965965667431629, m_out_ptr->d[1][2], 1e-9);
  EXPECT_NEAR(1.0016464854941129, m_out_ptr->d[2][0], 1e-9);
  EXPECT_NEAR(0.8340629514912308, m_out_ptr->d[2][1], 1e-9);
  EXPECT_NEAR(0.5019554757270761, m_out_ptr->d[2][2], 1e-9);
}

TEST(Mat3InvTest, Reuse7) {
  Mat3 m = {{{0.9115569395390188, 0.8468104512526617, 0.7879695564494713},
             {0.7730020654670251, 0.8442255328531749, -0.5501534810737319},
             {0.0546990779044889, 0.7441233161736318, -0.4496175452310767}}};
  const Mat3 *m_out_ptr = Mat3Inv(&m, &m);

  EXPECT_NEAR(0.0418083603256900, m.d[0][0], 1e-9);
  EXPECT_NEAR(1.3566340784865520, m.d[0][1], 1e-9);
  EXPECT_NEAR(-1.5867113133102164, m.d[0][2], 1e-9);
  EXPECT_NEAR(0.4453375509481592, m.d[1][0], 1e-9);
  EXPECT_NEAR(-0.6354045946911765, m.d[1][1], 1e-9);
  EXPECT_NEAR(1.5579518405815389, m.d[1][2], 1e-9);
  EXPECT_NEAR(0.7421261414859016, m.d[2][0], 1e-9);
  EXPECT_NEAR(-0.8865595775664784, m.d[2][1], 1e-9);
  EXPECT_NEAR(0.1612851746619220, m.d[2][2], 1e-9);

  EXPECT_NEAR(0.0418083603256900, m_out_ptr->d[0][0], 1e-9);
  EXPECT_NEAR(1.3566340784865520, m_out_ptr->d[0][1], 1e-9);
  EXPECT_NEAR(-1.5867113133102164, m_out_ptr->d[0][2], 1e-9);
  EXPECT_NEAR(0.4453375509481592, m_out_ptr->d[1][0], 1e-9);
  EXPECT_NEAR(-0.6354045946911765, m_out_ptr->d[1][1], 1e-9);
  EXPECT_NEAR(1.5579518405815389, m_out_ptr->d[1][2], 1e-9);
  EXPECT_NEAR(0.7421261414859016, m_out_ptr->d[2][0], 1e-9);
  EXPECT_NEAR(-0.8865595775664784, m_out_ptr->d[2][1], 1e-9);
  EXPECT_NEAR(0.1612851746619220, m_out_ptr->d[2][2], 1e-9);
}

TEST(Mat3InvTest, Singular7) {
  const Mat3 m = {
      {{-0.2797506851139984, 0.9301439850938691, 0.0000000000000000},
       {0.8920549712930876, -0.4133533930289199, 0.0000000000000000},
       {0.6713256792687805, -0.8226775282098815, 0.0000000000000000}}};
  Mat3 m_out = {
      {{-0.3486910678563302, -0.2345778224017978, 0.0011089358943981},
       {-0.9931256664418493, 0.1705518327480176, 0.6372271099514202},
       {-0.2808694584750868, 0.4533974757543691, -0.4486227580623947}}};
  const Mat3 *m_out_ptr = Mat3Inv(&m, &m_out);

  EXPECT_DOUBLE_EQ(INFINITY, m_out.d[0][0]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out.d[0][1]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out.d[0][2]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out.d[1][0]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out.d[1][1]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out.d[1][2]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out.d[2][0]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out.d[2][1]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out.d[2][2]);

  EXPECT_DOUBLE_EQ(INFINITY, m_out_ptr->d[0][0]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out_ptr->d[0][1]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out_ptr->d[0][2]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out_ptr->d[1][0]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out_ptr->d[1][1]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out_ptr->d[1][2]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out_ptr->d[2][0]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out_ptr->d[2][1]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out_ptr->d[2][2]);
}

TEST(Mat3TraceTest, Normal7) {
  const Mat3 m = {
      {{-0.1832417799634773, 0.4825981033103297, 0.3814136587985719},
       {0.0892555905901955, -0.4996676930307884, 0.0828978130389568},
       {0.7513544555124747, -0.0638501397755513, -0.4793279577801555}}};
  double m_trace = Mat3Trace(&m);

  EXPECT_NEAR(-1.1622374307744212, m_trace, 1e-9);
}

TEST(Mat3DiagTest, Normal7) {
  const Mat3 m = {
      {{-0.0893796733195060, -0.9762400008088006, 0.5997257491568220},
       {-0.9260883530436816, 0.7478913363677107, 0.2013698614472534},
       {-0.8292840510606347, -0.5012131850200003, 0.7003812133752707}}};
  Vec3 v_out = {-0.4777040825523364, 0.5612795228691272, -0.0674149604164502};
  const Vec3 *v_out_ptr = Mat3Diag(&m, &v_out);

  EXPECT_NEAR(-0.0893796733195060, v_out.x, 1e-9);
  EXPECT_NEAR(0.7478913363677107, v_out.y, 1e-9);
  EXPECT_NEAR(0.7003812133752707, v_out.z, 1e-9);

  EXPECT_NEAR(-0.0893796733195060, v_out_ptr->x, 1e-9);
  EXPECT_NEAR(0.7478913363677107, v_out_ptr->y, 1e-9);
  EXPECT_NEAR(0.7003812133752707, v_out_ptr->z, 1e-9);
}

TEST(Mat3ScaleTest, Normal9) {
  const Mat3 m = {
      {{0.0235556019128600, -0.2195291846595819, 0.7833860597274340},
       {0.0480161002403663, -0.3283388942653729, -0.0076054980385074},
       {-0.6910396300613597, -0.2118469298392693, -0.3692062214370837}}};
  Mat3 m_out = {
      {{-0.8534916344387335, 0.2695528478940874, -0.8253714075861738},
       {-0.3558815359033116, -0.9230584169932790, -0.4749625440601402},
       {-0.9615490107431999, 0.3289295031610497, -0.5157122052564920}}};
  const Mat3 *m_out_ptr = Mat3Scale(&m, -0.6094777027240030, &m_out);

  EXPECT_NEAR(-0.0143566141401310, m_out.d[0][0], 1e-9);
  EXPECT_NEAR(0.1337981431471954, m_out.d[0][1], 1e-9);
  EXPECT_NEAR(-0.4774563360286851, m_out.d[0][2], 1e-9);
  EXPECT_NEAR(-0.0292647424682639, m_out.d[1][0], 1e-9);
  EXPECT_NEAR(0.2001152349917988, m_out.d[1][1], 1e-9);
  EXPECT_NEAR(0.0046353814725814, m_out.d[1][2], 1e-9);
  EXPECT_NEAR(0.4211732462210424, m_out.d[2][0], 1e-9);
  EXPECT_NEAR(0.1291159801275709, m_out.d[2][1], 1e-9);
  EXPECT_NEAR(0.2250229596728833, m_out.d[2][2], 1e-9);

  EXPECT_NEAR(-0.0143566141401310, m_out_ptr->d[0][0], 1e-9);
  EXPECT_NEAR(0.1337981431471954, m_out_ptr->d[0][1], 1e-9);
  EXPECT_NEAR(-0.4774563360286851, m_out_ptr->d[0][2], 1e-9);
  EXPECT_NEAR(-0.0292647424682639, m_out_ptr->d[1][0], 1e-9);
  EXPECT_NEAR(0.2001152349917988, m_out_ptr->d[1][1], 1e-9);
  EXPECT_NEAR(0.0046353814725814, m_out_ptr->d[1][2], 1e-9);
  EXPECT_NEAR(0.4211732462210424, m_out_ptr->d[2][0], 1e-9);
  EXPECT_NEAR(0.1291159801275709, m_out_ptr->d[2][1], 1e-9);
  EXPECT_NEAR(0.2250229596728833, m_out_ptr->d[2][2], 1e-9);
}

TEST(Mat3Vec3MultTest, Normal9) {
  const Mat3 m = {
      {{-0.9357214621893382, 0.0842266704312016, -0.9385853082485316},
       {-0.7330348904160788, -0.2595247366273639, 0.5058894533275446},
       {-0.1559530957033644, 0.6539875015078864, -0.5857404695147963}}};
  const Vec3 v = {-0.9941999369625190, -0.4200198202333607,
                  -0.4049682469236504};
  Vec3 v_out = {-0.0000490154010115, 0.8982972318867299, -0.3015891435043381};
  const Vec3 *v_out_ptr = Mat3Vec3Mult(&m, &v, &v_out);

  EXPECT_NEAR(1.2750145946194500, v_out.x, 1e-9);
  EXPECT_NEAR(0.6329196100161086, v_out.y, 1e-9);
  EXPECT_NEAR(0.1175671361908303, v_out.z, 1e-9);

  EXPECT_NEAR(1.2750145946194500, v_out_ptr->x, 1e-9);
  EXPECT_NEAR(0.6329196100161086, v_out_ptr->y, 1e-9);
  EXPECT_NEAR(0.1175671361908303, v_out_ptr->z, 1e-9);
}

TEST(Mat3TransVec3MultTest, Normal9) {
  const Mat3 m = {
      {{-0.1409755791728609, -0.9943112847661890, -0.3918246766580837},
       {0.7007295541058083, -0.3412970520948078, -0.1699147841273982},
       {-0.9395227198594955, 0.2540317371909517, -0.0940505606120277}}};
  const Vec3 v = {-0.4882629192415209, 0.4286684131414815, -0.5746401502070684};
  Vec3 v_out = {-0.3303992493381041, -0.2501777356078501, -0.7663394446069545};
  const Vec3 *v_out_ptr = Mat3TransVec3Mult(&m, &v, &v_out);

  EXPECT_NEAR(0.9091012506915939, v_out.x, 1e-9);
  EXPECT_NEAR(0.1932052291866087, v_out.y, 1e-9);
  EXPECT_NEAR(0.1725215878519263, v_out.z, 1e-9);

  EXPECT_NEAR(0.9091012506915939, v_out_ptr->x, 1e-9);
  EXPECT_NEAR(0.1932052291866087, v_out_ptr->y, 1e-9);
  EXPECT_NEAR(0.1725215878519263, v_out_ptr->z, 1e-9);
}

TEST(Mat3Vec3LeftDivideTest, Normal9) {
  const Mat3 m = {
      {{0.9297623162587363, 0.5359821618159566, -0.6548181079112461},
       {0.4561913125099868, 0.7326230234879969, 0.4287347178794321},
       {-0.3557161264362740, -0.1937073400071994, 0.9212246595636062}}};
  const Vec3 v = {-0.8892173892929904, -0.1418335928515717,
                  -0.9816800161331591};
  Vec3 v_out = {0.2283200916496393, -0.0577708544805293, -0.9951204278032231};
  const Vec3 *v_out_ptr = Mat3Vec3LeftDivide(&m, &v, &v_out);

  EXPECT_NEAR(-4.6271072300872058, v_out.x, 1e-9);
  EXPECT_NEAR(3.8794336057358043, v_out.y, 1e-9);
  EXPECT_NEAR(-2.0365736985505620, v_out.z, 1e-9);

  EXPECT_NEAR(-4.6271072300872058, v_out_ptr->x, 1e-9);
  EXPECT_NEAR(3.8794336057358043, v_out_ptr->y, 1e-9);
  EXPECT_NEAR(-2.0365736985505620, v_out_ptr->z, 1e-9);
}

TEST(Mat3Vec3LeftDivideTest, Singular9) {
  const Mat3 m = {
      {{-0.2048501452877822, -0.1392826051592186, 0.0000000000000000},
       {-0.1581049805320554, 0.1804412013753292, 0.0000000000000000},
       {0.5771480251286532, -0.5287383647810160, 0.0000000000000000}}};
  const Vec3 v = {0.1960505631402132, 0.9508656534216269, 0.3817948166436578};
  Vec3 v_out = {-0.2178519626742239, -0.1366189485192835, 0.2088055576510623};
  const Vec3 *v_out_ptr = Mat3Vec3LeftDivide(&m, &v, &v_out);

  EXPECT_DOUBLE_EQ(INFINITY, v_out.x);
  EXPECT_DOUBLE_EQ(INFINITY, v_out.y);
  EXPECT_DOUBLE_EQ(INFINITY, v_out.z);

  EXPECT_DOUBLE_EQ(INFINITY, v_out_ptr->x);
  EXPECT_DOUBLE_EQ(INFINITY, v_out_ptr->y);
  EXPECT_DOUBLE_EQ(INFINITY, v_out_ptr->z);
}

TEST(Mat3DetTest, Normal9) {
  const Mat3 m = {
      {{-0.0786508669251831, -0.2759332311176055, 0.7167978181343586},
       {-0.0050170031847425, -0.2033709595098296, -0.8766638743064998},
       {0.4149085030930673, -0.3958277791694660, -0.7163791462233533}}};
  double m_det = Mat3Det(&m);

  EXPECT_NEAR(0.1790992634980078, m_det, 1e-9);
}

TEST(Mat3InvTest, Normal9) {
  const Mat3 m = {
      {{0.1399423283342514, -0.6785822227541447, 0.8498157753899620},
       {0.3703267927229954, 0.2416508948554530, -0.0576870725446632},
       {-0.6765124228985950, -0.5452792889962408, -0.6470214149818989}}};
  Mat3 m_out = {
      {{0.1869686775302271, -0.9785857202072228, 0.0165676978490641},
       {-0.1735719207939002, 0.3265144707659755, -0.0740072982421609},
       {-0.9609935337281561, -0.4790992220455048, -0.0132726307914535}}};
  const Mat3 *m_out_ptr = Mat3Inv(&m, &m_out);

  EXPECT_NEAR(0.7571830309508581, m_out.d[0][0], 1e-9);
  EXPECT_NEAR(3.6383553868395477, m_out.d[0][1], 1e-9);
  EXPECT_NEAR(0.6701169441630854, m_out.d[0][2], 1e-9);
  EXPECT_NEAR(-1.1233654022497250, m_out.d[1][0], 1e-9);
  EXPECT_NEAR(-1.9527999148743247, m_out.d[1][1], 1e-9);
  EXPECT_NEAR(-1.3013515634997879, m_out.d[1][2], 1e-9);
  EXPECT_NEAR(0.1550244839705116, m_out.d[2][0], 1e-9);
  EXPECT_NEAR(-2.1584622033282423, m_out.d[2][1], 1e-9);
  EXPECT_NEAR(-1.1494865007871000, m_out.d[2][2], 1e-9);

  EXPECT_NEAR(0.7571830309508581, m_out_ptr->d[0][0], 1e-9);
  EXPECT_NEAR(3.6383553868395477, m_out_ptr->d[0][1], 1e-9);
  EXPECT_NEAR(0.6701169441630854, m_out_ptr->d[0][2], 1e-9);
  EXPECT_NEAR(-1.1233654022497250, m_out_ptr->d[1][0], 1e-9);
  EXPECT_NEAR(-1.9527999148743247, m_out_ptr->d[1][1], 1e-9);
  EXPECT_NEAR(-1.3013515634997879, m_out_ptr->d[1][2], 1e-9);
  EXPECT_NEAR(0.1550244839705116, m_out_ptr->d[2][0], 1e-9);
  EXPECT_NEAR(-2.1584622033282423, m_out_ptr->d[2][1], 1e-9);
  EXPECT_NEAR(-1.1494865007871000, m_out_ptr->d[2][2], 1e-9);
}

TEST(Mat3InvTest, Reuse9) {
  Mat3 m = {{{0.9619257022697227, 0.6023915354161629, 0.3970071957688392},
             {0.1714695057914981, 0.7638883953688034, 0.8523507787977953},
             {-0.5031770519430501, -0.6048437557080641, -0.5105483410060812}}};
  const Mat3 *m_out_ptr = Mat3Inv(&m, &m);

  EXPECT_NEAR(4.7264252304899648, m.d[0][0], 1e-9);
  EXPECT_NEAR(2.5384388574979058, m.d[0][1], 1e-9);
  EXPECT_NEAR(7.9131883102891427, m.d[0][2], 1e-9);
  EXPECT_NEAR(-12.8513205214608792, m.d[1][0], 1e-9);
  EXPECT_NEAR(-10.9690189666537545, m.d[1][1], 1e-9);
  EXPECT_NEAR(-28.3058770743409447, m.d[1][2], 1e-9);
  EXPECT_NEAR(10.5667021572734168, m.d[2][0], 1e-9);
  EXPECT_NEAR(10.4931463234478564, m.d[2][1], 1e-9);
  EXPECT_NEAR(23.7761975857034074, m.d[2][2], 1e-9);

  EXPECT_NEAR(4.7264252304899648, m_out_ptr->d[0][0], 1e-9);
  EXPECT_NEAR(2.5384388574979058, m_out_ptr->d[0][1], 1e-9);
  EXPECT_NEAR(7.9131883102891427, m_out_ptr->d[0][2], 1e-9);
  EXPECT_NEAR(-12.8513205214608792, m_out_ptr->d[1][0], 1e-9);
  EXPECT_NEAR(-10.9690189666537545, m_out_ptr->d[1][1], 1e-9);
  EXPECT_NEAR(-28.3058770743409447, m_out_ptr->d[1][2], 1e-9);
  EXPECT_NEAR(10.5667021572734168, m_out_ptr->d[2][0], 1e-9);
  EXPECT_NEAR(10.4931463234478564, m_out_ptr->d[2][1], 1e-9);
  EXPECT_NEAR(23.7761975857034074, m_out_ptr->d[2][2], 1e-9);
}

TEST(Mat3InvTest, Singular9) {
  const Mat3 m = {
      {{0.0000000000000000, 0.4239151065933020, 0.9835005957079834},
       {0.0000000000000000, 0.8986922413963763, -0.2196161844934157},
       {0.0000000000000000, 0.5254168027159150, -0.9448886546419193}}};
  Mat3 m_out = {
      {{-0.8185401734426487, -0.7719892805053832, 0.7709709884174769},
       {-0.2586230304859183, -0.7439832702458129, 0.6737871570992935},
       {-0.5227609509898130, 0.1292804867468178, 0.5105426698715199}}};
  const Mat3 *m_out_ptr = Mat3Inv(&m, &m_out);

  EXPECT_DOUBLE_EQ(INFINITY, m_out.d[0][0]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out.d[0][1]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out.d[0][2]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out.d[1][0]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out.d[1][1]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out.d[1][2]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out.d[2][0]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out.d[2][1]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out.d[2][2]);

  EXPECT_DOUBLE_EQ(INFINITY, m_out_ptr->d[0][0]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out_ptr->d[0][1]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out_ptr->d[0][2]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out_ptr->d[1][0]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out_ptr->d[1][1]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out_ptr->d[1][2]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out_ptr->d[2][0]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out_ptr->d[2][1]);
  EXPECT_DOUBLE_EQ(INFINITY, m_out_ptr->d[2][2]);
}

TEST(Mat3TraceTest, Normal9) {
  const Mat3 m = {
      {{0.3433865898510349, -0.0035751832884370, -0.5639693340424938},
       {-0.9465259682150386, -0.4953907307745071, -0.7571237571489144},
       {-0.0711537587505606, -0.6894079270644602, -0.8981647457952970}}};
  double m_trace = Mat3Trace(&m);

  EXPECT_NEAR(-1.0501688867187693, m_trace, 1e-9);
}

TEST(Mat3DiagTest, Normal9) {
  const Mat3 m = {
      {{-0.4235494514137426, -0.4947846191615577, 0.1459011100092278},
       {-0.1650517758107288, -0.9392157323023576, 0.9502704874809635},
       {0.7702340519782875, 0.4359678302715568, -0.8280925378382602}}};
  Vec3 v_out = {-0.0767262195033553, -0.7725891941429199, -0.1126953573207536};
  const Vec3 *v_out_ptr = Mat3Diag(&m, &v_out);

  EXPECT_NEAR(-0.4235494514137426, v_out.x, 1e-9);
  EXPECT_NEAR(-0.9392157323023576, v_out.y, 1e-9);
  EXPECT_NEAR(-0.8280925378382602, v_out.z, 1e-9);

  EXPECT_NEAR(-0.4235494514137426, v_out_ptr->x, 1e-9);
  EXPECT_NEAR(-0.9392157323023576, v_out_ptr->y, 1e-9);
  EXPECT_NEAR(-0.8280925378382602, v_out_ptr->z, 1e-9);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

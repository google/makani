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

#include <gsl/gsl_linalg.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_randist.h>
#include <gsl/gsl_rng.h>
#include <gsl/gsl_vector.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>

#include <string>

#include "common/c_math/linalg.h"
#include "common/c_math/util/linalg_io.h"
#include "lib/util/random_matrix.h"
#include "lib/util/test_util.h"

using ::test_util::Rand;
using ::test_util::RandNormal;

const double kConsistencyTol = 1e-4;

// These are a few functions to make random_matrix.h easier to work with.
static void VecRandomGaussian(const gsl_rng *rng, Vec *v) {
  MatArrRandomGaussian(v->length, 1U, rng, v->d);
}

static void MatRandomGaussian(const gsl_rng *rng, Mat *m) {
  MatArrRandomGaussian(m->nr, m->nc, rng, m->d);
}

static void MatRandomOrthogonal(const gsl_rng *rng, Mat *m) {
  MatArrRandomOrthogonal(m->nr, rng, m->d);
}

static void MatRandomWithRank(int32_t r, const gsl_rng *rng, Mat *m) {
  MatArrRandomWithRank(m->nr, m->nc, r, rng, m->d);
}

TEST(VecAxpy, Normal_0) {
  VEC_INIT(22, x, {0.8748763783944802, 0.0319157960462900, 0.7119917165040284,
                   0.5671882990192584, 0.6728288551761079, 0.2445725070708274,
                   0.0260467464103307, 0.6265081315964255, 0.1108490931358967,
                   0.2593364913621732, 0.1583801926347032, 0.2960087242071343,
                   0.7357492947113683, 0.7222542690736602, 0.1202982126924455,
                   0.5268420036689354, 0.6564668714957285, 0.6118880502673113,
                   0.4227168405011201, 0.5999741903701343, 0.6489909644705955,
                   0.5244447700048280});
  VEC_INIT(22, y, {0.8567416191545223, 0.8081337059832551, 0.7425512419518339,
                   0.4953940573790519, 0.9385872291979516, 0.7498029406126708,
                   0.0434890466315348, 0.2593095991614560, 0.9395243948956877,
                   0.9874364768617904, 0.5734080927408878, 0.3939263567909877,
                   0.6354020567669875, 0.1697004849361995, 0.1923187377830073,
                   0.3015866209999493, 0.4762399428142793, 0.8064965556619733,
                   0.3784488228170308, 0.7223862122987647, 0.8387503016702088,
                   0.4561864003823905});
  VEC_INIT(22, zout,
           {1.1584178711154969, 0.8191389610882847, 0.9880613480268972,
            0.6909728207676074, 1.1705931356932346, 0.8341368180471329,
            0.0524705267890703, 0.4753431207315877, 0.9777475539343148,
            1.0768612946336720, 0.6280210004371459, 0.4959965517029063,
            0.8891042884111655, 0.4187493405663034, 0.2338001571228006,
            0.4832531112235436, 0.7026038843037157, 1.0174887577409721,
            0.5242107101332555, 0.9292702584269030, 1.0625363891141517,
            0.6370262727149065});
  const double a = 0.344821576408992;
  VecAxpy(a, &x, &y);
  for (int32_t i = 0; i < 22; i++) {
    EXPECT_NEAR(zout.d[i], y.d[i], 1e-9);
  }
}

TEST(VecAxpy, Consistency_0) {
  for (int32_t t = 0; t < test_util::kNumTests; t++) {
    const int32_t len =
        rand() % VEC_MAX_ELEMENTS;  // NOLINT(runtime/threadsafe_fn)
    VEC(len, x);
    VEC(len, y);
    for (int32_t i = 0; i < len; i++) {
      x.d[i] = Rand(-1e9, 1e9);
      y.d[i] = Rand(-1e9, 1e9);
    }
    const double a = Rand(-1e9, 1e9);
    VEC(len, z);
    VEC(len, w);
    VecZero(&z);
    VecZero(&w);

    // w = -y
    // y = a*x + y
    // w = y + w
    // z = w/a
    // => z == x
    VecAxpy(-1, &y, &w);
    VecAxpy(a, &x, &y);
    VecAxpy(1, &y, &w);
    VecAxpy(1 / a, &w, &z);
    for (int32_t i = 0; i < len; i++) {
      EXPECT_NEAR(x.d[i], z.d[i], kConsistencyTol);
    }
  }
}

TEST(VecSlice, basic_checks) {
  int32_t I[9] = {1, 0, 0, 1, 0, 1, 0, 0, 1};
  VEC_INIT(9, v_in, {0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0});
  VEC_INIT(4, v_out, {10.0, 10.0, 10.0, 10.0});

  VecSlice(&v_in, I, &v_out);

  EXPECT_EQ(VecGet(&v_out, 0), VecGet(&v_in, 0));
  EXPECT_EQ(VecGet(&v_out, 1), VecGet(&v_in, 3));
  EXPECT_EQ(VecGet(&v_out, 2), VecGet(&v_in, 5));
  EXPECT_EQ(VecGet(&v_out, 3), VecGet(&v_in, 8));
}

TEST(VecSlice, scalar) {
  int32_t I[9] = {0, 0, 0, 0, 0, 0, 0, 0, 1};
  VEC_INIT(9, v_in, {0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0});
  VEC_INIT(1, v_out, {10.0});

  VecSlice(&v_in, I, &v_out);

  EXPECT_EQ(VecGet(&v_out, 0), VecGet(&v_in, 8));
}

TEST(VecSliceSet, SomeElements) {
  VEC_INIT(9, v_out, {20.0, 21.0, 22.0, 23.0, 24.0, 25.0, 26.0, 27.0, 28.0});
  int32_t I[9] = {1, 0, 1, 0, 0, 0, 1, 0, 1};
  VEC_INIT(4, v_in, {1.0, 2.0, 3.0, 4.0});
  VEC_INIT(9, v_ans, {1.0, 21.0, 2.0, 23.0, 24.0, 25.0, 3.0, 27.0, 4.0});

  VecSliceSet(&v_in, I, &v_out);

  EXPECT_NEAR_VEC(v_out, v_ans, kConsistencyTol);
}

TEST(VecSliceSet, OneElements) {
  VEC_INIT(9, v_out, {20.0, 21.0, 22.0, 23.0, 24.0, 25.0, 26.0, 27.0, 28.0});
  int32_t I[9] = {0, 0, 1, 0, 0, 0, 0, 0, 0};
  VEC_INIT(1, v_in, {10.0});
  VEC_INIT(9, v_ans, {20.0, 21.0, 10.0, 23.0, 24.0, 25.0, 26.0, 27.0, 28.0});

  VecSliceSet(&v_in, I, &v_out);

  EXPECT_NEAR_VEC(v_out, v_ans, kConsistencyTol);
}

TEST(VecSliceSet, AllElements) {
  VEC_INIT(9, v_out, {-0.0, -1.0, -2.0, -3.0, -4.0, -5.0, -6.0, -7.0, -8.0});
  int32_t I[9] = {1, 1, 1, 1, 1, 1, 1, 1, 1};
  VEC_INIT(9, v_in, {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0});
  VEC_INIT(9, v_ans, {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0});

  VecSliceSet(&v_in, I, &v_out);

  EXPECT_NEAR_VEC(v_out, v_ans, kConsistencyTol);
}

TEST(VecPtr, Normal) {
  VEC_INIT(4, v, {1.0, 2.0, 3.0, 4.0});
  for (int32_t i = 0; i < v.length; ++i) {
    EXPECT_NEAR(*VecPtr(&v, i), VecGet(&v, i), 1e-9);
  }
}

#if !defined(NDEBUG)

TEST(VecPtrDeath, OutOfBounds) {
  VEC_INIT(4, v, {1.0, 2.0, 3.0, 4.0});
  EXPECT_DEATH(VecPtr(&v, 4), "");
}

#endif  // !defined(NDEBUG)

TEST(VecInit, Normal) {
  double a[5] = {1.0, 2.0, 3.0, 4.0, 5.0};
  VEC(5, v);
  VecInit(a, 5U, &v);
  for (int32_t i = 0; i < 5; ++i) {
    EXPECT_NEAR(v.d[i], a[i], 1e-9);
  }
}

TEST(VecResize, VariableLength) {
  double a_data[22];
  Vec a = {2, a_data, 1, 22};
  EXPECT_EQ(VecResize(3, &a), 1);
  EXPECT_EQ(a.length, 3U);
  EXPECT_EQ(VecResize(22, &a), 1);
}

#if !defined(NDEBUG)

TEST(VecResize, VariableLengthTooLarge) {
  double a_data[22];
  Vec a = {2, a_data, 1, 22};
  EXPECT_DEATH(VecResize(23, &a), "");
}

TEST(VecResizeDeath, NotVariableLength) {
  double a_data[22];
  Vec a = {2, a_data, 0, 22};
  EXPECT_DEATH(VecResize(3, &a), "");
}

TEST(VecResizeDeath, NotVariableLengthWithZeroLength) {
  double a_data[22];
  Vec a = {2, a_data, 0, 0};
  EXPECT_DEATH(VecResize(3, &a), "");
}

TEST(MatVecGenMult, DeathTest) {
  MAT(4, 4, A);
  VEC(4, x);
  VEC(4, y);
  VEC(4, z);
  EXPECT_DEATH(MatVecGenMult(kNoTrans, 1.0, &A, &x, 0.0, &y, &x), "");
  EXPECT_DEATH(MatVecGenMult(kNoTrans, 1.0, &A, &x, 1.0, NULL, &z), "");
  VecResize(2, &x);
  EXPECT_DEATH(MatVecGenMult(kNoTrans, 1.0, &A, &x, 0.0, &y, &z), "");
}

#endif  // !defined(NDEBUG)

TEST(MatVecGenMult, UnitaryTest) {
  gsl_rng *rng = gsl_rng_alloc(gsl_rng_taus);

  for (int32_t n = 1; n < 10; ++n) {
    for (int32_t rpt = 0; rpt < 100; ++rpt) {
      MAT(n, n, U);
      VEC(n, v);
      VEC(n, w);
      VEC(n, r);

      VecRandomGaussian(rng, &v);
      MatRandomOrthogonal(rng, &U);
      // Check that 2.0*U*v has twice the norm of v.
      MatVecGenMult(kNoTrans, 2.0, &U, &v, 0.0, &w, &w);
      EXPECT_NEAR(VecNorm(&w), 2.0 * VecNorm(&v), 1e-10);
      MatVecGenMult(kNoTrans, 2.0, &U, &v, 0.0, NULL, &r);
      EXPECT_NEAR_VEC(w, r, 0.0);
      // Check that 2.0*U'(2.0*U*v)- 3.0*v == v
      MatVecGenMult(kTrans, 2.0, &U, &w, -3.0, &v, &r);
      EXPECT_NEAR_VEC(r, v, 1e-10);
    }
  }
  gsl_rng_free(rng);
}

#if !defined(NDEBUG)

TEST(MatGenMult, DeathTest) {
  MAT(4, 4, A);
  MAT(4, 4, B);
  MAT(4, 4, C);
  MAT(4, 4, D);
  EXPECT_DEATH(MatGenMult(kNoTrans, kNoTrans, 1.0, &A, &B, 0.0, &C, &A), "");
  EXPECT_DEATH(MatGenMult(kNoTrans, kNoTrans, 1.0, &A, &B, 0.0, &C, &B), "");
  EXPECT_DEATH(MatGenMult(kNoTrans, kNoTrans, 1.0, &A, &B, 1.0, NULL, &D), "");
  MatResize(4, 2, &A);
  EXPECT_DEATH(MatGenMult(kNoTrans, kNoTrans, 1.0, &A, &B, 0.0, &C, &D), "");
  MatResize(4, 4, &A);
  MatResize(4, 2, &C);
  EXPECT_DEATH(MatGenMult(kNoTrans, kNoTrans, 1.0, &A, &B, 0.0, &C, &D), "");
}

#endif  // !defined(NDEBUG)

TEST(MatGenMult, SquareTest) {
  gsl_rng *rng = gsl_rng_alloc(gsl_rng_taus);

  for (int32_t n = 1; n < 10; ++n) {
    for (int32_t rpt = 0; rpt < 100; ++rpt) {
      MAT(n, n, U);
      MAT(n, n, Z);
      MAT(n, n, I);
      MAT(n, n, R);

      MatZero(&Z);
      MatI(n, &I);
      MatRandomOrthogonal(rng, &U);

      // Check that 0.5*U*U^T - 0.5*I == 0.0.
      MatGenMult(kNoTrans, kTrans, 0.5, &U, &U, -0.5, &I, &R);
      EXPECT_NEAR_MAT(R, Z, 1e-10);

      // Check that 1.0*U*U^T == I.
      MatResize(n - 1, n - 1, &R);
      MatGenMult(kNoTrans, kTrans, 1.0, &U, &U, 0.0, &I, &R);
      EXPECT_NEAR_MAT(R, I, 1e-10);
      MatResize(n - 1, n - 1, &R);
      MatGenMult(kNoTrans, kTrans, 1.0, &U, &U, 0.0, NULL, &R);
      EXPECT_NEAR_MAT(R, I, 1e-10);

      // Check that 2.0*U^T*U - 2.0*I == 0.0
      MatGenMult(kTrans, kNoTrans, 2.0, &U, &U, -2.0, &I, &R);
      EXPECT_NEAR_MAT(R, Z, 1e-10);

      // Check that U^T*U == I.
      MatGenMult(kTrans, kNoTrans, 1.0, &U, &U, 0.0, &I, &R);
      EXPECT_NEAR_MAT(R, I, 1e-10);

      // Lastly, check that 0.25*U*U - 0.5*(0.5*U*U) == 0.
      MatGenMult(kNoTrans, kNoTrans, 0.5, &U, &U, 0.0, &R, &R);
      MatGenMult(kNoTrans, kNoTrans, 0.25, &U, &U, -0.5, &R, &R);
      EXPECT_NEAR_MAT(R, Z, 1e-10);
    }
  }
  gsl_rng_free(rng);
}

TEST(MatGenMult, Trivial) {
  gsl_rng *rng = gsl_rng_alloc(gsl_rng_taus);
  for (int32_t r = 1; r < 10; ++r) {
    for (int32_t c = 1; c < 10; ++c) {
      MAT(r, c, A);
      MatRandomGaussian(rng, &A);
      MAT(r, c, AT);
      MatTrans(&A, &AT);
      MAT(r, r, I);
      MAT(r, c, R);

      MatI(r, &I);
      MatGenMult(kNoTrans, kNoTrans, 1.0, &I, &A, 0.0, NULL, &R);
      EXPECT_NEAR_MAT(R, A, 0.0);
      MatGenMult(kNoTrans, kTrans, 1.0, &I, &AT, 0.0, NULL, &R);
      EXPECT_NEAR_MAT(R, A, 0.0);

      MatI(c, &I);
      MatGenMult(kNoTrans, kNoTrans, 1.0, &A, &I, 0.0, NULL, &R);
      EXPECT_NEAR_MAT(R, A, 0.0);
      MatGenMult(kTrans, kNoTrans, 1.0, &AT, &I, 0.0, NULL, &R);
      EXPECT_NEAR_MAT(R, A, 0.0);

      MAT(c, c, Z);
      MatZero(&Z);
      MatGenMult(kTrans, kTrans, 1.0, &A, &AT, 0.0, NULL, &R);
      MatGenMult(kNoTrans, kNoTrans, 1.0, &AT, &A, -1.0, &R, &R);
      EXPECT_NEAR_MAT(R, Z, 1e-10);
    }
  }
  gsl_rng_free(rng);
}

TEST(MatArrGemm, A_NoTrans_B_NoTrans) {
  double a[4][2] = {{1.0, 2.0}, {3.0, 4.0}, {5.0, 6.0}, {7.0, 8.0}};
  double b[2][3] = {{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}};
  double c[4][3] = {
      {1.0, 1.0, 1.0}, {1.0, 1.0, 1.0}, {1.0, 1.0, 1.0}, {1.0, 1.0, 1.0}};
  double alpha = 2.0, beta = 0.5;
  double c_ans[4][3] = {{18.5, 24.5, 30.5},
                        {38.5, 52.5, 66.5},
                        {58.5, 80.5, 102.5},
                        {78.5, 108.5, 138.5}};
  MatArrGemm(kNoTrans, kNoTrans, alpha, &a[0][0], 4, 2, &b[0][0], 3, beta,
             &c[0][0]);
  for (int32_t i = 0; i < 4; ++i) {
    for (int32_t j = 0; j < 3; ++j) {
      EXPECT_NEAR(c[i][j], c_ans[i][j], 1e-6);
    }
  }
}

TEST(MatArrGemm, A_Trans_B_NoTrans) {
  double a[2][4] = {{1.0, 3.0, 5.0, 7.0}, {2.0, 4.0, 6.0, 8.0}};
  double b[2][3] = {{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}};
  double c[4][3] = {
      {1.0, 1.0, 1.0}, {1.0, 1.0, 1.0}, {1.0, 1.0, 1.0}, {1.0, 1.0, 1.0}};
  double alpha = 2.0, beta = 0.5;
  double c_ans[4][3] = {{18.5, 24.5, 30.5},
                        {38.5, 52.5, 66.5},
                        {58.5, 80.5, 102.5},
                        {78.5, 108.5, 138.5}};
  MatArrGemm(kTrans, kNoTrans, alpha, &a[0][0], 2, 4, &b[0][0], 3, beta,
             &c[0][0]);
  for (int32_t i = 0; i < 4; ++i)
    for (int32_t j = 0; j < 3; ++j) EXPECT_NEAR(c[i][j], c_ans[i][j], 1e-6);
}

TEST(MatArrGemm, A_NoTrans_B_Trans) {
  double a[4][2] = {{1.0, 2.0}, {3.0, 4.0}, {5.0, 6.0}, {7.0, 8.0}};
  double b[3][2] = {{1.0, 4.0}, {2.0, 5.0}, {3.0, 6.0}};
  double c[4][3] = {
      {1.0, 1.0, 1.0}, {1.0, 1.0, 1.0}, {1.0, 1.0, 1.0}, {1.0, 1.0, 1.0}};
  double alpha = 2.0, beta = 0.5;
  double c_ans[4][3] = {{18.5, 24.5, 30.5},
                        {38.5, 52.5, 66.5},
                        {58.5, 80.5, 102.5},
                        {78.5, 108.5, 138.5}};
  MatArrGemm(kNoTrans, kTrans, alpha, &a[0][0], 4, 2, &b[0][0], 3, beta,
             &c[0][0]);
  for (int32_t i = 0; i < 4; ++i)
    for (int32_t j = 0; j < 3; ++j) EXPECT_NEAR(c[i][j], c_ans[i][j], 1e-6);
}

TEST(MatArrGemm, A_Trans_B_Trans) {
  double a[2][4] = {{1.0, 3.0, 5.0, 7.0}, {2.0, 4.0, 6.0, 8.0}};
  double b[3][2] = {{1.0, 4.0}, {2.0, 5.0}, {3.0, 6.0}};
  double c[4][3] = {
      {1.0, 1.0, 1.0}, {1.0, 1.0, 1.0}, {1.0, 1.0, 1.0}, {1.0, 1.0, 1.0}};
  double alpha = 2.0, beta = 0.5;
  double c_ans[4][3] = {{18.5, 24.5, 30.5},
                        {38.5, 52.5, 66.5},
                        {58.5, 80.5, 102.5},
                        {78.5, 108.5, 138.5}};
  MatArrGemm(kTrans, kTrans, alpha, &a[0][0], 2, 4, &b[0][0], 3, beta,
             &c[0][0]);
  for (int32_t i = 0; i < 4; ++i)
    for (int32_t j = 0; j < 3; ++j) EXPECT_NEAR(c[i][j], c_ans[i][j], 1e-6);
}

TEST(MatArrIsUpperTriangular, Normal) {
  MAT_INIT(
      4, 3, a,
      {{1.0, 1.0, 1.0}, {0.0, 1.0, 1.0}, {0.0, 0.0, 1.0}, {0.0, 0.0, 1.0}});
  MAT_INIT(
      4, 3, b,
      {{1.0, 1.0, 1.0}, {0.0, 1.0, 1.0}, {0.0, 0.0, 1.0}, {0.0, 0.0, 0.0}});
  EXPECT_EQ(MatArrIsUpperTriangular(a.d, 4, 3), 0);
  EXPECT_EQ(MatArrIsUpperTriangular(b.d, 4, 3), 1);
}

TEST(MatMult3, TransposeCheck) {
  gsl_rng *rng = gsl_rng_alloc(gsl_rng_taus);
  MAT(3, 3, A);
  MAT(3, 3, B);
  MAT(3, 3, C);
  MAT(3, 3, At);
  MAT(3, 3, Bt);
  MAT(3, 3, Ct);
  MAT(3, 3, R1);
  MAT(3, 3, R2);
  MAT(3, 3, ABC);

  int32_t n0, n1, n2, n3;
  for (int32_t i = 0; i < 222; ++i) {
    n0 = (int32_t)Rand(1.0, 16.0);
    n1 = (int32_t)Rand(1.0, 16.0);
    n2 = (int32_t)Rand(1.0, 16.0);
    n3 = (int32_t)Rand(1.0, 16.0);

    MatResize(n0, n1, &A);
    MatResize(n1, n2, &B);
    MatResize(n2, n3, &C);

    MatRandomGaussian(rng, &A);
    MatRandomGaussian(rng, &B);
    MatRandomGaussian(rng, &C);

    // Check that (A*B*C)' == C'*B'*A'.
    MatTrans(MatMult3(&A, &B, &C, &ABC), &R1);
    MatMult3(MatTrans(&C, &Ct), MatTrans(&B, &Bt), MatTrans(&A, &At), &R2);
    EXPECT_NEAR_MAT(R1, R2, 1e-9);
  }
  gsl_rng_free(rng);
}

TEST(MatScale, CheckValues) {
  const int32_t nr = 4;
  const int32_t nc = 3;
  MAT(nr, nc, A);
  MAT(nr, nc, B);
  MAT(nr, nc, C);

  for (int32_t i = 0; i < nr; ++i) {
    for (int32_t j = 0; j < nc; ++j) {
      *MatPtr(&A, i, j) = Rand(1.0, 16.0);
      *MatPtr(&B, i, j) = MatGet(&A, i, j);
      *MatPtr(&C, i, j) = 0.0;
    }
  }

  double scale = 200.0 / 9.;
  MatScale(&A, scale, &C);  // Output into a different matrix.
  MatScale(&B, scale, &B);
  for (int32_t i = 0; i < nr; ++i) {
    for (int32_t j = 0; j < nc; ++j) {
      EXPECT_EQ(MatGet(&C, i, j), scale * MatGet(&A, i, j));
      EXPECT_EQ(MatGet(&B, i, j), scale * MatGet(&A, i, j));
    }
  }
}

TEST(MatAdd, TransposeCheck) {
  gsl_rng *rng = gsl_rng_alloc(gsl_rng_taus);
  MAT(3, 3, A);
  MAT(3, 3, B);
  MAT(3, 3, At);
  MAT(3, 3, Bt);
  MAT(3, 3, R1);
  MAT(3, 3, R2);
  MAT(3, 3, AB);

  int32_t n0, n1;
  for (int32_t i = 0; i < 222; ++i) {
    n0 = (int32_t)Rand(1.0, 16.0);
    n1 = (int32_t)Rand(1.0, 16.0);

    MatResize(n0, n1, &A);
    MatResize(n0, n1, &B);
    MatRandomGaussian(rng, &A);
    MatRandomGaussian(rng, &B);

    // Check that (A + B)' == A + B.
    MatTrans(MatAdd(&A, &B, &AB), &R1);
    MatAdd(MatTrans(&A, &At), MatTrans(&B, &Bt), &R2);
    EXPECT_NEAR_MAT(R1, R2, 1e-9);
  }
  gsl_rng_free(rng);
}

TEST(MatSub, TransposeCheck) {
  gsl_rng *rng = gsl_rng_alloc(gsl_rng_taus);
  MAT(3, 3, A);
  MAT(3, 3, B);
  MAT(3, 3, At);
  MAT(3, 3, Bt);
  MAT(3, 3, R1);
  MAT(3, 3, R2);
  MAT(3, 3, AB);

  int32_t n0, n1;
  for (int32_t i = 0; i < 222; ++i) {
    n0 = (int32_t)Rand(1.0, 16.0);
    n1 = (int32_t)Rand(1.0, 16.0);

    MatResize(n0, n1, &A);
    MatResize(n0, n1, &B);
    MatRandomGaussian(rng, &A);
    MatRandomGaussian(rng, &B);

    // Check that (A + B)' == A + B.
    MatTrans(MatSub(&A, &B, &AB), &R1);
    MatSub(MatTrans(&A, &At), MatTrans(&B, &Bt), &R2);
    EXPECT_NEAR_MAT(R1, R2, 1e-9);
  }
  gsl_rng_free(rng);
}

TEST(MatArrIsLowerTriangular, Normal) {
  MAT_INIT(
      4, 3, a,
      {{1.0, 0.0, 0.0}, {1.0, 1.0, 0.0}, {0.0, 1.0, 1.0}, {1.0, 0.0, 1.0}});
  MAT_INIT(
      4, 3, b,
      {{1.0, 0.0, 1.0}, {1.0, 1.0, 0.0}, {0.0, 1.0, 1.0}, {1.0, 0.0, 1.0}});
  EXPECT_EQ(MatArrIsLowerTriangular(a.d, 4, 3), 1);
  EXPECT_EQ(MatArrIsLowerTriangular(b.d, 4, 3), 0);
}

TEST(MatArrQrDecomp, Normal_M_GT_N) {
  double a[8][4] = {{-2.6, 1.8, -4.0, 0.0}, {-0.4, -1.0, -2.3, 2.8},
                    {4.7, -1.3, -1.6, 2.2}, {0.5, 4.9, 1.8, 4.1},
                    {0.3, -4.6, -3.6, 4.0}, {-2.6, 3.9, 2.3, -1.6},
                    {-0.1, 4.2, -3.9, 2.0}, {1.3, 3.0, 1.6, -3.0}};
  double q[8][8], r[8][4];
  double q_ans[8][8] = {
      {-0.422834104872492, 0.075519470878410, -0.543389571827535,
       0.219139132806951, -0.067332813788829, -0.167869226622365,
       -0.660495045967202, 0.056977646229904},
      {-0.065051400749614, -0.125918960836193, -0.265166890809663,
       -0.279775111115596, -0.648629008271706, 0.271602415827528,
       0.196770804804213, 0.546002679025345},
      {0.764353958807967, 0.073932735862217, -0.231855708303419,
       0.045539787762916, -0.123653310935304, 0.398599902165980,
       -0.380048365319518, -0.189360109806989},
      {0.081314250937018, 0.550540076291613, 0.080137466405926,
       -0.735960752691464, 0.065698652564982, -0.277622594465772,
       -0.227033870823766, 0.096428095279573},
      {0.048788550562211, -0.481809096514797, -0.335199537624324,
       -0.353967727414491, 0.659599045648870, 0.157419263988662,
       -0.012779364756213, 0.260408816893533},
      {-0.422834104872492, 0.301710399102350, 0.216793139524717,
       -0.055804996278761, 0.164394587293727, 0.797153748180358,
       -0.116511350700185, -0.063357071145004},
      {-0.016262850187403, 0.447829607694062, -0.636131606040983,
       0.058344814948921, 0.133287623077363, 0.034538973427684,
       0.561348465814765, -0.238843030706401},
      {0.211417052436246, 0.382309131262397, 0.101063320150087,
       0.445174526235633, 0.274368685792741, -0.025845694168661,
       0.014003537698797, 0.724296990111724}};
  double r_ans[8][4] = {
      {6.148983655857283, -2.598803459947087, 0.017889135206144,
       2.037735128481665},
      {0.0, 9.284191972194835, 2.153821291756628, -0.593947961670865},
      {0.0, 0.0, 7.646615841142891, -4.187106586539354},
      {0.0, 0.0, 0.0, -6.246038728501256},
      {0.0, 0.0, 0.0, 0.0},
      {0.0, 0.0, 0.0, 0.0},
      {0.0, 0.0, 0.0, 0.0},
      {0.0, 0.0, 0.0, 0.0}};

  MatArrQrDecomp(&a[0][0], 8, 4, &q[0][0], &r[0][0]);
  for (int32_t i = 0; i < 8; ++i) {
    for (int32_t j = 0; j < 4; ++j) {
      EXPECT_NEAR(r[i][j], r_ans[i][j], 1e-6);
    }
  }

  for (int32_t i = 0; i < 8; ++i) {
    for (int32_t j = 0; j < 8; ++j) {
      EXPECT_NEAR(q[i][j], q_ans[i][j], 1e-6);
    }
  }
}

TEST(MatArrQrDecomp, Normal_M_LT_N) {
  double a[2][3] = {{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}};
  double q[2][2], qt[2][2], r[2][3], I[2][2], a_out[2][3];
  MatArrQrDecomp(&a[0][0], 2, 3, &q[0][0], &r[0][0]);
  MatArrTrans(&q[0][0], 2, 2, &qt[0][0]);
  MatArrMult(&q[0][0], 2, 2, &qt[0][0], 2, &I[0][0]);
  MatArrMult(&q[0][0], 2, 2, &r[0][0], 3, &a_out[0][0]);

  // Q orthogonal
  for (int32_t i = 0; i < 2; ++i) {
    for (int32_t j = 0; j < 2; ++j) {
      if (i == j) {
        EXPECT_NEAR(I[i][j], 1.0, 1e-6);
      } else {
        EXPECT_NEAR(I[i][j], 0.0, 1e-6);
      }
    }
  }

  // QR = A
  for (int32_t i = 0; i < 2; ++i) {
    for (int32_t j = 0; j < 3; ++j) {
      EXPECT_NEAR(a[i][j], a_out[i][j], 1e-6);
    }
  }
}

TEST(MatArrQrDecomp, Rand66) {
  double a[6][6], q[6][6], qt[6][6], r[6][6], I[6][6], a_out[6][6];
  for (int32_t k = 0; k < 22; ++k) {
    for (int32_t i = 0; i < 6; ++i)
      for (int32_t j = 0; j < 6; ++j) a[i][j] = Rand(-1e9, 1e9);
    MatArrQrDecomp(&a[0][0], 6, 6, &q[0][0], &r[0][0]);
    MatArrTrans(&q[0][0], 6, 6, &qt[0][0]);
    MatArrMult(&q[0][0], 6, 6, &qt[0][0], 6, &I[0][0]);
    MatArrMult(&q[0][0], 6, 6, &r[0][0], 6, &a_out[0][0]);

    // Q orthogonal
    for (int32_t i = 0; i < 6; ++i) {
      for (int32_t j = 0; j < 6; ++j) {
        if (i == j)
          EXPECT_NEAR(I[i][j], 1.0, 1e-5);
        else
          EXPECT_NEAR(I[i][j], 0.0, 1e-5);
      }
    }

    // QR = A
    for (int32_t i = 0; i < 6; ++i)
      for (int32_t j = 0; j < 6; ++j) EXPECT_NEAR(a[i][j], a_out[i][j], 1e-5);
  }
}

TEST(MatArrQrDecomp, Q_NULL_Reuse_A) {
  double a[8][4] = {{-2.6, 1.8, -4.0, 0.0}, {-0.4, -1.0, -2.3, 2.8},
                    {4.7, -1.3, -1.6, 2.2}, {0.5, 4.9, 1.8, 4.1},
                    {0.3, -4.6, -3.6, 4.0}, {-2.6, 3.9, 2.3, -1.6},
                    {-0.1, 4.2, -3.9, 2.0}, {1.3, 3.0, 1.6, -3.0}};
  double r_ans[8][4] = {
      {6.148983655857283, -2.598803459947087, 0.017889135206144,
       2.037735128481665},
      {0.0, 9.284191972194835, 2.153821291756628, -0.593947961670865},
      {0.0, 0.0, 7.646615841142891, -4.187106586539354},
      {0.0, 0.0, 0.0, -6.246038728501256},
      {0.0, 0.0, 0.0, 0.0},
      {0.0, 0.0, 0.0, 0.0},
      {0.0, 0.0, 0.0, 0.0},
      {0.0, 0.0, 0.0, 0.0}};

  MatArrQrDecomp(&a[0][0], 8, 4, NULL, &a[0][0]);
  for (int32_t i = 0; i < 8; ++i)
    for (int32_t j = 0; j < 4; ++j) EXPECT_NEAR(a[i][j], r_ans[i][j], 1e-6);
}

TEST(MatArrQrDecomp, RankDeficient) {
  double q[4][4], r[4][10];
  double a[4][10] = {
      {-0.62, -0.76, -0.18, 0.20, 0.84, -1.15, -0.67, -0.44, 0.10, 0.49},
      {0.75, -1.40, -0.20, 1.59, -0.24, 0.10, 0.19, -1.79, -0.54, 0.74},
      {-0.62, -0.76, -0.18, 0.20, 0.84, -1.15, -0.67, -0.44, 0.10, 0.49},
      {0.89, 0.49, 0.29, 0.70, -1.17, 2.59, -1.93, -0.89, -0.60, -0.19}};
  double r_ans_gsl[4][10], q_ans_gsl[4][4];

  MatArrQrDecomp(&a[0][0], 4, 10, &q[0][0], &r[0][0]);

  gsl_matrix *gsl_ar = gsl_matrix_alloc(4, 10);
  gsl_vector *gsl_tau = gsl_vector_alloc(4);
  gsl_matrix *gsl_q = gsl_matrix_alloc(4, 4);
  gsl_matrix *gsl_r = gsl_matrix_alloc(4, 10);
  for (int32_t i = 0; i < 4; ++i) {
    for (int32_t j = 0; j < 10; ++j) {
      gsl_matrix_set(gsl_ar, i, j, a[i][j]);
    }
  }
  gsl_linalg_QR_decomp(gsl_ar, gsl_tau);
  gsl_linalg_QR_unpack(gsl_ar, gsl_tau, gsl_q, gsl_r);
  for (int32_t i = 0; i < 4; ++i) {
    for (int32_t j = 0; j < 10; ++j) {
      r_ans_gsl[i][j] = gsl_matrix_get(gsl_r, i, j);
      if (j < 4) q_ans_gsl[i][j] = gsl_matrix_get(gsl_q, i, j);
    }
  }
  gsl_matrix_free(gsl_ar);
  gsl_vector_free(gsl_tau);
  gsl_matrix_free(gsl_q);
  gsl_matrix_free(gsl_r);

  for (int32_t i = 0; i < 4; ++i) {
    for (int32_t j = 0; j < 10; ++j) {
      EXPECT_NEAR(r[i][j], r_ans_gsl[i][j], 1e-9);
    }
  }

  for (int32_t i = 0; i < 4; ++i) {
    for (int32_t j = 0; j < 4; ++j) {
      EXPECT_NEAR(q[i][j], q_ans_gsl[i][j], 1e-2);
    }
  }
}

TEST(MatArrQrDecomp, CompareToGSL_M_LT_N) {
  double a[4][10], q[4][4], r[4][10];
  gsl_matrix *gsl_ar = gsl_matrix_alloc(4, 10);
  gsl_vector *gsl_tau = gsl_vector_alloc(4);
  gsl_matrix *gsl_q = gsl_matrix_alloc(4, 4);
  gsl_matrix *gsl_r = gsl_matrix_alloc(4, 10);

  for (int32_t k = 0; k < 2222; ++k) {
    for (int32_t i = 0; i < 4; ++i) {
      int32_t zero_row = Rand() < 0.05;
      for (int32_t j = 0; j < 10; ++j) {
        double val = Rand(-100.0, 100.0);
        if (zero_row) val = 0.0;
        // Set zero column.
        if ((i == 0 && Rand() < 0.05) ||
            (i > 0 && gsl_matrix_get(gsl_ar, i - 1, j) == 0.0)) {
          val = 0.0;
        }
        gsl_matrix_set(gsl_ar, i, j, val);
        a[i][j] = val;
      }
    }
    MatArrQrDecomp(&a[0][0], 4, 10, &q[0][0], &r[0][0]);
    gsl_linalg_QR_decomp(gsl_ar, gsl_tau);
    gsl_linalg_QR_unpack(gsl_ar, gsl_tau, gsl_q, gsl_r);

    // We only check magnitude here because the QR decomposition isn't
    // unique, and there is a sign difference between this
    // implementation and GSL's depending on how many Householder
    // transformations have been applied.  This difference only seems
    // to appear for all zero rows.
    for (int32_t i = 0; i < 4; ++i) {
      for (int32_t j = 0; j < 10; ++j) {
        EXPECT_NEAR(fabs(gsl_matrix_get(gsl_r, i, j)), fabs(r[i][j]), 1e-9);
        if (j < 4) {
          EXPECT_NEAR(fabs(gsl_matrix_get(gsl_q, i, j)), fabs(q[i][j]), 1e-9);
        }
      }
    }
  }

  gsl_matrix_free(gsl_ar);
  gsl_vector_free(gsl_tau);
  gsl_matrix_free(gsl_q);
  gsl_matrix_free(gsl_r);
}

TEST(MatArrQrDecomp, CompareToGSL_M_GT_N) {
  double a[9][6], q[9][9], r[9][6];
  gsl_matrix *gsl_ar = gsl_matrix_alloc(9, 6);
  gsl_vector *gsl_tau = gsl_vector_alloc(6);
  gsl_matrix *gsl_q = gsl_matrix_alloc(9, 9);
  gsl_matrix *gsl_r = gsl_matrix_alloc(9, 6);

  for (int32_t k = 0; k < 2222; ++k) {
    for (int32_t i = 0; i < 9; ++i) {
      int32_t zero_row = Rand() < 0.05;
      for (int32_t j = 0; j < 6; ++j) {
        double val = Rand(-100.0, 100.0);
        if (zero_row) val = 0.0;
        // Set zero column.
        if ((i == 0 && Rand() < 0.05) ||
            (i > 0 && gsl_matrix_get(gsl_ar, i - 1, j) == 0.0)) {
          val = 0.0;
        }
        gsl_matrix_set(gsl_ar, i, j, val);
        a[i][j] = val;
      }
    }
    MatArrQrDecomp(&a[0][0], 9, 6, &q[0][0], &r[0][0]);
    gsl_linalg_QR_decomp(gsl_ar, gsl_tau);
    gsl_linalg_QR_unpack(gsl_ar, gsl_tau, gsl_q, gsl_r);

    // We only check magnitude here because the QR decomposition isn't
    // unique, and there is a sign difference between this
    // implementation and GSL's depending on how many Householder
    // transformations have been applied.  This difference only seems
    // to appear for all zero rows.
    for (int32_t i = 0; i < 9; ++i) {
      for (int32_t j = 0; j < 6; ++j) {
        EXPECT_NEAR(fabs(gsl_matrix_get(gsl_r, i, j)), fabs(r[i][j]), 1e-9);
        if (j < 9) {
          EXPECT_NEAR(fabs(gsl_matrix_get(gsl_q, i, j)), fabs(q[i][j]), 1e-9);
        }
      }
    }
  }

  gsl_matrix_free(gsl_ar);
  gsl_vector_free(gsl_tau);
  gsl_matrix_free(gsl_q);
  gsl_matrix_free(gsl_r);
}

TEST(MatArrQrDecomp, ZeroColumn) {
  double a[4][10] = {
      {-0.62, 0.00, -0.18, 0.20, 0.84, -1.15, -0.67, -0.44, 0.10, 0.49},
      {0.75, 0.00, -0.20, 1.59, -0.24, 0.10, 0.19, -1.79, -0.54, 0.74},
      {-0.62, 0.00, -0.18, 0.20, 0.84, -1.15, -0.67, -0.44, 0.10, 0.49},
      {0.89, 0.00, 0.29, 0.70, -1.17, 2.59, -1.93, -0.89, -0.60, -0.19}};

  // This is only accurate to 0.01 to keep it pretty.
  double r_ans[4][10] = {
      {1.46, 0.00, 0.23, 1.08, -1.55, 2.61, -0.51, -1.09, -0.73, -0.15},
      {0.00, 0.00, -0.35, 1.27, 0.62, -1.26, 0.13, -1.56, -0.24, 0.97},
      {0.00, 0.00, 0.13, 0.08, -0.19, 0.89, -1.50, -0.26, -0.15, -0.06},
      {0.00, 0.00, 0.00, 0.56, 0.05, 0.42, -1.46, -0.84, -0.24, 0.30}};

  MatArrQrDecomp(&a[0][0], 4, 10, NULL, &a[0][0]);
  for (int32_t i = 0; i < 4; ++i)
    for (int32_t j = 0; j < 10; ++j) EXPECT_NEAR(a[i][j], r_ans[i][j], 1e-2);
}

TEST(MatArrQrDecomp, Zero) {
  double a[4][10], q[4][4], q_ans[4][4], r_ans[4][10];
  MatArrZero(4U, 10U, &a[0][0]);
  MatArrI(4U, &q_ans[0][0]);
  MatArrZero(4U, 10U, &r_ans[0][0]);

  MatArrQrDecomp(&a[0][0], 4, 10, &q[0][0], &a[0][0]);
  for (int32_t i = 0; i < 4; ++i)
    for (int32_t j = 0; j < 10; ++j) EXPECT_NEAR(a[i][j], r_ans[i][j], 1e-6);

  for (int32_t i = 0; i < 4; ++i)
    for (int32_t j = 0; j < 4; ++j) EXPECT_NEAR(q[i][j], q_ans[i][j], 1e-6);
}

TEST(MatInit, Normal) {
  double d[2][3] = {{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}};
  MAT(2, 3, A);
  MatInit(&d[0][0], 2U, 3U, &A);
  for (int32_t i = 0; i < 2; ++i) {
    for (int32_t j = 0; j < 3; ++j) {
      EXPECT_NEAR(MatGet(&A, i, j), d[i][j], 1e-9);
    }
  }
}

TEST(MatSubmatSet, Sizes) {
  gsl_rng *rng = gsl_rng_alloc(gsl_rng_taus);
  int32_t n = 10;
  int32_t start_rows[9] = {0, 0, 0, 0, 0, 5, 5, 5, 5};
  int32_t start_cols[9] = {0, 0, 0, 5, 5, 0, 0, 5, 5};
  int32_t destn_rows[9] = {0, 0, 5, 0, 5, 0, 5, 0, 5};
  int32_t destn_cols[9] = {0, 0, 4, 0, 4, 0, 4, 0, 4};
  int32_t num_rows[9] = {8, 5, 5, 5, 5, 5, 5, 5, 5};
  int32_t num_cols[9] = {8, 5, 5, 5, 5, 5, 5, 5, 5};

  for (int32_t idx = 0; idx < 9; ++idx) {
    MAT(n, n, S);
    MAT(n, n, A);
    MAT(n, n, B);
    MatRandomGaussian(rng, &S);
    MatRandomGaussian(rng, &A);
    MatCopy(&A, &B);
    MatSubmatSet(&S, start_rows[idx], start_cols[idx], num_rows[idx],
                 num_cols[idx], destn_rows[idx], destn_cols[idx], &A);
    for (int32_t i = 0; i < A.nr; ++i) {
      for (int32_t j = 0; j < A.nc; ++j) {
        if (i >= destn_rows[idx] && i < destn_rows[idx] + num_rows[idx] &&
            j >= destn_cols[idx] && j < destn_cols[idx] + num_cols[idx]) {
          int32_t Si = i - destn_rows[idx] + start_rows[idx];
          int32_t Sj = j - destn_cols[idx] + start_cols[idx];
          EXPECT_EQ(MatGet(&A, i, j), MatGet(&S, Si, Sj));
        } else {  // Check values that shouldn't have been over-written.
          EXPECT_EQ(MatGet(&A, i, j), MatGet(&B, i, j));
        }
      }
    }
  }
  gsl_rng_free(rng);
}

TEST(MatI, Normal) {
  MAT(0, 0, A);
  int32_t n = 1;
  gsl_rng *rng = gsl_rng_alloc(gsl_rng_taus);
  for (int32_t exp = 0; exp < 4; ++exp) {
    n *= 2;
    MatRandomGaussian(rng, &A);
    MatI(n, &A);
    for (int32_t i = 0; i < n; ++i) {
      for (int32_t j = 0; j < n; ++j) {
        EXPECT_EQ(MatGet(&A, i, j), i == j ? 1.0 : 0.0);
      }
    }
  }
  gsl_rng_free(rng);
}

TEST(MatThinSvDecomp, CheckSingularValues_M_GT_N) {
  MAT_INIT(5, 4, A, {{0.49, -0.84, 1.44, 0.83},
                     {0.74, 1.35, -1.96, 1.38},
                     {1.71, -1.07, -0.20, -1.06},
                     {-0.19, 0.96, -1.21, -0.47},
                     {-2.14, 0.12, 2.91, -0.27}});
  MAT_INIT(5, 4, U, {{0}});
  VEC_INIT(4, s, {0});
  MAT_INIT(4, 4, V, {{0}});
  VEC_INIT(4, s_ans, {4.571598749, 2.858861081, 1.880174516, 0.476384268});

  MatThinSvDecomp(&A, &U, &s, &V);
  EXPECT_NEAR_VEC(s, s_ans, 1e-9);
}

TEST(MatThinSvDecomp, MultiplyTogether) {
  for (int32_t k = 0; k < 222; ++k) {
    MAT_INIT(5, 4, A,
             {{RandNormal(), RandNormal(), RandNormal(), RandNormal()},
              {RandNormal(), RandNormal(), RandNormal(), RandNormal()},
              {RandNormal(), RandNormal(), RandNormal(), RandNormal()},
              {RandNormal(), RandNormal(), RandNormal(), RandNormal()},
              {RandNormal(), RandNormal(), RandNormal(), RandNormal()}});
    MAT_INIT(5, 4, U, {{0}});
    VEC_INIT(4, s, {0});
    MAT_INIT(4, 4, V, {{0}});
    MAT_INIT(4, 4, Vt, {{0}});
    MAT_INIT(4, 4, S, {{0}});
    MAT_INIT(4, 4, SVt, {{0}});
    MAT_INIT(5, 4, A_out, {{0}});

    MatThinSvDecomp(&A, &U, &s, &V);

    for (int32_t i = 0; i < s.length; ++i) {
      *MatPtr(&S, i, i) = VecGet(&s, i);
    }
    MatTrans(&V, &Vt);
    MatMult(&S, &Vt, &SVt);
    MatMult(&U, &SVt, &A_out);
    EXPECT_NEAR_MAT(A, A_out, 1e-9);
  }
}

TEST(MatRank, FullRank_M_LT_N) {
  MAT_INIT(4, 5, A, {{0.19, 0.71, 0.66, 0.96, 0.75},
                     {0.49, 0.75, 0.16, 0.34, 0.26},
                     {0.45, 0.28, 0.12, 0.59, 0.51},
                     {0.65, 0.68, 0.50, 0.22, 0.70}});
  EXPECT_EQ(MatRank(&A, 10.0 * DBL_EPSILON), 4U);
}

TEST(MatRank, FullRankSquare) {
  MAT_INIT(4, 4, A, {{0.89, 0.15, 0.81, 0.20},
                     {0.96, 0.26, 0.24, 0.25},
                     {0.55, 0.84, 0.93, 0.62},
                     {0.14, 0.25, 0.35, 0.47}});
  EXPECT_EQ(MatRank(&A, 10.0 * DBL_EPSILON), 4U);
}

TEST(MatRank, FullRank_M_GT_N) {
  MAT_INIT(5, 4, A, {{0.35, 0.29, 0.08, 0.13},
                     {0.83, 0.76, 0.05, 0.57},
                     {0.59, 0.75, 0.53, 0.47},
                     {0.55, 0.38, 0.78, 0.01},
                     {0.92, 0.57, 0.93, 0.34}});
  EXPECT_EQ(MatRank(&A, 10.0 * DBL_EPSILON), 4U);
}

TEST(MatRank, PartialRank_M_LT_N) {
  MAT_INIT(4, 5, A, {{1.0, 2.0, 3.0, 4.0, 5.0},
                     {2.0, 3.0, 4.0, 5.0, 6.0},
                     {3.0, 4.0, 5.0, 6.0, 7.0},
                     {4.0, 5.0, 6.0, 7.0, 8.0}});
  EXPECT_EQ(MatRank(&A, 10.0 * DBL_EPSILON), 2U);
}

TEST(MatRank, PartialRankSquare) {
  MAT_INIT(4, 4, A, {{1.0, 2.0, 3.0, 4.0},
                     {2.0, 3.0, 4.0, 5.0},
                     {3.0, 4.0, 5.0, 6.0},
                     {4.0, 5.0, 6.0, 7.0}});
  EXPECT_EQ(MatRank(&A, 10.0 * DBL_EPSILON), 2U);
}

TEST(MatRank, PartialRank_M_GT_N) {
  MAT_INIT(5, 4, A, {{1.0, 2.0, 3.0, 4.0},
                     {2.0, 3.0, 4.0, 5.0},
                     {3.0, 4.0, 5.0, 6.0},
                     {4.0, 5.0, 6.0, 7.0},
                     {5.0, 6.0, 7.0, 8.0}});
  EXPECT_EQ(MatRank(&A, 10.0 * DBL_EPSILON), 2U);
}

TEST(MatRank, RandomWithRank) {
  gsl_rng *rng = gsl_rng_alloc(gsl_rng_taus);
  for (int32_t nr = 0; nr < 5; ++nr) {
    for (int32_t nc = 0; nc < 5; ++nc) {
      for (int32_t rank = 0; rank <= nc && rank <= nr; ++rank) {
        MAT(nr, nc, A);
        MatRandomWithRank(rank, rng, &A);
        EXPECT_EQ(MatRank(&A, 10.0 * DBL_EPSILON), rank);
      }
    }
  }
  gsl_rng_free(rng);
}

// The MATLAB convention for zero rank matrices (i.e. all zeros) is to
// return a vector of zeros.
TEST(MatVecLeftDivide, ZeroRank) {
  MAT_INIT(2, 3, A, {{0, 0, 0}, {0, 0, 0}});
  VEC_INIT(2, b, {1.0, 1.0});
  VEC_INIT(3, x, {1.0, 1.0, 1.0});
  MatVecLeftDivide(&A, &b, &x);
  for (int32_t i = 0; i < x.length; ++i) {
    EXPECT_NEAR(VecGet(&x, i), 0.0, 1e-9);
  }
}

// A \ b solves the least squares problem A*x = b.  So |A*x - b|
// should be a minimum for the correct solution x.
TEST(MatVecLeftDivide, Normal_M_GT_N) {
  for (int32_t i = 0; i < 222; ++i) {
    MAT_INIT(3, 2, A, {{Rand(-1e9, 1e9), Rand(-1e9, 1e9)},
                       {Rand(-1e9, 1e9), Rand(-1e9, 1e9)},
                       {Rand(-1e9, 1e9), Rand(-1e9, 1e9)}});
    VEC_INIT(3, b, {Rand(-1e9, 1e9), Rand(-1e9, 1e9), Rand(-1e9, 1e9)});
    VEC_INIT(2, x, {0});
    VEC_INIT(3, tmp, {0});
    MatVecLeftDivide(&A, &b, &x);
    double r_mod, r = VecNorm(VecSub(MatVecMult(&A, &x, &tmp), &b, &tmp));
    VEC_INIT(2, x_mod, {0});
    for (int32_t j = 0; j < x.length; ++j) {
      VecCopy(&x, &x_mod);
      x_mod.d[j] = x.d[j] + 0.01;
      r_mod = VecNorm(VecSub(MatVecMult(&A, &x_mod, &tmp), &b, &tmp));
      EXPECT_GE(r_mod, r);
      x_mod.d[j] = x.d[j] - 0.01;
      r_mod = VecNorm(VecSub(MatVecMult(&A, &x_mod, &tmp), &b, &tmp));
      EXPECT_GE(r_mod, r);
    }
  }
}

#if !defined(NDEBUG)

TEST(MatVecLeftDivideDeath, ReuseInput_M_GT_N) {
  MAT_INIT(3, 2, A, {{Rand(-1e9, 1e9), Rand(-1e9, 1e9)},
                     {Rand(-1e9, 1e9), Rand(-1e9, 1e9)},
                     {Rand(-1e9, 1e9), Rand(-1e9, 1e9)}});
  VEC_INIT(3, b, {Rand(-1e9, 1e9), Rand(-1e9, 1e9), Rand(-1e9, 1e9)});
  b.variable_len = 1;
  EXPECT_DEATH(MatVecLeftDivide(&A, &b, &b), "");
}

#endif  // !defined(NDEBUG)

TEST(MatVecLeftDivide, Normal_M_LT_N) {
  MAT_INIT(2, 3, A, {{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}});
  VEC_INIT(2, b, {1.0, 5.0});
  VEC_INIT(2, b_out, {0});
  VEC_INIT(3, x, {0});
  MatVecLeftDivide(&A, &b, &x);
  MatVecMult(&A, &x, &b_out);
  for (int32_t i = 0; i < 2; ++i) {
    EXPECT_NEAR(b.d[i], b_out.d[i], 1e-6);
  }
}

// b / A solves the least squares problem x*A = b.  So |x*A - b|
// should be a minimum for the correct solution x.
TEST(MatVecRightDivide, Normal_M_LT_N) {
  for (int32_t i = 0; i < 222; ++i) {
    MAT_INIT(2, 3, A, {{Rand(-1e9, 1e9), Rand(-1e9, 1e9), Rand(-1e9, 1e9)},
                       {Rand(-1e9, 1e9), Rand(-1e9, 1e9), Rand(-1e9, 1e9)}});
    VEC_INIT(3, b, {Rand(-1e9, 1e9), Rand(-1e9, 1e9), Rand(-1e9, 1e9)});
    VEC_INIT(2, x, {0});
    VEC_INIT(3, tmp, {0});
    MatVecRightDivide(&A, &b, &x);
    double r_mod, r = VecNorm(VecSub(MatTransVecMult(&A, &x, &tmp), &b, &tmp));
    VEC_INIT(2, x_mod, {0});
    for (int32_t j = 0; j < x.length; ++j) {
      VecCopy(&x, &x_mod);
      x_mod.d[j] = x.d[j] + 0.01;
      r_mod = VecNorm(VecSub(MatTransVecMult(&A, &x_mod, &tmp), &b, &tmp));
      EXPECT_GE(r_mod, r);
      x_mod.d[j] = x.d[j] - 0.01;
      r_mod = VecNorm(VecSub(MatTransVecMult(&A, &x_mod, &tmp), &b, &tmp));
      EXPECT_GE(r_mod, r);
    }
  }
}

TEST(MatVecRightDivide, CompareToMatlab_M_LT_N) {
  MAT_INIT(4, 6, A, {{0.93, 0.62, 0.59, 0.76, 0.08, 0.93},
                     {0.35, 0.47, 0.55, 0.75, 0.05, 0.13},
                     {0.20, 0.35, 0.92, 0.38, 0.53, 0.57},
                     {0.25, 0.83, 0.29, 0.57, 0.78, 0.47}});
  VEC_INIT(6, b, {0.31, 0.53, 0.17, 0.60, 0.26, 0.65});
  VEC_INIT(4, x, {0});
  VEC_INIT(4, x_ans, {0.435423878394150, -0.063587934213125, -0.150461760696058,
                      0.458164473366840});
  MatVecRightDivide(&A, &b, &x);
  for (int32_t i = 0; i < x.length; ++i) {
    EXPECT_NEAR(x.d[i], x_ans.d[i], 1e-6);
  }
}

#if !defined(NDEBUG)

TEST(MatVecRightDivideDeath, ReuseInput_M_LT_N) {
  MAT_INIT(2, 3, A, {{Rand(-1e9, 1e9), Rand(-1e9, 1e9), Rand(-1e9, 1e9)},
                     {Rand(-1e9, 1e9), Rand(-1e9, 1e9), Rand(-1e9, 1e9)}});
  VEC_INIT(3, b, {Rand(-1e9, 1e9), Rand(-1e9, 1e9), Rand(-1e9, 1e9)});
  b.variable_len = 1;
  EXPECT_DEATH(MatVecRightDivide(&A, &b, &b), "");
}

#endif  // !defined(NDEBUG)

TEST(MatVecRightDivide, Normal_M_GE_N) {
  for (int32_t i = 0; i < 222; ++i) {
    MAT_INIT(3, 2, A, {{Rand(-1e9, 1e9), Rand(-1e9, 1e9)},
                       {Rand(-1e9, 1e9), Rand(-1e9, 1e9)},
                       {Rand(-1e9, 1e9), Rand(-1e9, 1e9)}});
    VEC_INIT(2, b, {Rand(-1e9, 1e9), Rand(-1e9, 1e9)});
    VEC_INIT(3, x, {0});
    VEC_INIT(2, b_out, {0});
    MatVecRightDivide(&A, &b, &x);
    MatTransVecMult(&A, &x, &b_out);
    EXPECT_NEAR_VEC(b, b_out, 1e-5);
  }
}

TEST(MatVecRightDivide, CompareToMatlab_M_GE_N) {
  MAT_INIT(6, 4, A, {{0.93, 0.35, 0.20, 0.25},
                     {0.62, 0.47, 0.35, 0.83},
                     {0.59, 0.55, 0.92, 0.29},
                     {0.76, 0.75, 0.38, 0.57},
                     {0.08, 0.05, 0.53, 0.78},
                     {0.93, 0.13, 0.57, 0.47}});
  VEC_INIT(4, b, {0.15, 0.83, 0.54, 1.00});
  VEC_INIT(6, x, {0});

  // This is the minimal 2-norm least squares solution, which MATLAB
  // gives as the output of lsqr.  The "basic solution", which MATLAB
  // gives for the matrix right divide is actually:
  // x_ans = {0.0, 0.0, 0.132620886936791, 1.101668867382283,
  //          0.974163087482296, -0.906933020969704}
  VEC_INIT(6, x_ans,
           {-0.400278191952835, 0.650193499299446, 0.274808416282241,
            0.777597144336397, 0.483900993342686, -0.723316264321402});
  MatVecRightDivide(&A, &b, &x);
  for (int32_t i = 0; i < x.length; ++i) {
    EXPECT_NEAR(x.d[i], x_ans.d[i], 1e-6);
  }
}

#if !defined(NDEBUG)

TEST(MatVecRightDivideDeath, ReuseInput_M_GE_N) {
  MAT_INIT(3, 2, A, {{Rand(-1e9, 1e9), Rand(-1e9, 1e9)},
                     {Rand(-1e9, 1e9), Rand(-1e9, 1e9)},
                     {Rand(-1e9, 1e9), Rand(-1e9, 1e9)}});
  VEC_INIT(3, b, {Rand(-1e9, 1e9), Rand(-1e9, 1e9), 0.0});
  b.variable_len = 1;
  b.length = 2;
  EXPECT_DEATH(MatVecRightDivide(&A, &b, &b), "");
}

#endif  // !defined(NDEBUG)

TEST(MatRightDivide, Recovery) {
  for (int32_t i = 0; i < 222; ++i) {
    MAT_INIT(2, 3, A, {{RandNormal(), RandNormal(), RandNormal()},
                       {RandNormal(), RandNormal(), RandNormal()}});
    VEC_INIT(2, b, {RandNormal(), RandNormal()});
    VEC(3, x);
    MAT(3, 2, At);
    VEC(2, bt);
    VEC(3, xt);
    MatTrans(&A, &At);
    VecCopy(&b, &bt);

    MatVecLeftDivide(&A, &b, &x);
    MatVecRightDivide(&At, &bt, &xt);

    for (int32_t j = 0; j < 2; ++j) {
      EXPECT_NEAR(VecGet(&x, j), VecGet(&xt, j), 1e-9);
    }
  }
}

TEST(MatMatLeftDivide, AgreesWithMatVecLeftDivide) {
  int32_t rows[3] = {4, 10, 4};
  int32_t cols[3] = {4, 4, 10};
  int32_t cols_b[3] = {22, 22, 22};
  gsl_rng *rng = gsl_rng_alloc(gsl_rng_taus);

  for (int32_t idx = 0; idx < 3; ++idx) {
    int32_t nc = cols[idx];
    int32_t nr = rows[idx];
    int32_t nc_b = cols_b[idx];
    for (int32_t rank = 0; rank <= nc && rank <= nr; ++rank) {
      MAT(nr, nc, A);
      MAT(nr, nc_b, B);
      MAT(nc, nc_b, X);
      VEC(nr, b);
      VEC(nc, x);
      VEC(nc, X_column);

      MatRandomWithRank(rank, rng, &A);
      MatRandomGaussian(rng, &B);

      int32_t mm_err = MatMatLeftDivide(&A, &B, &X);
      for (int32_t j = 0; j < B.nc; ++j) {    // Iterate over columns.
        for (int32_t i = 0; i < X.nr; ++i) {  // Fetch out the expected answer.
          *VecPtr(&X_column, i) = MatGet(&X, i, j);
        }
        for (int32_t i = 0; i < B.nr; ++i) {  // Copy a column of B into b.
          *VecPtr(&b, i) = MatGet(&B, i, j);
        }
        int32_t mv_err = MatVecLeftDivide(&A, &b, &x);  // Compute x = A\b.
        EXPECT_EQ(mm_err, mv_err);  // Expect the error codes to match.
        if (mv_err == kLinalgErrorNone) {
          EXPECT_NEAR_VEC(x, X_column, 0.0);
        }
      }
    }
  }
  gsl_rng_free(rng);
}

TEST(MatMatRightDivide, AgreesWithLeftDivide) {
  int32_t rows[3] = {4, 10, 4};
  int32_t cols[3] = {4, 4, 10};
  int32_t cols_b[3] = {22, 22, 22};
  gsl_rng *rng = gsl_rng_alloc(gsl_rng_taus);

  for (int32_t idx = 0; idx < 3; ++idx) {
    int32_t nc = cols[idx];
    int32_t nr = rows[idx];
    int32_t nc_b = cols_b[idx];
    for (int32_t rank = 0; rank <= nc && rank <= nr; ++rank) {
      MAT(nr, nc, A);
      MAT(nr, nc_b, B);
      MAT(nc, nc_b, X);
      MatRandomWithRank(rank, rng, &A);
      MatRandomGaussian(rng, &B);

      MAT(nc, nr, AT);
      MAT(nc_b, nr, BT);
      MAT(nc_b, nc, XT);
      MatTrans(&A, &AT);
      MatTrans(&B, &BT);
      int32_t left_err = MatMatLeftDivide(&A, &B, &X);
      int32_t right_err = MatMatRightDivide(&AT, &BT, &XT);
      EXPECT_EQ(right_err, left_err);

      if (right_err == kLinalgErrorNone) {
        MAT(nc, nc_b, X_compare);
        MatTrans(&XT, &X_compare);
        EXPECT_EQ_MAT(X, X_compare);
      }
    }
  }
  gsl_rng_free(rng);
}

TEST(MatVecRightDivide, AgreesWithLeftDivide) {
  int32_t rows[3] = {4, 10, 4};
  int32_t cols[3] = {4, 4, 10};

  MAT(10, 10, A);
  MAT(10, 10, AT);
  VEC(10, b);
  VEC(10, lresult);
  VEC(10, rresult);

  int32_t errl = 0, errr = 0;
  gsl_rng *rng = gsl_rng_alloc(gsl_rng_taus);
  for (int32_t d = 0; d < 3; ++d) {
    MatResize(rows[d], cols[d], &A);
    MatResize(cols[d], rows[d], &AT);
    int32_t max_rank = rows[d] < cols[d] ? rows[d] : cols[d];

    for (int32_t rank = 0; rank <= max_rank; ++rank) {
      MatRandomWithRank(rank, rng, &A);
      MatTrans(&A, &AT);

      VecResize(rows[d], &b);
      VecRandomGaussian(rng, &b);
      errl = MatVecLeftDivide(&A, &b, &lresult);
      errr = MatVecRightDivide(&AT, &b, &rresult);
      EXPECT_NEAR(errl, errr, 0.0);
      if (rank == max_rank) {
        EXPECT_NEAR(errr, kLinalgErrorNone, 0.0);
      }
      if (errr != kLinalgErrorSingularMat) {
        EXPECT_NEAR_VEC(lresult, rresult, 1e-6 * VecNorm(&lresult));
      }
    }
  }
  gsl_rng_free(rng);
}

TEST(MatMatLeftDivide, CompareToOctave) {
  std::string path = test_util::TestRunfilesDir() +
                     "/common/c_math/test_data/mat_divide_comparison.dat";
  FILE *f = fopen(path.c_str(), "r");
  assert(f != NULL);

  int32_t num_problems;
  int32_t scanned = fscanf(f, "%d\n", &num_problems);
  EXPECT_EQ(1, scanned);
  EXPECT_LT(0, num_problems);
  for (int32_t i = 0U; i < num_problems; ++i) {
    int32_t nr, nc, ra, nb;
    scanned = fscanf(f, "%d\n%d\n%d\n%d\n", &nr, &nc, &ra, &nb);
    EXPECT_EQ(4, scanned);
    MAT(nr, nc, A);
    MAT(nr, nb, B);
    MAT(nc, nb, X_ans);

    MatScan(f, &A);
    MatScan(f, &B);
    MatScan(f, &X_ans);

    MAT(nc, nb, X);
    MatMatLeftDivide(&A, &B, &X);

    // Calculate residuals (i.e. A*X - B) to compare.
    MAT(nr, nb, R_ans);
    MatGenMult(kNoTrans, kNoTrans, 1.0, &A, &X_ans, -1.0, &B, &R_ans);
    MAT(nr, nb, R);
    MatGenMult(kNoTrans, kNoTrans, 1.0, &A, &X, -1.0, &B, &R);
    EXPECT_NEAR_MAT(R, R_ans, 1e-6);

    // In overdetermined or fully determined cases X should nearly match.
    if (nc <= nr && ra == nc) EXPECT_NEAR_MAT(X, X_ans, 1e-6);
  }
  fclose(f);
}

TEST(MatSqrtSum, Product) {
  int32_t nras[6] = {1, 1, 2, 4, 8, 4};
  int32_t nrbs[6] = {1, 2, 1, 4, 4, 8};
  int32_t ncs[6] = {1, 2, 2, 8, 8, 8};
  gsl_rng *rng = gsl_rng_alloc(gsl_rng_taus);

  for (int32_t idx = 0; idx < 6; ++idx) {
    int32_t nr_a = nras[idx];
    int32_t nr_b = nrbs[idx];
    int32_t nc = ncs[idx];
    // Generate two random upper triangular matrices.
    MAT(nr_a, nc, A);
    MAT(nr_b, nc, B);
    MatRandomGaussian(rng, &A);
    MatRandomGaussian(rng, &B);
    MAT(nc, nc, sqrt_S);
    MatSqrtSum(&A, &B, &sqrt_S);

    MatIsUpperTriangular(&sqrt_S);
    EXPECT_EQ(MatHasNonnegDiag(&sqrt_S), 1);
    // Compute S = A^T*A + B^T*B.
    MAT(nc, nc, S);
    MatGenMult(kTrans, kNoTrans, 1.0, &A, &A, 0.0, &S, &S);
    MatGenMult(kTrans, kNoTrans, 1.0, &B, &B, 1.0, &S, &S);
    // Compare to sqrt_S^T*sqrt_S.
    MAT(nc, nc, tmp);
    MatGenMult(kTrans, kNoTrans, 1.0, &sqrt_S, &sqrt_S, 0.0, &tmp, &tmp);
    EXPECT_NEAR_MAT(S, tmp, 1e-10);
  }
  gsl_rng_free(rng);
}

TEST(MatSlice, ReuseInput) {
  MAT_INIT(
      4, 3, a,
      {{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}, {7.0, 8.0, 9.0}, {10.0, 11.0, 12.0}});
  a.variable_dim = 1;
  int32_t rows[4] = {1, 0, 1, 0};
  int32_t cols[3] = {1, 0, 1};
  MatSlice(&a, rows, cols, &a);
  MAT_INIT(2, 2, ans, {{1.0, 3.0}, {7.0, 9.0}});

  for (int32_t i = 0; i < 2; ++i)
    for (int32_t j = 0; j < 2; ++j)
      EXPECT_NEAR(MatGet(&a, i, j), MatGet(&ans, i, j), 1e-9);
}

// The remaining tests are generated code comparing the results to
// MATLAB.

TEST(VecNorm, Normal3) {
  double d_v[22] = {
      0.2317475849164190,  0.2728879621897584,  0.5004625031449950,
      0.5505625610897258,  0.7338783342702269,  0.1854299166582933,
      -0.9198447640128118, 0.5094601849922462,  -0.0854957209943779,
      0.6888425420984552,  0.2611970564707073,  -0.0408559808777271,
      -0.1776006413724596, -0.2043299090933990, -0.2836530065176404,
      0.3205574523156975,  0.8833844404841125,  0.1495163130482855,
      0.2187788454503548,  -0.2152780573264821, 0.8219197716963402,
      0.2852868684863044};
  const Vec v = {22, &d_v[0], 0, 22};
  double v_norm = VecNorm(&v);

  EXPECT_NEAR(2.1960124284499836, v_norm, 1e-9);
}

TEST(VecNormBound, Unbounded3) {
  double d_v[22] = {
      -0.3210460121692242, -0.7637691459095506, 0.2236956624851079,
      -0.0402487808158665, 0.7991492955325215,  -0.9918549552634650,
      0.3764621377382822,  -0.4659603145023175, 0.5537595518069636,
      -0.8839112242757425, -0.6189511648907760, 0.0081440728617221,
      0.8826932072969620,  -0.1576268141660597, 0.0597372033265351,
      -0.8618765876191983, 0.1022774125584802,  0.5801197924928927,
      -0.8245191907506253, -0.1740965642562382, -0.9664022577437645,
      -0.1081035450393890};
  const Vec v = {22, &d_v[0], 0, 22};
  double v_norm = VecNormBound(&v, 2.1028073942421859);

  EXPECT_NEAR(2.7827328081875722, v_norm, 1e-9);
}

TEST(VecNormBound, Bounded3) {
  double d_v[22] = {
      -0.9545210505756778, -0.7471780138442117, 0.9442516692949543,
      -0.6640431543158589, 0.2075756491468896,  -0.2112405891565678,
      0.4392355741608911,  0.3341108251099092,  -0.9301211120990667,
      -0.9409683380691614, 0.8631247814912648,  0.9986992415066285,
      -0.4011196692210388, -0.2790027034102058, -0.0322655344215750,
      0.5072497428336891,  0.9555989925727844,  -0.9224132139416199,
      0.8056241111777676,  0.3229926651174486,  -0.9437773616197127,
      0.1835346971592524};
  const Vec v = {22, &d_v[0], 0, 22};
  double v_norm = VecNormBound(&v, 3.9611063728394855);

  EXPECT_NEAR(3.9611063728394855, v_norm, 1e-9);
}

TEST(VecNormSquared, Normal3) {
  double d_v[22] = {
      -0.6359073264089210, 0.4414842346146832,  0.2132570119201320,
      -0.8648648959976213, -0.5073893320627969, -0.7103116935257796,
      -0.1683101285905484, 0.5884593538655045,  0.2067148403953118,
      -0.8158460071292502, -0.4786452541648969, 0.5294211735511232,
      -0.8474797434833554, 0.2301034904467640,  0.6944658355703914,
      -0.2011254065554515, 0.3221409527566221,  -0.3562325316831352,
      0.2605372714951533,  -0.8660262751930830, -0.7516847228333041,
      -0.0972987801941649};
  const Vec v = {22, &d_v[0], 0, 22};
  double v_norm_sqrd = VecNormSquared(&v);

  EXPECT_NEAR(6.6640377586422215, v_norm_sqrd, 1e-9);
}

TEST(VecNormalize, Normal3) {
  double d_v[22] = {
      0.8902697283846168,  -0.0511144231293001, -0.0648884443683231,
      0.5317170488634311,  0.3774032052262990,  0.9271611152837100,
      0.9968302761657899,  0.9437541093407011,  -0.7655993117142299,
      -0.9734457758547381, 0.3420691653287908,  0.1852080367028233,
      0.7789580087308183,  0.6640159300455803,  0.7029988186495963,
      0.0394577707625083,  -0.3729101181325767, -0.8922485430597333,
      -0.1261440477220708, -0.3882684191047061, -0.2294724120759148,
      -0.4968116636297848};
  Vec v = {22, &d_v[0], 0, 22};
  const Vec *v_out_ptr = VecNormalize(&v, &v);

  EXPECT_NEAR(0.3036085390902362, v.d[0], 1e-9);
  EXPECT_NEAR(-0.0174315433154013, v.d[1], 1e-9);
  EXPECT_NEAR(-0.0221288955137802, v.d[2], 1e-9);
  EXPECT_NEAR(0.1813313777474133, v.d[3], 1e-9);
  EXPECT_NEAR(0.1287057530245787, v.d[4], 1e-9);
  EXPECT_NEAR(0.3161896026986434, v.d[5], 1e-9);
  EXPECT_NEAR(0.3399488651790506, v.d[6], 1e-9);
  EXPECT_NEAR(0.3218483087336302, v.d[7], 1e-9);
  EXPECT_NEAR(-0.2610922073918110, v.d[8], 1e-9);
  EXPECT_NEAR(-0.3319740528829210, v.d[9], 1e-9);
  EXPECT_NEAR(0.1166557912080583, v.d[10], 1e-9);
  EXPECT_NEAR(0.0631614663043132, v.d[11], 1e-9);
  EXPECT_NEAR(0.2656479216389019, v.d[12], 1e-9);
  EXPECT_NEAR(0.2264492434439902, v.d[13], 1e-9);
  EXPECT_NEAR(0.2397435715349364, v.d[14], 1e-9);
  EXPECT_NEAR(0.0134562770753754, v.d[15], 1e-9);
  EXPECT_NEAR(-0.1271734762717731, v.d[16], 1e-9);
  EXPECT_NEAR(-0.3042833739335286, v.d[17], 1e-9);
  EXPECT_NEAR(-0.0430188838536821, v.d[18], 1e-9);
  EXPECT_NEAR(-0.1324111151270414, v.d[19], 1e-9);
  EXPECT_NEAR(-0.0782569389597197, v.d[20], 1e-9);
  EXPECT_NEAR(-0.1694275999604291, v.d[21], 1e-9);

  EXPECT_NEAR(0.3036085390902362, v_out_ptr->d[0], 1e-9);
  EXPECT_NEAR(-0.0174315433154013, v_out_ptr->d[1], 1e-9);
  EXPECT_NEAR(-0.0221288955137802, v_out_ptr->d[2], 1e-9);
  EXPECT_NEAR(0.1813313777474133, v_out_ptr->d[3], 1e-9);
  EXPECT_NEAR(0.1287057530245787, v_out_ptr->d[4], 1e-9);
  EXPECT_NEAR(0.3161896026986434, v_out_ptr->d[5], 1e-9);
  EXPECT_NEAR(0.3399488651790506, v_out_ptr->d[6], 1e-9);
  EXPECT_NEAR(0.3218483087336302, v_out_ptr->d[7], 1e-9);
  EXPECT_NEAR(-0.2610922073918110, v_out_ptr->d[8], 1e-9);
  EXPECT_NEAR(-0.3319740528829210, v_out_ptr->d[9], 1e-9);
  EXPECT_NEAR(0.1166557912080583, v_out_ptr->d[10], 1e-9);
  EXPECT_NEAR(0.0631614663043132, v_out_ptr->d[11], 1e-9);
  EXPECT_NEAR(0.2656479216389019, v_out_ptr->d[12], 1e-9);
  EXPECT_NEAR(0.2264492434439902, v_out_ptr->d[13], 1e-9);
  EXPECT_NEAR(0.2397435715349364, v_out_ptr->d[14], 1e-9);
  EXPECT_NEAR(0.0134562770753754, v_out_ptr->d[15], 1e-9);
  EXPECT_NEAR(-0.1271734762717731, v_out_ptr->d[16], 1e-9);
  EXPECT_NEAR(-0.3042833739335286, v_out_ptr->d[17], 1e-9);
  EXPECT_NEAR(-0.0430188838536821, v_out_ptr->d[18], 1e-9);
  EXPECT_NEAR(-0.1324111151270414, v_out_ptr->d[19], 1e-9);
  EXPECT_NEAR(-0.0782569389597197, v_out_ptr->d[20], 1e-9);
  EXPECT_NEAR(-0.1694275999604291, v_out_ptr->d[21], 1e-9);
}

TEST(VecNormalize, Zero_3) {
  double d_v[22] = {0.0000000000000000, 0.0000000000000000, 0.0000000000000000,
                    0.0000000000000000, 0.0000000000000000, 0.0000000000000000,
                    0.0000000000000000, 0.0000000000000000, 0.0000000000000000,
                    0.0000000000000000, 0.0000000000000000, 0.0000000000000000,
                    0.0000000000000000, 0.0000000000000000, 0.0000000000000000,
                    0.0000000000000000, 0.0000000000000000, 0.0000000000000000,
                    0.0000000000000000, 0.0000000000000000, 0.0000000000000000,
                    0.0000000000000000};
  Vec v = {22, &d_v[0], 0, 22};
  const Vec *v_out_ptr = VecNormalize(&v, &v);

  EXPECT_NEAR(0.0000000000000000, v.d[0], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v.d[1], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v.d[2], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v.d[3], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v.d[4], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v.d[5], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v.d[6], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v.d[7], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v.d[8], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v.d[9], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v.d[10], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v.d[11], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v.d[12], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v.d[13], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v.d[14], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v.d[15], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v.d[16], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v.d[17], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v.d[18], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v.d[19], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v.d[20], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v.d[21], 1e-9);

  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[0], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[1], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[2], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[3], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[4], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[5], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[6], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[7], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[8], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[9], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[10], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[11], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[12], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[13], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[14], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[15], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[16], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[17], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[18], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[19], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[20], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[21], 1e-9);
}

TEST(VecScale, Normal3) {
  double d_v[22] = {
      -0.5424168543576759, -0.7548076571565581, -0.9950621386298837,
      0.3785951413341218,  0.2887634520126996,  -0.1936547255447369,
      0.1401173909637214,  -0.9979763064358147, 0.0129135603023602,
      0.9892006454553519,  0.8710460692622675,  -0.3636782126425873,
      -0.7829372380496851, -0.5765657162411022, -0.8891083308891483,
      0.2819795133382927,  -0.3357217629614622, -0.2326318436530428,
      0.0713835815746082,  -0.2943664462487412, -0.4107728450560240,
      -0.2806834929593409};
  const Vec v = {22, &d_v[0], 0, 22};
  double d_v_out[22] = {
      0.7726444811392215,  -0.6521704422307164, -0.1905694599542231,
      0.4475848343933917,  0.6584196828341082,  -0.9864196746828660,
      -0.8954641566972439, 0.8971949027951096,  0.0612471288695586,
      -0.6327108379186486, 0.3274679673621781,  -0.4790044566283911,
      0.8711808944122919,  0.1043461863388142,  -0.5023288428880606,
      -0.6935306236627714, -0.0452877914192231, 0.6101651740785661,
      0.3978295151534732,  0.1386417737667867,  -0.4754992363289197,
      -0.5656247345648993};
  Vec v_out = {22, &d_v_out[0], 0, 22};
  const Vec *v_out_ptr = VecScale(&v, 0.3229642692546391, &v_out);

  EXPECT_NEAR(-0.1751812629990268, v_out.d[0], 1e-9);
  EXPECT_NEAR(-0.2437759034213739, v_out.d[1], 1e-9);
  EXPECT_NEAR(-0.3213695164655587, v_out.d[2], 1e-9);
  EXPECT_NEAR(0.1222727031643314, v_out.d[3], 1e-9);
  EXPECT_NEAR(0.0932602772667286, v_out.d[4], 1e-9);
  EXPECT_NEAR(-0.0625435569232636, v_out.d[5], 1e-9);
  EXPECT_NEAR(0.0452529107824649, v_out.d[6], 1e-9);
  EXPECT_NEAR(-0.3223106885414866, v_out.d[7], 1e-9);
  EXPECT_NEAR(0.0041706185665275, v_out.d[8], 1e-9);
  EXPECT_NEAR(0.3194764636057050, v_out.d[9], 1e-9);
  EXPECT_NEAR(0.2813167572464139, v_out.d[10], 1e-9);
  EXPECT_NEAR(-0.1174550681899464, v_out.d[11], 1e-9);
  EXPECT_NEAR(-0.2528607529589619, v_out.d[12], 1e-9);
  EXPECT_NEAR(-0.1862101252230851, v_out.d[13], 1e-9);
  EXPECT_NEAR(-0.2871502223738256, v_out.d[14], 1e-9);
  EXPECT_NEAR(0.0910693074700805, v_out.d[15], 1e-9);
  EXPECT_NEAR(-0.1084261338477278, v_out.d[16], 1e-9);
  EXPECT_NEAR(-0.0751317733907644, v_out.d[17], 1e-9);
  EXPECT_NEAR(0.0230543462600223, v_out.d[18], 1e-9);
  EXPECT_NEAR(-0.0950698442058097, v_out.d[19], 1e-9);
  EXPECT_NEAR(-0.1326649517331679, v_out.d[20], 1e-9);
  EXPECT_NEAR(-0.0906507391954532, v_out.d[21], 1e-9);

  EXPECT_NEAR(-0.1751812629990268, v_out_ptr->d[0], 1e-9);
  EXPECT_NEAR(-0.2437759034213739, v_out_ptr->d[1], 1e-9);
  EXPECT_NEAR(-0.3213695164655587, v_out_ptr->d[2], 1e-9);
  EXPECT_NEAR(0.1222727031643314, v_out_ptr->d[3], 1e-9);
  EXPECT_NEAR(0.0932602772667286, v_out_ptr->d[4], 1e-9);
  EXPECT_NEAR(-0.0625435569232636, v_out_ptr->d[5], 1e-9);
  EXPECT_NEAR(0.0452529107824649, v_out_ptr->d[6], 1e-9);
  EXPECT_NEAR(-0.3223106885414866, v_out_ptr->d[7], 1e-9);
  EXPECT_NEAR(0.0041706185665275, v_out_ptr->d[8], 1e-9);
  EXPECT_NEAR(0.3194764636057050, v_out_ptr->d[9], 1e-9);
  EXPECT_NEAR(0.2813167572464139, v_out_ptr->d[10], 1e-9);
  EXPECT_NEAR(-0.1174550681899464, v_out_ptr->d[11], 1e-9);
  EXPECT_NEAR(-0.2528607529589619, v_out_ptr->d[12], 1e-9);
  EXPECT_NEAR(-0.1862101252230851, v_out_ptr->d[13], 1e-9);
  EXPECT_NEAR(-0.2871502223738256, v_out_ptr->d[14], 1e-9);
  EXPECT_NEAR(0.0910693074700805, v_out_ptr->d[15], 1e-9);
  EXPECT_NEAR(-0.1084261338477278, v_out_ptr->d[16], 1e-9);
  EXPECT_NEAR(-0.0751317733907644, v_out_ptr->d[17], 1e-9);
  EXPECT_NEAR(0.0230543462600223, v_out_ptr->d[18], 1e-9);
  EXPECT_NEAR(-0.0950698442058097, v_out_ptr->d[19], 1e-9);
  EXPECT_NEAR(-0.1326649517331679, v_out_ptr->d[20], 1e-9);
  EXPECT_NEAR(-0.0906507391954532, v_out_ptr->d[21], 1e-9);
}

TEST(VecAdd, Normal3) {
  double d_v0[22] = {
      0.1446651325696462,  -0.4338183776619027, -0.5431218260845314,
      -0.3483311323192728, 0.4579412539930312,  -0.9594609519893431,
      -0.7191725347761235, 0.4376754233693294,  0.1954418421389532,
      0.8430791939976203,  0.4559087118305252,  0.9460879068575427,
      0.3077012274666735,  -0.1073352500898010, 0.9483367200791142,
      0.3188191594298864,  0.8291068808698860,  -0.0057481793823870,
      -0.4567724884935798, -0.2825645933879546, -0.8137443796044186,
      0.3884737476669939};
  const Vec v0 = {22, &d_v0[0], 0, 22};
  double d_v1[22] = {
      -0.6049795684707804, 0.9233962692557960,  -0.5031016280192397,
      0.4627362638833896,  0.0498875952465365,  0.1564054897849587,
      0.9404252838711387,  -0.3895440680827962, 0.8579482121325355,
      0.0105756207077567,  -0.1065749482257137, 0.2440477450748526,
      -0.0137365018264168, -0.1551049713745651, 0.4257691946653870,
      0.7698135233274099,  -0.4067196052645288, -0.2206420904735540,
      -0.8077275914120787, 0.1620062141959595,  -0.5630180192038090,
      0.7249701861229663};
  const Vec v1 = {22, &d_v1[0], 0, 22};
  double d_v_out[22] = {
      -0.6335878247323636, 0.0018424652756888,  0.7257358960810825,
      -0.5638349827145632, -0.3885038144135886, -0.5369261711721744,
      0.7184807687090926,  0.1132638024555173,  -0.2505778936114673,
      0.8116559405626289,  -0.2465645137777626, -0.4866282743253405,
      -0.8151198832567284, -0.2592603334760499, 0.3567131447772489,
      -0.8469412895140154, -0.3357265872788724, 0.1067358008325698,
      -0.9722248317342366, -0.2842082715660674, 0.2891633907756446,
      -0.8707867357654591};
  Vec v_out = {22, &d_v_out[0], 0, 22};
  const Vec *v_out_ptr = VecAdd(&v0, &v1, &v_out);

  EXPECT_NEAR(-0.4603144359011342, v_out.d[0], 1e-9);
  EXPECT_NEAR(0.4895778915938933, v_out.d[1], 1e-9);
  EXPECT_NEAR(-1.0462234541037712, v_out.d[2], 1e-9);
  EXPECT_NEAR(0.1144051315641168, v_out.d[3], 1e-9);
  EXPECT_NEAR(0.5078288492395677, v_out.d[4], 1e-9);
  EXPECT_NEAR(-0.8030554622043844, v_out.d[5], 1e-9);
  EXPECT_NEAR(0.2212527490950151, v_out.d[6], 1e-9);
  EXPECT_NEAR(0.0481313552865332, v_out.d[7], 1e-9);
  EXPECT_NEAR(1.0533900542714887, v_out.d[8], 1e-9);
  EXPECT_NEAR(0.8536548147053771, v_out.d[9], 1e-9);
  EXPECT_NEAR(0.3493337636048115, v_out.d[10], 1e-9);
  EXPECT_NEAR(1.1901356519323953, v_out.d[11], 1e-9);
  EXPECT_NEAR(0.2939647256402567, v_out.d[12], 1e-9);
  EXPECT_NEAR(-0.2624402214643662, v_out.d[13], 1e-9);
  EXPECT_NEAR(1.3741059147445012, v_out.d[14], 1e-9);
  EXPECT_NEAR(1.0886326827572963, v_out.d[15], 1e-9);
  EXPECT_NEAR(0.4223872756053573, v_out.d[16], 1e-9);
  EXPECT_NEAR(-0.2263902698559410, v_out.d[17], 1e-9);
  EXPECT_NEAR(-1.2645000799056585, v_out.d[18], 1e-9);
  EXPECT_NEAR(-0.1205583791919951, v_out.d[19], 1e-9);
  EXPECT_NEAR(-1.3767623988082276, v_out.d[20], 1e-9);
  EXPECT_NEAR(1.1134439337899602, v_out.d[21], 1e-9);

  EXPECT_NEAR(-0.4603144359011342, v_out_ptr->d[0], 1e-9);
  EXPECT_NEAR(0.4895778915938933, v_out_ptr->d[1], 1e-9);
  EXPECT_NEAR(-1.0462234541037712, v_out_ptr->d[2], 1e-9);
  EXPECT_NEAR(0.1144051315641168, v_out_ptr->d[3], 1e-9);
  EXPECT_NEAR(0.5078288492395677, v_out_ptr->d[4], 1e-9);
  EXPECT_NEAR(-0.8030554622043844, v_out_ptr->d[5], 1e-9);
  EXPECT_NEAR(0.2212527490950151, v_out_ptr->d[6], 1e-9);
  EXPECT_NEAR(0.0481313552865332, v_out_ptr->d[7], 1e-9);
  EXPECT_NEAR(1.0533900542714887, v_out_ptr->d[8], 1e-9);
  EXPECT_NEAR(0.8536548147053771, v_out_ptr->d[9], 1e-9);
  EXPECT_NEAR(0.3493337636048115, v_out_ptr->d[10], 1e-9);
  EXPECT_NEAR(1.1901356519323953, v_out_ptr->d[11], 1e-9);
  EXPECT_NEAR(0.2939647256402567, v_out_ptr->d[12], 1e-9);
  EXPECT_NEAR(-0.2624402214643662, v_out_ptr->d[13], 1e-9);
  EXPECT_NEAR(1.3741059147445012, v_out_ptr->d[14], 1e-9);
  EXPECT_NEAR(1.0886326827572963, v_out_ptr->d[15], 1e-9);
  EXPECT_NEAR(0.4223872756053573, v_out_ptr->d[16], 1e-9);
  EXPECT_NEAR(-0.2263902698559410, v_out_ptr->d[17], 1e-9);
  EXPECT_NEAR(-1.2645000799056585, v_out_ptr->d[18], 1e-9);
  EXPECT_NEAR(-0.1205583791919951, v_out_ptr->d[19], 1e-9);
  EXPECT_NEAR(-1.3767623988082276, v_out_ptr->d[20], 1e-9);
  EXPECT_NEAR(1.1134439337899602, v_out_ptr->d[21], 1e-9);
}

TEST(VecAdd3, Normal3) {
  double d_v0[22] = {
      -0.8465497409979115, -0.5233505840631365, 0.3205924951655204,
      -0.1764157066097130, -0.4418238201905387, -0.7552295613617694,
      0.4698437118818046,  -0.4862090536386232, -0.5134621890701097,
      -0.0747205857929809, 0.8614516030998667,  -0.9445012677093043,
      0.9178178682454752,  0.3677111099637114,  -0.7275077191271311,
      0.8944768229490447,  0.5349712728567793,  -0.5068430166014533,
      0.1391492690605933,  0.9488655634746668,  -0.3609853836625756,
      0.1620396791059344};
  const Vec v0 = {22, &d_v0[0], 0, 22};
  double d_v1[22] = {
      -0.1152666803726785, -0.4271082532455386, -0.4932472312396556,
      -0.8381156002302570, 0.5068401220422574,  0.8101771515938763,
      0.6177786412233512,  -0.0948724548931887, -0.9321848017491436,
      0.3013599966126816,  -0.6377772127536068, -0.8594673130425536,
      -0.8644202429157575, -0.5215039440271954, 0.2320621638656115,
      -0.1842996997808488, -0.8703581692201734, 0.0113425003199297,
      0.0810763952602296,  -0.6119688737608362, 0.4196784976060979,
      -0.0026852403894346};
  const Vec v1 = {22, &d_v1[0], 0, 22};
  double d_v2[22] = {
      0.2389966090583828,  -0.5123981219477289, 0.5018198337865563,
      0.0150316926006699,  0.7958920963163227,  -0.2257798350860565,
      -0.8869981063675993, -0.5980420396159223, -0.9495531184486903,
      0.6351923438297407,  -0.9411472903777525, 0.6236470284008762,
      0.8562683937380398,  0.1579608810136111,  0.7722065706465300,
      0.4699189643263606,  0.3049555977992402,  -0.9625500062303003,
      -0.0235151381472101, 0.5908819871326363,  -0.5598601872344542,
      -0.4595995587998774};
  const Vec v2 = {22, &d_v2[0], 0, 22};
  double d_v0c[22] = {
      -0.8465497409979115, -0.5233505840631365, 0.3205924951655204,
      -0.1764157066097130, -0.4418238201905387, -0.7552295613617694,
      0.4698437118818046,  -0.4862090536386232, -0.5134621890701097,
      -0.0747205857929809, 0.8614516030998667,  -0.9445012677093043,
      0.9178178682454752,  0.3677111099637114,  -0.7275077191271311,
      0.8944768229490447,  0.5349712728567793,  -0.5068430166014533,
      0.1391492690605933,  0.9488655634746668,  -0.3609853836625756,
      0.1620396791059344};
  Vec v0c = {22, &d_v0c[0], 0, 22};
  double d_v1c[22] = {
      -0.1152666803726785, -0.4271082532455386, -0.4932472312396556,
      -0.8381156002302570, 0.5068401220422574,  0.8101771515938763,
      0.6177786412233512,  -0.0948724548931887, -0.9321848017491436,
      0.3013599966126816,  -0.6377772127536068, -0.8594673130425536,
      -0.8644202429157575, -0.5215039440271954, 0.2320621638656115,
      -0.1842996997808488, -0.8703581692201734, 0.0113425003199297,
      0.0810763952602296,  -0.6119688737608362, 0.4196784976060979,
      -0.0026852403894346};
  Vec v1c = {22, &d_v1c[0], 0, 22};
  double d_v2c[22] = {
      0.2389966090583828,  -0.5123981219477289, 0.5018198337865563,
      0.0150316926006699,  0.7958920963163227,  -0.2257798350860565,
      -0.8869981063675993, -0.5980420396159223, -0.9495531184486903,
      0.6351923438297407,  -0.9411472903777525, 0.6236470284008762,
      0.8562683937380398,  0.1579608810136111,  0.7722065706465300,
      0.4699189643263606,  0.3049555977992402,  -0.9625500062303003,
      -0.0235151381472101, 0.5908819871326363,  -0.5598601872344542,
      -0.4595995587998774};
  Vec v2c = {22, &d_v2c[0], 0, 22};
  double d_v_out[22] = {
      -0.3471574163172750, -0.8027721518482092, 0.2372844465456991,
      0.4022338575055855,  -0.2579168977394961, -0.8016559359493407,
      -0.3223399735398202, -0.7700932365136059, -0.1689954321437372,
      -0.1260605663543299, -0.6463724884849176, -0.6704194113893538,
      -0.7212286905788854, -0.5683133053280716, 0.3387311268734146,
      0.0035149135983790,  -0.3437711840928095, -0.7843361553307167,
      -0.7759439792455367, -0.3401469101761194, -0.2047298042126491,
      0.7044588101579374};
  Vec v_out = {22, &d_v_out[0], 0, 22};
  const Vec *v_out_ptr = VecAdd3(&v0, &v1, &v2, &v_out);
  VecAdd3(&v0c, &v1, &v2, &v0c);
  VecAdd3(&v0, &v1c, &v2, &v1c);
  VecAdd3(&v0, &v1, &v2c, &v2c);

  EXPECT_NEAR(-0.7228198123122072, v_out.d[0], 1e-9);
  EXPECT_NEAR(-1.4628569592564040, v_out.d[1], 1e-9);
  EXPECT_NEAR(0.3291650977124210, v_out.d[2], 1e-9);
  EXPECT_NEAR(-0.9994996142393000, v_out.d[3], 1e-9);
  EXPECT_NEAR(0.8609083981680414, v_out.d[4], 1e-9);
  EXPECT_NEAR(-0.1708322448539497, v_out.d[5], 1e-9);
  EXPECT_NEAR(0.2006242467375565, v_out.d[6], 1e-9);
  EXPECT_NEAR(-1.1791235481477342, v_out.d[7], 1e-9);
  EXPECT_NEAR(-2.3952001092679436, v_out.d[8], 1e-9);
  EXPECT_NEAR(0.8618317546494414, v_out.d[9], 1e-9);
  EXPECT_NEAR(-0.7174729000314926, v_out.d[10], 1e-9);
  EXPECT_NEAR(-1.1803215523509818, v_out.d[11], 1e-9);
  EXPECT_NEAR(0.9096660190677575, v_out.d[12], 1e-9);
  EXPECT_NEAR(0.0041680469501271, v_out.d[13], 1e-9);
  EXPECT_NEAR(0.2767610153850104, v_out.d[14], 1e-9);
  EXPECT_NEAR(1.1800960874945565, v_out.d[15], 1e-9);
  EXPECT_NEAR(-0.0304312985641539, v_out.d[16], 1e-9);
  EXPECT_NEAR(-1.4580505225118239, v_out.d[17], 1e-9);
  EXPECT_NEAR(0.1967105261736128, v_out.d[18], 1e-9);
  EXPECT_NEAR(0.9277786768464669, v_out.d[19], 1e-9);
  EXPECT_NEAR(-0.5011670732909319, v_out.d[20], 1e-9);
  EXPECT_NEAR(-0.3002451200833776, v_out.d[21], 1e-9);

  EXPECT_NEAR(-0.7228198123122072, v0c.d[0], 1e-9);
  EXPECT_NEAR(-1.4628569592564040, v0c.d[1], 1e-9);
  EXPECT_NEAR(0.3291650977124210, v0c.d[2], 1e-9);
  EXPECT_NEAR(-0.9994996142393000, v0c.d[3], 1e-9);
  EXPECT_NEAR(0.8609083981680414, v0c.d[4], 1e-9);
  EXPECT_NEAR(-0.1708322448539497, v0c.d[5], 1e-9);
  EXPECT_NEAR(0.2006242467375565, v0c.d[6], 1e-9);
  EXPECT_NEAR(-1.1791235481477342, v0c.d[7], 1e-9);
  EXPECT_NEAR(-2.3952001092679436, v0c.d[8], 1e-9);
  EXPECT_NEAR(0.8618317546494414, v0c.d[9], 1e-9);
  EXPECT_NEAR(-0.7174729000314926, v0c.d[10], 1e-9);
  EXPECT_NEAR(-1.1803215523509818, v0c.d[11], 1e-9);
  EXPECT_NEAR(0.9096660190677575, v0c.d[12], 1e-9);
  EXPECT_NEAR(0.0041680469501271, v0c.d[13], 1e-9);
  EXPECT_NEAR(0.2767610153850104, v0c.d[14], 1e-9);
  EXPECT_NEAR(1.1800960874945565, v0c.d[15], 1e-9);
  EXPECT_NEAR(-0.0304312985641539, v0c.d[16], 1e-9);
  EXPECT_NEAR(-1.4580505225118239, v0c.d[17], 1e-9);
  EXPECT_NEAR(0.1967105261736128, v0c.d[18], 1e-9);
  EXPECT_NEAR(0.9277786768464669, v0c.d[19], 1e-9);
  EXPECT_NEAR(-0.5011670732909319, v0c.d[20], 1e-9);
  EXPECT_NEAR(-0.3002451200833776, v0c.d[21], 1e-9);

  EXPECT_NEAR(-0.7228198123122072, v1c.d[0], 1e-9);
  EXPECT_NEAR(-1.4628569592564040, v1c.d[1], 1e-9);
  EXPECT_NEAR(0.3291650977124210, v1c.d[2], 1e-9);
  EXPECT_NEAR(-0.9994996142393000, v1c.d[3], 1e-9);
  EXPECT_NEAR(0.8609083981680414, v1c.d[4], 1e-9);
  EXPECT_NEAR(-0.1708322448539497, v1c.d[5], 1e-9);
  EXPECT_NEAR(0.2006242467375565, v1c.d[6], 1e-9);
  EXPECT_NEAR(-1.1791235481477342, v1c.d[7], 1e-9);
  EXPECT_NEAR(-2.3952001092679436, v1c.d[8], 1e-9);
  EXPECT_NEAR(0.8618317546494414, v1c.d[9], 1e-9);
  EXPECT_NEAR(-0.7174729000314926, v1c.d[10], 1e-9);
  EXPECT_NEAR(-1.1803215523509818, v1c.d[11], 1e-9);
  EXPECT_NEAR(0.9096660190677575, v1c.d[12], 1e-9);
  EXPECT_NEAR(0.0041680469501271, v1c.d[13], 1e-9);
  EXPECT_NEAR(0.2767610153850104, v1c.d[14], 1e-9);
  EXPECT_NEAR(1.1800960874945565, v1c.d[15], 1e-9);
  EXPECT_NEAR(-0.0304312985641539, v1c.d[16], 1e-9);
  EXPECT_NEAR(-1.4580505225118239, v1c.d[17], 1e-9);
  EXPECT_NEAR(0.1967105261736128, v1c.d[18], 1e-9);
  EXPECT_NEAR(0.9277786768464669, v1c.d[19], 1e-9);
  EXPECT_NEAR(-0.5011670732909319, v1c.d[20], 1e-9);
  EXPECT_NEAR(-0.3002451200833776, v1c.d[21], 1e-9);

  EXPECT_NEAR(-0.7228198123122072, v2c.d[0], 1e-9);
  EXPECT_NEAR(-1.4628569592564040, v2c.d[1], 1e-9);
  EXPECT_NEAR(0.3291650977124210, v2c.d[2], 1e-9);
  EXPECT_NEAR(-0.9994996142393000, v2c.d[3], 1e-9);
  EXPECT_NEAR(0.8609083981680414, v2c.d[4], 1e-9);
  EXPECT_NEAR(-0.1708322448539497, v2c.d[5], 1e-9);
  EXPECT_NEAR(0.2006242467375565, v2c.d[6], 1e-9);
  EXPECT_NEAR(-1.1791235481477342, v2c.d[7], 1e-9);
  EXPECT_NEAR(-2.3952001092679436, v2c.d[8], 1e-9);
  EXPECT_NEAR(0.8618317546494414, v2c.d[9], 1e-9);
  EXPECT_NEAR(-0.7174729000314926, v2c.d[10], 1e-9);
  EXPECT_NEAR(-1.1803215523509818, v2c.d[11], 1e-9);
  EXPECT_NEAR(0.9096660190677575, v2c.d[12], 1e-9);
  EXPECT_NEAR(0.0041680469501271, v2c.d[13], 1e-9);
  EXPECT_NEAR(0.2767610153850104, v2c.d[14], 1e-9);
  EXPECT_NEAR(1.1800960874945565, v2c.d[15], 1e-9);
  EXPECT_NEAR(-0.0304312985641539, v2c.d[16], 1e-9);
  EXPECT_NEAR(-1.4580505225118239, v2c.d[17], 1e-9);
  EXPECT_NEAR(0.1967105261736128, v2c.d[18], 1e-9);
  EXPECT_NEAR(0.9277786768464669, v2c.d[19], 1e-9);
  EXPECT_NEAR(-0.5011670732909319, v2c.d[20], 1e-9);
  EXPECT_NEAR(-0.3002451200833776, v2c.d[21], 1e-9);

  EXPECT_NEAR(-0.7228198123122072, v_out_ptr->d[0], 1e-9);
  EXPECT_NEAR(-1.4628569592564040, v_out_ptr->d[1], 1e-9);
  EXPECT_NEAR(0.3291650977124210, v_out_ptr->d[2], 1e-9);
  EXPECT_NEAR(-0.9994996142393000, v_out_ptr->d[3], 1e-9);
  EXPECT_NEAR(0.8609083981680414, v_out_ptr->d[4], 1e-9);
  EXPECT_NEAR(-0.1708322448539497, v_out_ptr->d[5], 1e-9);
  EXPECT_NEAR(0.2006242467375565, v_out_ptr->d[6], 1e-9);
  EXPECT_NEAR(-1.1791235481477342, v_out_ptr->d[7], 1e-9);
  EXPECT_NEAR(-2.3952001092679436, v_out_ptr->d[8], 1e-9);
  EXPECT_NEAR(0.8618317546494414, v_out_ptr->d[9], 1e-9);
  EXPECT_NEAR(-0.7174729000314926, v_out_ptr->d[10], 1e-9);
  EXPECT_NEAR(-1.1803215523509818, v_out_ptr->d[11], 1e-9);
  EXPECT_NEAR(0.9096660190677575, v_out_ptr->d[12], 1e-9);
  EXPECT_NEAR(0.0041680469501271, v_out_ptr->d[13], 1e-9);
  EXPECT_NEAR(0.2767610153850104, v_out_ptr->d[14], 1e-9);
  EXPECT_NEAR(1.1800960874945565, v_out_ptr->d[15], 1e-9);
  EXPECT_NEAR(-0.0304312985641539, v_out_ptr->d[16], 1e-9);
  EXPECT_NEAR(-1.4580505225118239, v_out_ptr->d[17], 1e-9);
  EXPECT_NEAR(0.1967105261736128, v_out_ptr->d[18], 1e-9);
  EXPECT_NEAR(0.9277786768464669, v_out_ptr->d[19], 1e-9);
  EXPECT_NEAR(-0.5011670732909319, v_out_ptr->d[20], 1e-9);
  EXPECT_NEAR(-0.3002451200833776, v_out_ptr->d[21], 1e-9);
}

TEST(VecSub, Normal3) {
  double d_v0[22] = {
      0.6850310575794785,  -0.2719127601990134, 0.4778140485953484,
      -0.7217917102564537, -0.5938857891136853, -0.0598314931599750,
      -0.9971225871556719, -0.3106930285082081, 0.6884527163171945,
      0.9613479693682221,  -0.2016570881627786, 0.7392944167052278,
      0.9369989621027746,  -0.6186677978433355, 0.3540488351351316,
      -0.5837095319975620, -0.7763251672105700, -0.6582453684679499,
      -0.3628633158773216, 0.9089834444678941,  -0.9441426456939486,
      -0.3519215047913173};
  const Vec v0 = {22, &d_v0[0], 0, 22};
  double d_v1[22] = {
      0.4721185627728113,  -0.1838419652618914, 0.1934534598003528,
      0.6581974625332765,  0.6714563719665763,  0.6729171318186675,
      -0.5971883395533659, -0.4397405050026206, -0.6321825916919963,
      -0.8057096763738516, -0.7253750349373895, -0.2785042299408627,
      0.6548898738173541,  0.7947375851361873,  0.4864384373113260,
      -0.8380039082895288, 0.3097424402720808,  -0.6884035257626460,
      -0.7236411963624896, -0.1036554990302125, 0.3370570411070881,
      -0.6832615953191601};
  const Vec v1 = {22, &d_v1[0], 0, 22};
  double d_v_out[22] = {
      0.3800037929919280,  -0.8865404743760299, -0.2369853726141355,
      0.4679751801889254,  -0.8216758108595013, 0.9728789724275477,
      -0.1017224809248931, -0.6159231126641795, -0.1632597244988048,
      0.1481661779096808,  -0.6135830796424688, -0.0229652425406806,
      0.0800406909422593,  0.6050925090746881,  -0.5317661135611589,
      0.2089759812159522,  -0.3560794564885215, -0.9973329677761826,
      -0.0263340917122770, 0.0136878713610302,  0.4173545086409260,
      0.9039926034139012};
  Vec v_out = {22, &d_v_out[0], 0, 22};
  const Vec *v_out_ptr = VecSub(&v0, &v1, &v_out);

  EXPECT_NEAR(0.2129124948066672, v_out.d[0], 1e-9);
  EXPECT_NEAR(-0.0880707949371220, v_out.d[1], 1e-9);
  EXPECT_NEAR(0.2843605887949956, v_out.d[2], 1e-9);
  EXPECT_NEAR(-1.3799891727897302, v_out.d[3], 1e-9);
  EXPECT_NEAR(-1.2653421610802615, v_out.d[4], 1e-9);
  EXPECT_NEAR(-0.7327486249786426, v_out.d[5], 1e-9);
  EXPECT_NEAR(-0.3999342476023060, v_out.d[6], 1e-9);
  EXPECT_NEAR(0.1290474764944125, v_out.d[7], 1e-9);
  EXPECT_NEAR(1.3206353080091908, v_out.d[8], 1e-9);
  EXPECT_NEAR(1.7670576457420737, v_out.d[9], 1e-9);
  EXPECT_NEAR(0.5237179467746109, v_out.d[10], 1e-9);
  EXPECT_NEAR(1.0177986466460904, v_out.d[11], 1e-9);
  EXPECT_NEAR(0.2821090882854205, v_out.d[12], 1e-9);
  EXPECT_NEAR(-1.4134053829795228, v_out.d[13], 1e-9);
  EXPECT_NEAR(-0.1323896021761943, v_out.d[14], 1e-9);
  EXPECT_NEAR(0.2542943762919667, v_out.d[15], 1e-9);
  EXPECT_NEAR(-1.0860676074826507, v_out.d[16], 1e-9);
  EXPECT_NEAR(0.0301581572946961, v_out.d[17], 1e-9);
  EXPECT_NEAR(0.3607778804851680, v_out.d[18], 1e-9);
  EXPECT_NEAR(1.0126389434981067, v_out.d[19], 1e-9);
  EXPECT_NEAR(-1.2811996868010367, v_out.d[20], 1e-9);
  EXPECT_NEAR(0.3313400905278427, v_out.d[21], 1e-9);

  EXPECT_NEAR(0.2129124948066672, v_out_ptr->d[0], 1e-9);
  EXPECT_NEAR(-0.0880707949371220, v_out_ptr->d[1], 1e-9);
  EXPECT_NEAR(0.2843605887949956, v_out_ptr->d[2], 1e-9);
  EXPECT_NEAR(-1.3799891727897302, v_out_ptr->d[3], 1e-9);
  EXPECT_NEAR(-1.2653421610802615, v_out_ptr->d[4], 1e-9);
  EXPECT_NEAR(-0.7327486249786426, v_out_ptr->d[5], 1e-9);
  EXPECT_NEAR(-0.3999342476023060, v_out_ptr->d[6], 1e-9);
  EXPECT_NEAR(0.1290474764944125, v_out_ptr->d[7], 1e-9);
  EXPECT_NEAR(1.3206353080091908, v_out_ptr->d[8], 1e-9);
  EXPECT_NEAR(1.7670576457420737, v_out_ptr->d[9], 1e-9);
  EXPECT_NEAR(0.5237179467746109, v_out_ptr->d[10], 1e-9);
  EXPECT_NEAR(1.0177986466460904, v_out_ptr->d[11], 1e-9);
  EXPECT_NEAR(0.2821090882854205, v_out_ptr->d[12], 1e-9);
  EXPECT_NEAR(-1.4134053829795228, v_out_ptr->d[13], 1e-9);
  EXPECT_NEAR(-0.1323896021761943, v_out_ptr->d[14], 1e-9);
  EXPECT_NEAR(0.2542943762919667, v_out_ptr->d[15], 1e-9);
  EXPECT_NEAR(-1.0860676074826507, v_out_ptr->d[16], 1e-9);
  EXPECT_NEAR(0.0301581572946961, v_out_ptr->d[17], 1e-9);
  EXPECT_NEAR(0.3607778804851680, v_out_ptr->d[18], 1e-9);
  EXPECT_NEAR(1.0126389434981067, v_out_ptr->d[19], 1e-9);
  EXPECT_NEAR(-1.2811996868010367, v_out_ptr->d[20], 1e-9);
  EXPECT_NEAR(0.3313400905278427, v_out_ptr->d[21], 1e-9);
}

TEST(VecLinComb, Normal3) {
  double d_v0[22] = {
      -0.2546758907651276, -0.3450568364528703, 0.6196509543058106,
      -0.1426559762558552, -0.9669037767104007, -0.3838818620123632,
      0.6017106180217549,  -0.5983296597751775, 0.2788989695395412,
      0.6255106490705395,  0.9774699450579134,  0.8610684119754286,
      0.8694007192219890,  0.0762356582174983,  -0.5988481342705065,
      0.8963891334930463,  -0.3799845733877527, -0.9281861104671731,
      0.7460652435654154,  -0.9997652786417290, 0.8570964924716704,
      0.3499203249183473};
  const Vec v0 = {22, &d_v0[0], 0, 22};
  double d_v1[22] = {
      0.0066265054475780,  0.8755492097201634,  -0.1990267019469585,
      0.8550309242257355,  0.2567825063477913,  0.7095981804947977,
      -0.8557549479982967, 0.4206967915512552,  -0.6539353521661624,
      0.1786154499414507,  0.4277092464882020,  -0.3632365982959869,
      0.5662873013060699,  -0.4741518531657776, 0.6898690415196225,
      -0.0425132650383964, -0.2077779692885229, -0.7659980925071974,
      -0.7048079908141596, -0.6984183735958076, -0.7337342763451460,
      -0.2727409267797585};
  const Vec v1 = {22, &d_v1[0], 0, 22};
  double d_v_out[22] = {
      -0.5821172137311839, 0.5003114109407243,  0.5415422939924490,
      -0.3608205815716192, 0.2568876140612077,  -0.1897919399906867,
      -0.2323179107372433, -0.5576028221657212, 0.5052940080867308,
      0.7793685525938296,  -0.5735947531081560, 0.8978032779394896,
      -0.8738325431669869, 0.8912994405452330,  0.9844155588767791,
      0.9108494076867282,  -0.4654028215431572, -0.3308609400685438,
      -0.9879708752173813, 0.8583057975427155,  -0.1938485170365383,
      0.6692728927693632};
  Vec v_out = {22, &d_v_out[0], 0, 22};
  const Vec *v_out_ptr =
      VecLinComb(0.9817550822339103, &v0, -0.3103284424823467, &v1, &v_out);

  EXPECT_NEAR(-0.2520857431957599, v_out.d[0], 1e-9);
  EXPECT_NEAR(-0.6104691254162684, v_out.d[1], 1e-9);
  EXPECT_NEAR(0.6701091200284200, v_out.d[2], 1e-9);
  EXPECT_NEAR(-0.4053936446894398, v_out.d[3], 1e-9);
  EXPECT_NEAR(-1.0289496120682213, v_out.d[4], 1e-9);
  EXPECT_NEAR(-0.5970864671493119, v_out.d[5], 1e-9);
  EXPECT_NEAR(0.8562975574358378, v_out.d[6], 1e-9);
  EXPECT_NEAR(-0.7179673644149884, v_out.d[7], 1e-9);
  EXPECT_NEAR(0.4767452200971152, v_out.d[8], 1e-9);
  EXPECT_NEAR(0.5586688043328202, v_out.d[9], 1e-9);
  EXPECT_NEAR(0.8269057419935257, v_out.d[10], 1e-9);
  EXPECT_NEAR(0.9580809374097389, v_out.d[11], 1e-9);
  EXPECT_NEAR(0.6778035183821606, v_out.d[12], 1e-9);
  EXPECT_NEAR(0.2219875509955305, v_out.d[13], 1e-9);
  EXPECT_NEAR(-0.8020081844779388, v_out.d[14], 1e-9);
  EXPECT_NEAR(0.8932276627902540, v_out.d[15], 1e-9);
  EXPECT_NEAR(-0.3085723725024583, v_out.d[16], 1e-9);
  EXPECT_NEAR(-0.6735404362178659, v_out.d[17], 1e-9);
  EXPECT_NEAR(0.9511753105868970, v_out.d[18], 1e-9);
  EXPECT_NEAR(-0.7647855572684782, v_out.d[19], 1e-9);
  EXPECT_NEAR(1.0691574526230216, v_out.d[20], 1e-9);
  EXPECT_NEAR(0.4281753243742830, v_out.d[21], 1e-9);

  EXPECT_NEAR(-0.2520857431957599, v_out_ptr->d[0], 1e-9);
  EXPECT_NEAR(-0.6104691254162684, v_out_ptr->d[1], 1e-9);
  EXPECT_NEAR(0.6701091200284200, v_out_ptr->d[2], 1e-9);
  EXPECT_NEAR(-0.4053936446894398, v_out_ptr->d[3], 1e-9);
  EXPECT_NEAR(-1.0289496120682213, v_out_ptr->d[4], 1e-9);
  EXPECT_NEAR(-0.5970864671493119, v_out_ptr->d[5], 1e-9);
  EXPECT_NEAR(0.8562975574358378, v_out_ptr->d[6], 1e-9);
  EXPECT_NEAR(-0.7179673644149884, v_out_ptr->d[7], 1e-9);
  EXPECT_NEAR(0.4767452200971152, v_out_ptr->d[8], 1e-9);
  EXPECT_NEAR(0.5586688043328202, v_out_ptr->d[9], 1e-9);
  EXPECT_NEAR(0.8269057419935257, v_out_ptr->d[10], 1e-9);
  EXPECT_NEAR(0.9580809374097389, v_out_ptr->d[11], 1e-9);
  EXPECT_NEAR(0.6778035183821606, v_out_ptr->d[12], 1e-9);
  EXPECT_NEAR(0.2219875509955305, v_out_ptr->d[13], 1e-9);
  EXPECT_NEAR(-0.8020081844779388, v_out_ptr->d[14], 1e-9);
  EXPECT_NEAR(0.8932276627902540, v_out_ptr->d[15], 1e-9);
  EXPECT_NEAR(-0.3085723725024583, v_out_ptr->d[16], 1e-9);
  EXPECT_NEAR(-0.6735404362178659, v_out_ptr->d[17], 1e-9);
  EXPECT_NEAR(0.9511753105868970, v_out_ptr->d[18], 1e-9);
  EXPECT_NEAR(-0.7647855572684782, v_out_ptr->d[19], 1e-9);
  EXPECT_NEAR(1.0691574526230216, v_out_ptr->d[20], 1e-9);
  EXPECT_NEAR(0.4281753243742830, v_out_ptr->d[21], 1e-9);
}

TEST(VecLinComb3, Normal3) {
  double d_v0[22] = {
      -0.9135249149624691, 0.7257252752841252,  -0.3549947713199082,
      0.7398554403311099,  -0.1740218165119618, -0.4648734790850455,
      0.1368778747829364,  -0.1909752975003844, -0.5295122853794691,
      0.4484936053341859,  0.6997935381694302,  -0.3566405592988697,
      -0.7848833400331017, 0.8528331192070229,  0.6742369871227898,
      -0.9849472133967649, 0.2149840027498287,  -0.1502379639188274,
      0.8544028489064766,  -0.6894011132876126, 0.0089960751980342,
      0.2432484587866901};
  const Vec v0 = {22, &d_v0[0], 0, 22};
  double d_v1[22] = {
      0.4257682991414249,  0.7952265343336373,  -0.6430889771733812,
      -0.5613097167199070, 0.8328195152302480,  0.2018834673729264,
      -0.0711698204366213, 0.5246089960737454,  0.5177067180307189,
      0.7341609439593113,  0.6829008578768252,  0.9983779183614392,
      0.3945894902672786,  0.4852045432166532,  -0.3670048513029052,
      -0.1735341363914757, -0.3422556928968137, 0.1311767146103733,
      0.7430265725366509,  0.3561297939584787,  0.3420277905085189,
      -0.9867874698814161};
  const Vec v1 = {22, &d_v1[0], 0, 22};
  double d_v2[22] = {
      -0.8935125901334866, 0.7544742875047181,  -0.0973162424213920,
      0.2202987481367391,  0.1124475685625832,  -0.6548314123345518,
      -0.0086317451795681, 0.0147115955659718,  0.0068483151660319,
      0.7816236714696858,  0.7936302358387870,  0.1467842407065463,
      -0.9113242183637216, -0.6712419366055846, 0.2335585645062992,
      0.3290432232434917,  -0.8578321987045006, -0.1290858755831279,
      -0.9203880989798663, -0.7737197344024416, 0.9998089539192976,
      -0.6370367069788818};
  const Vec v2 = {22, &d_v2[0], 0, 22};
  double d_v_out[22] = {
      0.7580794178326602,  0.4264013913211424,  -0.7804084699678975,
      -0.1311768273397793, 0.0733584347132794,  0.5332480347844235,
      -0.7904799596754584, -0.9056828098986094, 0.1434248436238714,
      0.8301356775407849,  -0.3344661840275676, 0.0474288901481563,
      0.3337308164725683,  0.9123598413300342,  0.4473502588900613,
      0.8861917631052856,  -0.8727397929083669, 0.7072108435140081,
      -0.9043258497340436, -0.7645814232667507, 0.7782009635556946,
      -0.6594139879781191};
  Vec v_out = {22, &d_v_out[0], 0, 22};
  const Vec *v_out_ptr =
      VecLinComb3(-0.9376456790571417, &v0, -0.5226982280948396, &v1,
                  -0.9787472846365177, &v2, &v_out);

  EXPECT_NEAR(1.5085373750671174, v_out.d[0], 1e-9);
  EXPECT_NEAR(-1.8345763292062214, v_out.d[1], 1e-9);
  EXPECT_NEAR(0.7642487903128057, v_out.d[2], 1e-9);
  EXPECT_NEAR(-0.6159434639591299, v_out.d[3], 1e-9);
  EXPECT_NEAR(-0.3822002330141451, v_out.d[4], 1e-9);
  EXPECT_NEAR(0.9712769450120209, v_out.d[5], 1e-9);
  EXPECT_NEAR(-0.0826943116565183, v_out.d[6], 1e-9);
  EXPECT_NEAR(-0.1095439643953373, v_out.d[7], 1e-9);
  EXPECT_NEAR(0.2191877523331910, v_out.d[8], 1e-9);
  EXPECT_NEAR(-1.5692847617289059, v_out.d[9], 1e-9);
  EXPECT_NEAR(-1.7898728940059896, v_out.d[10], 1e-9);
  EXPECT_NEAR(-0.3311125665123963, v_out.d[11], 1e-9);
  EXPECT_NEAR(1.4216473491054120, v_out.d[12], 1e-9);
  EXPECT_NEAR(-0.3962946213973263, v_out.d[13], 1e-9);
  EXPECT_NEAR(-0.6689574229720943, v_out.d[14], 1e-9);
  EXPECT_NEAR(0.6921873230690267, v_out.d[15], 1e-9);
  EXPECT_NEAR(0.8169185581435542, v_out.d[16], 1e-9);
  EXPECT_NEAR(0.1986465916166005, v_out.d[17], 1e-9);
  EXPECT_NEAR(-0.2886784596552483, v_out.d[18], 1e-9);
  EXPECT_NEAR(1.2175416518535132, v_out.d[19], 1e-9);
  EXPECT_NEAR(-1.1657727498996957, v_out.d[20], 1e-9);
  EXPECT_NEAR(0.9112091428639264, v_out.d[21], 1e-9);

  EXPECT_NEAR(1.5085373750671174, v_out_ptr->d[0], 1e-9);
  EXPECT_NEAR(-1.8345763292062214, v_out_ptr->d[1], 1e-9);
  EXPECT_NEAR(0.7642487903128057, v_out_ptr->d[2], 1e-9);
  EXPECT_NEAR(-0.6159434639591299, v_out_ptr->d[3], 1e-9);
  EXPECT_NEAR(-0.3822002330141451, v_out_ptr->d[4], 1e-9);
  EXPECT_NEAR(0.9712769450120209, v_out_ptr->d[5], 1e-9);
  EXPECT_NEAR(-0.0826943116565183, v_out_ptr->d[6], 1e-9);
  EXPECT_NEAR(-0.1095439643953373, v_out_ptr->d[7], 1e-9);
  EXPECT_NEAR(0.2191877523331910, v_out_ptr->d[8], 1e-9);
  EXPECT_NEAR(-1.5692847617289059, v_out_ptr->d[9], 1e-9);
  EXPECT_NEAR(-1.7898728940059896, v_out_ptr->d[10], 1e-9);
  EXPECT_NEAR(-0.3311125665123963, v_out_ptr->d[11], 1e-9);
  EXPECT_NEAR(1.4216473491054120, v_out_ptr->d[12], 1e-9);
  EXPECT_NEAR(-0.3962946213973263, v_out_ptr->d[13], 1e-9);
  EXPECT_NEAR(-0.6689574229720943, v_out_ptr->d[14], 1e-9);
  EXPECT_NEAR(0.6921873230690267, v_out_ptr->d[15], 1e-9);
  EXPECT_NEAR(0.8169185581435542, v_out_ptr->d[16], 1e-9);
  EXPECT_NEAR(0.1986465916166005, v_out_ptr->d[17], 1e-9);
  EXPECT_NEAR(-0.2886784596552483, v_out_ptr->d[18], 1e-9);
  EXPECT_NEAR(1.2175416518535132, v_out_ptr->d[19], 1e-9);
  EXPECT_NEAR(-1.1657727498996957, v_out_ptr->d[20], 1e-9);
  EXPECT_NEAR(0.9112091428639264, v_out_ptr->d[21], 1e-9);
}

TEST(VecDot, Normal3) {
  double d_v0[22] = {
      0.3784640351552149,  -0.0435540919377739, -0.9933388414904851,
      0.9549240218581962,  -0.9733076889711267, -0.6120405291546298,
      0.3474771705383828,  0.0774432147317772,  -0.0118402634776640,
      0.6963120065640798,  -0.7263963064473413, 0.3706255245206242,
      -0.8583736330570084, -0.5159344990238994, -0.4011593976840282,
      0.2840546434301676,  0.3393439238765741,  -0.2334820920159404,
      -0.8877232465961318, 0.2613587392493000,  -0.9165995169833414,
      0.4830401037156167};
  const Vec v0 = {22, &d_v0[0], 0, 22};
  double d_v1[22] = {
      -0.5415880406464073, -0.5674406404851284, -0.4637524658552958,
      0.1246396933376990,  0.2390732060256120,  0.2239945517999371,
      -0.4489659363916512, 0.6880908455697006,  -0.2319944682792190,
      0.2824842285563374,  0.3844308467821393,  -0.1539937205466881,
      0.7996493359438313,  0.9481554166731301,  -0.0051087153966600,
      -0.4523044227688546, 0.0930298295718273,  0.4251645407942539,
      0.3193579347660636,  0.1884729221074122,  -0.5654449704541045,
      -0.9846944251200980};
  const Vec v1 = {22, &d_v1[0], 0, 22};
  double v_dot = VecDot(&v0, &v1);

  EXPECT_NEAR(-1.7712702385591599, v_dot, 1e-9);
}

TEST(VecMult, Normal3) {
  double d_v0[22] = {
      -0.6704457107070751, -0.4246559547685449, 0.6520357142002147,
      -0.8820289406904060, 0.8558807004184910,  0.0473870050963132,
      -0.6504197975908903, -0.3536801560033955, 0.0867216431927567,
      -0.4160408370483861, -0.8327246323405437, 0.1153350233166208,
      0.5051534013653307,  -0.4499800957852127, -0.8268038914987141,
      -0.1908696126263900, -0.6656492021659237, 0.9784212702648944,
      -0.8352041606105456, -0.9590144359189827, 0.5106874313086625,
      0.7221334737822780};
  const Vec v0 = {22, &d_v0[0], 0, 22};
  double d_v1[22] = {
      0.3754104767905442,  0.9980211620158148,  0.8814724577150059,
      0.9824687222710027,  0.9036864074035296,  -0.3621604509148428,
      -0.3610934919499715, -0.1682882113678223, -0.1201663295150497,
      -0.6114221478835926, 0.6524097172500911,  -0.2475283371741537,
      0.6515616834724258,  0.4231365781268710,  -0.3179433616602723,
      -0.5414440610723601, -0.9047789621445452, -0.7050586585456948,
      -0.9404032975534320, -0.1781360314194576, 0.9253218633454419,
      -0.2016723997758005};
  const Vec v1 = {22, &d_v1[0], 0, 22};
  double d_v_out[22] = {
      -0.8158399461636343, -0.3240597534544443, -0.5256177985890369,
      0.7434415662858751,  -0.5604447555886014, 0.6233925599186674,
      0.5607536433886664,  -0.7298811425557814, 0.9779069131818043,
      0.1523435468206760,  -0.9340047718573692, -0.2640376975941507,
      0.6254299461158728,  -0.4642512577444522, 0.4141764825514924,
      0.4770013628034642,  0.8116790000234004,  0.6881845817199630,
      -0.2767369977059986, 0.0343775528365018,  0.1629312959728475,
      -0.0885853546764193};
  Vec v_out = {22, &d_v_out[0], 0, 22};
  const Vec *v_out_ptr = VecMult(&v0, &v1, &v_out);

  EXPECT_NEAR(-0.2516923439187183, v_out.d[0], 1e-9);
  EXPECT_NEAR(-0.4238156294350385, v_out.d[1], 1e-9);
  EXPECT_NEAR(0.5747515235140224, v_out.d[2], 1e-9);
  EXPECT_NEAR(-0.8665658463661492, v_out.d[3], 1e-9);
  EXPECT_NEAR(0.7734477553272027, v_out.d[4], 1e-9);
  EXPECT_NEAR(-0.0171616991331848, v_out.d[5], 1e-9);
  EXPECT_NEAR(0.2348623559454883, v_out.d[6], 1e-9);
  EXPECT_NEAR(0.0595202008501038, v_out.d[7], 1e-9);
  EXPECT_NEAR(-0.0104210215519874, v_out.d[8], 1e-9);
  EXPECT_NEAR(0.2543765821954120, v_out.d[9], 1e-9);
  EXPECT_NEAR(-0.5432776419324802, v_out.d[10], 1e-9);
  EXPECT_NEAR(-0.0285486865395054, v_out.d[11], 1e-9);
  EXPECT_NEAR(0.3291386006054169, v_out.d[12], 1e-9);
  EXPECT_NEAR(-0.1904030379557566, v_out.d[13], 1e-9);
  EXPECT_NEAR(0.2628768086968962, v_out.d[14], 1e-9);
  EXPECT_NEAR(0.1033452181957408, v_out.d[15], 1e-9);
  EXPECT_NEAR(0.6022653942880291, v_out.d[16], 1e-9);
  EXPECT_NEAR(-0.6898443883055412, v_out.d[17], 1e-9);
  EXPECT_NEAR(0.7854287467685033, v_out.d[18], 1e-9);
  EXPECT_NEAR(0.1708350256885773, v_out.d[19], 1e-9);
  EXPECT_NEAR(0.4725502455256289, v_out.d[20], 1e-9);
  EXPECT_NEAR(-0.1456343906161071, v_out.d[21], 1e-9);

  EXPECT_NEAR(-0.2516923439187183, v_out_ptr->d[0], 1e-9);
  EXPECT_NEAR(-0.4238156294350385, v_out_ptr->d[1], 1e-9);
  EXPECT_NEAR(0.5747515235140224, v_out_ptr->d[2], 1e-9);
  EXPECT_NEAR(-0.8665658463661492, v_out_ptr->d[3], 1e-9);
  EXPECT_NEAR(0.7734477553272027, v_out_ptr->d[4], 1e-9);
  EXPECT_NEAR(-0.0171616991331848, v_out_ptr->d[5], 1e-9);
  EXPECT_NEAR(0.2348623559454883, v_out_ptr->d[6], 1e-9);
  EXPECT_NEAR(0.0595202008501038, v_out_ptr->d[7], 1e-9);
  EXPECT_NEAR(-0.0104210215519874, v_out_ptr->d[8], 1e-9);
  EXPECT_NEAR(0.2543765821954120, v_out_ptr->d[9], 1e-9);
  EXPECT_NEAR(-0.5432776419324802, v_out_ptr->d[10], 1e-9);
  EXPECT_NEAR(-0.0285486865395054, v_out_ptr->d[11], 1e-9);
  EXPECT_NEAR(0.3291386006054169, v_out_ptr->d[12], 1e-9);
  EXPECT_NEAR(-0.1904030379557566, v_out_ptr->d[13], 1e-9);
  EXPECT_NEAR(0.2628768086968962, v_out_ptr->d[14], 1e-9);
  EXPECT_NEAR(0.1033452181957408, v_out_ptr->d[15], 1e-9);
  EXPECT_NEAR(0.6022653942880291, v_out_ptr->d[16], 1e-9);
  EXPECT_NEAR(-0.6898443883055412, v_out_ptr->d[17], 1e-9);
  EXPECT_NEAR(0.7854287467685033, v_out_ptr->d[18], 1e-9);
  EXPECT_NEAR(0.1708350256885773, v_out_ptr->d[19], 1e-9);
  EXPECT_NEAR(0.4725502455256289, v_out_ptr->d[20], 1e-9);
  EXPECT_NEAR(-0.1456343906161071, v_out_ptr->d[21], 1e-9);
}

TEST(MatArrMult, Normal0) {
  double m1[2][11] = {
      {0.9409693656406166, 0.5298131533238114, 0.0162663959650701,
       0.0291384140157863, 0.4889891885153336, 0.5237378659339506,
       0.9808261772234459, 0.8053448746586258, 0.1022479631976341,
       0.5218357884538594, 0.5259544137228005},
      {0.6206214518170813, 0.7576624902736504, 0.2487860670975326,
       0.5034047986949194, 0.9245599274217495, 0.6113528534578406,
       0.8371799429603878, 0.5156759079938488, 0.1774812216678331,
       0.4381633022369922, 0.4106683928060328}};
  double m2[11][2] = {{0.6918503464482521, 0.7484622738513611},
                      {0.4367917859198486, 0.8414515416806464},
                      {0.8829062485939624, 0.6902038262835815},
                      {0.0391810671752375, 0.0778604122928220},
                      {0.3305305328665571, 0.4580618903912170},
                      {0.4378088581393267, 0.6279081633159499},
                      {0.7320339084197404, 0.1562645891279174},
                      {0.5434053272824945, 0.1867968811438127},
                      {0.0684315897873724, 0.1267612422358735},
                      {0.6741097387065410, 0.1655457757582913},
                      {0.5747558546141021, 0.2452754242592510}};
  double m_out[2][2];
  double m_out2[2][2];
  Mat mn_m1 = {2, 11, &m1[0][0], 0, 0};
  Mat mn_m2 = {11, 2, &m2[0][0], 0, 0};
  Mat mn_m_out = {2, 2, &m_out2[0][0], 0, 0};
  MatArrMult(&m1[0][0], 2, 11, &m2[0][0], 2, &m_out[0][0]);
  MatMult(&mn_m1, &mn_m2, &mn_m_out);

  EXPECT_NEAR(m_out[0][0], 3.1055479973574291, 1e-9);
  EXPECT_NEAR(m_out2[0][0], 3.1055479973574291, 1e-9);
  EXPECT_NEAR(m_out[0][1], 2.2484914085294743, 1e-9);
  EXPECT_NEAR(m_out2[0][1], 2.2484914085294743, 1e-9);
  EXPECT_NEAR(m_out[1][0], 3.0095622849710186, 1e-9);
  EXPECT_NEAR(m_out2[1][0], 3.0095622849710186, 1e-9);
  EXPECT_NEAR(m_out[1][1], 2.5432444489353396, 1e-9);
  EXPECT_NEAR(m_out2[1][1], 2.5432444489353396, 1e-9);
}

#if !defined(NDEBUG)

TEST(MatArrMultDeath, Death0) {
  double m1[6][6] = {
      {0.2044689753479982, 0.7477855922072102, 0.0789313305486246,
       0.0399122767375300, 0.9673004220475294, 0.3619685774888184},
      {0.3815591182895898, 0.4854516528812912, 0.8331584189610883,
       0.9698745244713570, 0.9169168491773247, 0.6394074567634512},
      {0.2879578834956538, 0.3613456607273541, 0.2384493579436259,
       0.5371433378396429, 0.8973684825766124, 0.0181738862493132},
      {0.1437412835478459, 0.0924155144363837, 0.7940920087192628,
       0.3294839409595641, 0.8369834397780841, 0.5665857616712735},
      {0.3577438031574942, 0.7188183904453234, 0.8241116832760657,
       0.5972317457209153, 0.0221469489160384, 0.9009895892789334},
      {0.2781045242691418, 0.4689748591466979, 0.1371588919809412,
       0.7769788984851503, 0.7783579148932923, 0.8061985879212249}};
  double m_out[6][6] = {
      {0.7646464936232430, 0.7785104880405235, 0.2436559510721658,
       0.0629376271749953, 0.6238952760436013, 0.0241671096001469},
      {0.2607313347974086, 0.9292086268921317, 0.6157729868369749,
       0.8847169830528713, 0.3938013545332600, 0.4531212761259668},
      {0.2809455196061696, 0.0933434373271266, 0.6058295062561649,
       0.6926728983049846, 0.3604168521114390, 0.7698068330947090},
      {0.0351259301216953, 0.0701607056224521, 0.0241927117742406,
       0.1114057221456457, 0.9707525858446286, 0.8879439391208680},
      {0.7104161932925108, 0.3273865156187701, 0.8116273682782537,
       0.2805867991358953, 0.1071202354731567, 0.5512080072935219},
      {0.9775953430707374, 0.9243303849787662, 0.0457090040202834,
       0.7279209630997991, 0.6508733772461585, 0.7217979438983840}};
  ASSERT_DEATH(MatArrMult(&m1[0][0], 6, 6, &m_out[0][0], 6, &m_out[0][0]),
               "MatArrMult: Assertion `m_out != m1 && m_out != m2' failed.");
  ASSERT_DEATH(MatArrMult(&m_out[0][0], 6, 6, &m1[0][0], 6, &m_out[0][0]),
               "MatArrMult: Assertion `m_out != m1 && m_out != m2' failed.");
}

#endif  // !defined(NDEBUG)

TEST(MatArrZero, Normal0) {
  double m[4][6] = {
      {0.3536193190220163, 0.8203205111159658, 0.3275656976880676,
       0.7310289122607063, 0.7236888819648067, 0.8836024579116939},
      {0.6422384081161282, 0.8838566300062021, 0.7550430196169428,
       0.5161626138289219, 0.0286962654665822, 0.0475988723520224},
      {0.9146531236171459, 0.5495380618100684, 0.6453450735207763,
       0.7888282551473751, 0.4206053876926352, 0.6928650158951463},
      {0.7993633324538135, 0.1092268446553981, 0.6876128642217056,
       0.6460955000790455, 0.6865773423718708, 0.1209942561654850}};
  MatArrZero(4, 6, &m[0][0]);

  EXPECT_NEAR(m[0][0], 0.0, 1e-9);
  EXPECT_NEAR(m[0][1], 0.0, 1e-9);
  EXPECT_NEAR(m[0][2], 0.0, 1e-9);
  EXPECT_NEAR(m[0][3], 0.0, 1e-9);
  EXPECT_NEAR(m[0][4], 0.0, 1e-9);
  EXPECT_NEAR(m[0][5], 0.0, 1e-9);
  EXPECT_NEAR(m[1][0], 0.0, 1e-9);
  EXPECT_NEAR(m[1][1], 0.0, 1e-9);
  EXPECT_NEAR(m[1][2], 0.0, 1e-9);
  EXPECT_NEAR(m[1][3], 0.0, 1e-9);
  EXPECT_NEAR(m[1][4], 0.0, 1e-9);
  EXPECT_NEAR(m[1][5], 0.0, 1e-9);
  EXPECT_NEAR(m[2][0], 0.0, 1e-9);
  EXPECT_NEAR(m[2][1], 0.0, 1e-9);
  EXPECT_NEAR(m[2][2], 0.0, 1e-9);
  EXPECT_NEAR(m[2][3], 0.0, 1e-9);
  EXPECT_NEAR(m[2][4], 0.0, 1e-9);
  EXPECT_NEAR(m[2][5], 0.0, 1e-9);
  EXPECT_NEAR(m[3][0], 0.0, 1e-9);
  EXPECT_NEAR(m[3][1], 0.0, 1e-9);
  EXPECT_NEAR(m[3][2], 0.0, 1e-9);
  EXPECT_NEAR(m[3][3], 0.0, 1e-9);
  EXPECT_NEAR(m[3][4], 0.0, 1e-9);
  EXPECT_NEAR(m[3][5], 0.0, 1e-9);
}

TEST(MatArrCopy, Normal0) {
  double m[7][8] = {{0.9594977738540310, 0.6765246424046368, 0.6550203713439761,
                     0.3630523471993591, 0.5593862171062011, 0.0781867944210712,
                     0.5651944771004247, 0.0016302822527985},
                    {0.3278908432489985, 0.2086925742455817, 0.7072060577147399,
                     0.5180780759757477, 0.6880753585612164, 0.0802893713877700,
                     0.0929359650108125, 0.9128559887699869},
                    {0.2954360971838792, 0.4903577276202313, 0.9533070696021037,
                     0.7763931576545048, 0.3397219233096723, 0.0673341057785711,
                     0.5151251948099937, 0.7200891593372469},
                    {0.4265303126307951, 0.5707536389932085, 0.2129507555084448,
                     0.0961004380979178, 0.6984989593131185, 0.6856325378194734,
                     0.7967782024563372, 0.8218704254305520},
                    {0.5124509131218602, 0.6114801827671819, 0.5717316948043537,
                     0.9474868487614270, 0.3706052821013310, 0.0543517400997932,
                     0.7539780176768691, 0.6172732515254585},
                    {0.1006687612303563, 0.0047666340070460, 0.1442353310517963,
                     0.6421776740784934, 0.7898558745083831, 0.6487406048266543,
                     0.8703189258046322, 0.0394208099999783},
                    {0.0829889418104792, 0.7739213137505642, 0.3519370109254136,
                     0.8113125347207004, 0.7416587631659751, 0.8425274313506755,
                     0.2794329670744012, 0.6117320342423328}};
  double m_out[7][8];
  MatArrCopy(&m[0][0], 7, 8, &m_out[0][0]);
  MatArrCopy(&m[0][0], 7, 8, &m[0][0]);

  EXPECT_NEAR(m_out[0][0], m[0][0], 1e-9);
  EXPECT_NEAR(m_out[0][1], m[0][1], 1e-9);
  EXPECT_NEAR(m_out[0][2], m[0][2], 1e-9);
  EXPECT_NEAR(m_out[0][3], m[0][3], 1e-9);
  EXPECT_NEAR(m_out[0][4], m[0][4], 1e-9);
  EXPECT_NEAR(m_out[0][5], m[0][5], 1e-9);
  EXPECT_NEAR(m_out[0][6], m[0][6], 1e-9);
  EXPECT_NEAR(m_out[0][7], m[0][7], 1e-9);
  EXPECT_NEAR(m_out[1][0], m[1][0], 1e-9);
  EXPECT_NEAR(m_out[1][1], m[1][1], 1e-9);
  EXPECT_NEAR(m_out[1][2], m[1][2], 1e-9);
  EXPECT_NEAR(m_out[1][3], m[1][3], 1e-9);
  EXPECT_NEAR(m_out[1][4], m[1][4], 1e-9);
  EXPECT_NEAR(m_out[1][5], m[1][5], 1e-9);
  EXPECT_NEAR(m_out[1][6], m[1][6], 1e-9);
  EXPECT_NEAR(m_out[1][7], m[1][7], 1e-9);
  EXPECT_NEAR(m_out[2][0], m[2][0], 1e-9);
  EXPECT_NEAR(m_out[2][1], m[2][1], 1e-9);
  EXPECT_NEAR(m_out[2][2], m[2][2], 1e-9);
  EXPECT_NEAR(m_out[2][3], m[2][3], 1e-9);
  EXPECT_NEAR(m_out[2][4], m[2][4], 1e-9);
  EXPECT_NEAR(m_out[2][5], m[2][5], 1e-9);
  EXPECT_NEAR(m_out[2][6], m[2][6], 1e-9);
  EXPECT_NEAR(m_out[2][7], m[2][7], 1e-9);
  EXPECT_NEAR(m_out[3][0], m[3][0], 1e-9);
  EXPECT_NEAR(m_out[3][1], m[3][1], 1e-9);
  EXPECT_NEAR(m_out[3][2], m[3][2], 1e-9);
  EXPECT_NEAR(m_out[3][3], m[3][3], 1e-9);
  EXPECT_NEAR(m_out[3][4], m[3][4], 1e-9);
  EXPECT_NEAR(m_out[3][5], m[3][5], 1e-9);
  EXPECT_NEAR(m_out[3][6], m[3][6], 1e-9);
  EXPECT_NEAR(m_out[3][7], m[3][7], 1e-9);
  EXPECT_NEAR(m_out[4][0], m[4][0], 1e-9);
  EXPECT_NEAR(m_out[4][1], m[4][1], 1e-9);
  EXPECT_NEAR(m_out[4][2], m[4][2], 1e-9);
  EXPECT_NEAR(m_out[4][3], m[4][3], 1e-9);
  EXPECT_NEAR(m_out[4][4], m[4][4], 1e-9);
  EXPECT_NEAR(m_out[4][5], m[4][5], 1e-9);
  EXPECT_NEAR(m_out[4][6], m[4][6], 1e-9);
  EXPECT_NEAR(m_out[4][7], m[4][7], 1e-9);
  EXPECT_NEAR(m_out[5][0], m[5][0], 1e-9);
  EXPECT_NEAR(m_out[5][1], m[5][1], 1e-9);
  EXPECT_NEAR(m_out[5][2], m[5][2], 1e-9);
  EXPECT_NEAR(m_out[5][3], m[5][3], 1e-9);
  EXPECT_NEAR(m_out[5][4], m[5][4], 1e-9);
  EXPECT_NEAR(m_out[5][5], m[5][5], 1e-9);
  EXPECT_NEAR(m_out[5][6], m[5][6], 1e-9);
  EXPECT_NEAR(m_out[5][7], m[5][7], 1e-9);
  EXPECT_NEAR(m_out[6][0], m[6][0], 1e-9);
  EXPECT_NEAR(m_out[6][1], m[6][1], 1e-9);
  EXPECT_NEAR(m_out[6][2], m[6][2], 1e-9);
  EXPECT_NEAR(m_out[6][3], m[6][3], 1e-9);
  EXPECT_NEAR(m_out[6][4], m[6][4], 1e-9);
  EXPECT_NEAR(m_out[6][5], m[6][5], 1e-9);
  EXPECT_NEAR(m_out[6][6], m[6][6], 1e-9);
  EXPECT_NEAR(m_out[6][7], m[6][7], 1e-9);
}

TEST(MatVecMult, Normal0) {
  double m[4][7] = {{0.7774381052477454, 0.6960767594945834, 0.2473816124570860,
                     0.6414380429495570, 0.9500737662997737, 0.4608893079069479,
                     0.9095105721516976},
                    {0.0753964259969709, 0.6266149932075620, 0.2262112653328794,
                     0.7260974302367694, 0.3702012762506678, 0.5308289465360573,
                     0.7407433891553935},
                    {0.2439417083085346, 0.6129856066957758, 0.6088410687701543,
                     0.2451820644606624, 0.6090965319558468, 0.5149946073661903,
                     0.3485854975273455},
                    {0.5999480273197119, 0.0253827602360220, 0.0712445649964365,
                     0.8357870990873000, 0.7276408796113387, 0.2826859420401815,
                     0.5884062227180841}};
  double v[7] = {0.3757507571548225, 0.2032135128393781, 0.2925798754902746,
                 0.8141300072007867, 0.2258779994128911, 0.6950724674032522,
                 0.4106859465700258};
  double v_out[4];
  Mat mn_m = {4, 7, &m[0][0], 0, 0};
  Vec n_v = {7, v, 0, 0};
  Vec n_v_out = {4, v_out, 0, 0};
  MatVecMult(&mn_m, &n_v, &n_v_out);

  EXPECT_NEAR(v_out[0], 1.9366434403999437, 1e-9);
  EXPECT_NEAR(v_out[1], 1.5697872773136443, 1e-9);
  EXPECT_NEAR(v_out[2], 1.2326722035957038, 1e-9);
  EXPECT_NEAR(v_out[3], 1.5343685763027268, 1e-9);
}

TEST(VecNorm, Normal7) {
  double d_v[22] = {
      -0.1541478427359033, 0.7461893847948071,  0.1396995920205566,
      -0.2998742251944020, 0.4297648101354175,  -0.6318610334979700,
      -0.3622577425861160, 0.2447367814967920,  -0.9731436586889552,
      -0.2093948167953263, 0.1271726956424502,  0.4968353732354358,
      -0.0366268257366418, -0.4357469432052288, -0.9673476842384383,
      -0.8508888091565536, 0.4330239908245956,  0.4961155586286909,
      -0.1657027326021321, 0.6015445038533522,  0.6378881040935342,
      -0.5738911407795115};
  const Vec v = {22, &d_v[0], 0, 22};
  double v_norm = VecNorm(&v);

  EXPECT_NEAR(2.4757105590032644, v_norm, 1e-9);
}

TEST(VecNormBound, Unbounded7) {
  double d_v[22] = {
      0.8331239567048156,  0.9278317443370458,  -0.0487106137772402,
      -0.2765600680235407, 0.1251194451811175,  -0.2682615874098442,
      0.2198861886354115,  0.0126082779977112,  0.6073752529349570,
      -0.9739361116084018, 0.2353023483537191,  -0.5918096530285701,
      -0.7082263119708145, 0.7663319751474258,  0.2683279089806279,
      -0.7474368562314486, -0.3014593445537446, -0.9092650785116621,
      0.2476755744789521,  0.1796957725881521,  -0.6951292484789806,
      0.9856991797385120};
  const Vec v = {22, &d_v[0], 0, 22};
  double v_norm = VecNormBound(&v, 2.7096457232131153);

  EXPECT_NEAR(2.7710302392385100, v_norm, 1e-9);
}

TEST(VecNormBound, Bounded7) {
  double d_v[22] = {
      -0.5570978400671667, -0.9919274799443534, -0.1507311563643792,
      0.2013628754520331,  -0.0867098552193386, 0.6683325698160529,
      -0.9231392978244002, 0.4155540470121639,  -0.3316357067195097,
      0.9795843994640328,  0.7977277620278165,  0.5242614250279445,
      -0.0422234101242835, -0.5295694187541280, 0.8479028735311325,
      -0.1299785984179835, -0.8539885287614328, 0.7405644718586546,
      -0.5877887322406290, -0.0218382341353895, 0.0956189301666133,
      0.2080214746333844};
  const Vec v = {22, &d_v[0], 0, 22};
  double v_norm = VecNormBound(&v, 3.6212317882930387);

  EXPECT_NEAR(3.6212317882930387, v_norm, 1e-9);
}

TEST(VecNormSquared, Normal7) {
  double d_v[22] = {
      -0.5568790760813438, -0.1772971276373567, -0.6337365372209542,
      -0.0333336358547727, -0.1473944292386207, -0.2636638914894178,
      0.3032838416357293,  -0.8352689973211527, 0.0427219430914512,
      0.5874713499274944,  0.9500741503201207,  0.4894724629367782,
      -0.7939857080426607, -0.4157433596625408, -0.2644817291747448,
      -0.0636948255652801, 0.3213676293447809,  -0.0668036883750160,
      -0.9948622466714208, 0.5974217016426617,  0.7215668423248280,
      -0.7107915215734624};
  const Vec v = {22, &d_v[0], 0, 22};
  double v_norm_sqrd = VecNormSquared(&v);

  EXPECT_NEAR(6.4719030152007493, v_norm_sqrd, 1e-9);
}

TEST(VecNormalize, Normal7) {
  double d_v[22] = {
      -0.7251729714222293, 0.7056031531256879,  0.7506570338880052,
      -0.6824140765337994, 0.1287708076937328,  0.4489000761793938,
      0.9619801033612647,  -0.9300199414897499, -0.6106443794165441,
      0.4890249962484916,  0.2955587500219259,  0.3728113058088776,
      -0.1016943365829570, -0.0602502027231895, -0.3750133733436822,
      0.8353101418479383,  0.0004892248893016,  -0.7869269419314553,
      -0.5199485496748060, -0.5504961262907320, -0.1567188806158275,
      -0.4180515834520107};
  Vec v = {22, &d_v[0], 0, 22};
  const Vec *v_out_ptr = VecNormalize(&v, &v);

  EXPECT_NEAR(-0.2713119911493942, v.d[0], 1e-9);
  EXPECT_NEAR(0.2639902533327554, v.d[1], 1e-9);
  EXPECT_NEAR(0.2808464498270322, v.d[2], 1e-9);
  EXPECT_NEAR(-0.2553144273009025, v.d[3], 1e-9);
  EXPECT_NEAR(0.0481775598569026, v.d[4], 1e-9);
  EXPECT_NEAR(0.1679488595065592, v.d[5], 1e-9);
  EXPECT_NEAR(0.3599096320112025, v.d[6], 1e-9);
  EXPECT_NEAR(-0.3479522432273769, v.d[7], 1e-9);
  EXPECT_NEAR(-0.2284629309042808, v.d[8], 1e-9);
  EXPECT_NEAR(0.1829609633599427, v.d[9], 1e-9);
  EXPECT_NEAR(0.1105786290032385, v.d[10], 1e-9);
  EXPECT_NEAR(0.1394814502030291, v.d[11], 1e-9);
  EXPECT_NEAR(-0.0380473266851448, v.d[12], 1e-9);
  EXPECT_NEAR(-0.0225416598689879, v.d[13], 1e-9);
  EXPECT_NEAR(-0.1403053189227106, v.d[14], 1e-9);
  EXPECT_NEAR(0.3125180705060956, v.d[15], 1e-9);
  EXPECT_NEAR(0.0001830357501824, v.d[16], 1e-9);
  EXPECT_NEAR(-0.2944162619378925, v.d[17], 1e-9);
  EXPECT_NEAR(-0.1945305214986768, v.d[18], 1e-9);
  EXPECT_NEAR(-0.2059594138637647, v.d[19], 1e-9);
  EXPECT_NEAR(-0.0586338890529711, v.d[20], 1e-9);
  EXPECT_NEAR(-0.1564073841404693, v.d[21], 1e-9);

  EXPECT_NEAR(-0.2713119911493942, v_out_ptr->d[0], 1e-9);
  EXPECT_NEAR(0.2639902533327554, v_out_ptr->d[1], 1e-9);
  EXPECT_NEAR(0.2808464498270322, v_out_ptr->d[2], 1e-9);
  EXPECT_NEAR(-0.2553144273009025, v_out_ptr->d[3], 1e-9);
  EXPECT_NEAR(0.0481775598569026, v_out_ptr->d[4], 1e-9);
  EXPECT_NEAR(0.1679488595065592, v_out_ptr->d[5], 1e-9);
  EXPECT_NEAR(0.3599096320112025, v_out_ptr->d[6], 1e-9);
  EXPECT_NEAR(-0.3479522432273769, v_out_ptr->d[7], 1e-9);
  EXPECT_NEAR(-0.2284629309042808, v_out_ptr->d[8], 1e-9);
  EXPECT_NEAR(0.1829609633599427, v_out_ptr->d[9], 1e-9);
  EXPECT_NEAR(0.1105786290032385, v_out_ptr->d[10], 1e-9);
  EXPECT_NEAR(0.1394814502030291, v_out_ptr->d[11], 1e-9);
  EXPECT_NEAR(-0.0380473266851448, v_out_ptr->d[12], 1e-9);
  EXPECT_NEAR(-0.0225416598689879, v_out_ptr->d[13], 1e-9);
  EXPECT_NEAR(-0.1403053189227106, v_out_ptr->d[14], 1e-9);
  EXPECT_NEAR(0.3125180705060956, v_out_ptr->d[15], 1e-9);
  EXPECT_NEAR(0.0001830357501824, v_out_ptr->d[16], 1e-9);
  EXPECT_NEAR(-0.2944162619378925, v_out_ptr->d[17], 1e-9);
  EXPECT_NEAR(-0.1945305214986768, v_out_ptr->d[18], 1e-9);
  EXPECT_NEAR(-0.2059594138637647, v_out_ptr->d[19], 1e-9);
  EXPECT_NEAR(-0.0586338890529711, v_out_ptr->d[20], 1e-9);
  EXPECT_NEAR(-0.1564073841404693, v_out_ptr->d[21], 1e-9);
}

TEST(VecNormalize, Zero_7) {
  double d_v[22] = {0.0000000000000000, 0.0000000000000000, 0.0000000000000000,
                    0.0000000000000000, 0.0000000000000000, 0.0000000000000000,
                    0.0000000000000000, 0.0000000000000000, 0.0000000000000000,
                    0.0000000000000000, 0.0000000000000000, 0.0000000000000000,
                    0.0000000000000000, 0.0000000000000000, 0.0000000000000000,
                    0.0000000000000000, 0.0000000000000000, 0.0000000000000000,
                    0.0000000000000000, 0.0000000000000000, 0.0000000000000000,
                    0.0000000000000000};
  Vec v = {22, &d_v[0], 0, 22};
  const Vec *v_out_ptr = VecNormalize(&v, &v);

  EXPECT_NEAR(0.0000000000000000, v.d[0], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v.d[1], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v.d[2], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v.d[3], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v.d[4], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v.d[5], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v.d[6], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v.d[7], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v.d[8], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v.d[9], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v.d[10], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v.d[11], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v.d[12], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v.d[13], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v.d[14], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v.d[15], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v.d[16], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v.d[17], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v.d[18], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v.d[19], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v.d[20], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v.d[21], 1e-9);

  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[0], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[1], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[2], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[3], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[4], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[5], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[6], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[7], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[8], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[9], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[10], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[11], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[12], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[13], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[14], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[15], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[16], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[17], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[18], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[19], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[20], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[21], 1e-9);
}

TEST(VecScale, Normal7) {
  double d_v[22] = {
      0.5160279159931480,  -0.3414411588115336, -0.0699745353568049,
      0.5253793451979449,  -0.2466755818772881, 0.2271149570396833,
      0.5927547135809661,  0.8938373109571449,  0.8192509594925887,
      0.5872732132667224,  -0.0540626069008117, -0.2187409273521177,
      0.5842016661049072,  0.4984343833004776,  0.3911055091451499,
      -0.2255626972698646, 0.6880186147380665,  0.2851844026521584,
      0.1931483233032196,  -0.2359890440036447, -0.4130255763717110,
      -0.4697510525164419};
  const Vec v = {22, &d_v[0], 0, 22};
  double d_v_out[22] = {
      0.6433208561991519,  -0.7722596374501982, -0.4855500147743155,
      0.1994285783790870,  0.5529670046817552,  -0.8987287073981007,
      -0.0778346373227190, -0.3469877424828298, -0.8678662446667034,
      -0.1938426250973626, 0.5634235580538283,  -0.2593068770135210,
      -0.9890127415945198, 0.9644743942848497,  0.2826615604708849,
      0.3900036027025244,  -0.6441883162112154, 0.0496434181684202,
      0.5045067414620963,  0.7043608436143884,  0.4593346251961885,
      0.4784865650002732};
  Vec v_out = {22, &d_v_out[0], 0, 22};
  const Vec *v_out_ptr = VecScale(&v, 0.9753071921712479, &v_out);

  EXPECT_NEAR(0.5032857378292578, v_out.d[0], 1e-9);
  EXPECT_NEAR(-0.3330100178921740, v_out.d[1], 1e-9);
  EXPECT_NEAR(-0.0682466676023331, v_out.d[2], 1e-9);
  EXPECT_NEAR(0.5124062539897765, v_out.d[3], 1e-9);
  EXPECT_NEAR(-0.2405844691379466, v_out.d[4], 1e-9);
  EXPECT_NEAR(0.2215068510504671, v_out.d[5], 1e-9);
  EXPECT_NEAR(0.5781179353489243, v_out.d[6], 1e-9);
  EXPECT_NEAR(0.8717659580075117, v_out.d[7], 1e-9);
  EXPECT_NEAR(0.7990213529863175, v_out.d[8], 1e-9);
  EXPECT_NEAR(0.5727717886685535, v_out.d[9], 1e-9);
  EXPECT_NEAR(-0.0527276493378886, v_out.d[10], 1e-9);
  EXPECT_NEAR(-0.2133395996687288, v_out.d[11], 1e-9);
  EXPECT_NEAR(0.5697760866305419, v_out.d[12], 1e-9);
  EXPECT_NEAR(0.4861266388583964, v_out.d[13], 1e-9);
  EXPECT_NEAR(0.3814480159670625, v_out.d[14], 1e-9);
  EXPECT_NEAR(-0.2199929209328448, v_out.d[15], 1e-9);
  EXPECT_NEAR(0.6710295033017352, v_out.d[16], 1e-9);
  EXPECT_NEAR(0.2781423990017112, v_out.d[17], 1e-9);
  EXPECT_NEAR(0.1883789488734476, v_out.d[18], 1e-9);
  EXPECT_NEAR(-0.2301618118903718, v_out.d[19], 1e-9);
  EXPECT_NEAR(-0.4028268151860048, v_out.d[20], 1e-9);
  EXPECT_NEAR(-0.4581515800492994, v_out.d[21], 1e-9);

  EXPECT_NEAR(0.5032857378292578, v_out_ptr->d[0], 1e-9);
  EXPECT_NEAR(-0.3330100178921740, v_out_ptr->d[1], 1e-9);
  EXPECT_NEAR(-0.0682466676023331, v_out_ptr->d[2], 1e-9);
  EXPECT_NEAR(0.5124062539897765, v_out_ptr->d[3], 1e-9);
  EXPECT_NEAR(-0.2405844691379466, v_out_ptr->d[4], 1e-9);
  EXPECT_NEAR(0.2215068510504671, v_out_ptr->d[5], 1e-9);
  EXPECT_NEAR(0.5781179353489243, v_out_ptr->d[6], 1e-9);
  EXPECT_NEAR(0.8717659580075117, v_out_ptr->d[7], 1e-9);
  EXPECT_NEAR(0.7990213529863175, v_out_ptr->d[8], 1e-9);
  EXPECT_NEAR(0.5727717886685535, v_out_ptr->d[9], 1e-9);
  EXPECT_NEAR(-0.0527276493378886, v_out_ptr->d[10], 1e-9);
  EXPECT_NEAR(-0.2133395996687288, v_out_ptr->d[11], 1e-9);
  EXPECT_NEAR(0.5697760866305419, v_out_ptr->d[12], 1e-9);
  EXPECT_NEAR(0.4861266388583964, v_out_ptr->d[13], 1e-9);
  EXPECT_NEAR(0.3814480159670625, v_out_ptr->d[14], 1e-9);
  EXPECT_NEAR(-0.2199929209328448, v_out_ptr->d[15], 1e-9);
  EXPECT_NEAR(0.6710295033017352, v_out_ptr->d[16], 1e-9);
  EXPECT_NEAR(0.2781423990017112, v_out_ptr->d[17], 1e-9);
  EXPECT_NEAR(0.1883789488734476, v_out_ptr->d[18], 1e-9);
  EXPECT_NEAR(-0.2301618118903718, v_out_ptr->d[19], 1e-9);
  EXPECT_NEAR(-0.4028268151860048, v_out_ptr->d[20], 1e-9);
  EXPECT_NEAR(-0.4581515800492994, v_out_ptr->d[21], 1e-9);
}

TEST(VecAdd, Normal7) {
  double d_v0[22] = {
      -0.9557057289826338, 0.6525240278293580,  0.3002343652693513,
      -0.3067815657597566, 0.0380710463257814,  0.3729278929712445,
      0.5644151348515414,  -0.6894915523913270, -0.8539474454591545,
      0.0562476698153549,  0.0936442960333492,  0.2459202008229535,
      -0.1561987508053810, 0.5765146493831925,  -0.6339139640288378,
      -0.5914547821371305, 0.4363125823831484,  0.8536902294544331,
      -0.0009310107999667, -0.9720469263966880, -0.1970138470915577,
      0.5769477428343242};
  const Vec v0 = {22, &d_v0[0], 0, 22};
  double d_v1[22] = {
      0.8361965598431558,  0.1795548721306977,  0.0652184816869554,
      -0.8684114673927408, -0.8503388127555174, -0.9258173262476346,
      -0.1835913675067133, -0.0356730902124052, -0.0457133189347920,
      -0.7628627079935237, -0.4740374398303215, 0.8828486284729293,
      0.5434339503086938,  0.8590118546216019,  -0.2400013020579956,
      -0.2587544213372499, -0.9317421699458910, -0.5160303662406824,
      0.8611440129462309,  -0.4922045816320892, 0.2163727423635728,
      -0.4592047176329377};
  const Vec v1 = {22, &d_v1[0], 0, 22};
  double d_v_out[22] = {
      0.9989983064252097,  0.5831063959726377,  0.0930035168923262,
      -0.0011768969336952, -0.3625468830634844, 0.7468582190701767,
      -0.1940565421652058, -0.8140201014863078, 0.5156167119427160,
      -0.4518834865364278, 0.7173480752497186,  -0.8276963420246510,
      -0.6329042950863313, -0.2809075789157727, -0.4129676326385283,
      0.8322026197550763,  -0.9854936653320749, 0.1145403058190109,
      -0.3429985861935170, -0.4759522885798384, -0.6067220559790869,
      0.9703659151939410};
  Vec v_out = {22, &d_v_out[0], 0, 22};
  const Vec *v_out_ptr = VecAdd(&v0, &v1, &v_out);

  EXPECT_NEAR(-0.1195091691394781, v_out.d[0], 1e-9);
  EXPECT_NEAR(0.8320788999600557, v_out.d[1], 1e-9);
  EXPECT_NEAR(0.3654528469563068, v_out.d[2], 1e-9);
  EXPECT_NEAR(-1.1751930331524973, v_out.d[3], 1e-9);
  EXPECT_NEAR(-0.8122677664297360, v_out.d[4], 1e-9);
  EXPECT_NEAR(-0.5528894332763901, v_out.d[5], 1e-9);
  EXPECT_NEAR(0.3808237673448280, v_out.d[6], 1e-9);
  EXPECT_NEAR(-0.7251646426037321, v_out.d[7], 1e-9);
  EXPECT_NEAR(-0.8996607643939465, v_out.d[8], 1e-9);
  EXPECT_NEAR(-0.7066150381781688, v_out.d[9], 1e-9);
  EXPECT_NEAR(-0.3803931437969723, v_out.d[10], 1e-9);
  EXPECT_NEAR(1.1287688292958828, v_out.d[11], 1e-9);
  EXPECT_NEAR(0.3872351995033128, v_out.d[12], 1e-9);
  EXPECT_NEAR(1.4355265040047944, v_out.d[13], 1e-9);
  EXPECT_NEAR(-0.8739152660868335, v_out.d[14], 1e-9);
  EXPECT_NEAR(-0.8502092034743804, v_out.d[15], 1e-9);
  EXPECT_NEAR(-0.4954295875627426, v_out.d[16], 1e-9);
  EXPECT_NEAR(0.3376598632137506, v_out.d[17], 1e-9);
  EXPECT_NEAR(0.8602130021462642, v_out.d[18], 1e-9);
  EXPECT_NEAR(-1.4642515080287772, v_out.d[19], 1e-9);
  EXPECT_NEAR(0.0193588952720152, v_out.d[20], 1e-9);
  EXPECT_NEAR(0.1177430252013865, v_out.d[21], 1e-9);

  EXPECT_NEAR(-0.1195091691394781, v_out_ptr->d[0], 1e-9);
  EXPECT_NEAR(0.8320788999600557, v_out_ptr->d[1], 1e-9);
  EXPECT_NEAR(0.3654528469563068, v_out_ptr->d[2], 1e-9);
  EXPECT_NEAR(-1.1751930331524973, v_out_ptr->d[3], 1e-9);
  EXPECT_NEAR(-0.8122677664297360, v_out_ptr->d[4], 1e-9);
  EXPECT_NEAR(-0.5528894332763901, v_out_ptr->d[5], 1e-9);
  EXPECT_NEAR(0.3808237673448280, v_out_ptr->d[6], 1e-9);
  EXPECT_NEAR(-0.7251646426037321, v_out_ptr->d[7], 1e-9);
  EXPECT_NEAR(-0.8996607643939465, v_out_ptr->d[8], 1e-9);
  EXPECT_NEAR(-0.7066150381781688, v_out_ptr->d[9], 1e-9);
  EXPECT_NEAR(-0.3803931437969723, v_out_ptr->d[10], 1e-9);
  EXPECT_NEAR(1.1287688292958828, v_out_ptr->d[11], 1e-9);
  EXPECT_NEAR(0.3872351995033128, v_out_ptr->d[12], 1e-9);
  EXPECT_NEAR(1.4355265040047944, v_out_ptr->d[13], 1e-9);
  EXPECT_NEAR(-0.8739152660868335, v_out_ptr->d[14], 1e-9);
  EXPECT_NEAR(-0.8502092034743804, v_out_ptr->d[15], 1e-9);
  EXPECT_NEAR(-0.4954295875627426, v_out_ptr->d[16], 1e-9);
  EXPECT_NEAR(0.3376598632137506, v_out_ptr->d[17], 1e-9);
  EXPECT_NEAR(0.8602130021462642, v_out_ptr->d[18], 1e-9);
  EXPECT_NEAR(-1.4642515080287772, v_out_ptr->d[19], 1e-9);
  EXPECT_NEAR(0.0193588952720152, v_out_ptr->d[20], 1e-9);
  EXPECT_NEAR(0.1177430252013865, v_out_ptr->d[21], 1e-9);
}

TEST(VecAdd3, Normal7) {
  double d_v0[22] = {
      0.3658956212297195,  0.7824253219868536,  0.5373352345738402,
      -0.1731141252540287, -0.7938521115312682, -0.3118603776300011,
      -0.0792565671836938, 0.8603804064445961,  -0.9326483378904222,
      0.3071574348789297,  -0.0539285296855847, 0.3019763413172125,
      -0.9156841812778311, 0.7030876871365304,  -0.5450993406589790,
      0.0809525949283136,  0.4819767736355889,  -0.4310494965595135,
      -0.7264058088622851, 0.2554501245775089,  -0.7357784893566333,
      -0.9986659235012296};
  const Vec v0 = {22, &d_v0[0], 0, 22};
  double d_v1[22] = {
      -0.9359100259348703, 0.9410356034085521,  -0.0780844111497869,
      -0.0529959696190958, -0.5356875970539565, -0.1522283533481203,
      -0.5747813237571400, 0.4050280283044949,  0.1823000716304322,
      -0.4633450794416574, 0.9499292114216169,  0.0192895116585243,
      -0.2688783535019730, -0.0853227213778911, -0.0169545159116180,
      -0.5544186031341112, 0.2716427375448740,  0.2046294826573649,
      -0.0592432183383149, -0.0474824426320231, 0.8212067580157314,
      0.2724705936590730};
  const Vec v1 = {22, &d_v1[0], 0, 22};
  double d_v2[22] = {
      0.3970317071912324,  0.5117384801612357,  0.0482207918230939,
      -0.1167573971268696, 0.1079351848327550,  0.0653873468874082,
      0.4657926769828595,  0.6657029107998438,  0.1107856652189130,
      0.6936474292887405,  -0.9091902102718896, 0.4297886676672100,
      0.6442571156702621,  -0.8687296460487448, -0.5958627534635235,
      0.1947406953655790,  -0.2226635138233914, -0.7688513120912757,
      0.2313250596735721,  0.9199977266640922,  -0.8673870168661391,
      0.2880233987421510};
  const Vec v2 = {22, &d_v2[0], 0, 22};
  double d_v0c[22] = {
      0.3658956212297195,  0.7824253219868536,  0.5373352345738402,
      -0.1731141252540287, -0.7938521115312682, -0.3118603776300011,
      -0.0792565671836938, 0.8603804064445961,  -0.9326483378904222,
      0.3071574348789297,  -0.0539285296855847, 0.3019763413172125,
      -0.9156841812778311, 0.7030876871365304,  -0.5450993406589790,
      0.0809525949283136,  0.4819767736355889,  -0.4310494965595135,
      -0.7264058088622851, 0.2554501245775089,  -0.7357784893566333,
      -0.9986659235012296};
  Vec v0c = {22, &d_v0c[0], 0, 22};
  double d_v1c[22] = {
      -0.9359100259348703, 0.9410356034085521,  -0.0780844111497869,
      -0.0529959696190958, -0.5356875970539565, -0.1522283533481203,
      -0.5747813237571400, 0.4050280283044949,  0.1823000716304322,
      -0.4633450794416574, 0.9499292114216169,  0.0192895116585243,
      -0.2688783535019730, -0.0853227213778911, -0.0169545159116180,
      -0.5544186031341112, 0.2716427375448740,  0.2046294826573649,
      -0.0592432183383149, -0.0474824426320231, 0.8212067580157314,
      0.2724705936590730};
  Vec v1c = {22, &d_v1c[0], 0, 22};
  double d_v2c[22] = {
      0.3970317071912324,  0.5117384801612357,  0.0482207918230939,
      -0.1167573971268696, 0.1079351848327550,  0.0653873468874082,
      0.4657926769828595,  0.6657029107998438,  0.1107856652189130,
      0.6936474292887405,  -0.9091902102718896, 0.4297886676672100,
      0.6442571156702621,  -0.8687296460487448, -0.5958627534635235,
      0.1947406953655790,  -0.2226635138233914, -0.7688513120912757,
      0.2313250596735721,  0.9199977266640922,  -0.8673870168661391,
      0.2880233987421510};
  Vec v2c = {22, &d_v2c[0], 0, 22};
  double d_v_out[22] = {
      -0.5270608796808651, 0.6828612867713562,  -0.8060014427030893,
      -0.4816869604503269, 0.4254433059553755,  0.3051589133876320,
      0.4732595848952363,  0.3248756243272251,  0.5565648338084854,
      0.4513892942953914,  -0.9267708191637727, 0.4741086086694430,
      0.4157260870213648,  -0.2942997440456723, 0.1475241470618822,
      0.2460365167610210,  -0.4703774041682058, -0.6788580444178884,
      -0.6785419884148798, 0.7618813503933126,  -0.6744405144151158,
      0.8201724331115658};
  Vec v_out = {22, &d_v_out[0], 0, 22};
  const Vec *v_out_ptr = VecAdd3(&v0, &v1, &v2, &v_out);
  VecAdd3(&v0c, &v1, &v2, &v0c);
  VecAdd3(&v0, &v1c, &v2, &v1c);
  VecAdd3(&v0, &v1, &v2c, &v2c);

  EXPECT_NEAR(-0.1729826975139184, v_out.d[0], 1e-9);
  EXPECT_NEAR(2.2351994055566413, v_out.d[1], 1e-9);
  EXPECT_NEAR(0.5074716152471472, v_out.d[2], 1e-9);
  EXPECT_NEAR(-0.3428674919999941, v_out.d[3], 1e-9);
  EXPECT_NEAR(-1.2216045237524698, v_out.d[4], 1e-9);
  EXPECT_NEAR(-0.3987013840907132, v_out.d[5], 1e-9);
  EXPECT_NEAR(-0.1882452139579742, v_out.d[6], 1e-9);
  EXPECT_NEAR(1.9311113455489348, v_out.d[7], 1e-9);
  EXPECT_NEAR(-0.6395626010410771, v_out.d[8], 1e-9);
  EXPECT_NEAR(0.5374597847260127, v_out.d[9], 1e-9);
  EXPECT_NEAR(-0.0131895285358574, v_out.d[10], 1e-9);
  EXPECT_NEAR(0.7510545206429469, v_out.d[11], 1e-9);
  EXPECT_NEAR(-0.5403054191095420, v_out.d[12], 1e-9);
  EXPECT_NEAR(-0.2509646802901055, v_out.d[13], 1e-9);
  EXPECT_NEAR(-1.1579166100341205, v_out.d[14], 1e-9);
  EXPECT_NEAR(-0.2787253128402185, v_out.d[15], 1e-9);
  EXPECT_NEAR(0.5309559973570714, v_out.d[16], 1e-9);
  EXPECT_NEAR(-0.9952713259934243, v_out.d[17], 1e-9);
  EXPECT_NEAR(-0.5543239675270279, v_out.d[18], 1e-9);
  EXPECT_NEAR(1.1279654086095781, v_out.d[19], 1e-9);
  EXPECT_NEAR(-0.7819587482070409, v_out.d[20], 1e-9);
  EXPECT_NEAR(-0.4381719311000056, v_out.d[21], 1e-9);

  EXPECT_NEAR(-0.1729826975139184, v0c.d[0], 1e-9);
  EXPECT_NEAR(2.2351994055566413, v0c.d[1], 1e-9);
  EXPECT_NEAR(0.5074716152471472, v0c.d[2], 1e-9);
  EXPECT_NEAR(-0.3428674919999941, v0c.d[3], 1e-9);
  EXPECT_NEAR(-1.2216045237524698, v0c.d[4], 1e-9);
  EXPECT_NEAR(-0.3987013840907132, v0c.d[5], 1e-9);
  EXPECT_NEAR(-0.1882452139579742, v0c.d[6], 1e-9);
  EXPECT_NEAR(1.9311113455489348, v0c.d[7], 1e-9);
  EXPECT_NEAR(-0.6395626010410771, v0c.d[8], 1e-9);
  EXPECT_NEAR(0.5374597847260127, v0c.d[9], 1e-9);
  EXPECT_NEAR(-0.0131895285358574, v0c.d[10], 1e-9);
  EXPECT_NEAR(0.7510545206429469, v0c.d[11], 1e-9);
  EXPECT_NEAR(-0.5403054191095420, v0c.d[12], 1e-9);
  EXPECT_NEAR(-0.2509646802901055, v0c.d[13], 1e-9);
  EXPECT_NEAR(-1.1579166100341205, v0c.d[14], 1e-9);
  EXPECT_NEAR(-0.2787253128402185, v0c.d[15], 1e-9);
  EXPECT_NEAR(0.5309559973570714, v0c.d[16], 1e-9);
  EXPECT_NEAR(-0.9952713259934243, v0c.d[17], 1e-9);
  EXPECT_NEAR(-0.5543239675270279, v0c.d[18], 1e-9);
  EXPECT_NEAR(1.1279654086095781, v0c.d[19], 1e-9);
  EXPECT_NEAR(-0.7819587482070409, v0c.d[20], 1e-9);
  EXPECT_NEAR(-0.4381719311000056, v0c.d[21], 1e-9);

  EXPECT_NEAR(-0.1729826975139184, v1c.d[0], 1e-9);
  EXPECT_NEAR(2.2351994055566413, v1c.d[1], 1e-9);
  EXPECT_NEAR(0.5074716152471472, v1c.d[2], 1e-9);
  EXPECT_NEAR(-0.3428674919999941, v1c.d[3], 1e-9);
  EXPECT_NEAR(-1.2216045237524698, v1c.d[4], 1e-9);
  EXPECT_NEAR(-0.3987013840907132, v1c.d[5], 1e-9);
  EXPECT_NEAR(-0.1882452139579742, v1c.d[6], 1e-9);
  EXPECT_NEAR(1.9311113455489348, v1c.d[7], 1e-9);
  EXPECT_NEAR(-0.6395626010410771, v1c.d[8], 1e-9);
  EXPECT_NEAR(0.5374597847260127, v1c.d[9], 1e-9);
  EXPECT_NEAR(-0.0131895285358574, v1c.d[10], 1e-9);
  EXPECT_NEAR(0.7510545206429469, v1c.d[11], 1e-9);
  EXPECT_NEAR(-0.5403054191095420, v1c.d[12], 1e-9);
  EXPECT_NEAR(-0.2509646802901055, v1c.d[13], 1e-9);
  EXPECT_NEAR(-1.1579166100341205, v1c.d[14], 1e-9);
  EXPECT_NEAR(-0.2787253128402185, v1c.d[15], 1e-9);
  EXPECT_NEAR(0.5309559973570714, v1c.d[16], 1e-9);
  EXPECT_NEAR(-0.9952713259934243, v1c.d[17], 1e-9);
  EXPECT_NEAR(-0.5543239675270279, v1c.d[18], 1e-9);
  EXPECT_NEAR(1.1279654086095781, v1c.d[19], 1e-9);
  EXPECT_NEAR(-0.7819587482070409, v1c.d[20], 1e-9);
  EXPECT_NEAR(-0.4381719311000056, v1c.d[21], 1e-9);

  EXPECT_NEAR(-0.1729826975139184, v2c.d[0], 1e-9);
  EXPECT_NEAR(2.2351994055566413, v2c.d[1], 1e-9);
  EXPECT_NEAR(0.5074716152471472, v2c.d[2], 1e-9);
  EXPECT_NEAR(-0.3428674919999941, v2c.d[3], 1e-9);
  EXPECT_NEAR(-1.2216045237524698, v2c.d[4], 1e-9);
  EXPECT_NEAR(-0.3987013840907132, v2c.d[5], 1e-9);
  EXPECT_NEAR(-0.1882452139579742, v2c.d[6], 1e-9);
  EXPECT_NEAR(1.9311113455489348, v2c.d[7], 1e-9);
  EXPECT_NEAR(-0.6395626010410771, v2c.d[8], 1e-9);
  EXPECT_NEAR(0.5374597847260127, v2c.d[9], 1e-9);
  EXPECT_NEAR(-0.0131895285358574, v2c.d[10], 1e-9);
  EXPECT_NEAR(0.7510545206429469, v2c.d[11], 1e-9);
  EXPECT_NEAR(-0.5403054191095420, v2c.d[12], 1e-9);
  EXPECT_NEAR(-0.2509646802901055, v2c.d[13], 1e-9);
  EXPECT_NEAR(-1.1579166100341205, v2c.d[14], 1e-9);
  EXPECT_NEAR(-0.2787253128402185, v2c.d[15], 1e-9);
  EXPECT_NEAR(0.5309559973570714, v2c.d[16], 1e-9);
  EXPECT_NEAR(-0.9952713259934243, v2c.d[17], 1e-9);
  EXPECT_NEAR(-0.5543239675270279, v2c.d[18], 1e-9);
  EXPECT_NEAR(1.1279654086095781, v2c.d[19], 1e-9);
  EXPECT_NEAR(-0.7819587482070409, v2c.d[20], 1e-9);
  EXPECT_NEAR(-0.4381719311000056, v2c.d[21], 1e-9);

  EXPECT_NEAR(-0.1729826975139184, v_out_ptr->d[0], 1e-9);
  EXPECT_NEAR(2.2351994055566413, v_out_ptr->d[1], 1e-9);
  EXPECT_NEAR(0.5074716152471472, v_out_ptr->d[2], 1e-9);
  EXPECT_NEAR(-0.3428674919999941, v_out_ptr->d[3], 1e-9);
  EXPECT_NEAR(-1.2216045237524698, v_out_ptr->d[4], 1e-9);
  EXPECT_NEAR(-0.3987013840907132, v_out_ptr->d[5], 1e-9);
  EXPECT_NEAR(-0.1882452139579742, v_out_ptr->d[6], 1e-9);
  EXPECT_NEAR(1.9311113455489348, v_out_ptr->d[7], 1e-9);
  EXPECT_NEAR(-0.6395626010410771, v_out_ptr->d[8], 1e-9);
  EXPECT_NEAR(0.5374597847260127, v_out_ptr->d[9], 1e-9);
  EXPECT_NEAR(-0.0131895285358574, v_out_ptr->d[10], 1e-9);
  EXPECT_NEAR(0.7510545206429469, v_out_ptr->d[11], 1e-9);
  EXPECT_NEAR(-0.5403054191095420, v_out_ptr->d[12], 1e-9);
  EXPECT_NEAR(-0.2509646802901055, v_out_ptr->d[13], 1e-9);
  EXPECT_NEAR(-1.1579166100341205, v_out_ptr->d[14], 1e-9);
  EXPECT_NEAR(-0.2787253128402185, v_out_ptr->d[15], 1e-9);
  EXPECT_NEAR(0.5309559973570714, v_out_ptr->d[16], 1e-9);
  EXPECT_NEAR(-0.9952713259934243, v_out_ptr->d[17], 1e-9);
  EXPECT_NEAR(-0.5543239675270279, v_out_ptr->d[18], 1e-9);
  EXPECT_NEAR(1.1279654086095781, v_out_ptr->d[19], 1e-9);
  EXPECT_NEAR(-0.7819587482070409, v_out_ptr->d[20], 1e-9);
  EXPECT_NEAR(-0.4381719311000056, v_out_ptr->d[21], 1e-9);
}

TEST(VecSub, Normal7) {
  double d_v0[22] = {
      0.9838767963627388,  -0.8807200013331746, 0.7484387391108600,
      0.7484101091115596,  0.2949709226963591,  -0.2601186748182076,
      0.9115735349539162,  -0.2938097505924520, -0.0991823262976117,
      -0.1423395516785313, -0.0677689619994959, 0.9935404935410164,
      0.4657615737167453,  -0.5255308810904142, -0.5813409052559602,
      -0.0980713580148367, -0.0461834279465023, -0.3256649678328609,
      -0.2436136881611979, 0.9167609828839436,  0.2190008179111613,
      -0.3980079209945298};
  const Vec v0 = {22, &d_v0[0], 0, 22};
  double d_v1[22] = {
      -0.5982881268957769, 0.1763080140490081, -0.2884612054121387,
      0.3649693625277060,  0.9627710620789434, -0.2913075360840813,
      -0.6984458169999956, 0.3632069888137350, 0.9936776406765366,
      0.5552469015091492,  0.1581102775137635, 0.3698514189221818,
      -0.5375774841223637, 0.0350096256055827, 0.0016999349688338,
      -0.9877698230089518, 0.9538965120035416, -0.6790815053998789,
      -0.6322564602263463, 0.9586565200478463, -0.1655284465865079,
      0.8243549541332893};
  const Vec v1 = {22, &d_v1[0], 0, 22};
  double d_v_out[22] = {
      0.7140678527479982,  -0.8825425752742693, 0.3112418811088808,
      0.1043554712817358,  -0.7069794790816053, -0.7388680289668932,
      -0.0846295310064598, -0.6166998919363240, 0.8967560797604957,
      -0.1286361689525530, -0.6062031380260289, -0.4413095126750786,
      0.1636868435087098,  0.8321361242207270,  0.4744261346130221,
      -0.1785803604621348, -0.7064050875804320, -0.1598117465691402,
      -0.4289991639419746, 0.3451426359579348,  -0.0476014182337718,
      -0.2710492265873836};
  Vec v_out = {22, &d_v_out[0], 0, 22};
  const Vec *v_out_ptr = VecSub(&v0, &v1, &v_out);

  EXPECT_NEAR(1.5821649232585158, v_out.d[0], 1e-9);
  EXPECT_NEAR(-1.0570280153821827, v_out.d[1], 1e-9);
  EXPECT_NEAR(1.0368999445229987, v_out.d[2], 1e-9);
  EXPECT_NEAR(0.3834407465838536, v_out.d[3], 1e-9);
  EXPECT_NEAR(-0.6678001393825843, v_out.d[4], 1e-9);
  EXPECT_NEAR(0.0311888612658737, v_out.d[5], 1e-9);
  EXPECT_NEAR(1.6100193519539119, v_out.d[6], 1e-9);
  EXPECT_NEAR(-0.6570167394061870, v_out.d[7], 1e-9);
  EXPECT_NEAR(-1.0928599669741483, v_out.d[8], 1e-9);
  EXPECT_NEAR(-0.6975864531876805, v_out.d[9], 1e-9);
  EXPECT_NEAR(-0.2258792395132594, v_out.d[10], 1e-9);
  EXPECT_NEAR(0.6236890746188346, v_out.d[11], 1e-9);
  EXPECT_NEAR(1.0033390578391090, v_out.d[12], 1e-9);
  EXPECT_NEAR(-0.5605405066959970, v_out.d[13], 1e-9);
  EXPECT_NEAR(-0.5830408402247940, v_out.d[14], 1e-9);
  EXPECT_NEAR(0.8896984649941151, v_out.d[15], 1e-9);
  EXPECT_NEAR(-1.0000799399500440, v_out.d[16], 1e-9);
  EXPECT_NEAR(0.3534165375670180, v_out.d[17], 1e-9);
  EXPECT_NEAR(0.3886427720651484, v_out.d[18], 1e-9);
  EXPECT_NEAR(-0.0418955371639027, v_out.d[19], 1e-9);
  EXPECT_NEAR(0.3845292644976692, v_out.d[20], 1e-9);
  EXPECT_NEAR(-1.2223628751278190, v_out.d[21], 1e-9);

  EXPECT_NEAR(1.5821649232585158, v_out_ptr->d[0], 1e-9);
  EXPECT_NEAR(-1.0570280153821827, v_out_ptr->d[1], 1e-9);
  EXPECT_NEAR(1.0368999445229987, v_out_ptr->d[2], 1e-9);
  EXPECT_NEAR(0.3834407465838536, v_out_ptr->d[3], 1e-9);
  EXPECT_NEAR(-0.6678001393825843, v_out_ptr->d[4], 1e-9);
  EXPECT_NEAR(0.0311888612658737, v_out_ptr->d[5], 1e-9);
  EXPECT_NEAR(1.6100193519539119, v_out_ptr->d[6], 1e-9);
  EXPECT_NEAR(-0.6570167394061870, v_out_ptr->d[7], 1e-9);
  EXPECT_NEAR(-1.0928599669741483, v_out_ptr->d[8], 1e-9);
  EXPECT_NEAR(-0.6975864531876805, v_out_ptr->d[9], 1e-9);
  EXPECT_NEAR(-0.2258792395132594, v_out_ptr->d[10], 1e-9);
  EXPECT_NEAR(0.6236890746188346, v_out_ptr->d[11], 1e-9);
  EXPECT_NEAR(1.0033390578391090, v_out_ptr->d[12], 1e-9);
  EXPECT_NEAR(-0.5605405066959970, v_out_ptr->d[13], 1e-9);
  EXPECT_NEAR(-0.5830408402247940, v_out_ptr->d[14], 1e-9);
  EXPECT_NEAR(0.8896984649941151, v_out_ptr->d[15], 1e-9);
  EXPECT_NEAR(-1.0000799399500440, v_out_ptr->d[16], 1e-9);
  EXPECT_NEAR(0.3534165375670180, v_out_ptr->d[17], 1e-9);
  EXPECT_NEAR(0.3886427720651484, v_out_ptr->d[18], 1e-9);
  EXPECT_NEAR(-0.0418955371639027, v_out_ptr->d[19], 1e-9);
  EXPECT_NEAR(0.3845292644976692, v_out_ptr->d[20], 1e-9);
  EXPECT_NEAR(-1.2223628751278190, v_out_ptr->d[21], 1e-9);
}

TEST(VecLinComb, Normal7) {
  double d_v0[22] = {
      0.2116790265202226,  -0.3847197883145907, 0.3980729454515690,
      -0.5697357872721578, -0.0234965220181260, -0.0074856831375432,
      -0.6996269467566987, -0.5253332818236263, -0.8422591829384922,
      0.7725142794158832,  -0.8729315837135483, -0.2085850205472846,
      -0.6397791626846800, 0.2413931163110139,  0.2756722737204620,
      -0.9447457352741000, -0.7211539919350034, -0.7507388232016234,
      -0.2416843086308742, -0.8637790515273536, -0.9478083271709827,
      0.7385783097655922};
  const Vec v0 = {22, &d_v0[0], 0, 22};
  double d_v1[22] = {
      -0.7823273590742785, -0.8237335811513702, 0.1315524188079544,
      0.1408576517762652,  -0.7171888886408386, 0.3537426049046690,
      0.9269178997385525,  0.7311869750797499,  -0.5461392178099438,
      0.4239036950970727,  0.2492720262380457,  0.5859039690432601,
      0.2327395686450691,  -0.3513561415864506, 0.5464435863410304,
      0.8739130741412453,  0.1241247755286543,  0.3325034427831093,
      0.6968657455135645,  -0.0007890327659577, 0.6851879433438646,
      0.0388213727205859};
  const Vec v1 = {22, &d_v1[0], 0, 22};
  double d_v_out[22] = {
      -0.3279714840273116, -0.3038473545090528, 0.6968454591670932,
      0.2520570796590873,  0.2702454843874271,  -0.2349882872761990,
      0.3334956351252578,  -0.1439216148976263, 0.9812061808349364,
      0.3231129527076015,  -0.8354806390621279, -0.9072202386059556,
      -0.6716066006808359, -0.2820343607606057, -0.6497131868092225,
      -0.5596640617035036, -0.5830598349848393, -0.2763033890537310,
      -0.2107547102681508, -0.9546165960404460, 0.8722369249403605,
      0.7571440345555494};
  Vec v_out = {22, &d_v_out[0], 0, 22};
  const Vec *v_out_ptr =
      VecLinComb(-0.8708502254519244, &v0, -0.9495050559898544, &v1, &v_out);

  EXPECT_NEAR(0.5584830549116380, v_out.d[0], 1e-9);
  EXPECT_NEAR(1.1171725144814331, v_out.d[1], 1e-9);
  EXPECT_NEAR(-0.4715716010786580, v_out.d[2], 1e-9);
  EXPECT_NEAR(0.3624094862575662, v_out.d[3], 1e-9);
  EXPECT_NEAR(0.7014364273610421, v_out.d[4], 1e-9);
  EXPECT_NEAR(-0.3293614830280136, v_out.d[5], 1e-9);
  EXPECT_NEAR(-0.2708429479739400, v_out.d[6], 1e-9);
  EXPECT_NEAR(-0.2367791227986459, v_out.d[7], 1e-9);
  EXPECT_NEAR(1.2520435479358256, v_out.d[8], 1e-9);
  EXPECT_NEAR(-1.0752429361416049, v_out.d[9], 1e-9);
  EXPECT_NEAR(0.5235076172511889, v_out.d[10], 1e-9);
  EXPECT_NEAR(-0.3746724687616017, v_out.d[11], 1e-9);
  EXPECT_NEAR(0.3361644309060062, v_out.d[12], 1e-9);
  EXPECT_NEAR(0.1233971831274329, v_out.d[13], 1e-9);
  EXPECT_NEAR(-0.7589202097643458, v_out.d[14], 1e-9);
  EXPECT_NEAR(-0.0070528458345548, v_out.d[15], 1e-9);
  EXPECT_NEAR(0.5101600145240900, v_out.d[16], 1e-9);
  EXPECT_NEAR(0.3380673733840507, v_out.d[17], 1e-9);
  EXPECT_NEAR(-0.4512067140518795, v_out.d[18], 1e-9);
  EXPECT_NEAR(0.7529713723638637, v_out.d[19], 1e-9);
  EXPECT_NEAR(0.1748096788937723, v_out.d[20], 1e-9);
  EXPECT_NEAR(-0.6800521772519302, v_out.d[21], 1e-9);

  EXPECT_NEAR(0.5584830549116380, v_out_ptr->d[0], 1e-9);
  EXPECT_NEAR(1.1171725144814331, v_out_ptr->d[1], 1e-9);
  EXPECT_NEAR(-0.4715716010786580, v_out_ptr->d[2], 1e-9);
  EXPECT_NEAR(0.3624094862575662, v_out_ptr->d[3], 1e-9);
  EXPECT_NEAR(0.7014364273610421, v_out_ptr->d[4], 1e-9);
  EXPECT_NEAR(-0.3293614830280136, v_out_ptr->d[5], 1e-9);
  EXPECT_NEAR(-0.2708429479739400, v_out_ptr->d[6], 1e-9);
  EXPECT_NEAR(-0.2367791227986459, v_out_ptr->d[7], 1e-9);
  EXPECT_NEAR(1.2520435479358256, v_out_ptr->d[8], 1e-9);
  EXPECT_NEAR(-1.0752429361416049, v_out_ptr->d[9], 1e-9);
  EXPECT_NEAR(0.5235076172511889, v_out_ptr->d[10], 1e-9);
  EXPECT_NEAR(-0.3746724687616017, v_out_ptr->d[11], 1e-9);
  EXPECT_NEAR(0.3361644309060062, v_out_ptr->d[12], 1e-9);
  EXPECT_NEAR(0.1233971831274329, v_out_ptr->d[13], 1e-9);
  EXPECT_NEAR(-0.7589202097643458, v_out_ptr->d[14], 1e-9);
  EXPECT_NEAR(-0.0070528458345548, v_out_ptr->d[15], 1e-9);
  EXPECT_NEAR(0.5101600145240900, v_out_ptr->d[16], 1e-9);
  EXPECT_NEAR(0.3380673733840507, v_out_ptr->d[17], 1e-9);
  EXPECT_NEAR(-0.4512067140518795, v_out_ptr->d[18], 1e-9);
  EXPECT_NEAR(0.7529713723638637, v_out_ptr->d[19], 1e-9);
  EXPECT_NEAR(0.1748096788937723, v_out_ptr->d[20], 1e-9);
  EXPECT_NEAR(-0.6800521772519302, v_out_ptr->d[21], 1e-9);
}

TEST(VecLinComb3, Normal7) {
  double d_v0[22] = {
      -0.9270918603181790, -0.5923812538212612, 0.2988922650033563,
      0.0357829271243364,  0.9630737583444313,  0.6214820261840930,
      -0.5786789950864724, -0.5301027187292691, 0.6399386186613785,
      0.6326848319204883,  0.3853412436869255,  -0.8510956529388432,
      -0.6201765381887243, 0.7025487097136043,  0.6549453942396606,
      -0.4050234022787651, 0.5730387769379841,  -0.8997497640341094,
      0.0453927307827875,  -0.5432313298318505, -0.0942132432700951,
      0.7120796422413733};
  const Vec v0 = {22, &d_v0[0], 0, 22};
  double d_v1[22] = {
      0.4804136964233441,  -0.3411746591333120, -0.1289552482875220,
      0.3365587891587773,  -0.6459419116567640, -0.7541946114957578,
      0.9053258829950319,  -0.9375970708480754, -0.5205341335613756,
      0.5990934485500303,  -0.1954147151119949, 0.1207949181502335,
      0.3177667989408706,  -0.5605594291462010, -0.2788865525158422,
      -0.0631657161897634, -0.2365745749686241, -0.0790826899189849,
      -0.4047812295492350, -0.5758660735462464, 0.6161088228685658,
      -0.2157426326977170};
  const Vec v1 = {22, &d_v1[0], 0, 22};
  double d_v2[22] = {
      0.8465941915433570,  -0.5721704714304514, -0.0338710325391529,
      -0.1276712366805079, -0.7030386955439700, -0.9976723790992044,
      0.9014051457458530,  0.1273904691963066,  0.0293655060052180,
      -0.2410156363402478, -0.2784604702272022, 0.5392434920672295,
      -0.7279628021138145, 0.3856141351600524,  -0.2505564666268052,
      0.9071692135553899,  0.3019001375978181,  -0.0583598112726273,
      0.4405581433060368,  0.9765599779729937,  0.2017255531228870,
      0.0608105554569780};
  const Vec v2 = {22, &d_v2[0], 0, 22};
  double d_v_out[22] = {
      -0.3739898258266687, -0.2696902314029863, -0.4026557168334657,
      0.6753536450038433,  0.5770848664098447,  0.9427144607800546,
      0.0314828793618382,  -0.7554373305630839, -0.4004676368384681,
      -0.7077516757950508, 0.1631725286568535,  0.8557008616846919,
      -0.9404081485812070, -0.1761682412073626, 0.6853837625667534,
      -0.8620327854809786, 0.6813817271943583,  0.3288496745891518,
      -0.1775790719278851, 0.0425348151524263,  -0.9318338224748941,
      0.9084597421214577};
  Vec v_out = {22, &d_v_out[0], 0, 22};
  const Vec *v_out_ptr =
      VecLinComb3(-0.4663620557117527, &v0, -0.5056332266432428, &v1,
                  0.1131184235319838, &v2, &v_out);

  EXPECT_NEAR(0.2852127386841951, v_out.d[0], 1e-9);
  EXPECT_NEAR(0.3840503613239094, v_out.d[1], 1e-9);
  EXPECT_NEAR(-0.0780193906633364, v_out.d[2], 1e-9);
  EXPECT_NEAR(-0.2013050749942626, v_out.d[3], 1e-9);
  EXPECT_NEAR(-0.2020579936503549, v_out.d[4], 1e-9);
  EXPECT_NEAR(-0.0213449071166776, v_out.d[5], 1e-9);
  EXPECT_NEAR(-0.0859233925862935, v_out.d[6], 1e-9);
  EXPECT_NEAR(0.7357102349176223, v_out.d[7], 1e-9);
  EXPECT_NEAR(-0.0319219564521475, v_out.d[8], 1e-9);
  EXPECT_NEAR(-0.6252450611126299, v_out.d[9], 1e-9);
  EXPECT_NEAR(-0.1123993710287860, v_out.d[10], 1e-9);
  EXPECT_NEAR(0.3968391678080131, v_out.d[11], 1e-9);
  EXPECT_NEAR(0.0462073488202846, v_out.d[12], 1e-9);
  EXPECT_NEAR(-0.0005845245542336, v_out.d[13], 1e-9);
  EXPECT_NEAR(-0.1927699255311253, v_out.d[14], 1e-9);
  EXPECT_NEAR(0.3234437827024897, v_out.d[15], 1e-9);
  EXPECT_NEAR(-0.1134731087030535, v_out.d[16], 1e-9);
  EXPECT_NEAR(0.4529944154076915, v_out.d[17], 1e-9);
  EXPECT_NEAR(0.2333366345843246, v_out.d[18], 1e-9);
  EXPECT_NEAR(0.6549864257817083, v_out.d[19], 1e-9);
  EXPECT_NEAR(-0.2447687337083117, v_out.d[20], 1e-9);
  EXPECT_NEAR(-0.2161214881233240, v_out.d[21], 1e-9);

  EXPECT_NEAR(0.2852127386841951, v_out_ptr->d[0], 1e-9);
  EXPECT_NEAR(0.3840503613239094, v_out_ptr->d[1], 1e-9);
  EXPECT_NEAR(-0.0780193906633364, v_out_ptr->d[2], 1e-9);
  EXPECT_NEAR(-0.2013050749942626, v_out_ptr->d[3], 1e-9);
  EXPECT_NEAR(-0.2020579936503549, v_out_ptr->d[4], 1e-9);
  EXPECT_NEAR(-0.0213449071166776, v_out_ptr->d[5], 1e-9);
  EXPECT_NEAR(-0.0859233925862935, v_out_ptr->d[6], 1e-9);
  EXPECT_NEAR(0.7357102349176223, v_out_ptr->d[7], 1e-9);
  EXPECT_NEAR(-0.0319219564521475, v_out_ptr->d[8], 1e-9);
  EXPECT_NEAR(-0.6252450611126299, v_out_ptr->d[9], 1e-9);
  EXPECT_NEAR(-0.1123993710287860, v_out_ptr->d[10], 1e-9);
  EXPECT_NEAR(0.3968391678080131, v_out_ptr->d[11], 1e-9);
  EXPECT_NEAR(0.0462073488202846, v_out_ptr->d[12], 1e-9);
  EXPECT_NEAR(-0.0005845245542336, v_out_ptr->d[13], 1e-9);
  EXPECT_NEAR(-0.1927699255311253, v_out_ptr->d[14], 1e-9);
  EXPECT_NEAR(0.3234437827024897, v_out_ptr->d[15], 1e-9);
  EXPECT_NEAR(-0.1134731087030535, v_out_ptr->d[16], 1e-9);
  EXPECT_NEAR(0.4529944154076915, v_out_ptr->d[17], 1e-9);
  EXPECT_NEAR(0.2333366345843246, v_out_ptr->d[18], 1e-9);
  EXPECT_NEAR(0.6549864257817083, v_out_ptr->d[19], 1e-9);
  EXPECT_NEAR(-0.2447687337083117, v_out_ptr->d[20], 1e-9);
  EXPECT_NEAR(-0.2161214881233240, v_out_ptr->d[21], 1e-9);
}

TEST(VecDot, Normal7) {
  double d_v0[22] = {
      -0.3393665250316995, -0.2415118924458337, 0.4903614880042282,
      0.3124173714498473,  -0.1755574746787096, -0.7718677402814185,
      -0.5517152787545931, 0.3669332167303916,  -0.4508601361577573,
      0.3635351945204863,  -0.0581643067283941, -0.9845513175756098,
      0.2371807176952292,  0.6349597784547771,  0.0401310368542553,
      -0.1458163105682473, -0.9964961297718569, 0.7924301377601048,
      -0.8266146071812608, 0.4920063737639040,  -0.6578098291465830,
      -0.2211126655683737};
  const Vec v0 = {22, &d_v0[0], 0, 22};
  double d_v1[22] = {
      -0.2875028005277824, 0.3019694774808046,  0.2671078884534526,
      -0.5140335439544708, 0.1811772074109819,  -0.5958886028248236,
      0.9762774752665309,  0.5416380238419656,  -0.6536786758728090,
      0.2793041246582562,  0.9377092643907092,  -0.2447865838638490,
      -0.1069074337949079, 0.5104753338107393,  -0.4815683185124364,
      0.5918395501513904,  -0.4537438379127781, -0.8385560955071847,
      -0.7777891255327967, -0.9620387796105556, 0.1532952759568229,
      0.8194098379033907};
  const Vec v1 = {22, &d_v1[0], 0, 22};
  double v_dot = VecDot(&v0, &v1);

  EXPECT_NEAR(0.5343899510377720, v_dot, 1e-9);
}

TEST(VecMult, Normal7) {
  double d_v0[22] = {
      0.7199500863840400,  -0.7653622630874455, 0.7354967107112009,
      0.0548807420020012,  0.7703745422759458,  -0.4057156083459019,
      0.5757909873688420,  -0.5746181751318700, 0.3047936479597617,
      -0.9226848841273261, -0.0342879319221754, 0.9655180006300321,
      -0.5873385485557310, 0.1355113417988725,  0.5601538386171352,
      0.5858471753080197,  0.0943924867543535,  -0.0704767928728502,
      -0.0684450938666978, 0.5602987784752373,  0.8518416811010521,
      -0.6080659986641610};
  const Vec v0 = {22, &d_v0[0], 0, 22};
  double d_v1[22] = {
      -0.5930778504735035, -0.0559877711095198, -0.0476801443662707,
      -0.4010781910257750, 0.5247694820795958,  0.3627049904377531,
      0.3864648680737481,  -0.1943115246212419, 0.6941547915303763,
      -0.7964185835264295, 0.5755920975432587,  -0.6752677047686471,
      0.2322361376523865,  0.4160684950289599,  -0.0427998055721142,
      -0.4242383459961931, -0.4476880870851132, -0.5710738526077128,
      0.5308842969143670,  0.2010015985762237,  0.0357841083975521,
      -0.1495389002173966};
  const Vec v1 = {22, &d_v1[0], 0, 22};
  double d_v_out[22] = {
      -0.0347627107634358, 0.4015721554003109,  -0.7511518826005177,
      0.6858072448855457,  -0.0053871753552632, -0.0414321922216176,
      -0.7297318386049534, 0.2175087407976868,  -0.0739567978303353,
      -0.5526637499359914, -0.3007706404613104, -0.6677472289066111,
      -0.9054301746923716, -0.8089074236324147, -0.2908317353850434,
      -0.5112954159682581, 0.9697116935008185,  0.5784451151555299,
      0.4013090952502389,  0.2540282001340546,  -0.7163818889074196,
      -0.7690302344806781};
  Vec v_out = {22, &d_v_out[0], 0, 22};
  const Vec *v_out_ptr = VecMult(&v0, &v1, &v_out);

  EXPECT_NEAR(-0.4269864496808596, v_out.d[0], 1e-9);
  EXPECT_NEAR(0.0428509272016040, v_out.d[1], 1e-9);
  EXPECT_NEAR(-0.0350685893476273, v_out.d[2], 1e-9);
  EXPECT_NEAR(-0.0220114687243149, v_out.d[3], 1e-9);
  EXPECT_NEAR(0.4042690495574538, v_out.d[4], 1e-9);
  EXPECT_NEAR(-0.1471550758455475, v_out.d[5], 1e-9);
  EXPECT_NEAR(0.2225229879715527, v_out.d[6], 1e-9);
  EXPECT_NEAR(0.1116549336849494, v_out.d[7], 1e-9);
  EXPECT_NEAR(0.2115739711592913, v_out.d[8], 1e-9);
  EXPECT_NEAR(0.7348433884579327, v_out.d[9], 1e-9);
  EXPECT_NEAR(-0.0197358626555054, v_out.d[10], 1e-9);
  EXPECT_NEAR(-0.6519831241982549, v_out.d[11], 1e-9);
  EXPECT_NEAR(-0.1364012360109416, v_out.d[12], 1e-9);
  EXPECT_NEAR(0.0563820000416119, v_out.d[13], 1e-9);
  EXPECT_NEAR(-0.0239744753832868, v_out.d[14], 1e-9);
  EXPECT_NEAR(-0.2485388366592161, v_out.d[15], 1e-9);
  EXPECT_NEAR(-0.0422583918302634, v_out.d[16], 1e-9);
  EXPECT_NEAR(0.0402474536253344, v_out.d[17], 1e-9);
  EXPECT_NEAR(-0.0363364255346597, v_out.d[18], 1e-9);
  EXPECT_NEAR(0.1126209501538281, v_out.d[19], 1e-9);
  EXPECT_NEAR(0.0304823950540730, v_out.d[20], 1e-9);
  EXPECT_NEAR(0.0909295206998316, v_out.d[21], 1e-9);

  EXPECT_NEAR(-0.4269864496808596, v_out_ptr->d[0], 1e-9);
  EXPECT_NEAR(0.0428509272016040, v_out_ptr->d[1], 1e-9);
  EXPECT_NEAR(-0.0350685893476273, v_out_ptr->d[2], 1e-9);
  EXPECT_NEAR(-0.0220114687243149, v_out_ptr->d[3], 1e-9);
  EXPECT_NEAR(0.4042690495574538, v_out_ptr->d[4], 1e-9);
  EXPECT_NEAR(-0.1471550758455475, v_out_ptr->d[5], 1e-9);
  EXPECT_NEAR(0.2225229879715527, v_out_ptr->d[6], 1e-9);
  EXPECT_NEAR(0.1116549336849494, v_out_ptr->d[7], 1e-9);
  EXPECT_NEAR(0.2115739711592913, v_out_ptr->d[8], 1e-9);
  EXPECT_NEAR(0.7348433884579327, v_out_ptr->d[9], 1e-9);
  EXPECT_NEAR(-0.0197358626555054, v_out_ptr->d[10], 1e-9);
  EXPECT_NEAR(-0.6519831241982549, v_out_ptr->d[11], 1e-9);
  EXPECT_NEAR(-0.1364012360109416, v_out_ptr->d[12], 1e-9);
  EXPECT_NEAR(0.0563820000416119, v_out_ptr->d[13], 1e-9);
  EXPECT_NEAR(-0.0239744753832868, v_out_ptr->d[14], 1e-9);
  EXPECT_NEAR(-0.2485388366592161, v_out_ptr->d[15], 1e-9);
  EXPECT_NEAR(-0.0422583918302634, v_out_ptr->d[16], 1e-9);
  EXPECT_NEAR(0.0402474536253344, v_out_ptr->d[17], 1e-9);
  EXPECT_NEAR(-0.0363364255346597, v_out_ptr->d[18], 1e-9);
  EXPECT_NEAR(0.1126209501538281, v_out_ptr->d[19], 1e-9);
  EXPECT_NEAR(0.0304823950540730, v_out_ptr->d[20], 1e-9);
  EXPECT_NEAR(0.0909295206998316, v_out_ptr->d[21], 1e-9);
}

TEST(MatArrMult, Normal1) {
  double m1[6][3] = {
      {0.4131532583807510, 0.2770324739911472, 0.0854012131516738},
      {0.5612103119125406, 0.6342885148575477, 0.0041929119804535},
      {0.9901792914165084, 0.2013671676492756, 0.0455535375691162},
      {0.3846707964658805, 0.5545272257489852, 0.9231143121811970},
      {0.3136707945132855, 0.0702696589609046, 0.8888026796419715},
      {0.9595411745223804, 0.1631369106622036, 0.2717748742391329}};
  double m2[3][10] = {
      {0.6626131560171433, 0.0395300391990788, 0.4286264156601270,
       0.5214612256401577, 0.3931175086405422, 0.3178090671162441,
       0.8534039275067485, 0.6891042825524032, 0.2614614893207166,
       0.3264077290267140},
      {0.7329448390824125, 0.8247625380434265, 0.2124606511632106,
       0.7024720249837747, 0.8225189465271959, 0.2662251969429671,
       0.5609816069019357, 0.2076483242655903, 0.0731252162525405,
       0.1725981674974189},
      {0.5828599519720031, 0.6767072712303011, 0.7724557701082033,
       0.0806339549198328, 0.8169067655324139, 0.9448035667621500,
       0.9516408689060810, 0.3572225266780296, 0.7830154659825140,
       0.2269030223520830}};
  double m_out[6][10];
  double m_out2[6][10];
  Mat mn_m1 = {6, 3, &m1[0][0], 0, 0};
  Mat mn_m2 = {3, 10, &m2[0][0], 0, 0};
  Mat mn_m_out = {6, 10, &m_out2[0][0], 0, 0};
  MatArrMult(&m1[0][0], 6, 3, &m2[0][0], 10, &m_out[0][0]);
  MatMult(&mn_m1, &mn_m2, &mn_m_out);

  EXPECT_NEAR(m_out[0][0], 0.5265872535204151, 1e-9);
  EXPECT_NEAR(m_out2[0][0], 0.5265872535204151, 1e-9);
  EXPECT_NEAR(m_out[0][1], 0.3026095927800328, 1e-9);
  EXPECT_NEAR(m_out2[0][1], 0.3026095927800328, 1e-9);
  EXPECT_NEAR(m_out[0][2], 0.3019155599488090, 1e-9);
  EXPECT_NEAR(m_out2[0][2], 0.3019155599488090, 1e-9);
  EXPECT_NEAR(m_out[0][3], 0.4169372050546484, 1e-9);
  EXPECT_NEAR(m_out2[0][3], 0.4169372050546484, 1e-9);
  EXPECT_NEAR(m_out[0][4], 0.4600470670906623, 1e-9);
  EXPECT_NEAR(m_out2[0][4], 0.4600470670906623, 1e-9);
  EXPECT_NEAR(m_out[0][5], 0.2857442473614297, 1e-9);
  EXPECT_NEAR(m_out2[0][5], 0.2857442473614297, 1e-9);
  EXPECT_NEAR(m_out[0][6], 0.5892680204772081, 1e-9);
  EXPECT_NEAR(m_out2[0][6], 0.5892680204772081, 1e-9);
  EXPECT_NEAR(m_out[0][7], 0.3727382458354774, 1e-9);
  EXPECT_NEAR(m_out2[0][7], 0.3727382458354774, 1e-9);
  EXPECT_NEAR(m_out[0][8], 0.1951521965349468, 1e-9);
  EXPECT_NEAR(m_out2[0][8], 0.1951521965349468, 1e-9);
  EXPECT_NEAR(m_out[0][9], 0.2020495075328457, 1e-9);
  EXPECT_NEAR(m_out2[0][9], 0.2020495075328457, 1e-9);
  EXPECT_NEAR(m_out[1][0], 0.8392077098953715, 1e-9);
  EXPECT_NEAR(m_out2[1][0], 0.8392077098953715, 1e-9);
  EXPECT_NEAR(m_out[1][1], 0.5481594450193381, 1e-9);
  EXPECT_NEAR(m_out2[1][1], 0.5481594450193381, 1e-9);
  EXPECT_NEAR(m_out[1][2], 0.3785497543714116, 1e-9);
  EXPECT_NEAR(m_out2[1][2], 0.3785497543714116, 1e-9);
  EXPECT_NEAR(m_out[1][3], 0.7385574456233559, 1e-9);
  EXPECT_NEAR(m_out2[1][3], 0.7385574456233559, 1e-9);
  EXPECT_NEAR(m_out[1][4], 0.7457611388414837, 1e-9);
  EXPECT_NEAR(m_out2[1][4], 0.7457611388414837, 1e-9);
  EXPECT_NEAR(m_out[1][5], 0.3511827886658058, 1e-9);
  EXPECT_NEAR(m_out2[1][5], 0.3511827886658058, 1e-9);
  EXPECT_NEAR(m_out[1][6], 0.8387534210480043, 1e-9);
  EXPECT_NEAR(m_out2[1][6], 0.8387534210480043, 1e-9);
  EXPECT_NEAR(m_out[1][7], 0.5199391791743776, 1e-9);
  EXPECT_NEAR(m_out2[1][7], 0.5199391791743776, 1e-9);
  EXPECT_NEAR(m_out[1][8], 0.1964004837184561, 1e-9);
  EXPECT_NEAR(m_out2[1][8], 0.1964004837184561, 1e-9);
  EXPECT_NEAR(m_out[1][9], 0.2936118031476394, 1e-9);
  EXPECT_NEAR(m_out2[1][9], 0.2936118031476394, 1e-9);
  EXPECT_NEAR(m_out[2][0], 0.8302481843171807, 1e-9);
  EXPECT_NEAR(m_out2[2][0], 0.8302481843171807, 1e-9);
  EXPECT_NEAR(m_out[2][1], 0.2360483325761270, 1e-9);
  EXPECT_NEAR(m_out2[2][1], 0.2360483325761270, 1e-9);
  EXPECT_NEAR(m_out[2][2], 0.5023876930465035, 1e-9);
  EXPECT_NEAR(m_out2[2][2], 0.5023876930465035, 1e-9);
  EXPECT_NEAR(m_out[2][3], 0.6614680708241764, 1e-9);
  EXPECT_NEAR(m_out2[2][3], 0.6614680708241764, 1e-9);
  EXPECT_NEAR(m_out[2][4], 0.5920981197833086, 1e-9);
  EXPECT_NEAR(m_out2[2][4], 0.5920981197833086, 1e-9);
  EXPECT_NEAR(m_out[2][5], 0.4113361155221146, 1e-9);
  EXPECT_NEAR(m_out2[2][5], 0.4113361155221146, 1e-9);
  EXPECT_NEAR(m_out[2][6], 1.0013367815898992, 1e-9);
  EXPECT_NEAR(m_out2[2][6], 1.0013367815898992, 1e-9);
  EXPECT_NEAR(m_out[2][7], 0.7404230949238624, 1e-9);
  EXPECT_NEAR(m_out2[2][7], 0.7404230949238624, 1e-9);
  EXPECT_NEAR(m_out[2][8], 0.3092878943556404, 1e-9);
  EXPECT_NEAR(m_out2[2][8], 0.3092878943556404, 1e-9);
  EXPECT_NEAR(m_out[2][9], 0.3682940133242155, 1e-9);
  EXPECT_NEAR(m_out2[2][9], 0.3682940133242155, 1e-9);
  EXPECT_NEAR(m_out[3][0], 1.1993721623798930, 1e-9);
  EXPECT_NEAR(m_out2[3][0], 1.1993721623798930, 1e-9);
  EXPECT_NEAR(m_out[3][1], 1.0972375010157243, 1e-9);
  EXPECT_NEAR(m_out2[3][1], 1.0972375010157243, 1e-9);
  EXPECT_NEAR(m_out[3][2], 0.9957602570824856, 1e-9);
  EXPECT_NEAR(m_out2[3][2], 0.9957602570824856, 1e-9);
  EXPECT_NEAR(m_out[3][3], 0.6645651260078692, 1e-9);
  EXPECT_NEAR(m_out2[3][3], 0.6645651260078692, 1e-9);
  EXPECT_NEAR(m_out[3][4], 1.3614283016777646, 1e-9);
  EXPECT_NEAR(m_out2[3][4], 1.3614283016777646, 1e-9);
  EXPECT_NEAR(m_out[3][5], 1.1420426815349287, 1e-9);
  EXPECT_NEAR(m_out2[3][5], 1.1420426815349287, 1e-9);
  EXPECT_NEAR(m_out[3][6], 1.5178324488164234, 1e-9);
  EXPECT_NEAR(m_out2[3][6], 1.5178324488164234, 1e-9);
  EXPECT_NEAR(m_out[3][7], 0.7099821694139241, 1e-9);
  EXPECT_NEAR(m_out2[3][7], 0.7099821694139241, 1e-9);
  EXPECT_NEAR(m_out[3][8], 0.8639393059506592, 1e-9);
  EXPECT_NEAR(m_out2[3][8], 0.8639393059506592, 1e-9);
  EXPECT_NEAR(m_out[3][9], 0.4307273314994056, 1e-9);
  EXPECT_NEAR(m_out2[3][9], 0.4307273314994056, 1e-9);
  EXPECT_NEAR(m_out[4][0], 0.7773936661510364, 1e-9);
  EXPECT_NEAR(m_out2[4][0], 0.7773936661510364, 1e-9);
  EXPECT_NEAR(m_out[4][1], 0.6718144370774559, 1e-9);
  EXPECT_NEAR(m_out2[4][1], 0.6718144370774559, 1e-9);
  EXPECT_NEAR(m_out[4][2], 0.8359378842264180, 1e-9);
  EXPECT_NEAR(m_out2[4][2], 0.8359378842264180, 1e-9);
  EXPECT_NEAR(m_out[4][3], 0.2845973017824832, 1e-9);
  EXPECT_NEAR(m_out2[4][3], 0.2845973017824832, 1e-9);
  EXPECT_NEAR(m_out[4][4], 0.9071765293565760, 1e-9);
  EXPECT_NEAR(m_out2[4][4], 0.9071765293565760, 1e-9);
  EXPECT_NEAR(m_out[4][5], 0.9581389182553516, 1e-9);
  EXPECT_NEAR(m_out2[4][5], 0.9581389182553516, 1e-9);
  EXPECT_NEAR(m_out[4][6], 1.1529288285226782, 1e-9);
  EXPECT_NEAR(m_out2[4][6], 1.1529288285226782, 1e-9);
  EXPECT_NEAR(m_out[4][7], 0.5482436036805746, 1e-9);
  EXPECT_NEAR(m_out2[4][7], 0.5482436036805746, 1e-9);
  EXPECT_NEAR(m_out[4][8], 0.7830975614637299, 1e-9);
  EXPECT_NEAR(m_out2[4][8], 0.7830975614637299, 1e-9);
  EXPECT_NEAR(m_out[4][9], 0.3161850003518009, 1e-9);
  EXPECT_NEAR(m_out2[4][9], 0.3161850003518009, 1e-9);
  EXPECT_NEAR(m_out[5][0], 0.9137816528586000, 1e-9);
  EXPECT_NEAR(m_out2[5][0], 0.9137816528586000, 1e-9);
  EXPECT_NEAR(m_out[5][1], 0.3563919462636445, 1e-9);
  EXPECT_NEAR(m_out2[5][1], 0.3563919462636445, 1e-9);
  EXPECT_NEAR(m_out[5][2], 0.6558789383583321, 1e-9);
  EXPECT_NEAR(m_out2[5][2], 0.6558789383583321, 1e-9);
  EXPECT_NEAR(m_out[5][3], 0.6368769158588538, 1e-9);
  EXPECT_NEAR(m_out2[5][3], 0.6368769158588538, 1e-9);
  EXPECT_NEAR(m_out[5][4], 0.7334103693315035, 1e-9);
  EXPECT_NEAR(m_out2[5][4], 0.7334103693315035, 1e-9);
  EXPECT_NEAR(m_out[5][5], 0.6051559122417629, 1e-9);
  EXPECT_NEAR(m_out2[5][5], 0.6051559122417629, 1e-9);
  EXPECT_NEAR(m_out[5][6], 1.1690250906979078, 1e-9);
  EXPECT_NEAR(m_out2[5][6], 1.1690250906979078, 1e-9);
  EXPECT_NEAR(m_out[5][7], 0.7921831460369140, 1e-9);
  EXPECT_NEAR(m_out2[5][7], 0.7921831460369140, 1e-9);
  EXPECT_NEAR(m_out[5][8], 0.4756164162208100, 1e-9);
  EXPECT_NEAR(m_out2[5][8], 0.4756164162208100, 1e-9);
  EXPECT_NEAR(m_out[5][9], 0.4030253278791791, 1e-9);
  EXPECT_NEAR(m_out2[5][9], 0.4030253278791791, 1e-9);
}

#if !defined(NDEBUG)

TEST(MatArrMultDeath, Death1) {
  double m1[4][4] = {{0.6383601244647858, 0.4742713240242661,
                      0.9427368822799536, 0.4284900847261250},
                     {0.3197724383425889, 0.2218828807148873,
                      0.2640319815844738, 0.8121988924262720},
                     {0.5938896775104204, 0.5255292571276671,
                      0.7594743934506832, 0.3646909083052029},
                     {0.0262826720414517, 0.2729890691691303,
                      0.5870971798349912, 0.5245854393818770}};
  double m_out[4][4] = {{0.0942591868719515, 0.9597444474070018,
                         0.1258847745573192, 0.3679145744836456},
                        {0.8550960415417427, 0.3341733211727912,
                         0.7719398492711619, 0.8348783360557633},
                        {0.1237092975333646, 0.0554619719361580,
                         0.7932985604293615, 0.4117266408456941},
                        {0.0157633750641647, 0.9619301359286282,
                         0.9340138635065744, 0.3744458529184138}};
  ASSERT_DEATH(MatArrMult(&m1[0][0], 4, 4, &m_out[0][0], 4, &m_out[0][0]),
               "MatArrMult: Assertion `m_out != m1 && m_out != m2' failed.");
  ASSERT_DEATH(MatArrMult(&m_out[0][0], 4, 4, &m1[0][0], 4, &m_out[0][0]),
               "MatArrMult: Assertion `m_out != m1 && m_out != m2' failed.");
}

#endif  // !defined(NDEBUG)

TEST(MatArrZero, Normal1) {
  double m[11][7] = {
      {0.3078082493885717, 0.8205268733214018, 0.5197023421048064,
       0.0671327135090609, 0.5030276056060133, 0.5091803726711820,
       0.1259589475933762},
      {0.6820972121860189, 0.9799460850775068, 0.8157654685479849,
       0.1751577928706953, 0.2000823407253774, 0.7368298972134577,
       0.0635483050904347},
      {0.2600049900102760, 0.9432276653852742, 0.9227763628907842,
       0.7526836460595202, 0.2702264582941426, 0.0909111796875123,
       0.8066176478193293},
      {0.2674556582036548, 0.8449000836196868, 0.6057688840535517,
       0.3690845555156615, 0.3052021478797554, 0.0327398643765952,
       0.6364895634071323},
      {0.6914395703523782, 0.3809394464276248, 0.2847342129936274,
       0.2448907870416600, 0.9680445714279662, 0.4672968111430811,
       0.9400762945228681},
      {0.0809631282865836, 0.2357440214763359, 0.0316971220531749,
       0.1165989648223857, 0.6138320384752119, 0.6238283935506821,
       0.7535629534075741},
      {0.2719865332769873, 0.5572171177711618, 0.0040629291804535,
       0.7975358992113660, 0.1260556423232363, 0.7054492011915998,
       0.7660813563769530},
      {0.9884229256947773, 0.2838192348003620, 0.3506268462526919,
       0.9085032564116519, 0.4410764826405467, 0.1606017784671356,
       0.3969322981337936},
      {0.9332883077142847, 0.4799578157549340, 0.0987847472729766,
       0.7503959145358586, 0.3023094821563155, 0.4352486471093097,
       0.6889428664590461},
      {0.5133089620443163, 0.7422153753019254, 0.4397202752279118,
       0.3827341618353866, 0.8899635493055978, 0.2524944187788708,
       0.1400512363535208},
      {0.0384310349652917, 0.6844399844499491, 0.5464142406138531,
       0.3500771332681888, 0.4305400574132807, 0.0976877524764610,
       0.0011659293566557}};
  MatArrZero(11, 7, &m[0][0]);

  EXPECT_NEAR(m[0][0], 0.0, 1e-9);
  EXPECT_NEAR(m[0][1], 0.0, 1e-9);
  EXPECT_NEAR(m[0][2], 0.0, 1e-9);
  EXPECT_NEAR(m[0][3], 0.0, 1e-9);
  EXPECT_NEAR(m[0][4], 0.0, 1e-9);
  EXPECT_NEAR(m[0][5], 0.0, 1e-9);
  EXPECT_NEAR(m[0][6], 0.0, 1e-9);
  EXPECT_NEAR(m[1][0], 0.0, 1e-9);
  EXPECT_NEAR(m[1][1], 0.0, 1e-9);
  EXPECT_NEAR(m[1][2], 0.0, 1e-9);
  EXPECT_NEAR(m[1][3], 0.0, 1e-9);
  EXPECT_NEAR(m[1][4], 0.0, 1e-9);
  EXPECT_NEAR(m[1][5], 0.0, 1e-9);
  EXPECT_NEAR(m[1][6], 0.0, 1e-9);
  EXPECT_NEAR(m[2][0], 0.0, 1e-9);
  EXPECT_NEAR(m[2][1], 0.0, 1e-9);
  EXPECT_NEAR(m[2][2], 0.0, 1e-9);
  EXPECT_NEAR(m[2][3], 0.0, 1e-9);
  EXPECT_NEAR(m[2][4], 0.0, 1e-9);
  EXPECT_NEAR(m[2][5], 0.0, 1e-9);
  EXPECT_NEAR(m[2][6], 0.0, 1e-9);
  EXPECT_NEAR(m[3][0], 0.0, 1e-9);
  EXPECT_NEAR(m[3][1], 0.0, 1e-9);
  EXPECT_NEAR(m[3][2], 0.0, 1e-9);
  EXPECT_NEAR(m[3][3], 0.0, 1e-9);
  EXPECT_NEAR(m[3][4], 0.0, 1e-9);
  EXPECT_NEAR(m[3][5], 0.0, 1e-9);
  EXPECT_NEAR(m[3][6], 0.0, 1e-9);
  EXPECT_NEAR(m[4][0], 0.0, 1e-9);
  EXPECT_NEAR(m[4][1], 0.0, 1e-9);
  EXPECT_NEAR(m[4][2], 0.0, 1e-9);
  EXPECT_NEAR(m[4][3], 0.0, 1e-9);
  EXPECT_NEAR(m[4][4], 0.0, 1e-9);
  EXPECT_NEAR(m[4][5], 0.0, 1e-9);
  EXPECT_NEAR(m[4][6], 0.0, 1e-9);
  EXPECT_NEAR(m[5][0], 0.0, 1e-9);
  EXPECT_NEAR(m[5][1], 0.0, 1e-9);
  EXPECT_NEAR(m[5][2], 0.0, 1e-9);
  EXPECT_NEAR(m[5][3], 0.0, 1e-9);
  EXPECT_NEAR(m[5][4], 0.0, 1e-9);
  EXPECT_NEAR(m[5][5], 0.0, 1e-9);
  EXPECT_NEAR(m[5][6], 0.0, 1e-9);
  EXPECT_NEAR(m[6][0], 0.0, 1e-9);
  EXPECT_NEAR(m[6][1], 0.0, 1e-9);
  EXPECT_NEAR(m[6][2], 0.0, 1e-9);
  EXPECT_NEAR(m[6][3], 0.0, 1e-9);
  EXPECT_NEAR(m[6][4], 0.0, 1e-9);
  EXPECT_NEAR(m[6][5], 0.0, 1e-9);
  EXPECT_NEAR(m[6][6], 0.0, 1e-9);
  EXPECT_NEAR(m[7][0], 0.0, 1e-9);
  EXPECT_NEAR(m[7][1], 0.0, 1e-9);
  EXPECT_NEAR(m[7][2], 0.0, 1e-9);
  EXPECT_NEAR(m[7][3], 0.0, 1e-9);
  EXPECT_NEAR(m[7][4], 0.0, 1e-9);
  EXPECT_NEAR(m[7][5], 0.0, 1e-9);
  EXPECT_NEAR(m[7][6], 0.0, 1e-9);
  EXPECT_NEAR(m[8][0], 0.0, 1e-9);
  EXPECT_NEAR(m[8][1], 0.0, 1e-9);
  EXPECT_NEAR(m[8][2], 0.0, 1e-9);
  EXPECT_NEAR(m[8][3], 0.0, 1e-9);
  EXPECT_NEAR(m[8][4], 0.0, 1e-9);
  EXPECT_NEAR(m[8][5], 0.0, 1e-9);
  EXPECT_NEAR(m[8][6], 0.0, 1e-9);
  EXPECT_NEAR(m[9][0], 0.0, 1e-9);
  EXPECT_NEAR(m[9][1], 0.0, 1e-9);
  EXPECT_NEAR(m[9][2], 0.0, 1e-9);
  EXPECT_NEAR(m[9][3], 0.0, 1e-9);
  EXPECT_NEAR(m[9][4], 0.0, 1e-9);
  EXPECT_NEAR(m[9][5], 0.0, 1e-9);
  EXPECT_NEAR(m[9][6], 0.0, 1e-9);
  EXPECT_NEAR(m[10][0], 0.0, 1e-9);
  EXPECT_NEAR(m[10][1], 0.0, 1e-9);
  EXPECT_NEAR(m[10][2], 0.0, 1e-9);
  EXPECT_NEAR(m[10][3], 0.0, 1e-9);
  EXPECT_NEAR(m[10][4], 0.0, 1e-9);
  EXPECT_NEAR(m[10][5], 0.0, 1e-9);
  EXPECT_NEAR(m[10][6], 0.0, 1e-9);
}

TEST(MatArrCopy, Normal1) {
  double m[6][3] = {
      {0.9420978183731700, 0.6252552401222043, 0.1447821073673879},
      {0.6428133538523331, 0.8919727967575183, 0.0144496006965831},
      {0.8380670817301392, 0.1751117247596324, 0.2450662167044138},
      {0.4832432280463314, 0.0399691746186339, 0.3122292765707319},
      {0.5163697911498977, 0.9898510145994296, 0.2693926478708651},
      {0.4088692970363328, 0.9869187185994002, 0.9832366635824450}};
  double m_out[6][3];
  MatArrCopy(&m[0][0], 6, 3, &m_out[0][0]);
  MatArrCopy(&m[0][0], 6, 3, &m[0][0]);

  EXPECT_NEAR(m_out[0][0], m[0][0], 1e-9);
  EXPECT_NEAR(m_out[0][1], m[0][1], 1e-9);
  EXPECT_NEAR(m_out[0][2], m[0][2], 1e-9);
  EXPECT_NEAR(m_out[1][0], m[1][0], 1e-9);
  EXPECT_NEAR(m_out[1][1], m[1][1], 1e-9);
  EXPECT_NEAR(m_out[1][2], m[1][2], 1e-9);
  EXPECT_NEAR(m_out[2][0], m[2][0], 1e-9);
  EXPECT_NEAR(m_out[2][1], m[2][1], 1e-9);
  EXPECT_NEAR(m_out[2][2], m[2][2], 1e-9);
  EXPECT_NEAR(m_out[3][0], m[3][0], 1e-9);
  EXPECT_NEAR(m_out[3][1], m[3][1], 1e-9);
  EXPECT_NEAR(m_out[3][2], m[3][2], 1e-9);
  EXPECT_NEAR(m_out[4][0], m[4][0], 1e-9);
  EXPECT_NEAR(m_out[4][1], m[4][1], 1e-9);
  EXPECT_NEAR(m_out[4][2], m[4][2], 1e-9);
  EXPECT_NEAR(m_out[5][0], m[5][0], 1e-9);
  EXPECT_NEAR(m_out[5][1], m[5][1], 1e-9);
  EXPECT_NEAR(m_out[5][2], m[5][2], 1e-9);
}

TEST(MatVecMult, Normal1) {
  double m[5][8] = {{0.9509570276351892, 0.9864774449658069, 0.6915820181482772,
                     0.5464959020818011, 0.5215243377925536, 0.1575973177488884,
                     0.5660690400367572, 0.2057486839817455},
                    {0.8938968058986415, 0.0301171541617905, 0.9457079839347835,
                     0.3570419340511709, 0.1077766969200286, 0.5804397376938395,
                     0.1726276514870474, 0.9256889564534569},
                    {0.5921971663837968, 0.0794552119252938, 0.1901710150564790,
                     0.6263893294402747, 0.0412894096472259, 0.6098430810592833,
                     0.5116526147793563, 0.0423838748818495},
                    {0.7743443869611426, 0.6206946445815347, 0.9534525192507349,
                     0.0875523458627195, 0.8224594991651247, 0.3296423997504705,
                     0.0486095624489689, 0.3756607586203512},
                    {0.1073166891203107, 0.7322274285654141, 0.4101331756755444,
                     0.6250704943710710, 0.4170894050775904, 0.2045495067528662,
                     0.2053242489376533, 0.0614590136944551}};
  double v[8] = {0.0492703053559863, 0.3286616526140038, 0.5169999993238682,
                 0.8356775928415320, 0.3997548833868220, 0.4945881196883215,
                 0.7912837812525463, 0.0665105614103573};
  double v_out[5];
  Mat mn_m = {5, 8, &m[0][0], 0, 0};
  Vec n_v = {8, v, 0, 0};
  Vec n_v_out = {5, v_out, 0, 0};
  MatVecMult(&mn_m, &n_v, &n_v_out);

  EXPECT_NEAR(v_out[0], 1.9333469061477984, 1e-9);
  EXPECT_NEAR(v_out[1], 1.3695723056584053, 1e-9);
  EXPECT_NEAR(v_out[2], 1.4028777350709125, 1e-9);
  EXPECT_NEAR(v_out[3], 1.3635199796758570, 1e-9);
  EXPECT_NEAR(v_out[4], 1.4147975644471131, 1e-9);
}

TEST(VecNorm, Normal11) {
  double d_v[22] = {
      -0.1100109621770002, -0.8833672133011188, -0.2487986717536741,
      0.8531060222301550,  -0.3080589063297627, -0.6387224346445426,
      -0.0176284551096006, 0.4681759992925276,  -0.1720121033402846,
      0.7185631244816173,  0.8000489055785680,  -0.6713275131477123,
      -0.9767023888403223, -0.5924153179235023, 0.2917129134682783,
      -0.0229744924296180, 0.0731653062306019,  -0.6572743596753186,
      -0.3344568084912554, -0.9525409734186894, 0.9950094060303027,
      -0.6324684322123675};
  const Vec v = {22, &d_v[0], 0, 22};
  double v_norm = VecNorm(&v);

  EXPECT_NEAR(2.8597767476484721, v_norm, 1e-9);
}

TEST(VecNormBound, Unbounded11) {
  double d_v[22] = {
      -0.9649982298651285, -0.5524946660321028, -0.8484261092524812,
      0.0377309904029146,  0.3320159292486728,  0.0352285587751155,
      0.0880983965374103,  -0.8272792605898138, -0.3284528104327844,
      0.5937683378051541,  0.8394608985890082,  -0.4208143241743842,
      0.2466719013529106,  0.1506064551089876,  -0.7599159708204948,
      -0.0261459789556349, 0.7313963588730057,  -0.7204611026880139,
      0.0035688317711942,  -0.6638592481068311, 0.9053732737297073,
      -0.2247066636927779};
  const Vec v = {22, &d_v[0], 0, 22};
  double v_norm = VecNormBound(&v, 2.5772399641403418);

  EXPECT_NEAR(2.6702392200621854, v_norm, 1e-9);
}

TEST(VecNormBound, Bounded11) {
  double d_v[22] = {
      -0.9562070189799798, 0.3173931797630714,  -0.4764562383531761,
      0.5161078859970087,  0.4714548991284553,  0.8773868212849392,
      0.2797184527838759,  0.2021326280264690,  -0.8727591982691534,
      -0.6749206215474362, -0.9833890408509380, -0.4998133701660001,
      -0.2179104215093683, 0.4302277242409145,  0.2737519050045480,
      -0.8783716160795012, -0.6859841381015805, -0.1165460744903835,
      0.1026483012177835,  0.6085005246520996,  -0.8024722025816338,
      0.1908933796555754};
  const Vec v = {22, &d_v[0], 0, 22};
  double v_norm = VecNormBound(&v, 3.3003765935589366);

  EXPECT_NEAR(3.3003765935589366, v_norm, 1e-9);
}

TEST(VecNormSquared, Normal11) {
  double d_v[22] = {
      0.4356032053605570,  -0.7493148449752967, 0.9943044935531562,
      -0.2055138157847696, -0.4410203911578967, 0.3207115526663062,
      0.0928300162116196,  -0.8038308480009277, 0.0423446431172041,
      0.8331455033020696,  -0.7146003340252791, -0.3900964687724564,
      -0.5319924083365382, 0.9708721303689054,  0.2519215441135905,
      0.5906430812544756,  -0.8424121997971388, 0.1526871834063865,
      0.3362018231061179,  -0.8856633788498081, 0.0650147107591961,
      0.3970761067341739};
  const Vec v = {22, &d_v[0], 0, 22};
  double v_norm_sqrd = VecNormSquared(&v);

  EXPECT_NEAR(7.5232016180191188, v_norm_sqrd, 1e-9);
}

TEST(VecNormalize, Normal11) {
  double d_v[22] = {
      -0.2632489615556646, 0.8121793985349179,  -0.4218138698848943,
      0.6316630891714083,  0.7315002638765864,  0.5532072169421960,
      -0.3231264845465596, 0.4926828963309349,  -0.6851402207911683,
      -0.4747206331003289, 0.3134556281974819,  0.0640881215109703,
      0.7918892750225757,  0.2992830389151169,  -0.0444570541300666,
      0.1029802355240910,  0.1675167712101195,  -0.7845089469968247,
      0.2217753159218356,  -0.6721057958061174, -0.0292463531416576,
      -0.9889965857041441};
  Vec v = {22, &d_v[0], 0, 22};
  const Vec *v_out_ptr = VecNormalize(&v, &v);

  EXPECT_NEAR(-0.1062047921772525, v.d[0], 1e-9);
  EXPECT_NEAR(0.3276645184934855, v.d[1], 1e-9);
  EXPECT_NEAR(-0.1701759966074361, v.d[2], 1e-9);
  EXPECT_NEAR(0.2548372715890290, v.d[3], 1e-9);
  EXPECT_NEAR(0.2951154414570816, v.d[4], 1e-9);
  EXPECT_NEAR(0.2231851444317234, v.d[5], 1e-9);
  EXPECT_NEAR(-0.1303616961504216, v.d[6], 1e-9);
  EXPECT_NEAR(0.1987672973329070, v.d[7], 1e-9);
  EXPECT_NEAR(-0.2764120106358581, v.d[8], 1e-9);
  EXPECT_NEAR(-0.1915206270244423, v.d[9], 1e-9);
  EXPECT_NEAR(0.1264600994160592, v.d[10], 1e-9);
  EXPECT_NEAR(0.0258556219400845, v.d[11], 1e-9);
  EXPECT_NEAR(0.3194786994948285, v.d[12], 1e-9);
  EXPECT_NEAR(0.1207423298550619, v.d[13], 1e-9);
  EXPECT_NEAR(-0.0179356916236048, v.d[14], 1e-9);
  EXPECT_NEAR(0.0415462019206786, v.d[15], 1e-9);
  EXPECT_NEAR(0.0675827314472165, v.d[16], 1e-9);
  EXPECT_NEAR(-0.3165011902976684, v.d[17], 1e-9);
  EXPECT_NEAR(0.0894727226969236, v.d[18], 1e-9);
  EXPECT_NEAR(-0.2711534204841374, v.d[19], 1e-9);
  EXPECT_NEAR(-0.0117991077305560, v.d[20], 1e-9);
  EXPECT_NEAR(-0.3989993967232073, v.d[21], 1e-9);

  EXPECT_NEAR(-0.1062047921772525, v_out_ptr->d[0], 1e-9);
  EXPECT_NEAR(0.3276645184934855, v_out_ptr->d[1], 1e-9);
  EXPECT_NEAR(-0.1701759966074361, v_out_ptr->d[2], 1e-9);
  EXPECT_NEAR(0.2548372715890290, v_out_ptr->d[3], 1e-9);
  EXPECT_NEAR(0.2951154414570816, v_out_ptr->d[4], 1e-9);
  EXPECT_NEAR(0.2231851444317234, v_out_ptr->d[5], 1e-9);
  EXPECT_NEAR(-0.1303616961504216, v_out_ptr->d[6], 1e-9);
  EXPECT_NEAR(0.1987672973329070, v_out_ptr->d[7], 1e-9);
  EXPECT_NEAR(-0.2764120106358581, v_out_ptr->d[8], 1e-9);
  EXPECT_NEAR(-0.1915206270244423, v_out_ptr->d[9], 1e-9);
  EXPECT_NEAR(0.1264600994160592, v_out_ptr->d[10], 1e-9);
  EXPECT_NEAR(0.0258556219400845, v_out_ptr->d[11], 1e-9);
  EXPECT_NEAR(0.3194786994948285, v_out_ptr->d[12], 1e-9);
  EXPECT_NEAR(0.1207423298550619, v_out_ptr->d[13], 1e-9);
  EXPECT_NEAR(-0.0179356916236048, v_out_ptr->d[14], 1e-9);
  EXPECT_NEAR(0.0415462019206786, v_out_ptr->d[15], 1e-9);
  EXPECT_NEAR(0.0675827314472165, v_out_ptr->d[16], 1e-9);
  EXPECT_NEAR(-0.3165011902976684, v_out_ptr->d[17], 1e-9);
  EXPECT_NEAR(0.0894727226969236, v_out_ptr->d[18], 1e-9);
  EXPECT_NEAR(-0.2711534204841374, v_out_ptr->d[19], 1e-9);
  EXPECT_NEAR(-0.0117991077305560, v_out_ptr->d[20], 1e-9);
  EXPECT_NEAR(-0.3989993967232073, v_out_ptr->d[21], 1e-9);
}

TEST(VecNormalize, Zero_11) {
  double d_v[22] = {0.0000000000000000, 0.0000000000000000, 0.0000000000000000,
                    0.0000000000000000, 0.0000000000000000, 0.0000000000000000,
                    0.0000000000000000, 0.0000000000000000, 0.0000000000000000,
                    0.0000000000000000, 0.0000000000000000, 0.0000000000000000,
                    0.0000000000000000, 0.0000000000000000, 0.0000000000000000,
                    0.0000000000000000, 0.0000000000000000, 0.0000000000000000,
                    0.0000000000000000, 0.0000000000000000, 0.0000000000000000,
                    0.0000000000000000};
  Vec v = {22, &d_v[0], 0, 22};
  const Vec *v_out_ptr = VecNormalize(&v, &v);

  EXPECT_NEAR(0.0000000000000000, v.d[0], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v.d[1], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v.d[2], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v.d[3], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v.d[4], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v.d[5], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v.d[6], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v.d[7], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v.d[8], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v.d[9], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v.d[10], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v.d[11], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v.d[12], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v.d[13], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v.d[14], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v.d[15], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v.d[16], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v.d[17], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v.d[18], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v.d[19], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v.d[20], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v.d[21], 1e-9);

  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[0], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[1], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[2], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[3], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[4], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[5], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[6], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[7], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[8], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[9], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[10], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[11], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[12], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[13], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[14], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[15], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[16], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[17], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[18], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[19], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[20], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[21], 1e-9);
}

TEST(VecScale, Normal11) {
  double d_v[22] = {
      0.3891972254983314,  0.3093173656043993,  0.2514419847725373,
      0.0745060187582665,  0.2519470194641922,  0.8242540145802082,
      0.5391763594454302,  -0.2652649127328148, -0.2646232016210217,
      -0.5327070972629686, -0.3213231289798881, 0.4306218729056703,
      0.5050438916802757,  -0.6310698933329262, -0.5876877754283225,
      0.1998063098177549,  -0.7711880892682501, -0.2217329771641636,
      -0.9409634608230577, 0.6451379938511250,  -0.4318477601727486,
      -0.2326279670873086};
  const Vec v = {22, &d_v[0], 0, 22};
  double d_v_out[22] = {
      0.4177774658560467,  -0.9760926757976367, 0.3928427416344222,
      0.5738991416603991,  -0.2063911991395844, -0.8508898443078785,
      -0.0814162127853619, -0.7880534355232107, 0.1039748811156525,
      0.1055043116534093,  -0.5082563368815882, -0.3993842789353494,
      0.3991478656948395,  0.4971913633099256,  0.0741676578177424,
      -0.1776028927677584, 0.5782352904966861,  -0.2227233661454351,
      -0.2083102370938628, 0.8266495696917497,  -0.7736640010489264,
      0.4912087059486971};
  Vec v_out = {22, &d_v_out[0], 0, 22};
  const Vec *v_out_ptr = VecScale(&v, -0.7542552993400788, &v_out);

  EXPECT_NEAR(-0.2935540698205721, v_out.d[0], 1e-9);
  EXPECT_NEAR(-0.2333042621850308, v_out.d[1], 1e-9);
  EXPECT_NEAR(-0.1896514494912737, v_out.d[2], 1e-9);
  EXPECT_NEAR(-0.0561965594811538, v_out.d[3], 1e-9);
  EXPECT_NEAR(-0.1900323745838050, v_out.d[4], 1e-9);
  EXPECT_NEAR(-0.6216979584994566, v_out.d[5], 1e-9);
  EXPECT_NEAR(-0.4066766263906069, v_out.d[6], 1e-9);
  EXPECT_NEAR(0.2000774661577091, v_out.d[7], 1e-9);
  EXPECT_NEAR(0.1995934521509938, v_out.d[8], 1e-9);
  EXPECT_NEAR(0.4017971511066649, v_out.d[9], 1e-9);
  EXPECT_NEAR(0.2423596728336163, v_out.d[10], 1e-9);
  EXPECT_NEAR(-0.3247988296508518, v_out.d[11], 1e-9);
  EXPECT_NEAR(-0.3809320316991847, v_out.d[12], 1e-9);
  EXPECT_NEAR(0.4759878113003378, v_out.d[13], 1e-9);
  EXPECT_NEAR(0.4432666189741944, v_out.d[14], 1e-9);
  EXPECT_NEAR(-0.1507049680216273, v_out.d[15], 1e-9);
  EXPECT_NEAR(0.5816727031185275, v_out.d[16], 1e-9);
  EXPECT_NEAR(0.1672432730645231, v_out.d[17], 1e-9);
  EXPECT_NEAR(0.7097266768111720, v_out.d[18], 1e-9);
  EXPECT_NEAR(-0.4865987506678383, v_out.d[19], 1e-9);
  EXPECT_NEAR(0.3257234616184391, v_out.d[20], 1e-9);
  EXPECT_NEAR(0.1754608769503120, v_out.d[21], 1e-9);

  EXPECT_NEAR(-0.2935540698205721, v_out_ptr->d[0], 1e-9);
  EXPECT_NEAR(-0.2333042621850308, v_out_ptr->d[1], 1e-9);
  EXPECT_NEAR(-0.1896514494912737, v_out_ptr->d[2], 1e-9);
  EXPECT_NEAR(-0.0561965594811538, v_out_ptr->d[3], 1e-9);
  EXPECT_NEAR(-0.1900323745838050, v_out_ptr->d[4], 1e-9);
  EXPECT_NEAR(-0.6216979584994566, v_out_ptr->d[5], 1e-9);
  EXPECT_NEAR(-0.4066766263906069, v_out_ptr->d[6], 1e-9);
  EXPECT_NEAR(0.2000774661577091, v_out_ptr->d[7], 1e-9);
  EXPECT_NEAR(0.1995934521509938, v_out_ptr->d[8], 1e-9);
  EXPECT_NEAR(0.4017971511066649, v_out_ptr->d[9], 1e-9);
  EXPECT_NEAR(0.2423596728336163, v_out_ptr->d[10], 1e-9);
  EXPECT_NEAR(-0.3247988296508518, v_out_ptr->d[11], 1e-9);
  EXPECT_NEAR(-0.3809320316991847, v_out_ptr->d[12], 1e-9);
  EXPECT_NEAR(0.4759878113003378, v_out_ptr->d[13], 1e-9);
  EXPECT_NEAR(0.4432666189741944, v_out_ptr->d[14], 1e-9);
  EXPECT_NEAR(-0.1507049680216273, v_out_ptr->d[15], 1e-9);
  EXPECT_NEAR(0.5816727031185275, v_out_ptr->d[16], 1e-9);
  EXPECT_NEAR(0.1672432730645231, v_out_ptr->d[17], 1e-9);
  EXPECT_NEAR(0.7097266768111720, v_out_ptr->d[18], 1e-9);
  EXPECT_NEAR(-0.4865987506678383, v_out_ptr->d[19], 1e-9);
  EXPECT_NEAR(0.3257234616184391, v_out_ptr->d[20], 1e-9);
  EXPECT_NEAR(0.1754608769503120, v_out_ptr->d[21], 1e-9);
}

TEST(VecAdd, Normal11) {
  double d_v0[22] = {
      0.2852259321396726,  -0.7545532018165568, 0.2498859515090923,
      -0.8003988584264288, 0.6763823679817702,  -0.4340090348327306,
      0.2015146075941967,  -0.6712121111145815, 0.4950947757421023,
      -0.8344435815609315, -0.3427605956503834, 0.9081229731489324,
      0.7176210482974075,  0.8714332410350654,  0.1614592280229508,
      0.2751242081258358,  0.4570432548467853,  0.6665602105763884,
      0.0346481441651152,  0.4500164739413597,  0.1866882849423008,
      -0.1373328853058255};
  const Vec v0 = {22, &d_v0[0], 0, 22};
  double d_v1[22] = {
      0.2483659778533693,  0.9590135411390399,  -0.0624345643097062,
      0.5500204460228495,  0.9728939381043522,  0.6861117626605679,
      -0.4226027721853169, -0.2330118387592881, 0.5667113791024601,
      -0.6647436275315521, -0.7026509679434849, 0.8391704034259615,
      -0.2739648987245673, -0.7981529674931112, 0.7771871464714430,
      0.3455939501447254,  -0.1818695754298538, 0.1404363395848263,
      0.8648262070352184,  0.7782026922239489,  0.2413421744324884,
      -0.2466904721065077};
  const Vec v1 = {22, &d_v1[0], 0, 22};
  double d_v_out[22] = {
      -0.7122178004279416, 0.4794307672520730,  -0.6995559837969458,
      0.6918400988019788,  0.5792357295383386,  -0.7481257788167726,
      0.1097089608604480,  -0.2879632718382665, -0.7733259138671746,
      -0.1069506350603227, 0.2785304547600651,  0.9370798059092724,
      0.9921063400506829,  0.8847683691891943,  0.4115663150252937,
      0.2489981114675761,  0.6278509363625151,  -0.2053475477751243,
      0.3507819181261294,  0.9236587572502286,  -0.1757686002236918,
      -0.0652332868182077};
  Vec v_out = {22, &d_v_out[0], 0, 22};
  const Vec *v_out_ptr = VecAdd(&v0, &v1, &v_out);

  EXPECT_NEAR(0.5335919099930420, v_out.d[0], 1e-9);
  EXPECT_NEAR(0.2044603393224831, v_out.d[1], 1e-9);
  EXPECT_NEAR(0.1874513871993861, v_out.d[2], 1e-9);
  EXPECT_NEAR(-0.2503784124035793, v_out.d[3], 1e-9);
  EXPECT_NEAR(1.6492763060861224, v_out.d[4], 1e-9);
  EXPECT_NEAR(0.2521027278278374, v_out.d[5], 1e-9);
  EXPECT_NEAR(-0.2210881645911202, v_out.d[6], 1e-9);
  EXPECT_NEAR(-0.9042239498738696, v_out.d[7], 1e-9);
  EXPECT_NEAR(1.0618061548445623, v_out.d[8], 1e-9);
  EXPECT_NEAR(-1.4991872090924836, v_out.d[9], 1e-9);
  EXPECT_NEAR(-1.0454115635938683, v_out.d[10], 1e-9);
  EXPECT_NEAR(1.7472933765748939, v_out.d[11], 1e-9);
  EXPECT_NEAR(0.4436561495728402, v_out.d[12], 1e-9);
  EXPECT_NEAR(0.0732802735419542, v_out.d[13], 1e-9);
  EXPECT_NEAR(0.9386463744943938, v_out.d[14], 1e-9);
  EXPECT_NEAR(0.6207181582705612, v_out.d[15], 1e-9);
  EXPECT_NEAR(0.2751736794169315, v_out.d[16], 1e-9);
  EXPECT_NEAR(0.8069965501612146, v_out.d[17], 1e-9);
  EXPECT_NEAR(0.8994743512003336, v_out.d[18], 1e-9);
  EXPECT_NEAR(1.2282191661653086, v_out.d[19], 1e-9);
  EXPECT_NEAR(0.4280304593747892, v_out.d[20], 1e-9);
  EXPECT_NEAR(-0.3840233574123333, v_out.d[21], 1e-9);

  EXPECT_NEAR(0.5335919099930420, v_out_ptr->d[0], 1e-9);
  EXPECT_NEAR(0.2044603393224831, v_out_ptr->d[1], 1e-9);
  EXPECT_NEAR(0.1874513871993861, v_out_ptr->d[2], 1e-9);
  EXPECT_NEAR(-0.2503784124035793, v_out_ptr->d[3], 1e-9);
  EXPECT_NEAR(1.6492763060861224, v_out_ptr->d[4], 1e-9);
  EXPECT_NEAR(0.2521027278278374, v_out_ptr->d[5], 1e-9);
  EXPECT_NEAR(-0.2210881645911202, v_out_ptr->d[6], 1e-9);
  EXPECT_NEAR(-0.9042239498738696, v_out_ptr->d[7], 1e-9);
  EXPECT_NEAR(1.0618061548445623, v_out_ptr->d[8], 1e-9);
  EXPECT_NEAR(-1.4991872090924836, v_out_ptr->d[9], 1e-9);
  EXPECT_NEAR(-1.0454115635938683, v_out_ptr->d[10], 1e-9);
  EXPECT_NEAR(1.7472933765748939, v_out_ptr->d[11], 1e-9);
  EXPECT_NEAR(0.4436561495728402, v_out_ptr->d[12], 1e-9);
  EXPECT_NEAR(0.0732802735419542, v_out_ptr->d[13], 1e-9);
  EXPECT_NEAR(0.9386463744943938, v_out_ptr->d[14], 1e-9);
  EXPECT_NEAR(0.6207181582705612, v_out_ptr->d[15], 1e-9);
  EXPECT_NEAR(0.2751736794169315, v_out_ptr->d[16], 1e-9);
  EXPECT_NEAR(0.8069965501612146, v_out_ptr->d[17], 1e-9);
  EXPECT_NEAR(0.8994743512003336, v_out_ptr->d[18], 1e-9);
  EXPECT_NEAR(1.2282191661653086, v_out_ptr->d[19], 1e-9);
  EXPECT_NEAR(0.4280304593747892, v_out_ptr->d[20], 1e-9);
  EXPECT_NEAR(-0.3840233574123333, v_out_ptr->d[21], 1e-9);
}

TEST(VecAdd3, Normal11) {
  double d_v0[22] = {
      0.9629151615253588,  -0.7541808175839873, -0.6635635264905195,
      0.0120434466927799,  -0.4147107614903636, -0.2701209218449674,
      -0.9985360751907104, -0.1675095876788206, 0.0489087108096249,
      0.4493614146468365,  -0.0950612911435054, -0.4661375351745367,
      0.4894573062474397,  0.4256768122026002,  -0.7532329279471621,
      -0.3664897615170746, -0.7247888420699948, -0.3267883158244300,
      0.8051205796808429,  0.9312825102886586,  -0.2966068819949614,
      -0.4178959545798386};
  const Vec v0 = {22, &d_v0[0], 0, 22};
  double d_v1[22] = {
      -0.8512862047896395, -0.4276612850383357, -0.8348978679582282,
      0.1150384853471946,  0.5408084696434605,  0.9215002871440625,
      -0.3872937546720312, 0.4027479934720803,  0.1739279684478710,
      0.9239491638736366,  -0.0590473452286984, -0.3803472824472345,
      -0.0679769433333908, -0.1117649173770328, 0.5815201336972691,
      0.2401989895747152,  0.4126108992388799,  -0.7181966251617624,
      -0.0758090323724929, 0.1002323427760694,  0.4491664567526514,
      -0.6244424149843193};
  const Vec v1 = {22, &d_v1[0], 0, 22};
  double d_v2[22] = {
      0.7370866863984451,  -0.3581704761218722, -0.2985615001544728,
      0.0005001816415469,  -0.9660216061808584, -0.9842490902820147,
      0.9636341781468096,  0.6818138204995019,  0.4965405585367773,
      0.0896724690875095,  -0.4359005259845368, -0.7896676231520350,
      -0.8878903164465566, 0.1363919926619990,  0.5979357395803293,
      0.3510412054542997,  0.6419369757700493,  -0.7080829597855796,
      0.0311470093821395,  -0.3101026798383213, -0.1273275261414342,
      0.0196085286050334};
  const Vec v2 = {22, &d_v2[0], 0, 22};
  double d_v0c[22] = {
      0.9629151615253588,  -0.7541808175839873, -0.6635635264905195,
      0.0120434466927799,  -0.4147107614903636, -0.2701209218449674,
      -0.9985360751907104, -0.1675095876788206, 0.0489087108096249,
      0.4493614146468365,  -0.0950612911435054, -0.4661375351745367,
      0.4894573062474397,  0.4256768122026002,  -0.7532329279471621,
      -0.3664897615170746, -0.7247888420699948, -0.3267883158244300,
      0.8051205796808429,  0.9312825102886586,  -0.2966068819949614,
      -0.4178959545798386};
  Vec v0c = {22, &d_v0c[0], 0, 22};
  double d_v1c[22] = {
      -0.8512862047896395, -0.4276612850383357, -0.8348978679582282,
      0.1150384853471946,  0.5408084696434605,  0.9215002871440625,
      -0.3872937546720312, 0.4027479934720803,  0.1739279684478710,
      0.9239491638736366,  -0.0590473452286984, -0.3803472824472345,
      -0.0679769433333908, -0.1117649173770328, 0.5815201336972691,
      0.2401989895747152,  0.4126108992388799,  -0.7181966251617624,
      -0.0758090323724929, 0.1002323427760694,  0.4491664567526514,
      -0.6244424149843193};
  Vec v1c = {22, &d_v1c[0], 0, 22};
  double d_v2c[22] = {
      0.7370866863984451,  -0.3581704761218722, -0.2985615001544728,
      0.0005001816415469,  -0.9660216061808584, -0.9842490902820147,
      0.9636341781468096,  0.6818138204995019,  0.4965405585367773,
      0.0896724690875095,  -0.4359005259845368, -0.7896676231520350,
      -0.8878903164465566, 0.1363919926619990,  0.5979357395803293,
      0.3510412054542997,  0.6419369757700493,  -0.7080829597855796,
      0.0311470093821395,  -0.3101026798383213, -0.1273275261414342,
      0.0196085286050334};
  Vec v2c = {22, &d_v2c[0], 0, 22};
  double d_v_out[22] = {
      0.7211414260400928,  0.9784262807248592,  0.0207193080286847,
      -0.8708278316423215, -0.0325460726539604, 0.7946441824306603,
      -0.5356479866892940, -0.6174427921475623, 0.6234442009846166,
      0.6860052482763812,  0.1361287369955519,  0.5199493472401222,
      0.2093020656408271,  0.1526934373557713,  0.0512589162900106,
      0.7899006194207203,  -0.7946654385814442, -0.8322342272396903,
      -0.2354472373004224, -0.6085340046134469, 0.6104882042795712,
      0.0666329720104999};
  Vec v_out = {22, &d_v_out[0], 0, 22};
  const Vec *v_out_ptr = VecAdd3(&v0, &v1, &v2, &v_out);
  VecAdd3(&v0c, &v1, &v2, &v0c);
  VecAdd3(&v0, &v1c, &v2, &v1c);
  VecAdd3(&v0, &v1, &v2c, &v2c);

  EXPECT_NEAR(0.8487156431341645, v_out.d[0], 1e-9);
  EXPECT_NEAR(-1.5400125787441952, v_out.d[1], 1e-9);
  EXPECT_NEAR(-1.7970228946032205, v_out.d[2], 1e-9);
  EXPECT_NEAR(0.1275821136815214, v_out.d[3], 1e-9);
  EXPECT_NEAR(-0.8399238980277615, v_out.d[4], 1e-9);
  EXPECT_NEAR(-0.3328697249829196, v_out.d[5], 1e-9);
  EXPECT_NEAR(-0.4221956517159320, v_out.d[6], 1e-9);
  EXPECT_NEAR(0.9170522262927616, v_out.d[7], 1e-9);
  EXPECT_NEAR(0.7193772377942731, v_out.d[8], 1e-9);
  EXPECT_NEAR(1.4629830476079826, v_out.d[9], 1e-9);
  EXPECT_NEAR(-0.5900091623567405, v_out.d[10], 1e-9);
  EXPECT_NEAR(-1.6361524407738062, v_out.d[11], 1e-9);
  EXPECT_NEAR(-0.4664099535325077, v_out.d[12], 1e-9);
  EXPECT_NEAR(0.4503038874875664, v_out.d[13], 1e-9);
  EXPECT_NEAR(0.4262229453304363, v_out.d[14], 1e-9);
  EXPECT_NEAR(0.2247504335119404, v_out.d[15], 1e-9);
  EXPECT_NEAR(0.3297590329389344, v_out.d[16], 1e-9);
  EXPECT_NEAR(-1.7530679007717720, v_out.d[17], 1e-9);
  EXPECT_NEAR(0.7604585566904896, v_out.d[18], 1e-9);
  EXPECT_NEAR(0.7214121732264067, v_out.d[19], 1e-9);
  EXPECT_NEAR(0.0252320486162558, v_out.d[20], 1e-9);
  EXPECT_NEAR(-1.0227298409591246, v_out.d[21], 1e-9);

  EXPECT_NEAR(0.8487156431341645, v0c.d[0], 1e-9);
  EXPECT_NEAR(-1.5400125787441952, v0c.d[1], 1e-9);
  EXPECT_NEAR(-1.7970228946032205, v0c.d[2], 1e-9);
  EXPECT_NEAR(0.1275821136815214, v0c.d[3], 1e-9);
  EXPECT_NEAR(-0.8399238980277615, v0c.d[4], 1e-9);
  EXPECT_NEAR(-0.3328697249829196, v0c.d[5], 1e-9);
  EXPECT_NEAR(-0.4221956517159320, v0c.d[6], 1e-9);
  EXPECT_NEAR(0.9170522262927616, v0c.d[7], 1e-9);
  EXPECT_NEAR(0.7193772377942731, v0c.d[8], 1e-9);
  EXPECT_NEAR(1.4629830476079826, v0c.d[9], 1e-9);
  EXPECT_NEAR(-0.5900091623567405, v0c.d[10], 1e-9);
  EXPECT_NEAR(-1.6361524407738062, v0c.d[11], 1e-9);
  EXPECT_NEAR(-0.4664099535325077, v0c.d[12], 1e-9);
  EXPECT_NEAR(0.4503038874875664, v0c.d[13], 1e-9);
  EXPECT_NEAR(0.4262229453304363, v0c.d[14], 1e-9);
  EXPECT_NEAR(0.2247504335119404, v0c.d[15], 1e-9);
  EXPECT_NEAR(0.3297590329389344, v0c.d[16], 1e-9);
  EXPECT_NEAR(-1.7530679007717720, v0c.d[17], 1e-9);
  EXPECT_NEAR(0.7604585566904896, v0c.d[18], 1e-9);
  EXPECT_NEAR(0.7214121732264067, v0c.d[19], 1e-9);
  EXPECT_NEAR(0.0252320486162558, v0c.d[20], 1e-9);
  EXPECT_NEAR(-1.0227298409591246, v0c.d[21], 1e-9);

  EXPECT_NEAR(0.8487156431341645, v1c.d[0], 1e-9);
  EXPECT_NEAR(-1.5400125787441952, v1c.d[1], 1e-9);
  EXPECT_NEAR(-1.7970228946032205, v1c.d[2], 1e-9);
  EXPECT_NEAR(0.1275821136815214, v1c.d[3], 1e-9);
  EXPECT_NEAR(-0.8399238980277615, v1c.d[4], 1e-9);
  EXPECT_NEAR(-0.3328697249829196, v1c.d[5], 1e-9);
  EXPECT_NEAR(-0.4221956517159320, v1c.d[6], 1e-9);
  EXPECT_NEAR(0.9170522262927616, v1c.d[7], 1e-9);
  EXPECT_NEAR(0.7193772377942731, v1c.d[8], 1e-9);
  EXPECT_NEAR(1.4629830476079826, v1c.d[9], 1e-9);
  EXPECT_NEAR(-0.5900091623567405, v1c.d[10], 1e-9);
  EXPECT_NEAR(-1.6361524407738062, v1c.d[11], 1e-9);
  EXPECT_NEAR(-0.4664099535325077, v1c.d[12], 1e-9);
  EXPECT_NEAR(0.4503038874875664, v1c.d[13], 1e-9);
  EXPECT_NEAR(0.4262229453304363, v1c.d[14], 1e-9);
  EXPECT_NEAR(0.2247504335119404, v1c.d[15], 1e-9);
  EXPECT_NEAR(0.3297590329389344, v1c.d[16], 1e-9);
  EXPECT_NEAR(-1.7530679007717720, v1c.d[17], 1e-9);
  EXPECT_NEAR(0.7604585566904896, v1c.d[18], 1e-9);
  EXPECT_NEAR(0.7214121732264067, v1c.d[19], 1e-9);
  EXPECT_NEAR(0.0252320486162558, v1c.d[20], 1e-9);
  EXPECT_NEAR(-1.0227298409591246, v1c.d[21], 1e-9);

  EXPECT_NEAR(0.8487156431341645, v2c.d[0], 1e-9);
  EXPECT_NEAR(-1.5400125787441952, v2c.d[1], 1e-9);
  EXPECT_NEAR(-1.7970228946032205, v2c.d[2], 1e-9);
  EXPECT_NEAR(0.1275821136815214, v2c.d[3], 1e-9);
  EXPECT_NEAR(-0.8399238980277615, v2c.d[4], 1e-9);
  EXPECT_NEAR(-0.3328697249829196, v2c.d[5], 1e-9);
  EXPECT_NEAR(-0.4221956517159320, v2c.d[6], 1e-9);
  EXPECT_NEAR(0.9170522262927616, v2c.d[7], 1e-9);
  EXPECT_NEAR(0.7193772377942731, v2c.d[8], 1e-9);
  EXPECT_NEAR(1.4629830476079826, v2c.d[9], 1e-9);
  EXPECT_NEAR(-0.5900091623567405, v2c.d[10], 1e-9);
  EXPECT_NEAR(-1.6361524407738062, v2c.d[11], 1e-9);
  EXPECT_NEAR(-0.4664099535325077, v2c.d[12], 1e-9);
  EXPECT_NEAR(0.4503038874875664, v2c.d[13], 1e-9);
  EXPECT_NEAR(0.4262229453304363, v2c.d[14], 1e-9);
  EXPECT_NEAR(0.2247504335119404, v2c.d[15], 1e-9);
  EXPECT_NEAR(0.3297590329389344, v2c.d[16], 1e-9);
  EXPECT_NEAR(-1.7530679007717720, v2c.d[17], 1e-9);
  EXPECT_NEAR(0.7604585566904896, v2c.d[18], 1e-9);
  EXPECT_NEAR(0.7214121732264067, v2c.d[19], 1e-9);
  EXPECT_NEAR(0.0252320486162558, v2c.d[20], 1e-9);
  EXPECT_NEAR(-1.0227298409591246, v2c.d[21], 1e-9);

  EXPECT_NEAR(0.8487156431341645, v_out_ptr->d[0], 1e-9);
  EXPECT_NEAR(-1.5400125787441952, v_out_ptr->d[1], 1e-9);
  EXPECT_NEAR(-1.7970228946032205, v_out_ptr->d[2], 1e-9);
  EXPECT_NEAR(0.1275821136815214, v_out_ptr->d[3], 1e-9);
  EXPECT_NEAR(-0.8399238980277615, v_out_ptr->d[4], 1e-9);
  EXPECT_NEAR(-0.3328697249829196, v_out_ptr->d[5], 1e-9);
  EXPECT_NEAR(-0.4221956517159320, v_out_ptr->d[6], 1e-9);
  EXPECT_NEAR(0.9170522262927616, v_out_ptr->d[7], 1e-9);
  EXPECT_NEAR(0.7193772377942731, v_out_ptr->d[8], 1e-9);
  EXPECT_NEAR(1.4629830476079826, v_out_ptr->d[9], 1e-9);
  EXPECT_NEAR(-0.5900091623567405, v_out_ptr->d[10], 1e-9);
  EXPECT_NEAR(-1.6361524407738062, v_out_ptr->d[11], 1e-9);
  EXPECT_NEAR(-0.4664099535325077, v_out_ptr->d[12], 1e-9);
  EXPECT_NEAR(0.4503038874875664, v_out_ptr->d[13], 1e-9);
  EXPECT_NEAR(0.4262229453304363, v_out_ptr->d[14], 1e-9);
  EXPECT_NEAR(0.2247504335119404, v_out_ptr->d[15], 1e-9);
  EXPECT_NEAR(0.3297590329389344, v_out_ptr->d[16], 1e-9);
  EXPECT_NEAR(-1.7530679007717720, v_out_ptr->d[17], 1e-9);
  EXPECT_NEAR(0.7604585566904896, v_out_ptr->d[18], 1e-9);
  EXPECT_NEAR(0.7214121732264067, v_out_ptr->d[19], 1e-9);
  EXPECT_NEAR(0.0252320486162558, v_out_ptr->d[20], 1e-9);
  EXPECT_NEAR(-1.0227298409591246, v_out_ptr->d[21], 1e-9);
}

TEST(VecSub, Normal11) {
  double d_v0[22] = {
      0.6318996983431597,  -0.2423021491149286, -0.6035344330356749,
      -0.8215491167230045, 0.8206548803775950,  0.5905947262456115,
      -0.9771761504007392, -0.1473240893060332, 0.0818095587620145,
      -0.9485619298659385, 0.2830483773921690,  0.4607597392559069,
      0.4849062822709118,  -0.4840186112945990, -0.5471324126556860,
      -0.1309507437472548, 0.9497430384624823,  0.2291679596182201,
      -0.6023962347958212, -0.4917236337958244, -0.0242457374520009,
      -0.7658501181411488};
  const Vec v0 = {22, &d_v0[0], 0, 22};
  double d_v1[22] = {
      0.4927878394763319,  -0.4706773512356019, 0.3578910924522121,
      -0.3525762985813401, 0.9781665759903686,  0.2155118399946772,
      0.4800647119593726,  0.7211642800558828,  -0.4046670364294469,
      -0.0175314293463307, -0.5380859271366716, -0.2843099934028201,
      0.4016459115360167,  -0.0546021585660834, -0.8141040968174826,
      0.0225834976481600,  -0.1516913876886172, 0.3292998058166430,
      -0.0816672950224784, 0.9543741415673119,  -0.1966526930394010,
      -0.8879457282227836};
  const Vec v1 = {22, &d_v1[0], 0, 22};
  double d_v_out[22] = {
      -0.9642126908484909, -0.0210917973890692, 0.9923277852057231,
      -0.9718175552889334, 0.3010106430273052,  -0.0112601676273143,
      0.9958825687980244,  0.9583043300695449,  0.9176582527581230,
      -0.4266747036269554, 0.5851725532157812,  -0.6383804188170892,
      -0.1964186596454593, -0.2797108857813511, 0.5049244070474295,
      0.0914862646716266,  0.8752605034581451,  -0.5827262783743907,
      -0.5322429505169313, 0.3230166453945742,  0.9152884697243240,
      0.5446413898080475};
  Vec v_out = {22, &d_v_out[0], 0, 22};
  const Vec *v_out_ptr = VecSub(&v0, &v1, &v_out);

  EXPECT_NEAR(0.1391118588668279, v_out.d[0], 1e-9);
  EXPECT_NEAR(0.2283752021206733, v_out.d[1], 1e-9);
  EXPECT_NEAR(-0.9614255254878870, v_out.d[2], 1e-9);
  EXPECT_NEAR(-0.4689728181416644, v_out.d[3], 1e-9);
  EXPECT_NEAR(-0.1575116956127736, v_out.d[4], 1e-9);
  EXPECT_NEAR(0.3750828862509343, v_out.d[5], 1e-9);
  EXPECT_NEAR(-1.4572408623601119, v_out.d[6], 1e-9);
  EXPECT_NEAR(-0.8684883693619161, v_out.d[7], 1e-9);
  EXPECT_NEAR(0.4864765951914614, v_out.d[8], 1e-9);
  EXPECT_NEAR(-0.9310305005196078, v_out.d[9], 1e-9);
  EXPECT_NEAR(0.8211343045288406, v_out.d[10], 1e-9);
  EXPECT_NEAR(0.7450697326587270, v_out.d[11], 1e-9);
  EXPECT_NEAR(0.0832603707348951, v_out.d[12], 1e-9);
  EXPECT_NEAR(-0.4294164527285156, v_out.d[13], 1e-9);
  EXPECT_NEAR(0.2669716841617966, v_out.d[14], 1e-9);
  EXPECT_NEAR(-0.1535342413954148, v_out.d[15], 1e-9);
  EXPECT_NEAR(1.1014344261510995, v_out.d[16], 1e-9);
  EXPECT_NEAR(-0.1001318461984229, v_out.d[17], 1e-9);
  EXPECT_NEAR(-0.5207289397733428, v_out.d[18], 1e-9);
  EXPECT_NEAR(-1.4460977753631363, v_out.d[19], 1e-9);
  EXPECT_NEAR(0.1724069555874002, v_out.d[20], 1e-9);
  EXPECT_NEAR(0.1220956100816348, v_out.d[21], 1e-9);

  EXPECT_NEAR(0.1391118588668279, v_out_ptr->d[0], 1e-9);
  EXPECT_NEAR(0.2283752021206733, v_out_ptr->d[1], 1e-9);
  EXPECT_NEAR(-0.9614255254878870, v_out_ptr->d[2], 1e-9);
  EXPECT_NEAR(-0.4689728181416644, v_out_ptr->d[3], 1e-9);
  EXPECT_NEAR(-0.1575116956127736, v_out_ptr->d[4], 1e-9);
  EXPECT_NEAR(0.3750828862509343, v_out_ptr->d[5], 1e-9);
  EXPECT_NEAR(-1.4572408623601119, v_out_ptr->d[6], 1e-9);
  EXPECT_NEAR(-0.8684883693619161, v_out_ptr->d[7], 1e-9);
  EXPECT_NEAR(0.4864765951914614, v_out_ptr->d[8], 1e-9);
  EXPECT_NEAR(-0.9310305005196078, v_out_ptr->d[9], 1e-9);
  EXPECT_NEAR(0.8211343045288406, v_out_ptr->d[10], 1e-9);
  EXPECT_NEAR(0.7450697326587270, v_out_ptr->d[11], 1e-9);
  EXPECT_NEAR(0.0832603707348951, v_out_ptr->d[12], 1e-9);
  EXPECT_NEAR(-0.4294164527285156, v_out_ptr->d[13], 1e-9);
  EXPECT_NEAR(0.2669716841617966, v_out_ptr->d[14], 1e-9);
  EXPECT_NEAR(-0.1535342413954148, v_out_ptr->d[15], 1e-9);
  EXPECT_NEAR(1.1014344261510995, v_out_ptr->d[16], 1e-9);
  EXPECT_NEAR(-0.1001318461984229, v_out_ptr->d[17], 1e-9);
  EXPECT_NEAR(-0.5207289397733428, v_out_ptr->d[18], 1e-9);
  EXPECT_NEAR(-1.4460977753631363, v_out_ptr->d[19], 1e-9);
  EXPECT_NEAR(0.1724069555874002, v_out_ptr->d[20], 1e-9);
  EXPECT_NEAR(0.1220956100816348, v_out_ptr->d[21], 1e-9);
}

TEST(VecLinComb, Normal11) {
  double d_v0[22] = {
      -0.3350793637901217, -0.7993092615584603, -0.7213182980189456,
      0.6356183452723194,  0.3081359408953630,  -0.7038931179828223,
      0.4488891141843150,  -0.0115173197660317, 0.5163892464184332,
      -0.4427152791601148, -0.1780821443015748, -0.3987408315947307,
      0.1880549340448869,  0.5263147356837659,  -0.6491235501861530,
      -0.1661392170702580, 0.5832415205542716,  0.5711992979175835,
      0.0846954583747235,  0.1334246942947388,  -0.5813421870522038,
      -0.4620382806605434};
  const Vec v0 = {22, &d_v0[0], 0, 22};
  double d_v1[22] = {
      -0.8565493049911124, 0.0741349276487473,  0.6586328287664853,
      -0.3887523244884827, -0.0111000526598706, -0.4960792008774999,
      -0.3320207227949914, -0.1092296754695401, 0.3053938677499994,
      0.5613356088651615,  -0.4680689807572962, 0.9652727301085275,
      0.5303131990032210,  -0.5461572515817623, 0.9754791483188150,
      0.8610925282371358,  -0.1677552271968026, -0.5540568050483405,
      0.4963942529613341,  0.8008038505673303,  -0.0790230861312802,
      0.2135906330639479};
  const Vec v1 = {22, &d_v1[0], 0, 22};
  double d_v_out[22] = {
      0.5692521678936582,  0.3833547872595140,  -0.3997792350020810,
      0.9091464546309451,  0.0927972018831620,  -0.4131454859129617,
      -0.6894541204881726, -0.4265312244342809, 0.4984487771025465,
      -0.5916222379253562, -0.1705695071192694, -0.5400556072952565,
      0.0294482422005011,  -0.9178734475666335, -0.3496081446379111,
      -0.9837935739860628, -0.7423689467508956, -0.1707578438331248,
      0.6073586716295987,  -0.2250853292262684, 0.8905387387911916,
      -0.7456436211660380};
  Vec v_out = {22, &d_v_out[0], 0, 22};
  const Vec *v_out_ptr =
      VecLinComb(-0.8049952700194887, &v0, 0.7137969345756441, &v1, &v_out);

  EXPECT_NEAR(-0.3416649652833669, v_out.d[0], 1e-9);
  EXPECT_NEAR(0.6963574589379938, v_out.d[1], 1e-9);
  EXPECT_NEAR(1.0507879122681616, v_out.d[2], 1e-9);
  EXPECT_NEAR(-0.7891599790108663, v_out.d[3], 1e-9);
  EXPECT_NEAR(-0.2559711585060158, v_out.d[4], 1e-9);
  EXPECT_NEAR(0.2125308176823472, v_out.d[5], 1e-9);
  EXPECT_NEAR(-0.5983489878282663, v_out.d[6], 1e-9);
  EXPECT_NEAR(-0.0686964195798927, v_out.d[7], 1e-9);
  EXPECT_NEAR(-0.1977016942176177, v_out.d[8], 1e-9);
  EXPECT_NEAR(0.7570633425653549, v_out.d[9], 1e-9);
  EXPECT_NEAR(-0.1907509197968084, v_out.d[10], 1e-9);
  EXPECT_NEAR(1.0099931991783258, v_out.d[11], 1e-9);
  EXPECT_NEAR(0.2271526034035418, v_out.d[12], 1e-9);
  EXPECT_NEAR(-0.8135262447423097, v_out.d[13], 1e-9);
  EXPECT_NEAR(1.2188354133705415, v_out.d[14], 1e-9);
  EXPECT_NEAR(0.7483864909479576, v_out.d[15], 1e-9);
  EXPECT_NEAR(-0.5892498322572814, v_out.d[16], 1e-9);
  EXPECT_NEAR(-0.8552967820863882, v_out.d[17], 1e-9);
  EXPECT_NEAR(0.2861452527209822, v_out.d[18], 1e-9);
  EXPECT_NEAR(0.4642050859202715, v_out.d[19], 1e-9);
  EXPECT_NEAR(0.4115712741985940, v_out.d[20], 1e-9);
  EXPECT_NEAR(0.5243989696347917, v_out.d[21], 1e-9);

  EXPECT_NEAR(-0.3416649652833669, v_out_ptr->d[0], 1e-9);
  EXPECT_NEAR(0.6963574589379938, v_out_ptr->d[1], 1e-9);
  EXPECT_NEAR(1.0507879122681616, v_out_ptr->d[2], 1e-9);
  EXPECT_NEAR(-0.7891599790108663, v_out_ptr->d[3], 1e-9);
  EXPECT_NEAR(-0.2559711585060158, v_out_ptr->d[4], 1e-9);
  EXPECT_NEAR(0.2125308176823472, v_out_ptr->d[5], 1e-9);
  EXPECT_NEAR(-0.5983489878282663, v_out_ptr->d[6], 1e-9);
  EXPECT_NEAR(-0.0686964195798927, v_out_ptr->d[7], 1e-9);
  EXPECT_NEAR(-0.1977016942176177, v_out_ptr->d[8], 1e-9);
  EXPECT_NEAR(0.7570633425653549, v_out_ptr->d[9], 1e-9);
  EXPECT_NEAR(-0.1907509197968084, v_out_ptr->d[10], 1e-9);
  EXPECT_NEAR(1.0099931991783258, v_out_ptr->d[11], 1e-9);
  EXPECT_NEAR(0.2271526034035418, v_out_ptr->d[12], 1e-9);
  EXPECT_NEAR(-0.8135262447423097, v_out_ptr->d[13], 1e-9);
  EXPECT_NEAR(1.2188354133705415, v_out_ptr->d[14], 1e-9);
  EXPECT_NEAR(0.7483864909479576, v_out_ptr->d[15], 1e-9);
  EXPECT_NEAR(-0.5892498322572814, v_out_ptr->d[16], 1e-9);
  EXPECT_NEAR(-0.8552967820863882, v_out_ptr->d[17], 1e-9);
  EXPECT_NEAR(0.2861452527209822, v_out_ptr->d[18], 1e-9);
  EXPECT_NEAR(0.4642050859202715, v_out_ptr->d[19], 1e-9);
  EXPECT_NEAR(0.4115712741985940, v_out_ptr->d[20], 1e-9);
  EXPECT_NEAR(0.5243989696347917, v_out_ptr->d[21], 1e-9);
}

TEST(VecLinComb3, Normal11) {
  double d_v0[22] = {
      -0.9965934091386386, -0.2214262263475506, -0.2278234156535235,
      -0.2343528923925406, -0.9738840431434377, 0.1147667706581166,
      0.5670465687701365,  0.1312406416599068,  0.3553281198712777,
      -0.4079669738880496, -0.1996151030094542, 0.9535010810141056,
      0.5476646719607794,  0.5383889093447645,  -0.0558700980464435,
      -0.5549108288709486, -0.6778537540859018, 0.3357073486485065,
      0.1603309380287787,  0.3701698810589100,  0.3995159529529428,
      -0.1104444349909917};
  const Vec v0 = {22, &d_v0[0], 0, 22};
  double d_v1[22] = {
      -0.7426170732895825, 0.8062838122778611,  0.9547718962223324,
      0.5481732206804020,  -0.0648801481683698, 0.0690920561190227,
      -0.7853185662378648, 0.4089449837146715,  -0.3027047673472327,
      -0.2822462674734858, 0.5915328634808152,  -0.1693121396092936,
      0.1919708302356213,  0.5827497729339608,  -0.4874121215391953,
      0.3219959220280801,  0.8742605725920063,  -0.7137745547278880,
      0.4558037802555823,  -0.5260791815604982, -0.3405562123751475,
      -0.5107285290259342};
  const Vec v1 = {22, &d_v1[0], 0, 22};
  double d_v2[22] = {
      0.2275596816141394,  -0.0406989429403397, -0.5626648575159594,
      0.6746824228624024,  -0.6576586186527635, -0.5098089526756830,
      0.2314652355702038,  -0.6237421346797132, -0.1855262769174082,
      -0.1659251503291939, -0.8301543519864882, 0.1961479122659426,
      0.7814850815861203,  -0.8257434760555604, 0.5448771665055037,
      0.9483414136724899,  0.5005155136124388,  -0.7114397593887640,
      0.4930591151055497,  -0.3306405115884476, 0.0025719987257204,
      0.4852531952338659};
  const Vec v2 = {22, &d_v2[0], 0, 22};
  double d_v_out[22] = {
      -0.9190641767347441, 0.9146189809396257,  -0.7992253064220589,
      -0.8484426363604893, 0.0125483043396353,  -0.6118476885389406,
      0.2972067113287680,  -0.9786718222121817, -0.3957473280526387,
      -0.1544787666896854, 0.1812938175344216,  -0.8710203253346191,
      -0.7584087110487325, -0.1123534117798259, -0.0700643361821112,
      -0.8486335614861109, 0.0000785385478705,  0.1738226976402988,
      -0.3687515913566262, -0.4612015740388653, 0.3285338565125744,
      -0.2403418997161622};
  Vec v_out = {22, &d_v_out[0], 0, 22};
  const Vec *v_out_ptr =
      VecLinComb3(0.9986468120187642, &v0, 0.0324970977011423, &v1,
                  0.3380951009299182, &v2, &v_out);

  EXPECT_NEAR(-0.9424409169775287, v_out.d[0], 1e-9);
  EXPECT_NEAR(-0.2086848244380387, v_out.d[1], 1e-9);
  EXPECT_NEAR(-0.3867220439433514, v_out.d[2], 1e-9);
  EXPECT_NEAR(0.0118850916877194, v_out.d[3], 1e-9);
  EXPECT_NEAR(-1.1970257685258674, v_out.d[4], 1e-9);
  EXPECT_NEAR(-0.0555071483683715, v_out.d[5], 1e-9);
  EXPECT_NEAR(0.6190159361767982, v_out.d[6], 1e-9);
  EXPECT_NEAR(-0.0665315864876496, v_out.d[7], 1e-9);
  EXPECT_NEAR(0.2822847424114451, v_out.d[8], 1e-9);
  EXPECT_NEAR(-0.4726855828594724, v_out.d[9], 1e-9);
  EXPECT_NEAR(-0.4607930044154988, v_out.d[10], 1e-9);
  EXPECT_NEAR(1.0130253098630559, v_out.d[11], 1e-9);
  EXPECT_NEAR(0.8173783510689557, v_out.d[12], 1e-9);
  EXPECT_NEAR(0.2774182203705357, v_out.d[13], 1e-9);
  EXPECT_NEAR(0.1125863259684493, v_out.d[14], 1e-9);
  EXPECT_NEAR(-0.2230664112975250, v_out.d[15], 1e-9);
  EXPECT_NEAR(-0.4793037161972707, v_out.d[16], 1e-9);
  EXPECT_NEAR(0.0715231748014203, v_out.d[17], 1e-9);
  EXPECT_NEAR(0.3416271513959606, v_out.d[18], 1e-9);
  EXPECT_NEAR(0.2407849879261211, v_out.d[19], 1e-9);
  EXPECT_NEAR(0.3887778244295729, v_out.d[20], 1e-9);
  EXPECT_NEAR(0.0371695502036753, v_out.d[21], 1e-9);

  EXPECT_NEAR(-0.9424409169775287, v_out_ptr->d[0], 1e-9);
  EXPECT_NEAR(-0.2086848244380387, v_out_ptr->d[1], 1e-9);
  EXPECT_NEAR(-0.3867220439433514, v_out_ptr->d[2], 1e-9);
  EXPECT_NEAR(0.0118850916877194, v_out_ptr->d[3], 1e-9);
  EXPECT_NEAR(-1.1970257685258674, v_out_ptr->d[4], 1e-9);
  EXPECT_NEAR(-0.0555071483683715, v_out_ptr->d[5], 1e-9);
  EXPECT_NEAR(0.6190159361767982, v_out_ptr->d[6], 1e-9);
  EXPECT_NEAR(-0.0665315864876496, v_out_ptr->d[7], 1e-9);
  EXPECT_NEAR(0.2822847424114451, v_out_ptr->d[8], 1e-9);
  EXPECT_NEAR(-0.4726855828594724, v_out_ptr->d[9], 1e-9);
  EXPECT_NEAR(-0.4607930044154988, v_out_ptr->d[10], 1e-9);
  EXPECT_NEAR(1.0130253098630559, v_out_ptr->d[11], 1e-9);
  EXPECT_NEAR(0.8173783510689557, v_out_ptr->d[12], 1e-9);
  EXPECT_NEAR(0.2774182203705357, v_out_ptr->d[13], 1e-9);
  EXPECT_NEAR(0.1125863259684493, v_out_ptr->d[14], 1e-9);
  EXPECT_NEAR(-0.2230664112975250, v_out_ptr->d[15], 1e-9);
  EXPECT_NEAR(-0.4793037161972707, v_out_ptr->d[16], 1e-9);
  EXPECT_NEAR(0.0715231748014203, v_out_ptr->d[17], 1e-9);
  EXPECT_NEAR(0.3416271513959606, v_out_ptr->d[18], 1e-9);
  EXPECT_NEAR(0.2407849879261211, v_out_ptr->d[19], 1e-9);
  EXPECT_NEAR(0.3887778244295729, v_out_ptr->d[20], 1e-9);
  EXPECT_NEAR(0.0371695502036753, v_out_ptr->d[21], 1e-9);
}

TEST(VecDot, Normal11) {
  double d_v0[22] = {
      0.6657021476449307,  -0.2978858914816287, -0.0880730118487949,
      -0.3000668291351050, 0.7454111040496514,  -0.5018821785135554,
      0.5433229804056885,  -0.0410166573467627, -0.4234642976028438,
      0.2850759286757267,  0.6483002549907078,  -0.7232668147466750,
      0.5502206414174735,  -0.0291434544404250, 0.8561043747513462,
      0.4140834834202907,  -0.6406409212257291, -0.7300899467689799,
      0.3155434763293981,  -0.2188202273870803, -0.0733946609761436,
      -0.9618973154040860};
  const Vec v0 = {22, &d_v0[0], 0, 22};
  double d_v1[22] = {
      -0.2982416378919033, 0.4525069347945634,  -0.2492014009583998,
      0.2264010000394245,  -0.7371424958943023, 0.0505791716987785,
      -0.9047948845179186, -0.3345077577459594, 0.0583275900228093,
      0.6961208901087939,  0.6112062144504140,  0.9822123511330882,
      0.7856725152880879,  0.6008773170025099,  0.2510544416385232,
      0.6553792949861434,  -0.1119185245190482, -0.7359395120856795,
      0.1569824338982369,  0.0707084356516803,  -0.2154980535110953,
      -0.0495836696356993};
  const Vec v1 = {22, &d_v1[0], 0, 22};
  double v_dot = VecDot(&v0, &v1);

  EXPECT_NEAR(0.0352006306413496, v_dot, 1e-9);
}

TEST(VecMult, Normal11) {
  double d_v0[22] = {
      -0.6990952526812353, 0.1700952552696691,  0.9762753964284065,
      -0.0872245099255986, -0.5850735031144891, -0.8668048922428089,
      -0.4351344577887055, 0.1497144209071382,  0.8378157669070507,
      -0.2967083996891227, -0.5294271831696498, 0.0584836110362066,
      -0.6250150992313037, 0.9905419033411624,  -0.8746374390215981,
      -0.6527008264426191, -0.6261305171515652, 0.3729969185279045,
      -0.1618838496578974, 0.3004261066936893,  0.4235181391985536,
      0.1833677970273238};
  const Vec v0 = {22, &d_v0[0], 0, 22};
  double d_v1[22] = {
      0.9180766196213515,  -0.6329618767681489, 0.1463960342021391,
      -0.2848500534606229, 0.0954627197454585,  -0.6024903690030050,
      0.0010578079542358,  0.8801904547024946,  0.7491401584441575,
      0.0280862371539492,  -0.4780622268385784, -0.8362979889092048,
      0.0566371973790654,  -0.4980139018827165, 0.7256881493298741,
      0.4419263663515318,  0.8193118683008913,  0.3004193054581434,
      -0.7716118547782485, -0.7164233350632092, 0.7613207219634652,
      0.8063695081494477};
  const Vec v1 = {22, &d_v1[0], 0, 22};
  double d_v_out[22] = {
      0.1273314876507305,  -0.9792875890017361, 0.5673553012948753,
      -0.9386196764603358, -0.0011944831121333, -0.0704610070772551,
      -0.4788075218178633, -0.7799564645122139, -0.0815613032954483,
      0.5074643250218647,  -0.3505884659005161, -0.6049997919952923,
      0.1680549087916985,  -0.6566523486835696, -0.6022258531277245,
      0.5485169249756161,  -0.6385893432036793, -0.0024217896700727,
      -0.5962149630346398, 0.5330591719373468,  -0.7666339231687971,
      -0.6758188554246662};
  Vec v_out = {22, &d_v_out[0], 0, 22};
  const Vec *v_out_ptr = VecMult(&v0, &v1, &v_out);

  EXPECT_NEAR(-0.6418230063749231, v_out.d[0], 1e-9);
  EXPECT_NEAR(-0.1076638120048471, v_out.d[1], 1e-9);
  EXPECT_NEAR(0.1429228463262399, v_out.d[2], 1e-9);
  EXPECT_NEAR(0.0248459063153834, v_out.d[3], 1e-9);
  EXPECT_NEAR(-0.0558527078583121, v_out.d[4], 1e-9);
  EXPECT_NEAR(0.5222415993809799, v_out.d[5], 1e-9);
  EXPECT_NEAR(-0.0004602886906110, v_out.d[6], 1e-9);
  EXPECT_NEAR(0.1317772042137746, v_out.d[7], 1e-9);
  EXPECT_NEAR(0.6276414363677613, v_out.d[8], 1e-9);
  EXPECT_NEAR(-0.0083334224792374, v_out.d[9], 1e-9);
  EXPECT_NEAR(0.2530991381349587, v_out.d[10], 1e-9);
  EXPECT_NEAR(-0.0489097262937277, v_out.d[11], 1e-9);
  EXPECT_NEAR(-0.0353991035400595, v_out.d[12], 1e-9);
  EXPECT_NEAR(-0.4933036382612649, v_out.d[13], 1e-9);
  EXPECT_NEAR(-0.6347140244582041, v_out.d[14], 1e-9);
  EXPECT_NEAR(-0.2884457045444285, v_out.d[15], 1e-9);
  EXPECT_NEAR(-0.5129961638076521, v_out.d[16], 1e-9);
  EXPECT_NEAR(0.1120554752021808, v_out.d[17], 1e-9);
  EXPECT_NEAR(0.1249114974931733, v_out.d[18], 1e-9);
  EXPECT_NEAR(-0.2152322732975484, v_out.d[19], 1e-9);
  EXPECT_NEAR(0.3224331354992662, v_out.d[20], 1e-9);
  EXPECT_NEAR(0.1478622002993708, v_out.d[21], 1e-9);

  EXPECT_NEAR(-0.6418230063749231, v_out_ptr->d[0], 1e-9);
  EXPECT_NEAR(-0.1076638120048471, v_out_ptr->d[1], 1e-9);
  EXPECT_NEAR(0.1429228463262399, v_out_ptr->d[2], 1e-9);
  EXPECT_NEAR(0.0248459063153834, v_out_ptr->d[3], 1e-9);
  EXPECT_NEAR(-0.0558527078583121, v_out_ptr->d[4], 1e-9);
  EXPECT_NEAR(0.5222415993809799, v_out_ptr->d[5], 1e-9);
  EXPECT_NEAR(-0.0004602886906110, v_out_ptr->d[6], 1e-9);
  EXPECT_NEAR(0.1317772042137746, v_out_ptr->d[7], 1e-9);
  EXPECT_NEAR(0.6276414363677613, v_out_ptr->d[8], 1e-9);
  EXPECT_NEAR(-0.0083334224792374, v_out_ptr->d[9], 1e-9);
  EXPECT_NEAR(0.2530991381349587, v_out_ptr->d[10], 1e-9);
  EXPECT_NEAR(-0.0489097262937277, v_out_ptr->d[11], 1e-9);
  EXPECT_NEAR(-0.0353991035400595, v_out_ptr->d[12], 1e-9);
  EXPECT_NEAR(-0.4933036382612649, v_out_ptr->d[13], 1e-9);
  EXPECT_NEAR(-0.6347140244582041, v_out_ptr->d[14], 1e-9);
  EXPECT_NEAR(-0.2884457045444285, v_out_ptr->d[15], 1e-9);
  EXPECT_NEAR(-0.5129961638076521, v_out_ptr->d[16], 1e-9);
  EXPECT_NEAR(0.1120554752021808, v_out_ptr->d[17], 1e-9);
  EXPECT_NEAR(0.1249114974931733, v_out_ptr->d[18], 1e-9);
  EXPECT_NEAR(-0.2152322732975484, v_out_ptr->d[19], 1e-9);
  EXPECT_NEAR(0.3224331354992662, v_out_ptr->d[20], 1e-9);
  EXPECT_NEAR(0.1478622002993708, v_out_ptr->d[21], 1e-9);
}

TEST(MatArrMult, Normal2) {
  double m1[8][7] = {
      {0.4831125715303517, 0.1999784826068653, 0.4627220110096041,
       0.2519442957708448, 0.0583222480154345, 0.7594569456667069,
       0.5350955187844567},
      {0.5116960068894928, 0.6582338964073821, 0.7896813856173345,
       0.7735209836670335, 0.7225293021938548, 0.7192253887460557,
       0.1433981639604575},
      {0.7705139164554008, 0.0914946963854357, 0.4372247155911306,
       0.0431991437753999, 0.0698133018681714, 0.4577759666226728,
       0.2377131397808495},
      {0.8030464430266857, 0.6247568375070195, 0.6614078630240329,
       0.8052114639856935, 0.3037294737970303, 0.0854410769895837,
       0.7821225756654309},
      {0.4418650515981588, 0.9865124963647378, 0.9513316583892963,
       0.4126398838275529, 0.5321801570825589, 0.3315903863229150,
       0.9218706473323911},
      {0.6102596279215251, 0.4004716175261348, 0.7628318383810667,
       0.7382709993767972, 0.1999471511772175, 0.7755344004204676,
       0.1352265977265222},
      {0.6936845086166534, 0.6367153299880576, 0.3282309381026653,
       0.1640067985814962, 0.9594110824408199, 0.1238508932526943,
       0.8025768467254580},
      {0.1876016362598834, 0.2513640167312593, 0.6066270001156370,
       0.8508872734153617, 0.1886236204944255, 0.2859493648005904,
       0.1904580584169394}};
  double m2[7][11] = {
      {0.8713657898553244, 0.8927398300372571, 0.9337376775700823,
       0.5788629297600379, 0.1604561057547170, 0.8866683226648249,
       0.3328599248350279, 0.6436371459674621, 0.1039650030328666,
       0.0666194188703980, 0.5787023947370313},
      {0.4200230593365910, 0.1035101741177292, 0.5291241601884625,
       0.2936621677166076, 0.1551039706497863, 0.1383814967199101,
       0.7248023118100806, 0.5524514332490794, 0.7818252240875959,
       0.8092244751110597, 0.8252398323676531},
      {0.5223549776603156, 0.2238816786048954, 0.1216819246346900,
       0.3629361179709846, 0.1882291010494155, 0.7419813545034110,
       0.3570629994706117, 0.9056333348747985, 0.8594468174651866,
       0.2809350567167197, 0.3938697639931896},
      {0.9804151048868528, 0.7989929839918888, 0.1856805207161658,
       0.9158117123522626, 0.6401465263032784, 0.8573687145262743,
       0.2840480885888721, 0.1900204477951405, 0.8737854683756417,
       0.5870566534293916, 0.0623013789279837},
      {0.2579125393684698, 0.7643543770951781, 0.3849622140997084,
       0.5617466945632883, 0.4508941128674249, 0.8087151298606400,
       0.8159491644637252, 0.0968274683028041, 0.3447621805737755,
       0.0627251001658387, 0.8731841861244112},
      {0.1715907521041682, 0.7942265234555148, 0.6572145661827453,
       0.0832925697770317, 0.8743729068054631, 0.9791493131432926,
       0.0983831366582334, 0.4766175164075841, 0.2107818580008343,
       0.5747882024166631, 0.6326131406146797},
      {0.8014767828622411, 0.7759010392811382, 0.3508082664800011,
       0.2783495093157241, 0.3210770339176604, 0.1598199539460619,
       0.4397103279717542, 0.8999467511212943, 0.3524337939128221,
       0.1308946695796391, 0.4453365835489664}};
  double m_out[8][11];
  double m_out2[8][11];
  Mat mn_m1 = {8, 7, &m1[0][0], 0, 0};
  Mat mn_m2 = {7, 11, &m2[0][0], 0, 0};
  Mat mn_m_out = {8, 11, &m_out2[0][0], 0, 0};
  MatArrMult(&m1[0][0], 8, 7, &m2[0][0], 11, &m_out[0][0]);
  MatMult(&mn_m1, &mn_m2, &mn_m_out);

  EXPECT_NEAR(m_out[0][0], 1.5679029429386730, 1e-9);
  EXPECT_NEAR(m_out2[0][0], 1.5679029429386730, 1e-9);
  EXPECT_NEAR(m_out[0][1], 1.8198312321444297, 1e-9);
  EXPECT_NEAR(m_out2[0][1], 1.8198312321444297, 1e-9);
  EXPECT_NEAR(m_out[0][2], 1.3692938702655661, 1e-9);
  EXPECT_NEAR(m_out2[0][2], 1.3692938702655661, 1e-9);
  EXPECT_NEAR(m_out[0][3], 0.9820171663342729, 1e-9);
  EXPECT_NEAR(m_out2[0][3], 0.9820171663342729, 1e-9);
  EXPECT_NEAR(m_out[0][4], 1.2190674499829945, 1e-9);
  EXPECT_NEAR(m_out2[0][4], 1.2190674499829945, 1e-9);
  EXPECT_NEAR(m_out[0][5], 1.8916809689346479, 1e-9);
  EXPECT_NEAR(m_out2[0][5], 1.8916809689346479, 1e-9);
  EXPECT_NEAR(m_out[0][6], 0.9001316576392600, 1e-9);
  EXPECT_NEAR(m_out2[0][6], 0.9001316576392600, 1e-9);
  EXPECT_NEAR(m_out[0][7], 1.7375337944611973, 1e-9);
  EXPECT_NEAR(m_out2[0][7], 1.7375337944611973, 1e-9);
  EXPECT_NEAR(m_out[0][8], 1.1931780414311617, 1e-9);
  EXPECT_NEAR(m_out2[0][8], 1.1931780414311617, 1e-9);
  EXPECT_NEAR(m_out[0][9], 0.9821388835044149, 1e-9);
  EXPECT_NEAR(m_out2[0][9], 0.9821388835044149, 1e-9);
  EXPECT_NEAR(m_out[0][10], 1.4122234162801597, 1e-9);
  EXPECT_NEAR(m_out2[0][10], 1.4122234162801597, 1e-9);
  EXPECT_NEAR(m_out[1][0], 2.3179055606178318, 1e-9);
  EXPECT_NEAR(m_out2[1][0], 2.3179055606178318, 1e-9);
  EXPECT_NEAR(m_out[1][1], 2.5545374437881456, 1e-9);
  EXPECT_NEAR(m_out2[1][1], 2.5545374437881456, 1e-9);
  EXPECT_NEAR(m_out[1][2], 1.8669321717134990, 1e-9);
  EXPECT_NEAR(m_out2[1][2], 1.8669321717134990, 1e-9);
  EXPECT_NEAR(m_out[1][3], 1.9902031023857507, 1e-9);
  EXPECT_NEAR(m_out2[1][3], 1.9902031023857507, 1e-9);
  EXPECT_NEAR(m_out[1][4], 1.8287044872854037, 1e-9);
  EXPECT_NEAR(m_out2[1][4], 1.8287044872854037, 1e-9);
  EXPECT_NEAR(m_out[1][5], 3.1053808992696599, 1e-9);
  EXPECT_NEAR(m_out2[1][5], 3.1053808992696599, 1e-9);
  EXPECT_NEAR(m_out[1][6], 1.8724561891283962, 1e-9);
  EXPECT_NEAR(m_out2[1][6], 1.8724561891283962, 1e-9);
  EXPECT_NEAR(m_out[1][7], 2.0969422208016293, 1e-9);
  EXPECT_NEAR(m_out2[1][7], 2.0969422208016293, 1e-9);
  EXPECT_NEAR(m_out[1][8], 2.3736416896399124, 1e-9);
  EXPECT_NEAR(m_out2[1][8], 2.3736416896399124, 1e-9);
  EXPECT_NEAR(m_out[1][9], 1.7201907412959412, 1e-9);
  EXPECT_NEAR(m_out2[1][9], 1.7201907412959412, 1e-9);
  EXPECT_NEAR(m_out[1][10], 2.3482966208863427, 1e-9);
  EXPECT_NEAR(m_out2[1][10], 2.3482966208863427, 1e-9);
  EXPECT_NEAR(m_out[2][0], 1.2676463602064389, 1e-9);
  EXPECT_NEAR(m_out2[2][0], 1.2676463602064389, 1e-9);
  EXPECT_NEAR(m_out[2][1], 1.4311233003778931, 1e-9);
  EXPECT_NEAR(m_out2[2][1], 1.4311233003778931, 1e-9);
  EXPECT_NEAR(m_out[2][2], 1.2402177647344563, 1e-9);
  EXPECT_NEAR(m_out2[2][2], 1.2402177647344563, 1e-9);
  EXPECT_NEAR(m_out[2][3], 0.8146514607851487, 1e-9);
  EXPECT_NEAR(m_out2[2][3], 0.8146514607851487, 1e-9);
  EXPECT_NEAR(m_out[2][4], 0.7558465894242611, 1e-9);
  EXPECT_NEAR(m_out2[2][4], 0.7558465894242611, 1e-9);
  EXPECT_NEAR(m_out[2][5], 1.5999830358157556, 1e-9);
  EXPECT_NEAR(m_out2[2][5], 1.5999830358157556, 1e-9);
  EXPECT_NEAR(m_out[2][6], 0.6977026378510598, 1e-9);
  EXPECT_NEAR(m_out2[2][6], 0.6977026378510598, 1e-9);
  EXPECT_NEAR(m_out[2][7], 1.3895248095845896, 1e-9);
  EXPECT_NEAR(m_out2[2][7], 1.3895248095845896, 1e-9);
  EXPECT_NEAR(m_out[2][8], 0.7694955162685293, 1e-9);
  EXPECT_NEAR(m_out2[2][8], 0.7694955162685293, 1e-9);
  EXPECT_NEAR(m_out[2][9], 0.5721816862556489, 1e-9);
  EXPECT_NEAR(m_out2[2][9], 0.5721816862556489, 1e-9);
  EXPECT_NEAR(m_out[2][10], 1.1527215989563877, 1e-9);
  EXPECT_NEAR(m_out2[2][10], 1.1527215989563877, 1e-9);
  EXPECT_NEAR(m_out[3][0], 2.8169402720610477, 1e-9);
  EXPECT_NEAR(m_out2[3][0], 2.8169402720610477, 1e-9);
  EXPECT_NEAR(m_out[3][1], 2.4798818886533200, 1e-9);
  EXPECT_NEAR(m_out2[3][1], 2.4798818886533200, 1e-9);
  EXPECT_NEAR(m_out[3][2], 1.7578546793335290, 1e-9);
  EXPECT_NEAR(m_out2[3][2], 1.7578546793335290, 1e-9);
  EXPECT_NEAR(m_out[3][3], 2.0212312257568570, 1e-9);
  EXPECT_NEAR(m_out2[3][3], 2.0212312257568570, 1e-9);
  EXPECT_NEAR(m_out[3][4], 1.3284842915122999, 1e-9);
  EXPECT_NEAR(m_out2[3][4], 1.3284842915122999, 1e-9);
  EXPECT_NEAR(m_out[3][5], 2.4338950355215236, 1e-9);
  EXPECT_NEAR(m_out2[3][5], 2.4338950355215236, 1e-9);
  EXPECT_NEAR(m_out[3][6], 1.7851513772902805, 1e-9);
  EXPECT_NEAR(m_out2[3][6], 1.7851513772902805, 1e-9);
  EXPECT_NEAR(m_out[3][7], 2.3880187235005410, 1e-9);
  EXPECT_NEAR(m_out2[3][7], 2.3880187235005410, 1e-9);
  EXPECT_NEAR(m_out[3][8], 2.2423366307864985, 1e-9);
  EXPECT_NEAR(m_out2[3][8], 2.2423366307864985, 1e-9);
  EXPECT_NEAR(m_out[3][9], 1.3881220749611305, 1e-9);
  EXPECT_NEAR(m_out2[3][9], 1.3881220749611305, 1e-9);
  EXPECT_NEAR(m_out[3][10], 1.9585441881615786, 1e-9);
  EXPECT_NEAR(m_out2[3][10], 1.9585441881615786, 1e-9);
  EXPECT_NEAR(m_out[4][0], 2.6338869887744001, 1e-9);
  EXPECT_NEAR(m_out2[4][0], 2.6338869887744001, 1e-9);
  EXPECT_NEAR(m_out[4][1], 2.4246792175725425, 1e-9);
  EXPECT_NEAR(m_out2[4][1], 2.4246792175725425, 1e-9);
  EXPECT_NEAR(m_out[4][2], 1.8731478260666325, 1e-9);
  EXPECT_NEAR(m_out2[4][2], 1.8731478260666325, 1e-9);
  EXPECT_NEAR(m_out[4][3], 1.8518254559613647, 1e-9);
  EXPECT_NEAR(m_out2[4][3], 1.8518254559613647, 1e-9);
  EXPECT_NEAR(m_out[4][4], 1.4930122847077532, 1e-9);
  EXPECT_NEAR(m_out2[4][4], 1.4930122847077532, 1e-9);
  EXPECT_NEAR(m_out[4][5], 2.4903496674159271, 1e-9);
  EXPECT_NEAR(m_out2[4][5], 2.4903496674159271, 1e-9);
  EXPECT_NEAR(m_out[4][6], 2.1912115130633798, 1e-9);
  EXPECT_NEAR(m_out2[4][6], 2.1912115130633798, 1e-9);
  EXPECT_NEAR(m_out[4][7], 2.8085746188163698, 1e-9);
  EXPECT_NEAR(m_out2[4][7], 2.8085746188163698, 1e-9);
  EXPECT_NEAR(m_out[4][8], 2.5736637541568421, 1e-9);
  EXPECT_NEAR(m_out2[4][8], 2.5736637541568421, 1e-9);
  EXPECT_NEAR(m_out[4][9], 1.6818955022253799, 1e-9);
  EXPECT_NEAR(m_out2[4][9], 1.6818955022253799, 1e-9);
  EXPECT_NEAR(m_out[4][10], 2.5552290377514200, 1e-9);
  EXPECT_NEAR(m_out2[4][10], 2.5552290377514200, 1e-9);
  EXPECT_NEAR(m_out[5][0], 2.1152721109101704, 1e-9);
  EXPECT_NEAR(m_out2[5][0], 2.1152721109101704, 1e-9);
  EXPECT_NEAR(m_out[5][1], 2.2206163131959373, 1e-9);
  EXPECT_NEAR(m_out2[5][1], 2.2206163131959373, 1e-9);
  EXPECT_NEAR(m_out[5][2], 1.6457302167364869, 1e-9);
  EXPECT_NEAR(m_out2[5][2], 1.6457302167364869, 1e-9);
  EXPECT_NEAR(m_out[5][3], 1.6383926551968711, 1e-9);
  EXPECT_NEAR(m_out2[5][3], 1.6383926551968711, 1e-9);
  EXPECT_NEAR(m_out[5][4], 1.5879028045968093, 1e-9);
  EXPECT_NEAR(m_out2[5][4], 1.5879028045968093, 1e-9);
  EXPECT_NEAR(m_out[5][5], 2.7381693713555526, 1e-9);
  EXPECT_NEAR(m_out2[5][5], 2.7381693713555526, 1e-9);
  EXPECT_NEAR(m_out[5][6], 1.2743839680911906, 1e-9);
  EXPECT_NEAR(m_out2[5][6], 1.2743839680911906, 1e-9);
  EXPECT_NEAR(m_out[5][7], 1.9558498054789959, 1e-9);
  EXPECT_NEAR(m_out2[5][7], 1.9558498054789959, 1e-9);
  EXPECT_NEAR(m_out[5][8], 1.9573095434953653, 1e-9);
  EXPECT_NEAR(m_out2[5][8], 1.9573095434953653, 1e-9);
  EXPECT_NEAR(m_out[5][9], 1.4884498541039024, 1e-9);
  EXPECT_NEAR(m_out2[5][9], 1.4884498541039024, 1e-9);
  EXPECT_NEAR(m_out[5][10], 1.7555208302479111, 1e-9);
  EXPECT_NEAR(m_out2[5][10], 1.7555208302479111, 1e-9);
  EXPECT_NEAR(m_out[6][0], 2.1160784031685296, 1e-9);
  EXPECT_NEAR(m_out2[6][0], 2.1160784031685296, 1e-9);
  EXPECT_NEAR(m_out[6][1], 2.3441274139279438, 1e-9);
  EXPECT_NEAR(m_out2[6][1], 2.3441274139279438, 1e-9);
  EXPECT_NEAR(m_out[6][2], 1.7872976842610684, 1e-9);
  EXPECT_NEAR(m_out2[6][2], 1.7872976842610684, 1e-9);
  EXPECT_NEAR(m_out[6][3], 1.6305123954598721, 1e-9);
  EXPECT_NEAR(m_out2[6][3], 1.6305123954598721, 1e-9);
  EXPECT_NEAR(m_out[6][4], 1.1754076554212163, 1e-9);
  EXPECT_NEAR(m_out2[6][4], 1.1754076554212163, 1e-9);
  EXPECT_NEAR(m_out[6][5], 2.1127598040469500, 1e-9);
  EXPECT_NEAR(m_out2[6][5], 2.1127598040469500, 1e-9);
  EXPECT_NEAR(m_out[6][6], 2.0040942964154063, 1e-9);
  EXPECT_NEAR(m_out2[6][6], 2.0040942964154063, 1e-9);
  EXPECT_NEAR(m_out[6][7], 2.0008602153994373, 1e-9);
  EXPECT_NEAR(m_out2[6][7], 2.0008602153994373, 1e-9);
  EXPECT_NEAR(m_out[6][8], 1.6350521912932261, 1e-9);
  EXPECT_NEAR(m_out2[6][8], 1.6350521912932261, 1e-9);
  EXPECT_NEAR(m_out[6][9], 0.9863715667862938, 1e-9);
  EXPECT_NEAR(m_out2[6][9], 0.9863715667862938, 1e-9);
  EXPECT_NEAR(m_out[6][10], 2.3398869490292151, 1e-9);
  EXPECT_NEAR(m_out2[6][10], 2.3398869490292151, 1e-9);
  EXPECT_NEAR(m_out[7][0], 1.6705080752249317, 1e-9);
  EXPECT_NEAR(m_out2[7][0], 1.6705080752249317, 1e-9);
  EXPECT_NEAR(m_out[7][1], 1.5282242840114575, 1e-9);
  EXPECT_NEAR(m_out2[7][1], 1.5282242840114575, 1e-9);
  EXPECT_NEAR(m_out[7][2], 0.8673395389372212, 1e-9);
  EXPECT_NEAR(m_out2[7][2], 0.8673395389372212, 1e-9);
  EXPECT_NEAR(m_out[7][3], 1.3646211740512872, 1e-9);
  EXPECT_NEAR(m_out2[7][3], 1.3646211740512872, 1e-9);
  EXPECT_NEAR(m_out[7][4], 1.1241941381283245, 1e-9);
  EXPECT_NEAR(m_out2[7][4], 1.1241941381283245, 1e-9);
  EXPECT_NEAR(m_out[7][5], 1.8437235060536077, 1e-9);
  EXPECT_NEAR(m_out2[7][5], 1.8437235060536077, 1e-9);
  EXPECT_NEAR(m_out[7][6], 0.9687175031228932, 1e-9);
  EXPECT_NEAR(m_out2[7][6], 0.9687175031228932, 1e-9);
  EXPECT_NEAR(m_out[7][7], 1.2966359415103215, 1e-9);
  EXPECT_NEAR(m_out2[7][7], 1.2966359415103215, 1e-9);
  EXPECT_NEAR(m_out[7][8], 1.6733103979905137, 1e-9);
  EXPECT_NEAR(m_out2[7][8], 1.6733103979905137, 1e-9);
  EXPECT_NEAR(m_out[7][9], 1.0869713538385308, 1e-9);
  EXPECT_NEAR(m_out2[7][9], 1.0869713538385308, 1e-9);
  EXPECT_NEAR(m_out[7][10], 1.0383610283175444, 1e-9);
  EXPECT_NEAR(m_out2[7][10], 1.0383610283175444, 1e-9);
}

#if !defined(NDEBUG)

TEST(MatArrMultDeath, Death2) {
  double m1[2][2] = {{0.3720972672619831, 0.8917718710161776},
                     {0.4978391875735400, 0.9251131664880623}};
  double m_out[2][2] = {{0.7071115114601547, 0.5099518713070820},
                        {0.7564241332986313, 0.4297446858124492}};
  ASSERT_DEATH(MatArrMult(&m1[0][0], 2, 2, &m_out[0][0], 2, &m_out[0][0]),
               "MatArrMult: Assertion `m_out != m1 && m_out != m2' failed.");
  ASSERT_DEATH(MatArrMult(&m_out[0][0], 2, 2, &m1[0][0], 2, &m_out[0][0]),
               "MatArrMult: Assertion `m_out != m1 && m_out != m2' failed.");
}

#endif  // !defined(NDEBUG)

TEST(MatArrZero, Normal2) {
  double m[2][3] = {
      {0.9982226452938572, 0.7303351849658190, 0.7139922714809748},
      {0.0264717482607740, 0.0649745868331065, 0.3057886970111784}};
  MatArrZero(2, 3, &m[0][0]);

  EXPECT_NEAR(m[0][0], 0.0, 1e-9);
  EXPECT_NEAR(m[0][1], 0.0, 1e-9);
  EXPECT_NEAR(m[0][2], 0.0, 1e-9);
  EXPECT_NEAR(m[1][0], 0.0, 1e-9);
  EXPECT_NEAR(m[1][1], 0.0, 1e-9);
  EXPECT_NEAR(m[1][2], 0.0, 1e-9);
}

TEST(MatArrCopy, Normal2) {
  double m[7][5] = {{0.0138381593742636, 0.2093579828666029, 0.0514666443302573,
                     0.8175613067164339, 0.9985181620430735},
                    {0.4972281584955838, 0.9360576406889841, 0.6434250718293454,
                     0.9620047592901141, 0.2914620916929009},
                    {0.9190053589998228, 0.6617022740479204, 0.7151549515081964,
                     0.2231586912128338, 0.2770510836440253},
                    {0.8471257859976312, 0.4935767760868655, 0.3087070911420344,
                     0.9709038934179891, 0.0774063738155952},
                    {0.8909249486825752, 0.3084674185938360, 0.2989027178523461,
                     0.2975228822326064, 0.0962533969941179},
                    {0.7135477989519513, 0.4100067069885248, 0.9314568696977338,
                     0.9567926324112540, 0.5688955169386587},
                    {0.2417454788893482, 0.9447563855145724, 0.0231198358381542,
                     0.6055805988520496, 0.3747054331554711}};
  double m_out[7][5];
  MatArrCopy(&m[0][0], 7, 5, &m_out[0][0]);
  MatArrCopy(&m[0][0], 7, 5, &m[0][0]);

  EXPECT_NEAR(m_out[0][0], m[0][0], 1e-9);
  EXPECT_NEAR(m_out[0][1], m[0][1], 1e-9);
  EXPECT_NEAR(m_out[0][2], m[0][2], 1e-9);
  EXPECT_NEAR(m_out[0][3], m[0][3], 1e-9);
  EXPECT_NEAR(m_out[0][4], m[0][4], 1e-9);
  EXPECT_NEAR(m_out[1][0], m[1][0], 1e-9);
  EXPECT_NEAR(m_out[1][1], m[1][1], 1e-9);
  EXPECT_NEAR(m_out[1][2], m[1][2], 1e-9);
  EXPECT_NEAR(m_out[1][3], m[1][3], 1e-9);
  EXPECT_NEAR(m_out[1][4], m[1][4], 1e-9);
  EXPECT_NEAR(m_out[2][0], m[2][0], 1e-9);
  EXPECT_NEAR(m_out[2][1], m[2][1], 1e-9);
  EXPECT_NEAR(m_out[2][2], m[2][2], 1e-9);
  EXPECT_NEAR(m_out[2][3], m[2][3], 1e-9);
  EXPECT_NEAR(m_out[2][4], m[2][4], 1e-9);
  EXPECT_NEAR(m_out[3][0], m[3][0], 1e-9);
  EXPECT_NEAR(m_out[3][1], m[3][1], 1e-9);
  EXPECT_NEAR(m_out[3][2], m[3][2], 1e-9);
  EXPECT_NEAR(m_out[3][3], m[3][3], 1e-9);
  EXPECT_NEAR(m_out[3][4], m[3][4], 1e-9);
  EXPECT_NEAR(m_out[4][0], m[4][0], 1e-9);
  EXPECT_NEAR(m_out[4][1], m[4][1], 1e-9);
  EXPECT_NEAR(m_out[4][2], m[4][2], 1e-9);
  EXPECT_NEAR(m_out[4][3], m[4][3], 1e-9);
  EXPECT_NEAR(m_out[4][4], m[4][4], 1e-9);
  EXPECT_NEAR(m_out[5][0], m[5][0], 1e-9);
  EXPECT_NEAR(m_out[5][1], m[5][1], 1e-9);
  EXPECT_NEAR(m_out[5][2], m[5][2], 1e-9);
  EXPECT_NEAR(m_out[5][3], m[5][3], 1e-9);
  EXPECT_NEAR(m_out[5][4], m[5][4], 1e-9);
  EXPECT_NEAR(m_out[6][0], m[6][0], 1e-9);
  EXPECT_NEAR(m_out[6][1], m[6][1], 1e-9);
  EXPECT_NEAR(m_out[6][2], m[6][2], 1e-9);
  EXPECT_NEAR(m_out[6][3], m[6][3], 1e-9);
  EXPECT_NEAR(m_out[6][4], m[6][4], 1e-9);
}

TEST(MatVecMult, Normal2) {
  double m[6][10] = {
      {0.4732552636518118, 0.1587092153889162, 0.6122588851365660,
       0.2209319376174878, 0.7542872236174847, 0.1683933179899232,
       0.3999587804067308, 0.0130686195792998, 0.7368755628621279,
       0.0721095173203209},
      {0.4964799662581036, 0.8499109175680016, 0.3516340265800189,
       0.9372524974661773, 0.3962115558996797, 0.6980811552955185,
       0.4131192711727590, 0.0736445509212050, 0.1095635823655157,
       0.2166392835727420},
      {0.7757598401611016, 0.6169350115502961, 0.3649600813833465,
       0.9582284199924906, 0.0684915423625572, 0.9063176038983286,
       0.3815456577087466, 0.2309845444025832, 0.9967949907122085,
       0.4130517145132370},
      {0.3158822808567897, 0.7175674073486705, 0.0811532349227323,
       0.9250758941866227, 0.5796307838526239, 0.4663804436508350,
       0.4979512753313214, 0.5378007040689893, 0.3259558430663011,
       0.8592352102206009},
      {0.7629911497410196, 0.3660721702104379, 0.1781953671827378,
       0.7550633674956079, 0.6034013399070259, 0.5316430056237427,
       0.0716513799499980, 0.1513899708233374, 0.3423690886192421,
       0.2347395540119032},
      {0.9307957289196063, 0.6121873778392877, 0.5757929196630337,
       0.0088727779465044, 0.4458490475543198, 0.4187081355978762,
       0.1183451611005463, 0.3772671118641330, 0.9270893729951821,
       0.7094559915983498}};
  double v[10] = {0.0874374578281740, 0.1140860955380406, 0.8829330073220225,
                  0.5984856351906896, 0.9402387936083496, 0.6112856157001187,
                  0.5185543416001855, 0.9264610223821660, 0.7844484770319485,
                  0.0530527570853737};
  double v_out[6];
  Mat mn_m = {6, 10, &m[0][0], 0, 0};
  Vec n_v = {10, v, 0, 0};
  Vec n_v_out = {6, v_out, 0, 0};
  MatVecMult(&mn_m, &n_v, &n_v_out);

  EXPECT_NEAR(v_out[0], 2.3458158942876461, 1e-9);
  EXPECT_NEAR(v_out[1], 2.1909297474701406, 1e-9);
  EXPECT_NEAR(v_out[2], 2.8680509206011235, 1e-9);
  EXPECT_NEAR(v_out[3], 2.6226114643104794, 1e-9);
  EXPECT_NEAR(v_out[4], 2.0684705035163624, 1e-9);
  EXPECT_NEAR(v_out[5], 2.5158642297074176, 1e-9);
}

TEST(VecNorm, Normal15) {
  double d_v[22] = {
      0.5059340981797087,  0.1193430987470097,  -0.3698542754593563,
      0.6072098146296807,  -0.6943170498730125, -0.0514486255085846,
      -0.7529152306204798, 0.7573189467792225,  -0.7179987100634220,
      -0.6404856232388263, -0.6381549868888587, -0.6746340106217668,
      0.3379982321785682,  -0.6140010189382867, 0.4760543999149893,
      0.0971575920104528,  -0.5268242013207747, 0.9705619575956566,
      -0.6807721078659457, 0.2297596078555313,  -0.2708282946466039,
      -0.7523612629899283};
  const Vec v = {22, &d_v[0], 0, 22};
  double v_norm = VecNorm(&v);

  EXPECT_NEAR(2.7002663905496469, v_norm, 1e-9);
}

TEST(VecNormBound, Unbounded15) {
  double d_v[22] = {
      0.1263218783268356,  0.8200228088511290,  0.5334583000099997,
      0.0932109742527576,  0.7196156468732027,  -0.8969158254187615,
      0.0793684485979955,  0.8119156259557732,  -0.7158822504427409,
      -0.3842442590043729, -0.9963962571771097, 0.1602145600438629,
      -0.4978913221257857, 0.4304021271006091,  -0.0432794308746880,
      -0.7504726551158605, -0.7729923278488120, 0.4974166759271903,
      0.8032969225468480,  0.6454108865084305,  -0.7643948219002812,
      0.6271538539439745};
  const Vec v = {22, &d_v[0], 0, 22};
  double v_norm = VecNormBound(&v, 1.9810187482076924);

  EXPECT_NEAR(2.9207369026239092, v_norm, 1e-9);
}

TEST(VecNormBound, Bounded15) {
  double d_v[22] = {
      0.8219408340970709,  -0.8800983944728691, -0.3343961229840298,
      0.7393383344241795,  -0.6028765170695285, 0.9588712254059628,
      -0.0649484127595115, -0.6653176824721949, -0.2250201458803904,
      0.1131780395151631,  0.5269217943351405,  0.9083026481574588,
      -0.0025424082906826, -0.4882196289300740, -0.5179370689398146,
      -0.5604441797617730, -0.0192304397360663, 0.7994657084219154,
      -0.9633616271886922, 0.6647793350303126,  0.9142970777348232,
      -0.9734277914348342};
  const Vec v = {22, &d_v[0], 0, 22};
  double v_norm = VecNormBound(&v, 3.4330250061319330);

  EXPECT_NEAR(3.4330250061319330, v_norm, 1e-9);
}

TEST(VecNormSquared, Normal15) {
  double d_v[22] = {
      -0.4374252848630398, 0.7212825147443094,  -0.6110601691829187,
      -0.5671072588478430, 0.4840887518095145,  -0.8750395136518885,
      -0.8621192327310709, 0.6418561078903093,  0.3926173835478159,
      -0.6753202408476384, -0.1119157207895107, -0.6051330234024903,
      0.1514581584592003,  0.7207569993050322,  -0.1157991287404760,
      -0.2451083039651170, 0.0006392400348429,  -0.6592664112798368,
      0.9005576838932441,  -0.9518105008840081, -0.9182950130172276,
      -0.4881585197153437};
  const Vec v = {22, &d_v[0], 0, 22};
  double v_norm_sqrd = VecNormSquared(&v);

  EXPECT_NEAR(8.3998385634948036, v_norm_sqrd, 1e-9);
}

TEST(VecNormalize, Normal15) {
  double d_v[22] = {
      0.4402753421062735,  -0.7819011228616721, -0.8804081975436331,
      0.1534293576265411,  -0.7088942388898745, 0.6993927085159235,
      0.6950962539123129,  -0.4328751839996250, 0.9586993940336304,
      0.9166638622588497,  -0.2751973960606124, -0.1638146786020576,
      0.7717196321931803,  -0.3303261488498308, -0.5105799860336497,
      -0.0571556381377289, 0.3150642349861454,  0.7488265045367053,
      -0.2598883134916079, -0.2500932731303713, -0.7809134299831333,
      -0.9844724273364356};
  Vec v = {22, &d_v[0], 0, 22};
  const Vec *v_out_ptr = VecNormalize(&v, &v);

  EXPECT_NEAR(0.1513181871925901, v.d[0], 1e-9);
  EXPECT_NEAR(-0.2687315167578016, v.d[1], 1e-9);
  EXPECT_NEAR(-0.3025874031565485, v.d[2], 1e-9);
  EXPECT_NEAR(0.0527321201934760, v.d[3], 1e-9);
  EXPECT_NEAR(-0.2436397882900156, v.d[4], 1e-9);
  EXPECT_NEAR(0.2403742082898653, v.d[5], 1e-9);
  EXPECT_NEAR(0.2388975602476121, v.d[6], 1e-9);
  EXPECT_NEAR(-0.1487748276115615, v.d[7], 1e-9);
  EXPECT_NEAR(0.3294952964519801, v.d[8], 1e-9);
  EXPECT_NEAR(0.3150481088456822, v.d[9], 1e-9);
  EXPECT_NEAR(-0.0945825648395305, v.d[10], 1e-9);
  EXPECT_NEAR(-0.0563014500948744, v.d[11], 1e-9);
  EXPECT_NEAR(0.2652322412737284, v.d[12], 1e-9);
  EXPECT_NEAR(-0.1135297602339966, v.d[13], 1e-9);
  EXPECT_NEAR(-0.1754811830565357, v.d[14], 1e-9);
  EXPECT_NEAR(-0.0196438154121045, v.d[15], 1e-9);
  EXPECT_NEAR(0.1082843946227994, v.d[16], 1e-9);
  EXPECT_NEAR(0.2573641045764200, v.d[17], 1e-9);
  EXPECT_NEAR(-0.0893209878208378, v.d[18], 1e-9);
  EXPECT_NEAR(-0.0859545314032472, v.d[19], 1e-9);
  EXPECT_NEAR(-0.2683920566936324, v.d[20], 1e-9);
  EXPECT_NEAR(-0.3383532276256350, v.d[21], 1e-9);

  EXPECT_NEAR(0.1513181871925901, v_out_ptr->d[0], 1e-9);
  EXPECT_NEAR(-0.2687315167578016, v_out_ptr->d[1], 1e-9);
  EXPECT_NEAR(-0.3025874031565485, v_out_ptr->d[2], 1e-9);
  EXPECT_NEAR(0.0527321201934760, v_out_ptr->d[3], 1e-9);
  EXPECT_NEAR(-0.2436397882900156, v_out_ptr->d[4], 1e-9);
  EXPECT_NEAR(0.2403742082898653, v_out_ptr->d[5], 1e-9);
  EXPECT_NEAR(0.2388975602476121, v_out_ptr->d[6], 1e-9);
  EXPECT_NEAR(-0.1487748276115615, v_out_ptr->d[7], 1e-9);
  EXPECT_NEAR(0.3294952964519801, v_out_ptr->d[8], 1e-9);
  EXPECT_NEAR(0.3150481088456822, v_out_ptr->d[9], 1e-9);
  EXPECT_NEAR(-0.0945825648395305, v_out_ptr->d[10], 1e-9);
  EXPECT_NEAR(-0.0563014500948744, v_out_ptr->d[11], 1e-9);
  EXPECT_NEAR(0.2652322412737284, v_out_ptr->d[12], 1e-9);
  EXPECT_NEAR(-0.1135297602339966, v_out_ptr->d[13], 1e-9);
  EXPECT_NEAR(-0.1754811830565357, v_out_ptr->d[14], 1e-9);
  EXPECT_NEAR(-0.0196438154121045, v_out_ptr->d[15], 1e-9);
  EXPECT_NEAR(0.1082843946227994, v_out_ptr->d[16], 1e-9);
  EXPECT_NEAR(0.2573641045764200, v_out_ptr->d[17], 1e-9);
  EXPECT_NEAR(-0.0893209878208378, v_out_ptr->d[18], 1e-9);
  EXPECT_NEAR(-0.0859545314032472, v_out_ptr->d[19], 1e-9);
  EXPECT_NEAR(-0.2683920566936324, v_out_ptr->d[20], 1e-9);
  EXPECT_NEAR(-0.3383532276256350, v_out_ptr->d[21], 1e-9);
}

TEST(VecNormalize, Zero_15) {
  double d_v[22] = {0.0000000000000000, 0.0000000000000000, 0.0000000000000000,
                    0.0000000000000000, 0.0000000000000000, 0.0000000000000000,
                    0.0000000000000000, 0.0000000000000000, 0.0000000000000000,
                    0.0000000000000000, 0.0000000000000000, 0.0000000000000000,
                    0.0000000000000000, 0.0000000000000000, 0.0000000000000000,
                    0.0000000000000000, 0.0000000000000000, 0.0000000000000000,
                    0.0000000000000000, 0.0000000000000000, 0.0000000000000000,
                    0.0000000000000000};
  Vec v = {22, &d_v[0], 0, 22};
  const Vec *v_out_ptr = VecNormalize(&v, &v);

  EXPECT_NEAR(0.0000000000000000, v.d[0], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v.d[1], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v.d[2], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v.d[3], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v.d[4], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v.d[5], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v.d[6], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v.d[7], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v.d[8], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v.d[9], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v.d[10], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v.d[11], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v.d[12], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v.d[13], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v.d[14], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v.d[15], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v.d[16], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v.d[17], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v.d[18], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v.d[19], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v.d[20], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v.d[21], 1e-9);

  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[0], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[1], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[2], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[3], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[4], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[5], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[6], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[7], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[8], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[9], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[10], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[11], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[12], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[13], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[14], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[15], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[16], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[17], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[18], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[19], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[20], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[21], 1e-9);
}

TEST(VecScale, Normal15) {
  double d_v[22] = {
      0.8776965212099819,  -0.8309434883265541, -0.9243025991537344,
      0.4187795084316961,  0.8475799245808284,  -0.0246770775185068,
      -0.6091778193044961, 0.9724626738088307,  0.8132738415984337,
      -0.0806260006399535, -0.3419724898898473, -0.1282205296984140,
      -0.6409148214755278, 0.2295156409314931,  -0.9994560160259585,
      -0.0017847817605761, 0.2721037457704722,  -0.8612548514132230,
      -0.5042743777694185, 0.7739865725950896,  0.3715216986172671,
      0.3164208373966673};
  const Vec v = {22, &d_v[0], 0, 22};
  double d_v_out[22] = {
      0.6363105331077681,  -0.7188880682853094, -0.4861162715443825,
      0.3222876379276722,  0.2881758679784976,  0.2682685937470148,
      0.8619622340962261,  0.2533204550114019,  0.4795890583342834,
      0.9324048621114960,  0.6999333634190616,  0.3177942541775842,
      0.3305596847286310,  -0.4394683382023730, 0.0872196504628246,
      -0.1905609294505386, -0.5466525138596301, 0.1955807457761631,
      0.2404067586396650,  -0.1273376089765696, -0.8633258750890920,
      -0.9906315713721834};
  Vec v_out = {22, &d_v_out[0], 0, 22};
  const Vec *v_out_ptr = VecScale(&v, 0.6500054041610537, &v_out);

  EXPECT_NEAR(0.5705074819998451, v_out.d[0], 1e-9);
  EXPECT_NEAR(-0.5401177579646975, v_out.d[1], 1e-9);
  EXPECT_NEAR(-0.6008016845300355, v_out.d[2], 1e-9);
  EXPECT_NEAR(0.2722089436325120, v_out.d[3], 1e-9);
  EXPECT_NEAR(0.5509315314359567, v_out.d[4], 1e-9);
  EXPECT_NEAR(-0.0160402337459307, v_out.d[5], 1e-9);
  EXPECT_NEAR(-0.3959688746429683, v_out.d[6], 1e-9);
  EXPECT_NEAR(0.6321059933206479, v_out.d[7], 1e-9);
  EXPECT_NEAR(0.5286323921018027, v_out.d[8], 1e-9);
  EXPECT_NEAR(-0.0524073361318624, v_out.d[9], 1e-9);
  EXPECT_NEAR(-0.2222839665028120, v_out.d[10], 1e-9);
  EXPECT_NEAR(-0.0833440372283620, v_out.d[11], 1e-9);
  EXPECT_NEAR(-0.4165980975660100, v_out.d[12], 1e-9);
  EXPECT_NEAR(0.1491864069449585, v_out.d[13], 1e-9);
  EXPECT_NEAR(-0.6496518116381497, v_out.d[14], 1e-9);
  EXPECT_NEAR(-0.0011601177896225, v_out.d[15], 1e-9);
  EXPECT_NEAR(0.1768689052432724, v_out.d[16], 1e-9);
  EXPECT_NEAR(-0.5598203077785202, v_out.d[17], 1e-9);
  EXPECT_NEAR(-0.3277810707300747, v_out.d[18], 1e-9);
  EXPECT_NEAR(0.5030954549348999, v_out.d[19], 1e-9);
  EXPECT_NEAR(0.2414911118643179, v_out.d[20], 1e-9);
  EXPECT_NEAR(0.2056752542969998, v_out.d[21], 1e-9);

  EXPECT_NEAR(0.5705074819998451, v_out_ptr->d[0], 1e-9);
  EXPECT_NEAR(-0.5401177579646975, v_out_ptr->d[1], 1e-9);
  EXPECT_NEAR(-0.6008016845300355, v_out_ptr->d[2], 1e-9);
  EXPECT_NEAR(0.2722089436325120, v_out_ptr->d[3], 1e-9);
  EXPECT_NEAR(0.5509315314359567, v_out_ptr->d[4], 1e-9);
  EXPECT_NEAR(-0.0160402337459307, v_out_ptr->d[5], 1e-9);
  EXPECT_NEAR(-0.3959688746429683, v_out_ptr->d[6], 1e-9);
  EXPECT_NEAR(0.6321059933206479, v_out_ptr->d[7], 1e-9);
  EXPECT_NEAR(0.5286323921018027, v_out_ptr->d[8], 1e-9);
  EXPECT_NEAR(-0.0524073361318624, v_out_ptr->d[9], 1e-9);
  EXPECT_NEAR(-0.2222839665028120, v_out_ptr->d[10], 1e-9);
  EXPECT_NEAR(-0.0833440372283620, v_out_ptr->d[11], 1e-9);
  EXPECT_NEAR(-0.4165980975660100, v_out_ptr->d[12], 1e-9);
  EXPECT_NEAR(0.1491864069449585, v_out_ptr->d[13], 1e-9);
  EXPECT_NEAR(-0.6496518116381497, v_out_ptr->d[14], 1e-9);
  EXPECT_NEAR(-0.0011601177896225, v_out_ptr->d[15], 1e-9);
  EXPECT_NEAR(0.1768689052432724, v_out_ptr->d[16], 1e-9);
  EXPECT_NEAR(-0.5598203077785202, v_out_ptr->d[17], 1e-9);
  EXPECT_NEAR(-0.3277810707300747, v_out_ptr->d[18], 1e-9);
  EXPECT_NEAR(0.5030954549348999, v_out_ptr->d[19], 1e-9);
  EXPECT_NEAR(0.2414911118643179, v_out_ptr->d[20], 1e-9);
  EXPECT_NEAR(0.2056752542969998, v_out_ptr->d[21], 1e-9);
}

TEST(VecAdd, Normal15) {
  double d_v0[22] = {
      -0.1033962927766032, -0.8890247337980812, 0.6556788342103466,
      -0.6691362567933541, -0.5941321731650344, 0.3084458230288722,
      -0.4975238170192133, 0.6874791274467575,  0.0375337028140379,
      -0.9475453752167726, 0.4228984077876687,  -0.6374564901729642,
      -0.6049568690903475, 0.3254995815949524,  0.9312875057722936,
      0.5640740111097327,  0.1299620718570176,  0.9946905165933853,
      -0.3297952121377539, -0.7595101858848223, 0.8764235485891865,
      0.6145826907016101};
  const Vec v0 = {22, &d_v0[0], 0, 22};
  double d_v1[22] = {
      -0.0668439872619149, -0.2086683295917098, -0.5422772237332274,
      -0.9015479881176631, -0.8821662446209175, -0.8059570056295711,
      -0.3193854726072693, 0.1976293610590334,  -0.4964393844649364,
      -0.3772649605929050, 0.1491585015132952,  0.2854905003175765,
      -0.0295383955208786, 0.7589314152507829,  0.5108552678716678,
      0.2636914983223151,  -0.5242280983963765, 0.8628394070561878,
      -0.1171439255317819, -0.7300443626868875, 0.7520921967875751,
      0.1085188043553007};
  const Vec v1 = {22, &d_v1[0], 0, 22};
  double d_v_out[22] = {
      -0.1654651464670225, -0.1897075596050211, 0.4546435462402088,
      -0.5914168380839528, 0.3547753059220944,  -0.6449017969411919,
      -0.7594294861443665, 0.4306228910400027,  0.3261370150603078,
      0.0036796005230122,  -0.7697636357651065, -0.3994366038458106,
      -0.7759395529362245, -0.5058898695786587, -0.6064206275835839,
      0.3033891483860651,  -0.8799691187278951, -0.4329512178786312,
      -0.6966050897203555, 0.8145704393687481,  0.6117695925857911,
      -0.3310704813190071};
  Vec v_out = {22, &d_v_out[0], 0, 22};
  const Vec *v_out_ptr = VecAdd(&v0, &v1, &v_out);

  EXPECT_NEAR(-0.1702402800385181, v_out.d[0], 1e-9);
  EXPECT_NEAR(-1.0976930633897910, v_out.d[1], 1e-9);
  EXPECT_NEAR(0.1134016104771192, v_out.d[2], 1e-9);
  EXPECT_NEAR(-1.5706842449110172, v_out.d[3], 1e-9);
  EXPECT_NEAR(-1.4762984177859519, v_out.d[4], 1e-9);
  EXPECT_NEAR(-0.4975111826006988, v_out.d[5], 1e-9);
  EXPECT_NEAR(-0.8169092896264827, v_out.d[6], 1e-9);
  EXPECT_NEAR(0.8851084885057909, v_out.d[7], 1e-9);
  EXPECT_NEAR(-0.4589056816508985, v_out.d[8], 1e-9);
  EXPECT_NEAR(-1.3248103358096777, v_out.d[9], 1e-9);
  EXPECT_NEAR(0.5720569093009640, v_out.d[10], 1e-9);
  EXPECT_NEAR(-0.3519659898553877, v_out.d[11], 1e-9);
  EXPECT_NEAR(-0.6344952646112261, v_out.d[12], 1e-9);
  EXPECT_NEAR(1.0844309968457353, v_out.d[13], 1e-9);
  EXPECT_NEAR(1.4421427736439614, v_out.d[14], 1e-9);
  EXPECT_NEAR(0.8277655094320477, v_out.d[15], 1e-9);
  EXPECT_NEAR(-0.3942660265393589, v_out.d[16], 1e-9);
  EXPECT_NEAR(1.8575299236495730, v_out.d[17], 1e-9);
  EXPECT_NEAR(-0.4469391376695357, v_out.d[18], 1e-9);
  EXPECT_NEAR(-1.4895545485717099, v_out.d[19], 1e-9);
  EXPECT_NEAR(1.6285157453767616, v_out.d[20], 1e-9);
  EXPECT_NEAR(0.7231014950569108, v_out.d[21], 1e-9);

  EXPECT_NEAR(-0.1702402800385181, v_out_ptr->d[0], 1e-9);
  EXPECT_NEAR(-1.0976930633897910, v_out_ptr->d[1], 1e-9);
  EXPECT_NEAR(0.1134016104771192, v_out_ptr->d[2], 1e-9);
  EXPECT_NEAR(-1.5706842449110172, v_out_ptr->d[3], 1e-9);
  EXPECT_NEAR(-1.4762984177859519, v_out_ptr->d[4], 1e-9);
  EXPECT_NEAR(-0.4975111826006988, v_out_ptr->d[5], 1e-9);
  EXPECT_NEAR(-0.8169092896264827, v_out_ptr->d[6], 1e-9);
  EXPECT_NEAR(0.8851084885057909, v_out_ptr->d[7], 1e-9);
  EXPECT_NEAR(-0.4589056816508985, v_out_ptr->d[8], 1e-9);
  EXPECT_NEAR(-1.3248103358096777, v_out_ptr->d[9], 1e-9);
  EXPECT_NEAR(0.5720569093009640, v_out_ptr->d[10], 1e-9);
  EXPECT_NEAR(-0.3519659898553877, v_out_ptr->d[11], 1e-9);
  EXPECT_NEAR(-0.6344952646112261, v_out_ptr->d[12], 1e-9);
  EXPECT_NEAR(1.0844309968457353, v_out_ptr->d[13], 1e-9);
  EXPECT_NEAR(1.4421427736439614, v_out_ptr->d[14], 1e-9);
  EXPECT_NEAR(0.8277655094320477, v_out_ptr->d[15], 1e-9);
  EXPECT_NEAR(-0.3942660265393589, v_out_ptr->d[16], 1e-9);
  EXPECT_NEAR(1.8575299236495730, v_out_ptr->d[17], 1e-9);
  EXPECT_NEAR(-0.4469391376695357, v_out_ptr->d[18], 1e-9);
  EXPECT_NEAR(-1.4895545485717099, v_out_ptr->d[19], 1e-9);
  EXPECT_NEAR(1.6285157453767616, v_out_ptr->d[20], 1e-9);
  EXPECT_NEAR(0.7231014950569108, v_out_ptr->d[21], 1e-9);
}

TEST(VecAdd3, Normal15) {
  double d_v0[22] = {
      -0.3743309897895268, -0.5744736874750187, 0.8196818316842767,
      0.9083060396058953,  -0.0261602247713963, 0.1416683884918486,
      0.7526592717728415,  0.6639040152106530,  0.0566021072462608,
      -0.1209412854602143, -0.4227513582122238, -0.5921500134475495,
      0.3326732228520024,  -0.0493423678163973, -0.2152542434628208,
      0.1378398910995613,  -0.1953127180433518, 0.2212420925172718,
      -0.8406872963417507, -0.7119433828235964, -0.0236368225821857,
      0.6879318781778969};
  const Vec v0 = {22, &d_v0[0], 0, 22};
  double d_v1[22] = {
      0.8686294092910785,  -0.9233265569519324, 0.2670211818291088,
      -0.7101463897517415, 0.6081787395012843,  0.9562650256796899,
      -0.8520602360429150, 0.6489775550563277,  0.1311914956263098,
      -0.4634697718156366, -0.3526132437248850, 0.9743326082209511,
      0.9976991688481485,  0.2192465016564293,  0.2243561734703017,
      0.0646326266690438,  -0.4859146829109799, -0.0165722012616649,
      -0.7717094447342554, -0.6087042188964489, 0.8601952269496020,
      0.5358710697525901};
  const Vec v1 = {22, &d_v1[0], 0, 22};
  double d_v2[22] = {
      -0.9188825595187891, -0.2379319214724573, -0.9494802095348602,
      0.4023701657493550,  -0.0269450427595566, 0.2272838919413438,
      0.7185222568642813,  0.6845142606377286,  0.0759560526085457,
      0.1065457057340622,  -0.0292102708753181, 0.5409875566616389,
      0.7594359672852495,  -0.4693480067153977, 0.6438122189555264,
      0.8333400198279577,  0.8686895142023086,  0.5748150947076045,
      0.8649077137871806,  0.5588978445804171,  -0.2073295270240647,
      0.5808083551708301};
  const Vec v2 = {22, &d_v2[0], 0, 22};
  double d_v0c[22] = {
      -0.3743309897895268, -0.5744736874750187, 0.8196818316842767,
      0.9083060396058953,  -0.0261602247713963, 0.1416683884918486,
      0.7526592717728415,  0.6639040152106530,  0.0566021072462608,
      -0.1209412854602143, -0.4227513582122238, -0.5921500134475495,
      0.3326732228520024,  -0.0493423678163973, -0.2152542434628208,
      0.1378398910995613,  -0.1953127180433518, 0.2212420925172718,
      -0.8406872963417507, -0.7119433828235964, -0.0236368225821857,
      0.6879318781778969};
  Vec v0c = {22, &d_v0c[0], 0, 22};
  double d_v1c[22] = {
      0.8686294092910785,  -0.9233265569519324, 0.2670211818291088,
      -0.7101463897517415, 0.6081787395012843,  0.9562650256796899,
      -0.8520602360429150, 0.6489775550563277,  0.1311914956263098,
      -0.4634697718156366, -0.3526132437248850, 0.9743326082209511,
      0.9976991688481485,  0.2192465016564293,  0.2243561734703017,
      0.0646326266690438,  -0.4859146829109799, -0.0165722012616649,
      -0.7717094447342554, -0.6087042188964489, 0.8601952269496020,
      0.5358710697525901};
  Vec v1c = {22, &d_v1c[0], 0, 22};
  double d_v2c[22] = {
      -0.9188825595187891, -0.2379319214724573, -0.9494802095348602,
      0.4023701657493550,  -0.0269450427595566, 0.2272838919413438,
      0.7185222568642813,  0.6845142606377286,  0.0759560526085457,
      0.1065457057340622,  -0.0292102708753181, 0.5409875566616389,
      0.7594359672852495,  -0.4693480067153977, 0.6438122189555264,
      0.8333400198279577,  0.8686895142023086,  0.5748150947076045,
      0.8649077137871806,  0.5588978445804171,  -0.2073295270240647,
      0.5808083551708301};
  Vec v2c = {22, &d_v2c[0], 0, 22};
  double d_v_out[22] = {
      -0.2021280790926949, -0.3230720946472951, -0.9210093132830490,
      -0.3005546429160626, -0.3851509715512409, -0.8186060553082992,
      0.1919305612103956,  0.1395784674653138,  -0.8682492048603150,
      -0.9069249263414483, -0.5196421186625853, 0.3241473884936028,
      -0.5608506558890736, 0.0966366921150250,  0.9425414739759654,
      -0.1843606235022757, 0.9434585919777361,  -0.1363637497264376,
      0.9352683202982255,  0.6471200441518616,  0.1392817258850196,
      -0.4985840193475133};
  Vec v_out = {22, &d_v_out[0], 0, 22};
  const Vec *v_out_ptr = VecAdd3(&v0, &v1, &v2, &v_out);
  VecAdd3(&v0c, &v1, &v2, &v0c);
  VecAdd3(&v0, &v1c, &v2, &v1c);
  VecAdd3(&v0, &v1, &v2c, &v2c);

  EXPECT_NEAR(-0.4245841400172374, v_out.d[0], 1e-9);
  EXPECT_NEAR(-1.7357321658994085, v_out.d[1], 1e-9);
  EXPECT_NEAR(0.1372228039785253, v_out.d[2], 1e-9);
  EXPECT_NEAR(0.6005298156035088, v_out.d[3], 1e-9);
  EXPECT_NEAR(0.5550734719703314, v_out.d[4], 1e-9);
  EXPECT_NEAR(1.3252173061128822, v_out.d[5], 1e-9);
  EXPECT_NEAR(0.6191212925942078, v_out.d[6], 1e-9);
  EXPECT_NEAR(1.9973958309047093, v_out.d[7], 1e-9);
  EXPECT_NEAR(0.2637496554811163, v_out.d[8], 1e-9);
  EXPECT_NEAR(-0.4778653515417888, v_out.d[9], 1e-9);
  EXPECT_NEAR(-0.8045748728124269, v_out.d[10], 1e-9);
  EXPECT_NEAR(0.9231701514350406, v_out.d[11], 1e-9);
  EXPECT_NEAR(2.0898083589854002, v_out.d[12], 1e-9);
  EXPECT_NEAR(-0.2994438728753657, v_out.d[13], 1e-9);
  EXPECT_NEAR(0.6529141489630073, v_out.d[14], 1e-9);
  EXPECT_NEAR(1.0358125375965628, v_out.d[15], 1e-9);
  EXPECT_NEAR(0.1874621132479768, v_out.d[16], 1e-9);
  EXPECT_NEAR(0.7794849859632114, v_out.d[17], 1e-9);
  EXPECT_NEAR(-0.7474890272888255, v_out.d[18], 1e-9);
  EXPECT_NEAR(-0.7617497571396281, v_out.d[19], 1e-9);
  EXPECT_NEAR(0.6292288773433516, v_out.d[20], 1e-9);
  EXPECT_NEAR(1.8046113031013171, v_out.d[21], 1e-9);

  EXPECT_NEAR(-0.4245841400172374, v0c.d[0], 1e-9);
  EXPECT_NEAR(-1.7357321658994085, v0c.d[1], 1e-9);
  EXPECT_NEAR(0.1372228039785253, v0c.d[2], 1e-9);
  EXPECT_NEAR(0.6005298156035088, v0c.d[3], 1e-9);
  EXPECT_NEAR(0.5550734719703314, v0c.d[4], 1e-9);
  EXPECT_NEAR(1.3252173061128822, v0c.d[5], 1e-9);
  EXPECT_NEAR(0.6191212925942078, v0c.d[6], 1e-9);
  EXPECT_NEAR(1.9973958309047093, v0c.d[7], 1e-9);
  EXPECT_NEAR(0.2637496554811163, v0c.d[8], 1e-9);
  EXPECT_NEAR(-0.4778653515417888, v0c.d[9], 1e-9);
  EXPECT_NEAR(-0.8045748728124269, v0c.d[10], 1e-9);
  EXPECT_NEAR(0.9231701514350406, v0c.d[11], 1e-9);
  EXPECT_NEAR(2.0898083589854002, v0c.d[12], 1e-9);
  EXPECT_NEAR(-0.2994438728753657, v0c.d[13], 1e-9);
  EXPECT_NEAR(0.6529141489630073, v0c.d[14], 1e-9);
  EXPECT_NEAR(1.0358125375965628, v0c.d[15], 1e-9);
  EXPECT_NEAR(0.1874621132479768, v0c.d[16], 1e-9);
  EXPECT_NEAR(0.7794849859632114, v0c.d[17], 1e-9);
  EXPECT_NEAR(-0.7474890272888255, v0c.d[18], 1e-9);
  EXPECT_NEAR(-0.7617497571396281, v0c.d[19], 1e-9);
  EXPECT_NEAR(0.6292288773433516, v0c.d[20], 1e-9);
  EXPECT_NEAR(1.8046113031013171, v0c.d[21], 1e-9);

  EXPECT_NEAR(-0.4245841400172374, v1c.d[0], 1e-9);
  EXPECT_NEAR(-1.7357321658994085, v1c.d[1], 1e-9);
  EXPECT_NEAR(0.1372228039785253, v1c.d[2], 1e-9);
  EXPECT_NEAR(0.6005298156035088, v1c.d[3], 1e-9);
  EXPECT_NEAR(0.5550734719703314, v1c.d[4], 1e-9);
  EXPECT_NEAR(1.3252173061128822, v1c.d[5], 1e-9);
  EXPECT_NEAR(0.6191212925942078, v1c.d[6], 1e-9);
  EXPECT_NEAR(1.9973958309047093, v1c.d[7], 1e-9);
  EXPECT_NEAR(0.2637496554811163, v1c.d[8], 1e-9);
  EXPECT_NEAR(-0.4778653515417888, v1c.d[9], 1e-9);
  EXPECT_NEAR(-0.8045748728124269, v1c.d[10], 1e-9);
  EXPECT_NEAR(0.9231701514350406, v1c.d[11], 1e-9);
  EXPECT_NEAR(2.0898083589854002, v1c.d[12], 1e-9);
  EXPECT_NEAR(-0.2994438728753657, v1c.d[13], 1e-9);
  EXPECT_NEAR(0.6529141489630073, v1c.d[14], 1e-9);
  EXPECT_NEAR(1.0358125375965628, v1c.d[15], 1e-9);
  EXPECT_NEAR(0.1874621132479768, v1c.d[16], 1e-9);
  EXPECT_NEAR(0.7794849859632114, v1c.d[17], 1e-9);
  EXPECT_NEAR(-0.7474890272888255, v1c.d[18], 1e-9);
  EXPECT_NEAR(-0.7617497571396281, v1c.d[19], 1e-9);
  EXPECT_NEAR(0.6292288773433516, v1c.d[20], 1e-9);
  EXPECT_NEAR(1.8046113031013171, v1c.d[21], 1e-9);

  EXPECT_NEAR(-0.4245841400172374, v2c.d[0], 1e-9);
  EXPECT_NEAR(-1.7357321658994085, v2c.d[1], 1e-9);
  EXPECT_NEAR(0.1372228039785253, v2c.d[2], 1e-9);
  EXPECT_NEAR(0.6005298156035088, v2c.d[3], 1e-9);
  EXPECT_NEAR(0.5550734719703314, v2c.d[4], 1e-9);
  EXPECT_NEAR(1.3252173061128822, v2c.d[5], 1e-9);
  EXPECT_NEAR(0.6191212925942078, v2c.d[6], 1e-9);
  EXPECT_NEAR(1.9973958309047093, v2c.d[7], 1e-9);
  EXPECT_NEAR(0.2637496554811163, v2c.d[8], 1e-9);
  EXPECT_NEAR(-0.4778653515417888, v2c.d[9], 1e-9);
  EXPECT_NEAR(-0.8045748728124269, v2c.d[10], 1e-9);
  EXPECT_NEAR(0.9231701514350406, v2c.d[11], 1e-9);
  EXPECT_NEAR(2.0898083589854002, v2c.d[12], 1e-9);
  EXPECT_NEAR(-0.2994438728753657, v2c.d[13], 1e-9);
  EXPECT_NEAR(0.6529141489630073, v2c.d[14], 1e-9);
  EXPECT_NEAR(1.0358125375965628, v2c.d[15], 1e-9);
  EXPECT_NEAR(0.1874621132479768, v2c.d[16], 1e-9);
  EXPECT_NEAR(0.7794849859632114, v2c.d[17], 1e-9);
  EXPECT_NEAR(-0.7474890272888255, v2c.d[18], 1e-9);
  EXPECT_NEAR(-0.7617497571396281, v2c.d[19], 1e-9);
  EXPECT_NEAR(0.6292288773433516, v2c.d[20], 1e-9);
  EXPECT_NEAR(1.8046113031013171, v2c.d[21], 1e-9);

  EXPECT_NEAR(-0.4245841400172374, v_out_ptr->d[0], 1e-9);
  EXPECT_NEAR(-1.7357321658994085, v_out_ptr->d[1], 1e-9);
  EXPECT_NEAR(0.1372228039785253, v_out_ptr->d[2], 1e-9);
  EXPECT_NEAR(0.6005298156035088, v_out_ptr->d[3], 1e-9);
  EXPECT_NEAR(0.5550734719703314, v_out_ptr->d[4], 1e-9);
  EXPECT_NEAR(1.3252173061128822, v_out_ptr->d[5], 1e-9);
  EXPECT_NEAR(0.6191212925942078, v_out_ptr->d[6], 1e-9);
  EXPECT_NEAR(1.9973958309047093, v_out_ptr->d[7], 1e-9);
  EXPECT_NEAR(0.2637496554811163, v_out_ptr->d[8], 1e-9);
  EXPECT_NEAR(-0.4778653515417888, v_out_ptr->d[9], 1e-9);
  EXPECT_NEAR(-0.8045748728124269, v_out_ptr->d[10], 1e-9);
  EXPECT_NEAR(0.9231701514350406, v_out_ptr->d[11], 1e-9);
  EXPECT_NEAR(2.0898083589854002, v_out_ptr->d[12], 1e-9);
  EXPECT_NEAR(-0.2994438728753657, v_out_ptr->d[13], 1e-9);
  EXPECT_NEAR(0.6529141489630073, v_out_ptr->d[14], 1e-9);
  EXPECT_NEAR(1.0358125375965628, v_out_ptr->d[15], 1e-9);
  EXPECT_NEAR(0.1874621132479768, v_out_ptr->d[16], 1e-9);
  EXPECT_NEAR(0.7794849859632114, v_out_ptr->d[17], 1e-9);
  EXPECT_NEAR(-0.7474890272888255, v_out_ptr->d[18], 1e-9);
  EXPECT_NEAR(-0.7617497571396281, v_out_ptr->d[19], 1e-9);
  EXPECT_NEAR(0.6292288773433516, v_out_ptr->d[20], 1e-9);
  EXPECT_NEAR(1.8046113031013171, v_out_ptr->d[21], 1e-9);
}

TEST(VecSub, Normal15) {
  double d_v0[22] = {
      -0.9000873693851155, -0.5977454075497792, -0.4035191206462279,
      0.8447727254739417,  -0.5820548867089854, -0.6968714057801491,
      0.6705801271886531,  0.4977159437855969,  -0.3805988120077906,
      -0.0502384287769009, 0.9765992742657066,  -0.6152860414600632,
      -0.5259841497905826, -0.2676216070506996, -0.6557034038246115,
      -0.4840655209924598, -0.6918727711640982, -0.2015178086709661,
      0.7379044226545386,  0.2451538523613082,  0.7304130539149269,
      -0.1766781163438158};
  const Vec v0 = {22, &d_v0[0], 0, 22};
  double d_v1[22] = {
      -0.6118859175668674, 0.8721984980769877,  -0.2886348932491065,
      -0.0458603273112204, -0.6930346848033422, 0.4870970586311856,
      0.0820219446113208,  -0.8457408806158191, -0.8056967389021832,
      -0.5611030322830872, -0.7039690397873233, -0.0482959874446727,
      0.4896316398113767,  0.1001352159924953,  -0.7544403877120611,
      -0.8389499908970861, -0.6876875770951807, -0.4471663954768204,
      0.3788981893453105,  -0.1809387113729910, -0.2793211989973530,
      0.4277800008032291};
  const Vec v1 = {22, &d_v1[0], 0, 22};
  double d_v_out[22] = {
      -0.6342709333954717, 0.9290176227062705,  -0.1699143891688943,
      0.2155790227448038,  0.2902766201905789,  -0.9539055450527609,
      0.1420018641708691,  0.9206833241935553,  0.9609068205421640,
      0.5609887579603361,  0.2175808658006970,  0.6039278349837767,
      -0.2181577504552417, -0.9311268633467957, -0.2082305954766528,
      0.7786759193440771,  -0.4235515941407779, -0.3625657027722298,
      -0.2688118777330430, -0.0531091917751831, 0.4327722148845334,
      0.2524936078376321};
  Vec v_out = {22, &d_v_out[0], 0, 22};
  const Vec *v_out_ptr = VecSub(&v0, &v1, &v_out);

  EXPECT_NEAR(-0.2882014518182481, v_out.d[0], 1e-9);
  EXPECT_NEAR(-1.4699439056267669, v_out.d[1], 1e-9);
  EXPECT_NEAR(-0.1148842273971213, v_out.d[2], 1e-9);
  EXPECT_NEAR(0.8906330527851620, v_out.d[3], 1e-9);
  EXPECT_NEAR(0.1109797980943568, v_out.d[4], 1e-9);
  EXPECT_NEAR(-1.1839684644113346, v_out.d[5], 1e-9);
  EXPECT_NEAR(0.5885581825773323, v_out.d[6], 1e-9);
  EXPECT_NEAR(1.3434568244014160, v_out.d[7], 1e-9);
  EXPECT_NEAR(0.4250979268943926, v_out.d[8], 1e-9);
  EXPECT_NEAR(0.5108646035061863, v_out.d[9], 1e-9);
  EXPECT_NEAR(1.6805683140530299, v_out.d[10], 1e-9);
  EXPECT_NEAR(-0.5669900540153905, v_out.d[11], 1e-9);
  EXPECT_NEAR(-1.0156157896019593, v_out.d[12], 1e-9);
  EXPECT_NEAR(-0.3677568230431949, v_out.d[13], 1e-9);
  EXPECT_NEAR(0.0987369838874497, v_out.d[14], 1e-9);
  EXPECT_NEAR(0.3548844699046263, v_out.d[15], 1e-9);
  EXPECT_NEAR(-0.0041851940689175, v_out.d[16], 1e-9);
  EXPECT_NEAR(0.2456485868058542, v_out.d[17], 1e-9);
  EXPECT_NEAR(0.3590062333092281, v_out.d[18], 1e-9);
  EXPECT_NEAR(0.4260925637342992, v_out.d[19], 1e-9);
  EXPECT_NEAR(1.0097342529122799, v_out.d[20], 1e-9);
  EXPECT_NEAR(-0.6044581171470449, v_out.d[21], 1e-9);

  EXPECT_NEAR(-0.2882014518182481, v_out_ptr->d[0], 1e-9);
  EXPECT_NEAR(-1.4699439056267669, v_out_ptr->d[1], 1e-9);
  EXPECT_NEAR(-0.1148842273971213, v_out_ptr->d[2], 1e-9);
  EXPECT_NEAR(0.8906330527851620, v_out_ptr->d[3], 1e-9);
  EXPECT_NEAR(0.1109797980943568, v_out_ptr->d[4], 1e-9);
  EXPECT_NEAR(-1.1839684644113346, v_out_ptr->d[5], 1e-9);
  EXPECT_NEAR(0.5885581825773323, v_out_ptr->d[6], 1e-9);
  EXPECT_NEAR(1.3434568244014160, v_out_ptr->d[7], 1e-9);
  EXPECT_NEAR(0.4250979268943926, v_out_ptr->d[8], 1e-9);
  EXPECT_NEAR(0.5108646035061863, v_out_ptr->d[9], 1e-9);
  EXPECT_NEAR(1.6805683140530299, v_out_ptr->d[10], 1e-9);
  EXPECT_NEAR(-0.5669900540153905, v_out_ptr->d[11], 1e-9);
  EXPECT_NEAR(-1.0156157896019593, v_out_ptr->d[12], 1e-9);
  EXPECT_NEAR(-0.3677568230431949, v_out_ptr->d[13], 1e-9);
  EXPECT_NEAR(0.0987369838874497, v_out_ptr->d[14], 1e-9);
  EXPECT_NEAR(0.3548844699046263, v_out_ptr->d[15], 1e-9);
  EXPECT_NEAR(-0.0041851940689175, v_out_ptr->d[16], 1e-9);
  EXPECT_NEAR(0.2456485868058542, v_out_ptr->d[17], 1e-9);
  EXPECT_NEAR(0.3590062333092281, v_out_ptr->d[18], 1e-9);
  EXPECT_NEAR(0.4260925637342992, v_out_ptr->d[19], 1e-9);
  EXPECT_NEAR(1.0097342529122799, v_out_ptr->d[20], 1e-9);
  EXPECT_NEAR(-0.6044581171470449, v_out_ptr->d[21], 1e-9);
}

TEST(VecLinComb, Normal15) {
  double d_v0[22] = {
      0.9234299862074802,  -0.9869229564244266, -0.2121356905298766,
      0.5873344759470611,  0.1695870344076935,  -0.7908522343339206,
      0.7642782877330678,  -0.6555606078143668, -0.8911904506200627,
      0.3103073505619314,  -0.0503868672676795, 0.5378529799018956,
      0.3437224745620961,  -0.6452129467439123, -0.5857299380989238,
      0.3989418616405196,  0.5288629277563985,  -0.1648514048457359,
      -0.5123510543450576, -0.3405184671190595, 0.9353280470561534,
      -0.4784616829355623};
  const Vec v0 = {22, &d_v0[0], 0, 22};
  double d_v1[22] = {
      0.7446419153718729,  0.0762688582600028,  -0.8437838115520195,
      -0.5503191347723799, -0.4021474410932051, 0.6320338365089695,
      0.7657570694174893,  0.8097481765454795,  -0.5807954233293302,
      -0.6121430086492692, 0.5851315730045150,  0.4722303009449131,
      -0.5359580437016629, -0.4618854523548495, 0.5171063410556493,
      -0.7749515959812974, 0.8214943545196973,  -0.1216427193760188,
      0.9216395178636414,  -0.1318243354079314, 0.2396750872331386,
      0.5755245840108911};
  const Vec v1 = {22, &d_v1[0], 0, 22};
  double d_v_out[22] = {
      -0.2012632258190235, 0.4216101166613866,  0.2273027968857231,
      -0.7992070412028691, -0.9747920625029274, 0.0883607333476741,
      -0.3398233240376554, 0.9788461364918641,  -0.2812809244522878,
      0.2975847764219854,  0.6388348963681780,  -0.7457082887745727,
      0.4519489275707689,  0.8412215588810903,  0.7041622683909041,
      0.6086052905253616,  0.3661386246916034,  -0.1447590157130993,
      0.6540256216823472,  -0.7460725982724290, 0.0303318331610938,
      -0.5014471147783048};
  Vec v_out = {22, &d_v_out[0], 0, 22};
  const Vec *v_out_ptr =
      VecLinComb(-0.9464974435761642, &v0, -0.2505908999007374, &v1, &v_out);

  EXPECT_NEAR(-1.0606246089437987, v_out.d[0], 1e-9);
  EXPECT_NEAR(0.9150077734365740, v_out.d[1], 1e-9);
  EXPECT_NEAR(0.4122304334362872, v_out.d[2], 1e-9);
  EXPECT_NEAR(-0.4180056127928335, v_out.d[3], 1e-9);
  EXPECT_NEAR(-0.0597392053742199, v_out.d[4], 1e-9);
  EXPECT_NEAR(0.5901576901850553, v_out.d[5], 1e-9);
  EXPECT_NEAR(-0.9152791986507968, v_out.d[6], 1e-9);
  EXPECT_NEAR(0.4175709151520217, v_out.d[7], 1e-9);
  EXPECT_NEAR(0.9890515310417057, v_out.d[8], 1e-9);
  EXPECT_NEAR(-0.1403076466243954, v_out.d[9], 1e-9);
  EXPECT_NEAR(-0.0989376063808653, v_out.d[10], 1e-9);
  EXPECT_NEAR(-0.6274130865711480, v_out.d[11], 1e-9);
  EXPECT_NEAR(-0.1910262349924587, v_out.d[12], 1e-9);
  EXPECT_NEAR(0.7264366958120175, v_out.d[13], 1e-9);
  EXPECT_NEAR(0.4248097456871435, v_out.d[14], 1e-9);
  EXPECT_NEAR(-0.1834016343618016, v_out.d[15], 1e-9);
  EXPECT_NEAR(-0.7064264186861031, v_out.d[16], 1e-9);
  EXPECT_NEAR(0.1865139917712377, v_out.d[17], 1e-9);
  EXPECT_NEAR(0.2539844869856178, v_out.d[18], 1e-9);
  EXPECT_NEAR(0.3553338374573541, v_out.d[19], 1e-9);
  EXPECT_NEAR(-0.9453460012372753, v_out.d[20], 1e-9);
  EXPECT_NEAR(0.3086415363253723, v_out.d[21], 1e-9);

  EXPECT_NEAR(-1.0606246089437987, v_out_ptr->d[0], 1e-9);
  EXPECT_NEAR(0.9150077734365740, v_out_ptr->d[1], 1e-9);
  EXPECT_NEAR(0.4122304334362872, v_out_ptr->d[2], 1e-9);
  EXPECT_NEAR(-0.4180056127928335, v_out_ptr->d[3], 1e-9);
  EXPECT_NEAR(-0.0597392053742199, v_out_ptr->d[4], 1e-9);
  EXPECT_NEAR(0.5901576901850553, v_out_ptr->d[5], 1e-9);
  EXPECT_NEAR(-0.9152791986507968, v_out_ptr->d[6], 1e-9);
  EXPECT_NEAR(0.4175709151520217, v_out_ptr->d[7], 1e-9);
  EXPECT_NEAR(0.9890515310417057, v_out_ptr->d[8], 1e-9);
  EXPECT_NEAR(-0.1403076466243954, v_out_ptr->d[9], 1e-9);
  EXPECT_NEAR(-0.0989376063808653, v_out_ptr->d[10], 1e-9);
  EXPECT_NEAR(-0.6274130865711480, v_out_ptr->d[11], 1e-9);
  EXPECT_NEAR(-0.1910262349924587, v_out_ptr->d[12], 1e-9);
  EXPECT_NEAR(0.7264366958120175, v_out_ptr->d[13], 1e-9);
  EXPECT_NEAR(0.4248097456871435, v_out_ptr->d[14], 1e-9);
  EXPECT_NEAR(-0.1834016343618016, v_out_ptr->d[15], 1e-9);
  EXPECT_NEAR(-0.7064264186861031, v_out_ptr->d[16], 1e-9);
  EXPECT_NEAR(0.1865139917712377, v_out_ptr->d[17], 1e-9);
  EXPECT_NEAR(0.2539844869856178, v_out_ptr->d[18], 1e-9);
  EXPECT_NEAR(0.3553338374573541, v_out_ptr->d[19], 1e-9);
  EXPECT_NEAR(-0.9453460012372753, v_out_ptr->d[20], 1e-9);
  EXPECT_NEAR(0.3086415363253723, v_out_ptr->d[21], 1e-9);
}

TEST(VecLinComb3, Normal15) {
  double d_v0[22] = {
      -0.3425973332616812, -0.5984197654543781, -0.4107857364980745,
      0.4111010533352921,  -0.5010262069829647, 0.6048951252334733,
      -0.7996596461103329, -0.3604466614588564, 0.9612927288848783,
      0.0022763596544471,  0.6855918320046919,  0.0639676098526929,
      -0.0297064870639532, 0.8490060711345935,  -0.4930360606791564,
      -0.1750607430642583, 0.5551932505361905,  0.8082115080628385,
      -0.2734205532604228, -0.3503044115063565, -0.4223041188264949,
      -0.3415871836881001};
  const Vec v0 = {22, &d_v0[0], 0, 22};
  double d_v1[22] = {
      -0.3043726716492201, 0.2196547577572883,  0.0951316345889031,
      0.3700133874629019,  0.0147088040817398,  -0.1814076745981925,
      0.5805583831024368,  0.2172989542457213,  0.6817553733520043,
      0.9205436927403892,  -0.3436691237608884, 0.9816586555095368,
      0.5223543039356411,  0.5821286632547047,  -0.3217616255243991,
      -0.7812174287544036, 0.1712740738714282,  -0.8936625178432633,
      -0.9040266347329282, 0.8460719903255713,  0.4230470486075371,
      -0.8135335871046103};
  const Vec v1 = {22, &d_v1[0], 0, 22};
  double d_v2[22] = {
      -0.0663650098526083, 0.2614198929742506,  0.3584440492207250,
      0.5785569007218740,  -0.1135284205634981, -0.2138993589123694,
      0.1672967864378012,  0.7521408177719375,  0.5433223560475273,
      0.6316993241020221,  -0.3746708837034223, 0.4850631358508011,
      0.1786012876297927,  0.6683305061451530,  -0.8879542330062540,
      0.6126501800768753,  0.1571818828991218,  -0.6955572037991717,
      -0.7907667642946241, 0.6046494427063891,  0.0600320369420078,
      0.5318338008214787};
  const Vec v2 = {22, &d_v2[0], 0, 22};
  double d_v_out[22] = {
      0.7563735465959058,  -0.4781025611987781, -0.4075890075509532,
      0.4243980691064990,  0.3936877166097914,  -0.6456417315571990,
      0.3418805052086482,  -0.3798358303889398, 0.0122513739897905,
      0.3671044083849544,  -0.1097338837743340, 0.3975321478835470,
      -0.9066898051054764, -0.9703047343019173, 0.5755408532829160,
      -0.6508004736545565, 0.5152061761405082,  0.3519446222735438,
      -0.6399053255303746, 0.7064429672322929,  0.8757170426255039,
      0.0272047321554774};
  Vec v_out = {22, &d_v_out[0], 0, 22};
  const Vec *v_out_ptr =
      VecLinComb3(0.9282993244087845, &v0, 0.4335383585781956, &v1,
                  -0.6801010529545481, &v2, &v_out);

  EXPECT_NEAR(-0.4048551883938349, v_out.d[0], 1e-9);
  EXPECT_NEAR(-0.6380758453272297, v_out.d[1], 1e-9);
  EXPECT_NEAR(-0.5838670842597008, v_out.d[2], 1e-9);
  EXPECT_NEAR(0.1485626693524492, v_out.d[3], 1e-9);
  EXPECT_NEAR(-0.3815146603096344, v_out.d[4], 1e-9);
  EXPECT_NEAR(0.6283497298362178, v_out.d[5], 1e-9);
  EXPECT_NEAR(-0.6044079013844088, v_out.d[6], 1e-9);
  EXPECT_NEAR(-0.7519267225099995, v_out.d[7], 1e-9);
  EXPECT_NEAR(0.8184203898561330, v_out.d[8], 1e-9);
  EXPECT_NEAR(-0.0284152307929478, v_out.d[9], 1e-9);
  EXPECT_NEAR(0.7422547491789426, v_out.d[10], 1e-9);
  EXPECT_NEAR(0.1550758217624176, v_out.d[11], 1e-9);
  EXPECT_NEAR(0.0774171918764276, v_out.d[12], 1e-9);
  EXPECT_NEAR(0.5859745864509986, v_out.d[13], 1e-9);
  EXPECT_NEAR(0.0067175598220721, v_out.d[14], 1e-9);
  EXPECT_NEAR(-0.9178605238349897, v_out.d[15], 1e-9);
  EXPECT_NEAR(0.4827398361772110, v_out.d[16], 1e-9);
  EXPECT_NEAR(0.8358744024994500, v_out.d[17], 1e-9);
  EXPECT_NEAR(-0.1079450291659753, v_out.d[18], 1e-9);
  EXPECT_NEAR(-0.3696054092670284, v_out.d[19], 1e-9);
  EXPECT_NEAR(-0.2494453566822863, v_out.d[20], 1e-9);
  EXPECT_NEAR(-1.0314938957814341, v_out.d[21], 1e-9);

  EXPECT_NEAR(-0.4048551883938349, v_out_ptr->d[0], 1e-9);
  EXPECT_NEAR(-0.6380758453272297, v_out_ptr->d[1], 1e-9);
  EXPECT_NEAR(-0.5838670842597008, v_out_ptr->d[2], 1e-9);
  EXPECT_NEAR(0.1485626693524492, v_out_ptr->d[3], 1e-9);
  EXPECT_NEAR(-0.3815146603096344, v_out_ptr->d[4], 1e-9);
  EXPECT_NEAR(0.6283497298362178, v_out_ptr->d[5], 1e-9);
  EXPECT_NEAR(-0.6044079013844088, v_out_ptr->d[6], 1e-9);
  EXPECT_NEAR(-0.7519267225099995, v_out_ptr->d[7], 1e-9);
  EXPECT_NEAR(0.8184203898561330, v_out_ptr->d[8], 1e-9);
  EXPECT_NEAR(-0.0284152307929478, v_out_ptr->d[9], 1e-9);
  EXPECT_NEAR(0.7422547491789426, v_out_ptr->d[10], 1e-9);
  EXPECT_NEAR(0.1550758217624176, v_out_ptr->d[11], 1e-9);
  EXPECT_NEAR(0.0774171918764276, v_out_ptr->d[12], 1e-9);
  EXPECT_NEAR(0.5859745864509986, v_out_ptr->d[13], 1e-9);
  EXPECT_NEAR(0.0067175598220721, v_out_ptr->d[14], 1e-9);
  EXPECT_NEAR(-0.9178605238349897, v_out_ptr->d[15], 1e-9);
  EXPECT_NEAR(0.4827398361772110, v_out_ptr->d[16], 1e-9);
  EXPECT_NEAR(0.8358744024994500, v_out_ptr->d[17], 1e-9);
  EXPECT_NEAR(-0.1079450291659753, v_out_ptr->d[18], 1e-9);
  EXPECT_NEAR(-0.3696054092670284, v_out_ptr->d[19], 1e-9);
  EXPECT_NEAR(-0.2494453566822863, v_out_ptr->d[20], 1e-9);
  EXPECT_NEAR(-1.0314938957814341, v_out_ptr->d[21], 1e-9);
}

TEST(VecDot, Normal15) {
  double d_v0[22] = {
      0.7400512573628752,  -0.7853999281016970, -0.1940100443911528,
      0.5533933442139358,  -0.1795310725290018, -0.5387695786818494,
      0.6731649668446564,  -0.1716828793083678, -0.7018035742966597,
      0.9145398146353334,  0.0777236774389458,  0.3508857621372496,
      -0.8326958888959093, -0.3446889435927782, -0.9003709132749902,
      0.7367434712602596,  -0.8264489845692693, 0.6462531751041236,
      0.5221425633923047,  0.1110078256121201,  -0.0800712926890039,
      -0.6797042130099247};
  const Vec v0 = {22, &d_v0[0], 0, 22};
  double d_v1[22] = {
      -0.5956061471932561, -0.7153707727687342, 0.2328441507806842,
      -0.4285429053969159, 0.1611030600122221,  -0.6024432189509066,
      0.5402151418571870,  -0.3866625318664549, -0.3941778702452623,
      -0.1584994889974736, -0.3914605418077330, 0.5133299689757547,
      0.3061043086396489,  0.9118908804603452,  -0.3250245892281518,
      0.6496034105506523,  -0.5116969616402889, 0.5179413488149018,
      -0.1003561852156585, 0.4991968961297335,  -0.5532220799983762,
      -0.5224124149797413};
  const Vec v1 = {22, &d_v1[0], 0, 22};
  double v_dot = VecDot(&v0, &v1);

  EXPECT_NEAR(2.2078478506613592, v_dot, 1e-9);
}

TEST(VecMult, Normal15) {
  double d_v0[22] = {
      -0.6696322035700115, -0.0279154495633565, 0.3752331228729473,
      -0.6390529993433420, -0.3093244101454551, -0.6744471991284111,
      -0.8498345534454641, -0.4407937717634929, -0.8172951269039859,
      0.3100232911507426,  -0.4041583543010303, 0.6455645919901083,
      0.5487622058744117,  -0.6528400704287443, -0.1593080246449170,
      0.4721124030205908,  -0.7716849868580362, 0.9868163761948328,
      0.7305316012530294,  0.5777334545402841,  0.8585439421100602,
      0.4559680661590300};
  const Vec v0 = {22, &d_v0[0], 0, 22};
  double d_v1[22] = {
      -0.2573753980960685, -0.1210237282194104, 0.5662547124767736,
      0.3519596525127644,  0.2490610728929352,  -0.8052155621564994,
      -0.6251873577519176, 0.8478130313338261,  0.4361921833501166,
      -0.3774404570623640, 0.7181337319469989,  -0.5558951375949188,
      0.6778314776436474,  0.2168820686909800,  0.6480243167843258,
      -0.5025007107663761, 0.4704890936268953,  -0.3363525503389457,
      0.4354432849263068,  0.0933895493697638,  0.7204733460248094,
      -0.8323469304983191};
  const Vec v1 = {22, &d_v1[0], 0, 22};
  double d_v_out[22] = {
      0.2810761065232004,  0.6098839563658058,  0.3576434717437438,
      0.0271398783852510,  -0.5908885018299384, -0.6540500177021080,
      -0.2851614358171415, 0.5699893555130982,  0.2953233114531664,
      0.2060701620452472,  0.6646922258417218,  0.6144938188021951,
      0.1194922121365560,  -0.4156314177753531, -0.9371834050722581,
      0.3382445687247444,  -0.4132847724362496, 0.3964416891549747,
      -0.2269501409962078, -0.1045326259568267, -0.1610903692006604,
      -0.9393971793205376};
  Vec v_out = {22, &d_v_out[0], 0, 22};
  const Vec *v_out_ptr = VecMult(&v0, &v1, &v_out);

  EXPECT_NEAR(0.1723468549717793, v_out.d[0], 1e-9);
  EXPECT_NEAR(0.0033784317810783, v_out.d[1], 1e-9);
  EXPECT_NEAR(0.2124775241041826, v_out.d[2], 1e-9);
  EXPECT_NEAR(-0.2249208715861225, v_out.d[3], 1e-9);
  EXPECT_NEAR(-0.0770406694628014, v_out.d[4], 1e-9);
  EXPECT_NEAR(0.5430753805910600, v_out.d[5], 1e-9);
  EXPECT_NEAR(0.5313058189948505, v_out.d[6], 1e-9);
  EXPECT_NEAR(-0.3737107038318776, v_out.d[7], 1e-9);
  EXPECT_NEAR(-0.3564977458456602, v_out.d[8], 1e-9);
  EXPECT_NEAR(-0.1170153327119146, v_out.d[9], 1e-9);
  EXPECT_NEAR(-0.2902397472717563, v_out.d[10], 1e-9);
  EXPECT_NEAR(-0.3588662176907489, v_out.d[11], 1e-9);
  EXPECT_NEAR(0.3719682968828399, v_out.d[12], 1e-9);
  EXPECT_NEAR(-0.1415893049989511, v_out.d[13], 1e-9);
  EXPECT_NEAR(-0.1032354738287829, v_out.d[14], 1e-9);
  EXPECT_NEAR(-0.2372368180794687, v_out.d[15], 1e-9);
  EXPECT_NEAR(-0.3630693700323200, v_out.d[16], 1e-9);
  EXPECT_NEAR(-0.3319182048493685, v_out.d[17], 1e-9);
  EXPECT_NEAR(0.3181050801920940, v_out.d[18], 1e-9);
  EXPECT_NEAR(0.0539542669753541, v_out.d[19], 1e-9);
  EXPECT_NEAR(0.6185580266813654, v_out.d[20], 1e-9);
  EXPECT_NEAR(-0.3795236202727231, v_out.d[21], 1e-9);

  EXPECT_NEAR(0.1723468549717793, v_out_ptr->d[0], 1e-9);
  EXPECT_NEAR(0.0033784317810783, v_out_ptr->d[1], 1e-9);
  EXPECT_NEAR(0.2124775241041826, v_out_ptr->d[2], 1e-9);
  EXPECT_NEAR(-0.2249208715861225, v_out_ptr->d[3], 1e-9);
  EXPECT_NEAR(-0.0770406694628014, v_out_ptr->d[4], 1e-9);
  EXPECT_NEAR(0.5430753805910600, v_out_ptr->d[5], 1e-9);
  EXPECT_NEAR(0.5313058189948505, v_out_ptr->d[6], 1e-9);
  EXPECT_NEAR(-0.3737107038318776, v_out_ptr->d[7], 1e-9);
  EXPECT_NEAR(-0.3564977458456602, v_out_ptr->d[8], 1e-9);
  EXPECT_NEAR(-0.1170153327119146, v_out_ptr->d[9], 1e-9);
  EXPECT_NEAR(-0.2902397472717563, v_out_ptr->d[10], 1e-9);
  EXPECT_NEAR(-0.3588662176907489, v_out_ptr->d[11], 1e-9);
  EXPECT_NEAR(0.3719682968828399, v_out_ptr->d[12], 1e-9);
  EXPECT_NEAR(-0.1415893049989511, v_out_ptr->d[13], 1e-9);
  EXPECT_NEAR(-0.1032354738287829, v_out_ptr->d[14], 1e-9);
  EXPECT_NEAR(-0.2372368180794687, v_out_ptr->d[15], 1e-9);
  EXPECT_NEAR(-0.3630693700323200, v_out_ptr->d[16], 1e-9);
  EXPECT_NEAR(-0.3319182048493685, v_out_ptr->d[17], 1e-9);
  EXPECT_NEAR(0.3181050801920940, v_out_ptr->d[18], 1e-9);
  EXPECT_NEAR(0.0539542669753541, v_out_ptr->d[19], 1e-9);
  EXPECT_NEAR(0.6185580266813654, v_out_ptr->d[20], 1e-9);
  EXPECT_NEAR(-0.3795236202727231, v_out_ptr->d[21], 1e-9);
}

TEST(MatArrMult, Normal3) {
  double m1[8][7] = {
      {0.2842478051712545, 0.1285756565220659, 0.8513865928955024,
       0.2919214445618242, 0.1229829770095746, 0.6357954619654590,
       0.8830518531611322},
      {0.0345598339602303, 0.3771543907250314, 0.9399481293095983,
       0.7440707745774174, 0.2259388093172139, 0.2021259322871856,
       0.1860175550609836},
      {0.6647673908104158, 0.4898168983662253, 0.2856710298344712,
       0.6494313502845658, 0.7570027875941560, 0.3800394381530703,
       0.4447101026818499},
      {0.1009070329751124, 0.4689033604510742, 0.9623587022046306,
       0.1868598629188989, 0.0457481629735539, 0.0933327074511215,
       0.1476023765267982},
      {0.3159316660450661, 0.0836005199290611, 0.3485664607214924,
       0.4238519429101498, 0.0542012906083822, 0.9466071668853572,
       0.8704774410197788},
      {0.6918261255994796, 0.9309489576374744, 0.6792090510860233,
       0.4898358861287512, 0.2310456266414976, 0.5659783200382967,
       0.8158520582767106},
      {0.3788365873677527, 0.4188763975969613, 0.4927361941620255,
       0.1921160105588332, 0.8660510707325392, 0.1771995466495460,
       0.5278398655216106},
      {0.9392994003994011, 0.9678468078391143, 0.2214989485371839,
       0.1989387196439757, 0.1803714016267677, 0.2354439359146351,
       0.1707756995958296}};
  double m2[7][11] = {
      {0.5766635932725547, 0.9266583646688454, 0.2442438442741369,
       0.5949013633190992, 0.0675910114840355, 0.7825839559368662,
       0.9287795832599065, 0.9588325661282913, 0.9897130330482041,
       0.5288534477358705, 0.9960887665735689},
      {0.1948737036608182, 0.5268134770530257, 0.6171577305787550,
       0.8855935284464500, 0.2669991264022263, 0.3279671757019125,
       0.7114036208505654, 0.6373664123948032, 0.8358376613545312,
       0.0615292292091697, 0.7572197591812680},
      {0.1395325473799570, 0.7747062566694967, 0.3097175235613369,
       0.5588934372605058, 0.4228694445888671, 0.8240666504695006,
       0.3256019916326228, 0.8330954412174850, 0.0581942492177804,
       0.7549974969331740, 0.4176000189524620},
      {0.3182110273605270, 0.2922296121510032, 0.0731241230403998,
       0.7093924363750482, 0.3585790935282722, 0.3722412972269666,
       0.5179022734949397, 0.2052033950489565, 0.2256362733882332,
       0.6453160923268135, 0.5280048376901000},
      {0.9278590702085613, 0.1571780223810385, 0.9565867903943631,
       0.4571577224912426, 0.0125272156679402, 0.1966185378892418,
       0.4358209538631685, 0.0958559806701452, 0.2074604154907804,
       0.5164466150690934, 0.0949981343774424},
      {0.5526936857944076, 0.4960607181969946, 0.1342403438488263,
       0.1975462932186237, 0.0355268522293164, 0.0425038933048613,
       0.1771582491747354, 0.0752290145308516, 0.3767956048954628,
       0.5358647922224448, 0.3738555069239931},
      {0.2371730814191283, 0.5896218540680498, 0.9869741997019319,
       0.0933670495667999, 0.4425063454884024, 0.3573712637785377,
       0.1348561753890962, 0.1396120092744022, 0.7699882176440331,
       0.3553834849290232, 0.2958248596936508}};
  double m_out[8][11];
  double m_out2[8][11];
  Mat mn_m1 = {8, 7, &m1[0][0], 0, 0};
  Mat mn_m2 = {7, 11, &m2[0][0], 0, 0};
  Mat mn_m_out = {8, 11, &m_out2[0][0], 0, 0};
  MatArrMult(&m1[0][0], 8, 7, &m2[0][0], 11, &m_out[0][0]);
  MatMult(&mn_m1, &mn_m2, &mn_m_out);

  EXPECT_NEAR(m_out[0][0], 1.0756072750431376, 1e-9);
  EXPECT_NEAR(m_out2[0][0], 1.0756072750431376, 1e-9);
  EXPECT_NEAR(m_out[0][1], 1.9314086513619695, 1e-9);
  EXPECT_NEAR(m_out2[0][1], 1.9314086513619695, 1e-9);
  EXPECT_NEAR(m_out[0][2], 1.5083557725750496, 1e-9);
  EXPECT_NEAR(m_out2[0][2], 1.5083557725750496, 1e-9);
  EXPECT_NEAR(m_out[0][3], 1.2301560208527893, 1e-9);
  EXPECT_NEAR(m_out2[0][3], 1.2301560208527893, 1e-9);
  EXPECT_NEAR(m_out[0][4], 0.9331289813930932, 1e-9);
  EXPECT_NEAR(m_out2[0][4], 0.9331289813930932, 1e-9);
  EXPECT_NEAR(m_out[0][5], 1.4416627541951106, 1e-9);
  EXPECT_NEAR(m_out2[0][5], 1.4416627541951106, 1e-9);
  EXPECT_NEAR(m_out[0][6], 1.0691926605609736, 1e-9);
  EXPECT_NEAR(m_out2[0][6], 1.0691926605609736, 1e-9);
  EXPECT_NEAR(m_out[0][7], 1.3065889815612219, 1e-9);
  EXPECT_NEAR(m_out2[0][7], 1.3065889815612219, 1e-9);
  EXPECT_NEAR(m_out[0][8], 1.4492245615811075, 1e-9);
  EXPECT_NEAR(m_out2[0][8], 1.4492245615811075, 1e-9);
  EXPECT_NEAR(m_out[0][9], 1.7074495355058841, 1e-9);
  EXPECT_NEAR(m_out2[0][9], 1.7074495355058841, 1e-9);
  EXPECT_NEAR(m_out[0][10], 1.4007785442784253, 1e-9);
  EXPECT_NEAR(m_out2[0][10], 1.4007785442784253, 1e-9);
  EXPECT_NEAR(m_out[1][0], 0.8268232102811529, 1e-9);
  EXPECT_NEAR(m_out2[1][0], 0.8268232102811529, 1e-9);
  EXPECT_NEAR(m_out[1][1], 1.4217977518415603, 1e-9);
  EXPECT_NEAR(m_out2[1][1], 1.4217977518415603, 1e-9);
  EXPECT_NEAR(m_out[1][2], 1.0135907669365389, 1e-9);
  EXPECT_NEAR(m_out2[1][2], 1.0135907669365389, 1e-9);
  EXPECT_NEAR(m_out[1][3], 1.5683210108998482, 1e-9);
  EXPECT_NEAR(m_out2[1][3], 1.5683210108998482, 1e-9);
  EXPECT_NEAR(m_out[1][4], 0.8596446250356613, 1e-9);
  EXPECT_NEAR(m_out2[1][4], 0.8596446250356613, 1e-9);
  EXPECT_NEAR(m_out[1][5], 1.3217862349369922, 1e-9);
  EXPECT_NEAR(m_out2[1][5], 1.3217862349369922, 1e-9);
  EXPECT_NEAR(m_out[1][6], 1.1511751557928396, 1e-9);
  EXPECT_NEAR(m_out2[1][6], 1.1511751557928396, 1e-9);
  EXPECT_NEAR(m_out[1][7], 1.2721085912831556, 1e-9);
  EXPECT_NEAR(m_out2[1][7], 1.2721085912831556, 1e-9);
  EXPECT_NEAR(m_out[1][8], 0.8382979422547575, 1e-9);
  EXPECT_NEAR(m_out2[1][8], 0.8382979422547575, 1e-9);
  EXPECT_NEAR(m_out[1][9], 1.5224075068058684, 1e-9);
  EXPECT_NEAR(m_out2[1][9], 1.5224075068058684, 1e-9);
  EXPECT_NEAR(m_out[1][10], 1.2574670198488929, 1e-9);
  EXPECT_NEAR(m_out2[1][10], 1.2574670198488929, 1e-9);
  EXPECT_NEAR(m_out[2][0], 1.7432267749088115, 1e-9);
  EXPECT_NEAR(m_out2[2][0], 1.7432267749088115, 1e-9);
  EXPECT_NEAR(m_out[2][1], 1.8548662453659812, 1e-9);
  EXPECT_NEAR(m_out2[2][1], 1.8548662453659812, 1e-9);
  EXPECT_NEAR(m_out[2][2], 1.8146989398043438, 1e-9);
  EXPECT_NEAR(m_out2[2][2], 1.8146989398043438, 1e-9);
  EXPECT_NEAR(m_out[2][3], 1.9122773768067265, 1e-9);
  EXPECT_NEAR(m_out2[2][3], 1.9122773768067265, 1e-9);
  EXPECT_NEAR(m_out[2][4], 0.7491588234039414, 1e-9);
  EXPECT_NEAR(m_out2[2][4], 0.7491588234039414, 1e-9);
  EXPECT_NEAR(m_out[2][5], 1.4819578446443751, 1e-9);
  EXPECT_NEAR(m_out2[2][5], 1.4819578446443751, 1e-9);
  EXPECT_NEAR(m_out[2][6], 1.8524536263617879, 1e-9);
  EXPECT_NEAR(m_out2[2][6], 1.8524536263617879, 1e-9);
  EXPECT_NEAR(m_out[2][7], 1.4840903209938381, 1e-9);
  EXPECT_NEAR(m_out2[2][7], 1.4840903209938381, 1e-9);
  EXPECT_NEAR(m_out[2][8], 1.8731628844143313, 1e-9);
  EXPECT_NEAR(m_out2[2][8], 1.8731628844143313, 1e-9);
  EXPECT_NEAR(m_out[2][9], 1.7691159043413962, 1e-9);
  EXPECT_NEAR(m_out2[2][9], 1.7691159043413962, 1e-9);
  EXPECT_NEAR(m_out[2][10], 1.8408154794373757, 1e-9);
  EXPECT_NEAR(m_out2[2][10], 1.8408154794373757, 1e-9);
  EXPECT_NEAR(m_out[3][0], 0.4723471334081266, 1e-9);
  EXPECT_NEAR(m_out2[3][0], 0.4723471334081266, 1e-9);
  EXPECT_NEAR(m_out[3][1], 1.2812011314928282, 1e-9);
  EXPECT_NEAR(m_out2[3][1], 1.2812011314928282, 1e-9);
  EXPECT_NEAR(m_out[3][2], 0.8277274136493200, 1e-9);
  EXPECT_NEAR(m_out2[3][2], 0.8277274136493200, 1e-9);
  EXPECT_NEAR(m_out[3][3], 1.1988333041286250, 1e-9);
  EXPECT_NEAR(m_out2[3][3], 1.1988333041286250, 1e-9);
  EXPECT_NEAR(m_out[3][4], 0.6751772288229932, 1e-9);
  EXPECT_NEAR(m_out2[3][4], 0.6751772288229932, 1e-9);
  EXPECT_NEAR(m_out[3][5], 1.1610685940907521, 1e-9);
  EXPECT_NEAR(m_out2[3][5], 1.1610685940907521, 1e-9);
  EXPECT_NEAR(m_out[3][6], 0.8937987574661206, 1e-9);
  EXPECT_NEAR(m_out2[3][6], 0.8937987574661206, 1e-9);
  EXPECT_NEAR(m_out[3][7], 1.2677107548617961, 1e-9);
  EXPECT_NEAR(m_out2[3][7], 1.2677107548617961, 1e-9);
  EXPECT_NEAR(m_out[3][8], 0.7482725768111556, 1e-9);
  EXPECT_NEAR(m_out2[3][8], 0.7482725768111556, 1e-9);
  EXPECT_NEAR(m_out[3][9], 1.0554740252528052, 1e-9);
  EXPECT_NEAR(m_out2[3][9], 1.0554740252528052, 1e-9);
  EXPECT_NEAR(m_out[3][10], 1.0390225646827358, 1e-9);
  EXPECT_NEAR(m_out2[3][10], 1.0390225646827358, 1e-9);
  EXPECT_NEAR(m_out[4][0], 1.1619173412803243, 1e-9);
  EXPECT_NEAR(m_out2[4][0], 1.1619173412803243, 1e-9);
  EXPECT_NEAR(m_out[4][1], 1.7220477138862504, 1e-9);
  EXPECT_NEAR(m_out2[4][1], 1.7220477138862504, 1e-9);
  EXPECT_NEAR(m_out[4][2], 1.3057699003327181, 1e-9);
  EXPECT_NEAR(m_out2[4][2], 1.3057699003327181, 1e-9);
  EXPECT_NEAR(m_out[4][3], 1.0505243139649862, 1e-9);
  EXPECT_NEAR(m_out2[4][3], 1.0505243139649862, 1e-9);
  EXPECT_NEAR(m_out[4][4], 0.7625587132320155, 1e-9);
  EXPECT_NEAR(m_out2[4][4], 0.7625587132320155, 1e-9);
  EXPECT_NEAR(m_out[4][5], 1.0816535639634197, 1e-9);
  EXPECT_NEAR(m_out2[4][5], 1.0816535639634197, 1e-9);
  EXPECT_NEAR(m_out[4][6], 0.9946229973697760, 1e-9);
  EXPECT_NEAR(m_out2[4][6], 0.9946229973697760, 1e-9);
  EXPECT_NEAR(m_out[4][7], 0.9315116673546433, 1e-9);
  EXPECT_NEAR(m_out2[4][7], 0.9315116673546433, 1e-9);
  EXPECT_NEAR(m_out[4][8], 1.5366585024786168, 1e-9);
  EXPECT_NEAR(m_out2[4][8], 1.5366585024786168, 1e-9);
  EXPECT_NEAR(m_out[4][9], 1.5535095436811610, 1e-9);
  EXPECT_NEAR(m_out2[4][9], 1.5535095436811610, 1e-9);
  EXPECT_NEAR(m_out[4][10], 1.3639093766234902, 1e-9);
  EXPECT_NEAR(m_out2[4][10], 1.3639093766234902, 1e-9);
  EXPECT_NEAR(m_out[5][0], 1.5516999312033120, 1e-9);
  EXPECT_NEAR(m_out2[5][0], 1.5516999312033120, 1e-9);
  EXPECT_NEAR(m_out[5][1], 2.5989740858427997, 1e-9);
  EXPECT_NEAR(m_out2[5][1], 2.5989740858427997, 1e-9);
  EXPECT_NEAR(m_out[5][2], 2.0919156343649878, 1e-9);
  EXPECT_NEAR(m_out2[5][2], 2.0919156343649878, 1e-9);
  EXPECT_NEAR(m_out[5][3], 2.2567069425582917, 1e-9);
  EXPECT_NEAR(m_out2[5][3], 2.2567069425582917, 1e-9);
  EXPECT_NEAR(m_out[5][4], 1.1422069475350507, 1e-9);
  EXPECT_NEAR(m_out2[5][4], 1.1422069475350507, 1e-9);
  EXPECT_NEAR(m_out[5][5], 1.9498296164754287, 1e-9);
  EXPECT_NEAR(m_out2[5][5], 1.9498296164754287, 1e-9);
  EXPECT_NEAR(m_out[5][6], 2.0906583206471341, 1e-9);
  EXPECT_NEAR(m_out2[5][6], 2.0906583206471341, 1e-9);
  EXPECT_NEAR(m_out[5][7], 2.1016908090293205, 1e-9);
  EXPECT_NEAR(m_out2[5][7], 2.1016908090293205, 1e-9);
  EXPECT_NEAR(m_out[5][8], 2.5022697747796014, 1e-9);
  EXPECT_NEAR(m_out2[5][8], 2.5022697747796014, 1e-9);
  EXPECT_NEAR(m_out[5][9], 1.9646062512680427, 1e-9);
  EXPECT_NEAR(m_out2[5][9], 1.9646062512680427, 1e-9);
  EXPECT_NEAR(m_out[5][10], 2.4112189437038634, 1e-9);
  EXPECT_NEAR(m_out2[5][10], 2.4112189437038634, 1e-9);
  EXPECT_NEAR(m_out[6][0], 1.4566752513639634, 1e-9);
  EXPECT_NEAR(m_out2[6][0], 1.4566752513639634, 1e-9);
  EXPECT_NEAR(m_out[6][1], 1.5448414728726307, 1e-9);
  EXPECT_NEAR(m_out2[6][1], 1.5448414728726307, 1e-9);
  EXPECT_NEAR(m_out[6][2], 1.8909033309890746, 1e-9);
  EXPECT_NEAR(m_out2[6][2], 1.8909033309890746, 1e-9);
  EXPECT_NEAR(m_out[6][3], 1.4882071987912389, 1e-9);
  EXPECT_NEAR(m_out2[6][3], 1.4882071987912389, 1e-9);
  EXPECT_NEAR(m_out[6][4], 0.6654144865753241, 1e-9);
  EXPECT_NEAR(m_out2[6][4], 0.6654144865753241, 1e-9);
  EXPECT_NEAR(m_out[6][5], 1.2778582880640821, 1e-9);
  EXPECT_NEAR(m_out2[6][5], 1.2778582880640821, 1e-9);
  EXPECT_NEAR(m_out[6][6], 1.3897971091521810, 1e-9);
  EXPECT_NEAR(m_out2[6][6], 1.3897971091521810, 1e-9);
  EXPECT_NEAR(m_out[6][7], 1.2501772448373702, 1e-9);
  EXPECT_NEAR(m_out2[6][7], 1.2501772448373702, 1e-9);
  EXPECT_NEAR(m_out[6][8], 1.4499447326316959, 1e-9);
  EXPECT_NEAR(m_out2[6][8], 1.4499447326316959, 1e-9);
  EXPECT_NEAR(m_out[6][9], 1.4519220367815238, 1e-9);
  EXPECT_NEAR(m_out2[6][9], 1.4519220367815238, 1e-9);
  EXPECT_NEAR(m_out[6][10], 1.3064095974457763, 1e-9);
  EXPECT_NEAR(m_out2[6][10], 1.3064095974457763, 1e-9);
  EXPECT_NEAR(m_out[7][0], 1.1624694829529356, 1e-9);
  EXPECT_NEAR(m_out2[7][0], 1.1624694829529356, 1e-9);
  EXPECT_NEAR(m_out[7][1], 1.8558547873339730, 1e-9);
  EXPECT_NEAR(m_out2[7][1], 1.8558547873339730, 1e-9);
  EXPECT_NEAR(m_out[7][2], 1.2825797456894594, 1e-9);
  EXPECT_NEAR(m_out2[7][2], 1.2825797456894594, 1e-9);
  EXPECT_NEAR(m_out[7][3], 1.8257433743073523, 1e-9);
  EXPECT_NEAR(m_out2[7][3], 1.8257433743073523, 1e-9);
  EXPECT_NEAR(m_out[7][4], 0.5730963159405302, 1e-9);
  EXPECT_NEAR(m_out2[7][4], 0.5730963159405302, 1e-9);
  EXPECT_NEAR(m_out[7][5], 1.4155877011088005, 1e-9);
  EXPECT_NEAR(m_out2[7][5], 1.4155877011088005, 1e-9);
  EXPECT_NEAR(m_out[7][6], 1.8794337726316201, 1e-9);
  EXPECT_NEAR(m_out2[7][6], 1.8794337726316201, 1e-9);
  EXPECT_NEAR(m_out[7][7], 1.8017007984660531, 1e-9);
  EXPECT_NEAR(m_out2[7][7], 1.8017007984660531, 1e-9);
  EXPECT_NEAR(m_out[7][8], 2.0540068699938594, 1e-9);
  EXPECT_NEAR(m_out2[7][8], 2.0540068699938594, 1e-9);
  EXPECT_NEAR(m_out[7][9], 1.1319212822163094, 1e-9);
  EXPECT_NEAR(m_out2[7][9], 1.1319212822163094, 1e-9);
  EXPECT_NEAR(m_out[7][10], 2.0217135354615525, 1e-9);
  EXPECT_NEAR(m_out2[7][10], 2.0217135354615525, 1e-9);
}

#if !defined(NDEBUG)

TEST(MatArrMultDeath, Death3) {
  double m1[8][8] = {
      {0.3442305217684726, 0.6582318509182781, 0.4667130725324032,
       0.9879404648607374, 0.2689083467307019, 0.8365024323989426,
       0.3558693284386506, 0.0071355066320992},
      {0.3572568411052924, 0.3556352998859752, 0.2008286503209874,
       0.0834707618512553, 0.8543196737280391, 0.2819653455991868,
       0.5041159697262222, 0.5615870148798574},
      {0.6497704710593961, 0.4793801088415761, 0.8512751805797752,
       0.7681629431776491, 0.4455389858581106, 0.4469762082763745,
       0.2468722575705651, 0.3163133685245259},
      {0.8807920424356488, 0.5405097344253571, 0.0797633711742393,
       0.9538863563003517, 0.5645742775330655, 0.3906028024365358,
       0.1384969293752344, 0.2435359535186730},
      {0.7491608135284877, 0.2475594451279733, 0.8979408296554364,
       0.6285398396200602, 0.7864191663595468, 0.6602500559986093,
       0.0464742878238773, 0.7621749029708457},
      {0.5386977210026151, 0.2180587825221414, 0.3605964710482378,
       0.6055821664077672, 0.8626730010272183, 0.0705917134609355,
       0.1396112056038904, 0.7056056901144712},
      {0.6615902447642156, 0.4184269931619944, 0.3736886795816831,
       0.6007568593733061, 0.0104536281061265, 0.8008541042477475,
       0.1955590195866422, 0.5110842535124128},
      {0.1113723333459078, 0.0352687130791325, 0.3329969820354548,
       0.5500236389415184, 0.3237387568733607, 0.1310497676428670,
       0.8241480869729567, 0.2327148717941966}};
  double m_out[8][8] = {
      {0.2221775575388063, 0.5575082109322789, 0.1572150670382323,
       0.3125655825508382, 0.2512909577578974, 0.9076540670968775,
       0.3934275964980046, 0.2760929067211479},
      {0.3967836552629519, 0.7155642625847284, 0.7032106105947472,
       0.4150704255957317, 0.7370462069825983, 0.8237703483160601,
       0.3625850434399447, 0.9148682666075495},
      {0.0574944060467841, 0.3388650329059866, 0.4108565814636432,
       0.6205643022865170, 0.3315196999110331, 0.8706660741953934,
       0.6855949673941997, 0.4878215565253218},
      {0.5618864596516706, 0.7852638002075247, 0.2923860130471660,
       0.7894114173777947, 0.2992009828650521, 0.2090770600423196,
       0.5725428536085545, 0.5511049799785475},
      {0.7163952963446084, 0.8460782239407800, 0.6468942211931835,
       0.3096773018564989, 0.0297973481817638, 0.5900525555746490,
       0.3022967709745110, 0.8014713600245352},
      {0.8488683415907422, 0.2636909420065198, 0.6296742435415252,
       0.1674825270027096, 0.6665721980158658, 0.3296231954336915,
       0.0055163904716741, 0.4387721192250744},
      {0.9222916303286662, 0.0496220837816407, 0.6654856253647120,
       0.8999152267419662, 0.0613315815802591, 0.5987168819191627,
       0.6266587641596482, 0.9446061761760552},
      {0.4256080130211634, 0.4886015065565402, 0.3014782590414677,
       0.1792105105671199, 0.7889669425815292, 0.1642904159771178,
       0.8922040534955183, 0.4083132550480562}};
  ASSERT_DEATH(MatArrMult(&m1[0][0], 8, 8, &m_out[0][0], 8, &m_out[0][0]),
               "MatArrMult: Assertion `m_out != m1 && m_out != m2' failed.");
  ASSERT_DEATH(MatArrMult(&m_out[0][0], 8, 8, &m1[0][0], 8, &m_out[0][0]),
               "MatArrMult: Assertion `m_out != m1 && m_out != m2' failed.");
}

#endif  // !defined(NDEBUG)

TEST(MatArrZero, Normal3) {
  double m[6][7] = {{0.1028802661152852, 0.8221269796801330, 0.6935907784906943,
                     0.3382645125050293, 0.0803414394710327, 0.9372776574811984,
                     0.3631807715570381},
                    {0.8332979201642144, 0.9559901931841136, 0.7482120740550113,
                     0.9178730834947952, 0.4508314212616469, 0.2196992486735422,
                     0.9424242662574658},
                    {0.4014437403272914, 0.9570246015837003, 0.7793417907772615,
                     0.1050602435235298, 0.0845160267251873, 0.9206520445936406,
                     0.4845371478353383},
                    {0.1683841063154956, 0.0981651942590270, 0.0470440331379551,
                     0.8911525805114389, 0.2152131880538066, 0.7269812450057004,
                     0.8953030995049042},
                    {0.0165485946793396, 0.8319168315385496, 0.3097019490227330,
                     0.6586766227719184, 0.3097684369838336, 0.4588842489045346,
                     0.2623767242133037},
                    {0.4429332129555932, 0.6686436637314873, 0.0242444304333582,
                     0.6922395959312783, 0.8286028676396249, 0.5811131448761285,
                     0.7383154780661552}};
  MatArrZero(6, 7, &m[0][0]);

  EXPECT_NEAR(m[0][0], 0.0, 1e-9);
  EXPECT_NEAR(m[0][1], 0.0, 1e-9);
  EXPECT_NEAR(m[0][2], 0.0, 1e-9);
  EXPECT_NEAR(m[0][3], 0.0, 1e-9);
  EXPECT_NEAR(m[0][4], 0.0, 1e-9);
  EXPECT_NEAR(m[0][5], 0.0, 1e-9);
  EXPECT_NEAR(m[0][6], 0.0, 1e-9);
  EXPECT_NEAR(m[1][0], 0.0, 1e-9);
  EXPECT_NEAR(m[1][1], 0.0, 1e-9);
  EXPECT_NEAR(m[1][2], 0.0, 1e-9);
  EXPECT_NEAR(m[1][3], 0.0, 1e-9);
  EXPECT_NEAR(m[1][4], 0.0, 1e-9);
  EXPECT_NEAR(m[1][5], 0.0, 1e-9);
  EXPECT_NEAR(m[1][6], 0.0, 1e-9);
  EXPECT_NEAR(m[2][0], 0.0, 1e-9);
  EXPECT_NEAR(m[2][1], 0.0, 1e-9);
  EXPECT_NEAR(m[2][2], 0.0, 1e-9);
  EXPECT_NEAR(m[2][3], 0.0, 1e-9);
  EXPECT_NEAR(m[2][4], 0.0, 1e-9);
  EXPECT_NEAR(m[2][5], 0.0, 1e-9);
  EXPECT_NEAR(m[2][6], 0.0, 1e-9);
  EXPECT_NEAR(m[3][0], 0.0, 1e-9);
  EXPECT_NEAR(m[3][1], 0.0, 1e-9);
  EXPECT_NEAR(m[3][2], 0.0, 1e-9);
  EXPECT_NEAR(m[3][3], 0.0, 1e-9);
  EXPECT_NEAR(m[3][4], 0.0, 1e-9);
  EXPECT_NEAR(m[3][5], 0.0, 1e-9);
  EXPECT_NEAR(m[3][6], 0.0, 1e-9);
  EXPECT_NEAR(m[4][0], 0.0, 1e-9);
  EXPECT_NEAR(m[4][1], 0.0, 1e-9);
  EXPECT_NEAR(m[4][2], 0.0, 1e-9);
  EXPECT_NEAR(m[4][3], 0.0, 1e-9);
  EXPECT_NEAR(m[4][4], 0.0, 1e-9);
  EXPECT_NEAR(m[4][5], 0.0, 1e-9);
  EXPECT_NEAR(m[4][6], 0.0, 1e-9);
  EXPECT_NEAR(m[5][0], 0.0, 1e-9);
  EXPECT_NEAR(m[5][1], 0.0, 1e-9);
  EXPECT_NEAR(m[5][2], 0.0, 1e-9);
  EXPECT_NEAR(m[5][3], 0.0, 1e-9);
  EXPECT_NEAR(m[5][4], 0.0, 1e-9);
  EXPECT_NEAR(m[5][5], 0.0, 1e-9);
  EXPECT_NEAR(m[5][6], 0.0, 1e-9);
}

TEST(MatArrCopy, Normal3) {
  double m[6][3] = {
      {0.5720659645428335, 0.5291476269114611, 0.4649440135354268},
      {0.0808232889639530, 0.6519151056884754, 0.2971638749307413},
      {0.7775746252188722, 0.2479422580351576, 0.9338628346716953},
      {0.8348483446943231, 0.5009322760941521, 0.5812264122258487},
      {0.4813496705727774, 0.6147969938540104, 0.9224350462991738},
      {0.9758211471592840, 0.6605735265256545, 0.6201691275848491}};
  double m_out[6][3];
  MatArrCopy(&m[0][0], 6, 3, &m_out[0][0]);
  MatArrCopy(&m[0][0], 6, 3, &m[0][0]);

  EXPECT_NEAR(m_out[0][0], m[0][0], 1e-9);
  EXPECT_NEAR(m_out[0][1], m[0][1], 1e-9);
  EXPECT_NEAR(m_out[0][2], m[0][2], 1e-9);
  EXPECT_NEAR(m_out[1][0], m[1][0], 1e-9);
  EXPECT_NEAR(m_out[1][1], m[1][1], 1e-9);
  EXPECT_NEAR(m_out[1][2], m[1][2], 1e-9);
  EXPECT_NEAR(m_out[2][0], m[2][0], 1e-9);
  EXPECT_NEAR(m_out[2][1], m[2][1], 1e-9);
  EXPECT_NEAR(m_out[2][2], m[2][2], 1e-9);
  EXPECT_NEAR(m_out[3][0], m[3][0], 1e-9);
  EXPECT_NEAR(m_out[3][1], m[3][1], 1e-9);
  EXPECT_NEAR(m_out[3][2], m[3][2], 1e-9);
  EXPECT_NEAR(m_out[4][0], m[4][0], 1e-9);
  EXPECT_NEAR(m_out[4][1], m[4][1], 1e-9);
  EXPECT_NEAR(m_out[4][2], m[4][2], 1e-9);
  EXPECT_NEAR(m_out[5][0], m[5][0], 1e-9);
  EXPECT_NEAR(m_out[5][1], m[5][1], 1e-9);
  EXPECT_NEAR(m_out[5][2], m[5][2], 1e-9);
}

TEST(MatVecMult, Normal3) {
  double m[10][9] = {
      {0.6163590309509581, 0.1446942908524701, 0.5831671745234773,
       0.8106815983221842, 0.5111636697862727, 0.3045486759670928,
       0.7815109935073308, 0.2744541916588802, 0.8750984244369957},
      {0.4114088500610815, 0.8533069293344172, 0.6519751547944623,
       0.3284995523157346, 0.9474429629254125, 0.9717504351808379,
       0.3363283186795949, 0.8154262130743656, 0.5453132091977562},
      {0.1056138618790298, 0.6608765908133941, 0.0678968017536147,
       0.0385374628422537, 0.3980122725958533, 0.1317124698924356,
       0.3169317785616643, 0.3041971579403936, 0.4823168080497149},
      {0.3966307894736205, 0.4841866341570337, 0.8380529538625326,
       0.7577344410473144, 0.9934381333127507, 0.6012834873056252,
       0.3978582493265602, 0.7921366926535136, 0.4680679887869811},
      {0.0386548103357757, 0.1251608113596940, 0.2705459454417438,
       0.9333933121526743, 0.5530319904035217, 0.7644632803857827,
       0.1863803432713906, 0.1196862118189198, 0.0803407786398536},
      {0.3000747918752669, 0.1592404615240265, 0.3405217125247876,
       0.3831884056276227, 0.3312585191161341, 0.4173179467877656,
       0.2837625666884899, 0.5042434723616324, 0.2976394387657252},
      {0.1272580561933133, 0.3670151769710701, 0.9669708198289023,
       0.8432531522961727, 0.8615338007343247, 0.6648447077341648,
       0.1926278702457741, 0.0291655014016432, 0.7427283586430196},
      {0.1846394904784253, 0.2951518904964938, 0.4293383446650760,
       0.4552369377548596, 0.6889855335765132, 0.1490913891302471,
       0.9189075766153760, 0.8826602536560348, 0.1723191122759795},
      {0.5330571161352753, 0.2432022504953305, 0.5373980733884414,
       0.7727998767622609, 0.6206322218923170, 0.9331103141746826,
       0.0513179244398716, 0.3824814392087466, 0.6816033997413735},
      {0.3272803505634981, 0.0562961002921846, 0.2375984932322128,
       0.7787380507549319, 0.0218346720492720, 0.9433075002202296,
       0.9064297012406272, 0.9618145714967243, 0.5333773315860085}};
  double v[9] = {0.3126200684356959, 0.7010002479834819, 0.5981429874288033,
                 0.1173924808281638, 0.1619257552303964, 0.8436491149658407,
                 0.8323052064261408, 0.8654914840751281, 0.4453750915617919};
  double v_out[10];
  Mat mn_m = {10, 9, &m[0][0], 0, 0};
  Vec n_v = {9, v, 0, 0};
  Vec n_v_out = {10, v_out, 0, 0};
  MatVecMult(&mn_m, &n_v, &n_v_out);

  EXPECT_NEAR(v_out[0], 2.3555454756891403, 1e-9);
  EXPECT_NEAR(v_out[1], 3.3570937625527808, 1e-9);
  EXPECT_NEAR(v_out[2], 1.4588711290694298, 1e-9);
  EXPECT_NEAR(v_out[3], 2.9469658977862618, 1e-9);
  EXPECT_NEAR(v_out[4], 1.4002039472464280, 1e-9);
  EXPECT_NEAR(v_out[5], 1.6649670044814520, 1e-9);
  EXPECT_NEAR(v_out[6], 2.1912000904962095, 1e-9);
  EXPECT_NEAR(v_out[7], 2.4177091270395104, 1e-9);
  EXPECT_NEAR(v_out[8], 2.3143207864896800, 1e-9);
  EXPECT_NEAR(v_out[9], 2.9990914419470260, 1e-9);
}

TEST(VecNorm, Normal19) {
  double d_v[22] = {
      -0.1098964035922598, 0.3633512871658295,  -0.5562441910198082,
      0.4026663673212927,  0.8190949858643917,  0.2359340972206811,
      -0.5810224500812835, 0.7714552156230294,  -0.2697952249204654,
      0.4541185928620124,  0.8015644774870618,  -0.1580407986650341,
      0.0673230093457400,  -0.5984909013197883, 0.8448365443990067,
      0.3554060201298985,  0.2888915252728672,  0.4780182761380305,
      0.8127676943325031,  -0.8834757453604325, -0.8682375455113005,
      0.3102764345096398};
  const Vec v = {22, &d_v[0], 0, 22};
  double v_norm = VecNorm(&v);

  EXPECT_NEAR(2.6509302055124846, v_norm, 1e-9);
}

TEST(VecNormBound, Unbounded19) {
  double d_v[22] = {
      -0.5066580935468326, -0.1188423200377164, -0.4118824846372602,
      -0.6889014445828561, -0.9537771329429319, -0.0330681787707920,
      0.6390219011788116,  0.3263690396100258,  -0.7564575944147134,
      -0.4695550202086516, -0.5328725208617497, 0.3012426066571519,
      0.7850637093133905,  0.9729582942460511,  -0.5211914009335092,
      -0.6410688242611542, 0.3699411559107919,  -0.0133944832750319,
      0.3784461735845122,  -0.3935850028531296, 0.8637585763243276,
      -0.4197810300421883};
  const Vec v = {22, &d_v[0], 0, 22};
  double v_norm = VecNormBound(&v, 2.5350562553841991);

  EXPECT_NEAR(2.6666763421880151, v_norm, 1e-9);
}

TEST(VecNormBound, Bounded19) {
  double d_v[22] = {
      0.3933177632815721,  0.8067421067493357,  0.3476771724887746,
      -0.0486199452131575, 0.3763849510781623,  0.3165679074833954,
      0.3265531733354534,  0.2213211169985383,  -0.1503496628060965,
      -0.6375697081783671, 0.4431443789606004,  -0.6964232287585459,
      0.8565815982247029,  0.5536320051416403,  -0.6329077056892605,
      -0.4962088653276269, -0.3329163541148847, -0.2174636338596017,
      -0.7076927607093180, 0.8592306920356914,  0.8875545894396466,
      0.9495363062454032};
  const Vec v = {22, &d_v[0], 0, 22};
  double v_norm = VecNormBound(&v, 3.4908413279578903);

  EXPECT_NEAR(3.4908413279578903, v_norm, 1e-9);
}

TEST(VecNormSquared, Normal19) {
  double d_v[22] = {
      -0.7619988856088185, 0.6359471894891624,  0.4597898951925781,
      0.1175237546623100,  -0.7605083133844399, -0.0264826434248711,
      -0.3858478785090600, -0.7700687339095889, 0.0014534239497421,
      -0.9304992217380497, 0.7732566682217183,  0.6281685969746409,
      -0.1027925917738053, 0.9198477408008852,  -0.3423965335635610,
      0.2244949745928897,  0.3797918272212937,  0.1909213525623377,
      0.1915089034292878,  -0.8385155887593021, 0.1098017224056036,
      0.2173813854632582};
  const Vec v = {22, &d_v[0], 0, 22};
  double v_norm_sqrd = VecNormSquared(&v);

  EXPECT_NEAR(6.3937084463711882, v_norm_sqrd, 1e-9);
}

TEST(VecNormalize, Normal19) {
  double d_v[22] = {
      -0.7153002234869090, 0.0369091970475732,  0.4708681355156437,
      0.2685657905492982,  0.1393288695470480,  -0.4831397685659877,
      0.4758201215823035,  0.7009385269195587,  0.9007593066809678,
      -0.7317109625577558, 0.8172203223004584,  0.0657303820783732,
      0.5827917677815213,  -0.6603034696769194, -0.6434977771368118,
      -0.7228292498301998, -0.3962579004451521, -0.2880651794371631,
      0.5288087787986877,  -0.8485636844819524, 0.9477718608158456,
      0.3806320077572998};
  Vec v = {22, &d_v[0], 0, 22};
  const Vec *v_out_ptr = VecNormalize(&v, &v);

  EXPECT_NEAR(-0.2562594370956545, v.d[0], 1e-9);
  EXPECT_NEAR(0.0132228814538275, v.d[1], 1e-9);
  EXPECT_NEAR(0.1686905713034878, v.d[2], 1e-9);
  EXPECT_NEAR(0.0962148704131813, v.d[3], 1e-9);
  EXPECT_NEAR(0.0499151775841069, v.d[4], 1e-9);
  EXPECT_NEAR(-0.1730869375766533, v.d[5], 1e-9);
  EXPECT_NEAR(0.1704646419947589, v.d[6], 1e-9);
  EXPECT_NEAR(0.2511142964159170, v.d[7], 1e-9);
  EXPECT_NEAR(0.3227009657057114, v.d[8], 1e-9);
  EXPECT_NEAR(-0.2621386562242583, v.d[9], 1e-9);
  EXPECT_NEAR(0.2927727587655052, v.d[10], 1e-9);
  EXPECT_NEAR(0.0235481971882740, v.d[11], 1e-9);
  EXPECT_NEAR(0.2087877026343572, v.d[12], 1e-9);
  EXPECT_NEAR(-0.2365566092330623, v.d[13], 1e-9);
  EXPECT_NEAR(-0.2305358962947426, v.d[14], 1e-9);
  EXPECT_NEAR(-0.2589567437499216, v.d[15], 1e-9);
  EXPECT_NEAR(-0.1419611278992407, v.d[16], 1e-9);
  EXPECT_NEAR(-0.1032006118627714, v.d[17], 1e-9);
  EXPECT_NEAR(0.1894480604599898, v.d[18], 1e-9);
  EXPECT_NEAR(-0.3040016555078559, v.d[19], 1e-9);
  EXPECT_NEAR(0.3395434190748782, v.d[20], 1e-9);
  EXPECT_NEAR(0.1363630834238926, v.d[21], 1e-9);

  EXPECT_NEAR(-0.2562594370956545, v_out_ptr->d[0], 1e-9);
  EXPECT_NEAR(0.0132228814538275, v_out_ptr->d[1], 1e-9);
  EXPECT_NEAR(0.1686905713034878, v_out_ptr->d[2], 1e-9);
  EXPECT_NEAR(0.0962148704131813, v_out_ptr->d[3], 1e-9);
  EXPECT_NEAR(0.0499151775841069, v_out_ptr->d[4], 1e-9);
  EXPECT_NEAR(-0.1730869375766533, v_out_ptr->d[5], 1e-9);
  EXPECT_NEAR(0.1704646419947589, v_out_ptr->d[6], 1e-9);
  EXPECT_NEAR(0.2511142964159170, v_out_ptr->d[7], 1e-9);
  EXPECT_NEAR(0.3227009657057114, v_out_ptr->d[8], 1e-9);
  EXPECT_NEAR(-0.2621386562242583, v_out_ptr->d[9], 1e-9);
  EXPECT_NEAR(0.2927727587655052, v_out_ptr->d[10], 1e-9);
  EXPECT_NEAR(0.0235481971882740, v_out_ptr->d[11], 1e-9);
  EXPECT_NEAR(0.2087877026343572, v_out_ptr->d[12], 1e-9);
  EXPECT_NEAR(-0.2365566092330623, v_out_ptr->d[13], 1e-9);
  EXPECT_NEAR(-0.2305358962947426, v_out_ptr->d[14], 1e-9);
  EXPECT_NEAR(-0.2589567437499216, v_out_ptr->d[15], 1e-9);
  EXPECT_NEAR(-0.1419611278992407, v_out_ptr->d[16], 1e-9);
  EXPECT_NEAR(-0.1032006118627714, v_out_ptr->d[17], 1e-9);
  EXPECT_NEAR(0.1894480604599898, v_out_ptr->d[18], 1e-9);
  EXPECT_NEAR(-0.3040016555078559, v_out_ptr->d[19], 1e-9);
  EXPECT_NEAR(0.3395434190748782, v_out_ptr->d[20], 1e-9);
  EXPECT_NEAR(0.1363630834238926, v_out_ptr->d[21], 1e-9);
}

TEST(VecNormalize, Zero_19) {
  double d_v[22] = {0.0000000000000000, 0.0000000000000000, 0.0000000000000000,
                    0.0000000000000000, 0.0000000000000000, 0.0000000000000000,
                    0.0000000000000000, 0.0000000000000000, 0.0000000000000000,
                    0.0000000000000000, 0.0000000000000000, 0.0000000000000000,
                    0.0000000000000000, 0.0000000000000000, 0.0000000000000000,
                    0.0000000000000000, 0.0000000000000000, 0.0000000000000000,
                    0.0000000000000000, 0.0000000000000000, 0.0000000000000000,
                    0.0000000000000000};
  Vec v = {22, &d_v[0], 0, 22};
  const Vec *v_out_ptr = VecNormalize(&v, &v);

  EXPECT_NEAR(0.0000000000000000, v.d[0], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v.d[1], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v.d[2], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v.d[3], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v.d[4], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v.d[5], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v.d[6], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v.d[7], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v.d[8], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v.d[9], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v.d[10], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v.d[11], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v.d[12], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v.d[13], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v.d[14], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v.d[15], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v.d[16], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v.d[17], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v.d[18], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v.d[19], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v.d[20], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v.d[21], 1e-9);

  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[0], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[1], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[2], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[3], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[4], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[5], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[6], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[7], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[8], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[9], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[10], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[11], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[12], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[13], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[14], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[15], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[16], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[17], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[18], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[19], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[20], 1e-9);
  EXPECT_NEAR(0.0000000000000000, v_out_ptr->d[21], 1e-9);
}

TEST(VecScale, Normal19) {
  double d_v[22] = {
      0.0028931644488521,  0.6419260548613888,  -0.3398296719081944,
      0.6501444630969033,  -0.1669043650857214, 0.4011728239611889,
      -0.5634197611209222, 0.0441849218518799,  -0.5272456464283659,
      0.0758126865950042,  -0.4132862757130216, -0.2055124588309478,
      0.3868656681907350,  -0.0552773829003612, 0.3596512682454795,
      -0.2525881226780691, -0.9811348592664537, 0.5041148244537135,
      0.3089880346524518,  0.2396066463188371,  0.9364859539218593,
      0.6053853215422864};
  const Vec v = {22, &d_v[0], 0, 22};
  double d_v_out[22] = {
      -0.0373135165103700, 0.9559661090856648,  -0.2462011991299660,
      0.9829876296003932,  -0.7031394912763094, -0.0879497023809186,
      -0.8687110503260176, -0.9854114328499699, -0.8919199935353428,
      0.3747603367000201,  -0.5037177991976853, -0.5967438120718347,
      -0.5050551337506881, 0.1426719282832107,  0.6903881372583283,
      -0.5210041206469869, -0.3636046164340609, 0.5307562284522105,
      0.7037824224040885,  -0.8075619555730935, 0.0855439134013414,
      -0.4520797476797751};
  Vec v_out = {22, &d_v_out[0], 0, 22};
  const Vec *v_out_ptr = VecScale(&v, -0.8320318066054408, &v_out);

  EXPECT_NEAR(-0.0024072048431851, v_out.d[0], 1e-9);
  EXPECT_NEAR(-0.5341028951334247, v_out.d[1], 1e-9);
  EXPECT_NEAR(0.2827490958559092, v_out.d[2], 1e-9);
  EXPECT_NEAR(-0.5409408721850408, v_out.d[3], 1e-9);
  EXPECT_NEAR(0.1388697404126068, v_out.d[4], 1e-9);
  EXPECT_NEAR(-0.3337885494814345, v_out.d[5], 1e-9);
  EXPECT_NEAR(0.4687831617226468, v_out.d[6], 1e-9);
  EXPECT_NEAR(-0.0367632603531399, v_out.d[7], 1e-9);
  EXPECT_NEAR(0.4386851477226468, v_out.d[8], 1e-9);
  EXPECT_NEAR(-0.0630785665912534, v_out.d[9], 1e-9);
  EXPECT_NEAR(0.3438673266267397, v_out.d[10], 1e-9);
  EXPECT_NEAR(0.1709929024010398, v_out.d[11], 1e-9);
  EXPECT_NEAR(-0.3218845408183583, v_out.d[12], 1e-9);
  EXPECT_NEAR(0.0459925407590082, v_out.d[13], 1e-9);
  EXPECT_NEAR(-0.2992412944662244, v_out.d[14], 1e-9);
  EXPECT_NEAR(0.2101613520389105, v_out.d[15], 1e-9);
  EXPECT_NEAR(0.8163354094790425, v_out.d[16], 1e-9);
  EXPECT_NEAR(-0.4194395681268079, v_out.d[17], 1e-9);
  EXPECT_NEAR(-0.2570878726913441, v_out.d[18], 1e-9);
  EXPECT_NEAR(-0.1993603508113329, v_out.d[19], 1e-9);
  EXPECT_NEAR(-0.7791861001022242, v_out.d[20], 1e-9);
  EXPECT_NEAR(-0.5036998427752443, v_out.d[21], 1e-9);

  EXPECT_NEAR(-0.0024072048431851, v_out_ptr->d[0], 1e-9);
  EXPECT_NEAR(-0.5341028951334247, v_out_ptr->d[1], 1e-9);
  EXPECT_NEAR(0.2827490958559092, v_out_ptr->d[2], 1e-9);
  EXPECT_NEAR(-0.5409408721850408, v_out_ptr->d[3], 1e-9);
  EXPECT_NEAR(0.1388697404126068, v_out_ptr->d[4], 1e-9);
  EXPECT_NEAR(-0.3337885494814345, v_out_ptr->d[5], 1e-9);
  EXPECT_NEAR(0.4687831617226468, v_out_ptr->d[6], 1e-9);
  EXPECT_NEAR(-0.0367632603531399, v_out_ptr->d[7], 1e-9);
  EXPECT_NEAR(0.4386851477226468, v_out_ptr->d[8], 1e-9);
  EXPECT_NEAR(-0.0630785665912534, v_out_ptr->d[9], 1e-9);
  EXPECT_NEAR(0.3438673266267397, v_out_ptr->d[10], 1e-9);
  EXPECT_NEAR(0.1709929024010398, v_out_ptr->d[11], 1e-9);
  EXPECT_NEAR(-0.3218845408183583, v_out_ptr->d[12], 1e-9);
  EXPECT_NEAR(0.0459925407590082, v_out_ptr->d[13], 1e-9);
  EXPECT_NEAR(-0.2992412944662244, v_out_ptr->d[14], 1e-9);
  EXPECT_NEAR(0.2101613520389105, v_out_ptr->d[15], 1e-9);
  EXPECT_NEAR(0.8163354094790425, v_out_ptr->d[16], 1e-9);
  EXPECT_NEAR(-0.4194395681268079, v_out_ptr->d[17], 1e-9);
  EXPECT_NEAR(-0.2570878726913441, v_out_ptr->d[18], 1e-9);
  EXPECT_NEAR(-0.1993603508113329, v_out_ptr->d[19], 1e-9);
  EXPECT_NEAR(-0.7791861001022242, v_out_ptr->d[20], 1e-9);
  EXPECT_NEAR(-0.5036998427752443, v_out_ptr->d[21], 1e-9);
}

TEST(VecAdd, Normal19) {
  double d_v0[22] = {
      -0.8866392645293864, -0.6369814255696502, -0.8213302057966267,
      -0.7297964699469346, -0.7581571399621778, -0.7612491279128886,
      -0.2672658174717331, 0.8467210586869864,  0.3454231351578747,
      -0.5394122713944995, -0.0380368234532971, 0.4861190953475392,
      -0.4566475734985693, 0.5899953126817847,  0.8919686383518000,
      -0.6734498182463144, -0.4184219337530115, 0.0061703384343381,
      0.8213640827349109,  -0.8950260554733078, 0.7644050544115319,
      -0.1112655943479264};
  const Vec v0 = {22, &d_v0[0], 0, 22};
  double d_v1[22] = {
      0.2287967560362831,  -0.2979722596373386, 0.1221742541634767,
      0.9634334650894589,  -0.0844611004977942, -0.2482872074920563,
      -0.4610284146689101, -0.8985184979982235, -0.8698100263063007,
      -0.1714343085269687, -0.1153273352421258, -0.3217399216230221,
      0.5754018313914087,  0.7166346426264854,  0.6074395898224403,
      0.6354083034616091,  -0.4906396481248092, -0.7273828709782264,
      -0.6290214180308282, -0.0869416177928277, -0.4859447610919865,
      0.7513718998047960};
  const Vec v1 = {22, &d_v1[0], 0, 22};
  double d_v_out[22] = {
      -0.3858842353227283, 0.6750737260667066,  -0.9505467790450917,
      0.3825746919573638,  0.0255466530106321,  0.7159937288720883,
      -0.0803692259833644, 0.3340399132417349,  0.7338827184941845,
      0.0015440029607965,  0.2394975629953968,  0.8653635378088185,
      0.9316829721054882,  0.4444803692892829,  0.9257224828308843,
      0.3824373562811747,  0.4583390689034683,  0.2497932938168770,
      -0.9351193901366419, -0.4843901565409410, -0.4269552673260004,
      0.2810665001612154};
  Vec v_out = {22, &d_v_out[0], 0, 22};
  const Vec *v_out_ptr = VecAdd(&v0, &v1, &v_out);

  EXPECT_NEAR(-0.6578425084931032, v_out.d[0], 1e-9);
  EXPECT_NEAR(-0.9349536852069888, v_out.d[1], 1e-9);
  EXPECT_NEAR(-0.6991559516331500, v_out.d[2], 1e-9);
  EXPECT_NEAR(0.2336369951425243, v_out.d[3], 1e-9);
  EXPECT_NEAR(-0.8426182404599720, v_out.d[4], 1e-9);
  EXPECT_NEAR(-1.0095363354049449, v_out.d[5], 1e-9);
  EXPECT_NEAR(-0.7282942321406431, v_out.d[6], 1e-9);
  EXPECT_NEAR(-0.0517974393112370, v_out.d[7], 1e-9);
  EXPECT_NEAR(-0.5243868911484260, v_out.d[8], 1e-9);
  EXPECT_NEAR(-0.7108465799214683, v_out.d[9], 1e-9);
  EXPECT_NEAR(-0.1533641586954229, v_out.d[10], 1e-9);
  EXPECT_NEAR(0.1643791737245170, v_out.d[11], 1e-9);
  EXPECT_NEAR(0.1187542578928393, v_out.d[12], 1e-9);
  EXPECT_NEAR(1.3066299553082701, v_out.d[13], 1e-9);
  EXPECT_NEAR(1.4994082281742402, v_out.d[14], 1e-9);
  EXPECT_NEAR(-0.0380415147847053, v_out.d[15], 1e-9);
  EXPECT_NEAR(-0.9090615818778207, v_out.d[16], 1e-9);
  EXPECT_NEAR(-0.7212125325438883, v_out.d[17], 1e-9);
  EXPECT_NEAR(0.1923426647040827, v_out.d[18], 1e-9);
  EXPECT_NEAR(-0.9819676732661355, v_out.d[19], 1e-9);
  EXPECT_NEAR(0.2784602933195455, v_out.d[20], 1e-9);
  EXPECT_NEAR(0.6401063054568696, v_out.d[21], 1e-9);

  EXPECT_NEAR(-0.6578425084931032, v_out_ptr->d[0], 1e-9);
  EXPECT_NEAR(-0.9349536852069888, v_out_ptr->d[1], 1e-9);
  EXPECT_NEAR(-0.6991559516331500, v_out_ptr->d[2], 1e-9);
  EXPECT_NEAR(0.2336369951425243, v_out_ptr->d[3], 1e-9);
  EXPECT_NEAR(-0.8426182404599720, v_out_ptr->d[4], 1e-9);
  EXPECT_NEAR(-1.0095363354049449, v_out_ptr->d[5], 1e-9);
  EXPECT_NEAR(-0.7282942321406431, v_out_ptr->d[6], 1e-9);
  EXPECT_NEAR(-0.0517974393112370, v_out_ptr->d[7], 1e-9);
  EXPECT_NEAR(-0.5243868911484260, v_out_ptr->d[8], 1e-9);
  EXPECT_NEAR(-0.7108465799214683, v_out_ptr->d[9], 1e-9);
  EXPECT_NEAR(-0.1533641586954229, v_out_ptr->d[10], 1e-9);
  EXPECT_NEAR(0.1643791737245170, v_out_ptr->d[11], 1e-9);
  EXPECT_NEAR(0.1187542578928393, v_out_ptr->d[12], 1e-9);
  EXPECT_NEAR(1.3066299553082701, v_out_ptr->d[13], 1e-9);
  EXPECT_NEAR(1.4994082281742402, v_out_ptr->d[14], 1e-9);
  EXPECT_NEAR(-0.0380415147847053, v_out_ptr->d[15], 1e-9);
  EXPECT_NEAR(-0.9090615818778207, v_out_ptr->d[16], 1e-9);
  EXPECT_NEAR(-0.7212125325438883, v_out_ptr->d[17], 1e-9);
  EXPECT_NEAR(0.1923426647040827, v_out_ptr->d[18], 1e-9);
  EXPECT_NEAR(-0.9819676732661355, v_out_ptr->d[19], 1e-9);
  EXPECT_NEAR(0.2784602933195455, v_out_ptr->d[20], 1e-9);
  EXPECT_NEAR(0.6401063054568696, v_out_ptr->d[21], 1e-9);
}

TEST(VecAdd3, Normal19) {
  double d_v0[22] = {
      -0.2892271900546171, -0.6325082970186118, 0.9935481275994835,
      -0.3783458560091595, 0.9462461122151862,  -0.1814498631456694,
      -0.2620753729273628, 0.0462256936614533,  -0.3547980078846165,
      0.4219485504695906,  0.7682054779004106,  -0.2305015858655470,
      0.9259748279918443,  0.8183515568220485,  -0.2513608844264854,
      -0.7663094153763816, -0.3129997910585101, 0.1144906915884443,
      -0.1106110759720109, 0.6532270045085335,  -0.4656397839795907,
      0.5246965537770727};
  const Vec v0 = {22, &d_v0[0], 0, 22};
  double d_v1[22] = {
      -0.8910570401506284, 0.4080454499005701,  0.4168988628808359,
      -0.7124670278481229, -0.1581255529077208, 0.3289321140078780,
      0.3890981907853817,  -0.0863225062175730, 0.9801294774249480,
      0.3411589071511119,  -0.6902222508486513, 0.4772794117657864,
      0.1079410454300784,  0.2853502277358353,  -0.6378628316897565,
      0.0779482012609447,  0.1123514806407342,  -0.4581922966633127,
      0.9837198803518250,  -0.6659362468040753, 0.4630896742905641,
      0.6824762030804095};
  const Vec v1 = {22, &d_v1[0], 0, 22};
  double d_v2[22] = {
      -0.4023879934160963, -0.1818765866458198, 0.3233023116574312,
      0.9748390619916341,  0.5098660428642552,  -0.0410457014912728,
      -0.2461075453191706, 0.8461861889373794,  -0.7317196875082344,
      0.6161172484575130,  -0.2980442443879352, -0.6513229107666481,
      -0.5422880099134384, 0.0697183758586493,  -0.4131405671247983,
      0.7170886373655756,  0.2631994157787036,  0.6687115475995791,
      -0.4868374228455739, 0.2645366643107494,  0.4956586158244254,
      -0.5018390312310288};
  const Vec v2 = {22, &d_v2[0], 0, 22};
  double d_v0c[22] = {
      -0.2892271900546171, -0.6325082970186118, 0.9935481275994835,
      -0.3783458560091595, 0.9462461122151862,  -0.1814498631456694,
      -0.2620753729273628, 0.0462256936614533,  -0.3547980078846165,
      0.4219485504695906,  0.7682054779004106,  -0.2305015858655470,
      0.9259748279918443,  0.8183515568220485,  -0.2513608844264854,
      -0.7663094153763816, -0.3129997910585101, 0.1144906915884443,
      -0.1106110759720109, 0.6532270045085335,  -0.4656397839795907,
      0.5246965537770727};
  Vec v0c = {22, &d_v0c[0], 0, 22};
  double d_v1c[22] = {
      -0.8910570401506284, 0.4080454499005701,  0.4168988628808359,
      -0.7124670278481229, -0.1581255529077208, 0.3289321140078780,
      0.3890981907853817,  -0.0863225062175730, 0.9801294774249480,
      0.3411589071511119,  -0.6902222508486513, 0.4772794117657864,
      0.1079410454300784,  0.2853502277358353,  -0.6378628316897565,
      0.0779482012609447,  0.1123514806407342,  -0.4581922966633127,
      0.9837198803518250,  -0.6659362468040753, 0.4630896742905641,
      0.6824762030804095};
  Vec v1c = {22, &d_v1c[0], 0, 22};
  double d_v2c[22] = {
      -0.4023879934160963, -0.1818765866458198, 0.3233023116574312,
      0.9748390619916341,  0.5098660428642552,  -0.0410457014912728,
      -0.2461075453191706, 0.8461861889373794,  -0.7317196875082344,
      0.6161172484575130,  -0.2980442443879352, -0.6513229107666481,
      -0.5422880099134384, 0.0697183758586493,  -0.4131405671247983,
      0.7170886373655756,  0.2631994157787036,  0.6687115475995791,
      -0.4868374228455739, 0.2645366643107494,  0.4956586158244254,
      -0.5018390312310288};
  Vec v2c = {22, &d_v2c[0], 0, 22};
  double d_v_out[22] = {
      -0.2916687957802941, 0.2764803136449714,  0.5402902906419464,
      0.6019578804641905,  0.6557058607122634,  0.6718368280329763,
      0.3860041087948214,  -0.9825868218024048, 0.3529312248081005,
      -0.0337785588674349, 0.4497906030521885,  -0.0286682283346145,
      0.5432067133647747,  0.7515925902914937,  -0.5501685971359216,
      -0.7753719314952572, 0.5554050639922763,  -0.9549963930402592,
      -0.0954007640668033, -0.8199859428698510, -0.4534806851384001,
      -0.4087857062335289};
  Vec v_out = {22, &d_v_out[0], 0, 22};
  const Vec *v_out_ptr = VecAdd3(&v0, &v1, &v2, &v_out);
  VecAdd3(&v0c, &v1, &v2, &v0c);
  VecAdd3(&v0, &v1c, &v2, &v1c);
  VecAdd3(&v0, &v1, &v2c, &v2c);

  EXPECT_NEAR(-1.5826722236213417, v_out.d[0], 1e-9);
  EXPECT_NEAR(-0.4063394337638615, v_out.d[1], 1e-9);
  EXPECT_NEAR(1.7337493021377506, v_out.d[2], 1e-9);
  EXPECT_NEAR(-0.1159738218656483, v_out.d[3], 1e-9);
  EXPECT_NEAR(1.2979866021717206, v_out.d[4], 1e-9);
  EXPECT_NEAR(0.1064365493709358, v_out.d[5], 1e-9);
  EXPECT_NEAR(-0.1190847274611517, v_out.d[6], 1e-9);
  EXPECT_NEAR(0.8060893763812598, v_out.d[7], 1e-9);
  EXPECT_NEAR(-0.1063882179679028, v_out.d[8], 1e-9);
  EXPECT_NEAR(1.3792247060782155, v_out.d[9], 1e-9);
  EXPECT_NEAR(-0.2200610173361759, v_out.d[10], 1e-9);
  EXPECT_NEAR(-0.4045450848664087, v_out.d[11], 1e-9);
  EXPECT_NEAR(0.4916278635084843, v_out.d[12], 1e-9);
  EXPECT_NEAR(1.1734201604165331, v_out.d[13], 1e-9);
  EXPECT_NEAR(-1.3023642832410403, v_out.d[14], 1e-9);
  EXPECT_NEAR(0.0287274232501387, v_out.d[15], 1e-9);
  EXPECT_NEAR(0.0625511053609278, v_out.d[16], 1e-9);
  EXPECT_NEAR(0.3250099425247106, v_out.d[17], 1e-9);
  EXPECT_NEAR(0.3862713815342402, v_out.d[18], 1e-9);
  EXPECT_NEAR(0.2518274220152077, v_out.d[19], 1e-9);
  EXPECT_NEAR(0.4931085061353988, v_out.d[20], 1e-9);
  EXPECT_NEAR(0.7053337256264534, v_out.d[21], 1e-9);

  EXPECT_NEAR(-1.5826722236213417, v0c.d[0], 1e-9);
  EXPECT_NEAR(-0.4063394337638615, v0c.d[1], 1e-9);
  EXPECT_NEAR(1.7337493021377506, v0c.d[2], 1e-9);
  EXPECT_NEAR(-0.1159738218656483, v0c.d[3], 1e-9);
  EXPECT_NEAR(1.2979866021717206, v0c.d[4], 1e-9);
  EXPECT_NEAR(0.1064365493709358, v0c.d[5], 1e-9);
  EXPECT_NEAR(-0.1190847274611517, v0c.d[6], 1e-9);
  EXPECT_NEAR(0.8060893763812598, v0c.d[7], 1e-9);
  EXPECT_NEAR(-0.1063882179679028, v0c.d[8], 1e-9);
  EXPECT_NEAR(1.3792247060782155, v0c.d[9], 1e-9);
  EXPECT_NEAR(-0.2200610173361759, v0c.d[10], 1e-9);
  EXPECT_NEAR(-0.4045450848664087, v0c.d[11], 1e-9);
  EXPECT_NEAR(0.4916278635084843, v0c.d[12], 1e-9);
  EXPECT_NEAR(1.1734201604165331, v0c.d[13], 1e-9);
  EXPECT_NEAR(-1.3023642832410403, v0c.d[14], 1e-9);
  EXPECT_NEAR(0.0287274232501387, v0c.d[15], 1e-9);
  EXPECT_NEAR(0.0625511053609278, v0c.d[16], 1e-9);
  EXPECT_NEAR(0.3250099425247106, v0c.d[17], 1e-9);
  EXPECT_NEAR(0.3862713815342402, v0c.d[18], 1e-9);
  EXPECT_NEAR(0.2518274220152077, v0c.d[19], 1e-9);
  EXPECT_NEAR(0.4931085061353988, v0c.d[20], 1e-9);
  EXPECT_NEAR(0.7053337256264534, v0c.d[21], 1e-9);

  EXPECT_NEAR(-1.5826722236213417, v1c.d[0], 1e-9);
  EXPECT_NEAR(-0.4063394337638615, v1c.d[1], 1e-9);
  EXPECT_NEAR(1.7337493021377506, v1c.d[2], 1e-9);
  EXPECT_NEAR(-0.1159738218656483, v1c.d[3], 1e-9);
  EXPECT_NEAR(1.2979866021717206, v1c.d[4], 1e-9);
  EXPECT_NEAR(0.1064365493709358, v1c.d[5], 1e-9);
  EXPECT_NEAR(-0.1190847274611517, v1c.d[6], 1e-9);
  EXPECT_NEAR(0.8060893763812598, v1c.d[7], 1e-9);
  EXPECT_NEAR(-0.1063882179679028, v1c.d[8], 1e-9);
  EXPECT_NEAR(1.3792247060782155, v1c.d[9], 1e-9);
  EXPECT_NEAR(-0.2200610173361759, v1c.d[10], 1e-9);
  EXPECT_NEAR(-0.4045450848664087, v1c.d[11], 1e-9);
  EXPECT_NEAR(0.4916278635084843, v1c.d[12], 1e-9);
  EXPECT_NEAR(1.1734201604165331, v1c.d[13], 1e-9);
  EXPECT_NEAR(-1.3023642832410403, v1c.d[14], 1e-9);
  EXPECT_NEAR(0.0287274232501387, v1c.d[15], 1e-9);
  EXPECT_NEAR(0.0625511053609278, v1c.d[16], 1e-9);
  EXPECT_NEAR(0.3250099425247106, v1c.d[17], 1e-9);
  EXPECT_NEAR(0.3862713815342402, v1c.d[18], 1e-9);
  EXPECT_NEAR(0.2518274220152077, v1c.d[19], 1e-9);
  EXPECT_NEAR(0.4931085061353988, v1c.d[20], 1e-9);
  EXPECT_NEAR(0.7053337256264534, v1c.d[21], 1e-9);

  EXPECT_NEAR(-1.5826722236213417, v2c.d[0], 1e-9);
  EXPECT_NEAR(-0.4063394337638615, v2c.d[1], 1e-9);
  EXPECT_NEAR(1.7337493021377506, v2c.d[2], 1e-9);
  EXPECT_NEAR(-0.1159738218656483, v2c.d[3], 1e-9);
  EXPECT_NEAR(1.2979866021717206, v2c.d[4], 1e-9);
  EXPECT_NEAR(0.1064365493709358, v2c.d[5], 1e-9);
  EXPECT_NEAR(-0.1190847274611517, v2c.d[6], 1e-9);
  EXPECT_NEAR(0.8060893763812598, v2c.d[7], 1e-9);
  EXPECT_NEAR(-0.1063882179679028, v2c.d[8], 1e-9);
  EXPECT_NEAR(1.3792247060782155, v2c.d[9], 1e-9);
  EXPECT_NEAR(-0.2200610173361759, v2c.d[10], 1e-9);
  EXPECT_NEAR(-0.4045450848664087, v2c.d[11], 1e-9);
  EXPECT_NEAR(0.4916278635084843, v2c.d[12], 1e-9);
  EXPECT_NEAR(1.1734201604165331, v2c.d[13], 1e-9);
  EXPECT_NEAR(-1.3023642832410403, v2c.d[14], 1e-9);
  EXPECT_NEAR(0.0287274232501387, v2c.d[15], 1e-9);
  EXPECT_NEAR(0.0625511053609278, v2c.d[16], 1e-9);
  EXPECT_NEAR(0.3250099425247106, v2c.d[17], 1e-9);
  EXPECT_NEAR(0.3862713815342402, v2c.d[18], 1e-9);
  EXPECT_NEAR(0.2518274220152077, v2c.d[19], 1e-9);
  EXPECT_NEAR(0.4931085061353988, v2c.d[20], 1e-9);
  EXPECT_NEAR(0.7053337256264534, v2c.d[21], 1e-9);

  EXPECT_NEAR(-1.5826722236213417, v_out_ptr->d[0], 1e-9);
  EXPECT_NEAR(-0.4063394337638615, v_out_ptr->d[1], 1e-9);
  EXPECT_NEAR(1.7337493021377506, v_out_ptr->d[2], 1e-9);
  EXPECT_NEAR(-0.1159738218656483, v_out_ptr->d[3], 1e-9);
  EXPECT_NEAR(1.2979866021717206, v_out_ptr->d[4], 1e-9);
  EXPECT_NEAR(0.1064365493709358, v_out_ptr->d[5], 1e-9);
  EXPECT_NEAR(-0.1190847274611517, v_out_ptr->d[6], 1e-9);
  EXPECT_NEAR(0.8060893763812598, v_out_ptr->d[7], 1e-9);
  EXPECT_NEAR(-0.1063882179679028, v_out_ptr->d[8], 1e-9);
  EXPECT_NEAR(1.3792247060782155, v_out_ptr->d[9], 1e-9);
  EXPECT_NEAR(-0.2200610173361759, v_out_ptr->d[10], 1e-9);
  EXPECT_NEAR(-0.4045450848664087, v_out_ptr->d[11], 1e-9);
  EXPECT_NEAR(0.4916278635084843, v_out_ptr->d[12], 1e-9);
  EXPECT_NEAR(1.1734201604165331, v_out_ptr->d[13], 1e-9);
  EXPECT_NEAR(-1.3023642832410403, v_out_ptr->d[14], 1e-9);
  EXPECT_NEAR(0.0287274232501387, v_out_ptr->d[15], 1e-9);
  EXPECT_NEAR(0.0625511053609278, v_out_ptr->d[16], 1e-9);
  EXPECT_NEAR(0.3250099425247106, v_out_ptr->d[17], 1e-9);
  EXPECT_NEAR(0.3862713815342402, v_out_ptr->d[18], 1e-9);
  EXPECT_NEAR(0.2518274220152077, v_out_ptr->d[19], 1e-9);
  EXPECT_NEAR(0.4931085061353988, v_out_ptr->d[20], 1e-9);
  EXPECT_NEAR(0.7053337256264534, v_out_ptr->d[21], 1e-9);
}

TEST(VecSub, Normal19) {
  double d_v0[22] = {
      -0.6483886660856353, -0.9147896276368344, 0.7638094684671219,
      -0.4578607858993433, -0.7051583172010809, -0.6045208737194598,
      -0.3308234333737470, -0.4782334303193305, 0.1924209272335549,
      -0.9460641751934233, -0.7001025143975679, 0.1838607413587741,
      -0.4265357589095791, -0.9779094910135404, 0.3621524663681781,
      -0.6220706336942654, 0.4133227309884153,  -0.5235377808730477,
      0.9058653238594798,  0.2313375543283778,  0.8857038311406349,
      0.4054878173125349};
  const Vec v0 = {22, &d_v0[0], 0, 22};
  double d_v1[22] = {
      0.5817624360519518,  -0.0232481809047496, -0.4980933258818814,
      0.5157707982501980,  0.5855785039842893,  -0.2775020720918491,
      0.6656738678809806,  0.7966602944231498,  0.4838394855636117,
      -0.6714598548544268, -0.7742406068390131, -0.9351627905718569,
      -0.3808578837683476, -0.5228663772073581, 0.4697823703634278,
      0.6077002197577002,  0.5007520768652651,  -0.1015226969351786,
      0.0459828812804794,  -0.1239453008892673, 0.3285480209126002,
      0.9772641959392645};
  const Vec v1 = {22, &d_v1[0], 0, 22};
  double d_v_out[22] = {
      0.8275389291147421,  0.7359854959126870,  0.0040279393724294,
      -0.8953552290196822, -0.0094605802504633, -0.0343984257379322,
      0.5873031007866782,  -0.7221841383865797, -0.3596387943552943,
      0.4874280121933958,  0.7480826124155402,  0.1674869882127359,
      0.7282159709263143,  -0.8651194230506538, 0.0137584644772144,
      0.9200369761650491,  -0.2254883546709934, 0.8103240806580678,
      -0.6334987177643534, -0.5211071050473950, -0.6340994684758965,
      -0.6777750802551723};
  Vec v_out = {22, &d_v_out[0], 0, 22};
  const Vec *v_out_ptr = VecSub(&v0, &v1, &v_out);

  EXPECT_NEAR(-1.2301511021375870, v_out.d[0], 1e-9);
  EXPECT_NEAR(-0.8915414467320848, v_out.d[1], 1e-9);
  EXPECT_NEAR(1.2619027943490033, v_out.d[2], 1e-9);
  EXPECT_NEAR(-0.9736315841495413, v_out.d[3], 1e-9);
  EXPECT_NEAR(-1.2907368211853703, v_out.d[4], 1e-9);
  EXPECT_NEAR(-0.3270188016276108, v_out.d[5], 1e-9);
  EXPECT_NEAR(-0.9964973012547276, v_out.d[6], 1e-9);
  EXPECT_NEAR(-1.2748937247424803, v_out.d[7], 1e-9);
  EXPECT_NEAR(-0.2914185583300568, v_out.d[8], 1e-9);
  EXPECT_NEAR(-0.2746043203389965, v_out.d[9], 1e-9);
  EXPECT_NEAR(0.0741380924414452, v_out.d[10], 1e-9);
  EXPECT_NEAR(1.1190235319306310, v_out.d[11], 1e-9);
  EXPECT_NEAR(-0.0456778751412314, v_out.d[12], 1e-9);
  EXPECT_NEAR(-0.4550431138061823, v_out.d[13], 1e-9);
  EXPECT_NEAR(-0.1076299039952497, v_out.d[14], 1e-9);
  EXPECT_NEAR(-1.2297708534519656, v_out.d[15], 1e-9);
  EXPECT_NEAR(-0.0874293458768498, v_out.d[16], 1e-9);
  EXPECT_NEAR(-0.4220150839378691, v_out.d[17], 1e-9);
  EXPECT_NEAR(0.8598824425790004, v_out.d[18], 1e-9);
  EXPECT_NEAR(0.3552828552176450, v_out.d[19], 1e-9);
  EXPECT_NEAR(0.5571558102280347, v_out.d[20], 1e-9);
  EXPECT_NEAR(-0.5717763786267296, v_out.d[21], 1e-9);

  EXPECT_NEAR(-1.2301511021375870, v_out_ptr->d[0], 1e-9);
  EXPECT_NEAR(-0.8915414467320848, v_out_ptr->d[1], 1e-9);
  EXPECT_NEAR(1.2619027943490033, v_out_ptr->d[2], 1e-9);
  EXPECT_NEAR(-0.9736315841495413, v_out_ptr->d[3], 1e-9);
  EXPECT_NEAR(-1.2907368211853703, v_out_ptr->d[4], 1e-9);
  EXPECT_NEAR(-0.3270188016276108, v_out_ptr->d[5], 1e-9);
  EXPECT_NEAR(-0.9964973012547276, v_out_ptr->d[6], 1e-9);
  EXPECT_NEAR(-1.2748937247424803, v_out_ptr->d[7], 1e-9);
  EXPECT_NEAR(-0.2914185583300568, v_out_ptr->d[8], 1e-9);
  EXPECT_NEAR(-0.2746043203389965, v_out_ptr->d[9], 1e-9);
  EXPECT_NEAR(0.0741380924414452, v_out_ptr->d[10], 1e-9);
  EXPECT_NEAR(1.1190235319306310, v_out_ptr->d[11], 1e-9);
  EXPECT_NEAR(-0.0456778751412314, v_out_ptr->d[12], 1e-9);
  EXPECT_NEAR(-0.4550431138061823, v_out_ptr->d[13], 1e-9);
  EXPECT_NEAR(-0.1076299039952497, v_out_ptr->d[14], 1e-9);
  EXPECT_NEAR(-1.2297708534519656, v_out_ptr->d[15], 1e-9);
  EXPECT_NEAR(-0.0874293458768498, v_out_ptr->d[16], 1e-9);
  EXPECT_NEAR(-0.4220150839378691, v_out_ptr->d[17], 1e-9);
  EXPECT_NEAR(0.8598824425790004, v_out_ptr->d[18], 1e-9);
  EXPECT_NEAR(0.3552828552176450, v_out_ptr->d[19], 1e-9);
  EXPECT_NEAR(0.5571558102280347, v_out_ptr->d[20], 1e-9);
  EXPECT_NEAR(-0.5717763786267296, v_out_ptr->d[21], 1e-9);
}

TEST(VecLinComb, Normal19) {
  double d_v0[22] = {
      -0.8432046647413658, 0.9921920416585557,  0.3792015984613115,
      0.4818178112436351,  0.0905731089401931,  -0.7251699160489224,
      -0.8024261553722594, 0.5126572420878215,  0.6890286663662724,
      -0.4543050928163208, -0.6213524702880837, -0.5547483487433191,
      -0.4117953183011158, 0.6400954072148080,  -0.8372009788103552,
      -0.1055006543118819, -0.3228106140024967, 0.7839209099489635,
      0.4548884089811807,  -0.0228702018156779, -0.1086601385906318,
      0.8370490278542668};
  const Vec v0 = {22, &d_v0[0], 0, 22};
  double d_v1[22] = {
      -0.8627439772375365, -0.7201180413083876, 0.8675233892007930,
      0.1471494260319131,  0.4651505576968407,  0.2925686763895765,
      0.4605416088737042,  0.5367054845307035,  0.1937445377171421,
      0.4025516715057287,  0.8488667804141317,  0.5818917865459972,
      0.2407293048580676,  -0.8120772881204181, -0.0088026205855940,
      0.2547927077799210,  0.8386174127120101,  -0.0050381312710273,
      -0.0917673484907464, -0.7175905946792964, -0.8131386404185970,
      -0.5084151515366637};
  const Vec v1 = {22, &d_v1[0], 0, 22};
  double d_v_out[22] = {
      0.0374565725599103,  0.5690536286559802,  0.0790746353735239,
      0.0914213699359443,  0.6230575810758905,  0.5356454021996127,
      0.2444400885638760,  -0.4699650743088688, -0.6519068845189353,
      0.1803656212341134,  0.9872555572975956,  -0.8955760909855299,
      -0.3192867678831519, 0.9908617282666861,  0.0948078434155915,
      0.8378585757670214,  0.5213857183806552,  0.3459021790181529,
      -0.1265015387071817, -0.7993546484688219, 0.7392748698002154,
      0.7646458365370696};
  Vec v_out = {22, &d_v_out[0], 0, 22};
  const Vec *v_out_ptr =
      VecLinComb(-0.4838674894461308, &v0, 0.6378338973318050, &v1, &v_out);

  EXPECT_NEAR(-0.1422880291832889, v_out.d[0], 1e-9);
  EXPECT_NEAR(-0.9394051690724308, v_out.d[1], 1e-9);
  EXPECT_NEAR(0.3698524989190035, v_out.d[2], 1e-9);
  EXPECT_NEAR(-0.1392790828008142, v_out.d[3], 1e-9);
  EXPECT_NEAR(0.2528634102276163, v_out.d[4], 1e-9);
  EXPECT_NEAR(0.5374963657992248, v_out.d[5], 1e-9);
  EXPECT_NEAR(0.6820169785372605, v_out.d[6], 1e-9);
  EXPECT_NEAR(0.0942707782421619, v_out.d[7], 1e-9);
  EXPECT_NEAR(-0.2098217373721902, v_out.d[8], 1e-9);
  EXPECT_NEAR(0.4765845662175560, v_out.d[9], 1e-9);
  EXPECT_NEAR(0.8420882667264937, v_out.d[10], 1e-9);
  EXPECT_NEAR(0.6395749968188165, v_out.d[11], 1e-9);
  EXPECT_NEAR(0.3527996775516288, v_out.d[12], 1e-9);
  EXPECT_NEAR(-0.8276917793115173, v_out.d[13], 1e-9);
  EXPECT_NEAR(0.3994797259839673, v_out.d[14], 1e-9);
  EXPECT_NEAR(0.2135637625518051, v_out.d[15], 1e-9);
  EXPECT_NEAR(0.6910961740843682, v_out.d[16], 1e-9);
  EXPECT_NEAR(-0.3825273335251999, v_out.d[17], 1e-9);
  EXPECT_NEAR(-0.2786380379675274, v_out.d[18], 1e-9);
  EXPECT_NEAR(-0.4466374585572648, v_out.d[19], 1e-9);
  EXPECT_NEAR(-0.4660702796265611, v_out.d[20], 1e-9);
  EXPECT_NEAR(-0.7293052292183388, v_out.d[21], 1e-9);

  EXPECT_NEAR(-0.1422880291832889, v_out_ptr->d[0], 1e-9);
  EXPECT_NEAR(-0.9394051690724308, v_out_ptr->d[1], 1e-9);
  EXPECT_NEAR(0.3698524989190035, v_out_ptr->d[2], 1e-9);
  EXPECT_NEAR(-0.1392790828008142, v_out_ptr->d[3], 1e-9);
  EXPECT_NEAR(0.2528634102276163, v_out_ptr->d[4], 1e-9);
  EXPECT_NEAR(0.5374963657992248, v_out_ptr->d[5], 1e-9);
  EXPECT_NEAR(0.6820169785372605, v_out_ptr->d[6], 1e-9);
  EXPECT_NEAR(0.0942707782421619, v_out_ptr->d[7], 1e-9);
  EXPECT_NEAR(-0.2098217373721902, v_out_ptr->d[8], 1e-9);
  EXPECT_NEAR(0.4765845662175560, v_out_ptr->d[9], 1e-9);
  EXPECT_NEAR(0.8420882667264937, v_out_ptr->d[10], 1e-9);
  EXPECT_NEAR(0.6395749968188165, v_out_ptr->d[11], 1e-9);
  EXPECT_NEAR(0.3527996775516288, v_out_ptr->d[12], 1e-9);
  EXPECT_NEAR(-0.8276917793115173, v_out_ptr->d[13], 1e-9);
  EXPECT_NEAR(0.3994797259839673, v_out_ptr->d[14], 1e-9);
  EXPECT_NEAR(0.2135637625518051, v_out_ptr->d[15], 1e-9);
  EXPECT_NEAR(0.6910961740843682, v_out_ptr->d[16], 1e-9);
  EXPECT_NEAR(-0.3825273335251999, v_out_ptr->d[17], 1e-9);
  EXPECT_NEAR(-0.2786380379675274, v_out_ptr->d[18], 1e-9);
  EXPECT_NEAR(-0.4466374585572648, v_out_ptr->d[19], 1e-9);
  EXPECT_NEAR(-0.4660702796265611, v_out_ptr->d[20], 1e-9);
  EXPECT_NEAR(-0.7293052292183388, v_out_ptr->d[21], 1e-9);
}

TEST(VecLinComb3, Normal19) {
  double d_v0[22] = {
      -0.8573368202539113, -0.3095243540909678, -0.6223803024827901,
      0.0526286741366113,  -0.3273748383702419, 0.5569554080844599,
      0.6477298232541340,  0.8690623208807380,  0.1678139443763478,
      -0.7882803331137200, -0.1884974184203503, -0.4882413983791571,
      0.3394043581617932,  -0.1310480915138712, 0.0830281005852114,
      0.9373618401923818,  -0.5415135984592649, -0.2496150131864721,
      -0.2003749397337262, 0.9627916367163181,  0.5087436995593415,
      0.9786267712971097};
  const Vec v0 = {22, &d_v0[0], 0, 22};
  double d_v1[22] = {
      -0.5776822344582984, -0.4603259622755671, -0.1212378437915544,
      -0.1088051340237584, 0.9373917478316034,  -0.6689476839394894,
      -0.9058712657101948, 0.4042096608277306,  -0.4138936847449795,
      -0.8847737487701099, -0.9320016394351864, 0.3551338003058304,
      -0.8168923849046679, 0.2304640143969323,  0.2739301362970756,
      -0.9175958393065033, -0.6131017868490019, -0.3492931055223703,
      0.5135691423777353,  -0.9458708890805596, 0.3634707442764578,
      -0.3588290503146461};
  const Vec v1 = {22, &d_v1[0], 0, 22};
  double d_v2[22] = {
      -0.3863652717354071, -0.3864505344088873, -0.5162233393818663,
      0.3599644782289684,  -0.5030956002741067, -0.9530012097148668,
      -0.0197301064497755, 0.2129637841789760,  0.8137628741273200,
      -0.9279964235047859, 0.3270886212325799,  -0.3725613494607858,
      0.1500763513735379,  0.9325933374806799,  0.0424967113927124,
      0.8689023904713953,  -0.2046654504936161, -0.9675323024240181,
      0.9970156583499121,  0.4956236939433107,  -0.8129878226638094,
      -0.2233858869029943};
  const Vec v2 = {22, &d_v2[0], 0, 22};
  double d_v_out[22] = {
      0.2917533649241226,  0.1117484353990807,  0.9154040896204270,
      0.3335426197727331,  -0.1853482021473896, 0.6746933360783400,
      -0.0808684900127257, -0.1128920460401936, -0.5860784412297435,
      0.4877203190012356,  -0.6649991950428540, -0.8335313567898401,
      -0.2584103668462427, -0.2900231062465963, 0.0477445505657956,
      -0.8347657437670852, -0.5434381406571684, -0.3837843702076791,
      -0.9307359422697703, 0.7419365944369598,  0.3260330082599752,
      -0.3536222084357037};
  Vec v_out = {22, &d_v_out[0], 0, 22};
  const Vec *v_out_ptr =
      VecLinComb3(0.2375679606540950, &v0, -0.2639335236890503, &v1,
                  -0.6192471893033080, &v2, &v_out);

  EXPECT_NEAR(0.1880495562983144, v_out.d[0], 1e-9);
  EXPECT_NEAR(0.2872707909322408, v_out.d[1], 1e-9);
  EXPECT_NEAR(0.2038109640692408, v_out.d[2], 1e-9);
  EXPECT_NEAR(-0.1816867821874069, v_out.d[3], 1e-9);
  EXPECT_NEAR(-0.0136423433827140, v_out.d[4], 1e-9);
  EXPECT_NEAR(0.8990158003782567, v_out.d[5], 1e-9);
  EXPECT_NEAR(0.4051874612965523, v_out.d[6], 1e-9);
  EXPECT_NEAR(-0.0321003416146943, v_out.d[7], 1e-9);
  EXPECT_NEAR(-0.3548127373805284, v_out.d[8], 1e-9);
  EXPECT_NEAR(0.6209104789577695, v_out.d[9], 1e-9);
  EXPECT_NEAR(-0.0013431798539531, v_out.d[10], 1e-9);
  EXPECT_NEAR(0.0209853398809978, v_out.d[11], 1e-9);
  EXPECT_NEAR(0.2033025280592912, v_out.d[12], 1e-9);
  EXPECT_NEAR(-0.6694658102497703, v_out.d[13], 1e-9);
  EXPECT_NEAR(-0.0788904986690683, v_out.d[14], 1e-9);
  EXPECT_NEAR(-0.0731939191182861, v_out.d[15], 1e-9);
  EXPECT_NEAR(0.1599103386963491, v_out.d[16], 1e-9);
  EXPECT_NEAR(0.6320312893456866, v_out.d[17], 1e-9);
  EXPECT_NEAR(-0.8005499233290181, v_out.d[18], 1e-9);
  EXPECT_NEAR(0.1714619029529228, v_out.d[19], 1e-9);
  EXPECT_NEAR(0.5283695130275454, v_out.d[20], 1e-9);
  EXPECT_NEAR(0.4655284645448018, v_out.d[21], 1e-9);

  EXPECT_NEAR(0.1880495562983144, v_out_ptr->d[0], 1e-9);
  EXPECT_NEAR(0.2872707909322408, v_out_ptr->d[1], 1e-9);
  EXPECT_NEAR(0.2038109640692408, v_out_ptr->d[2], 1e-9);
  EXPECT_NEAR(-0.1816867821874069, v_out_ptr->d[3], 1e-9);
  EXPECT_NEAR(-0.0136423433827140, v_out_ptr->d[4], 1e-9);
  EXPECT_NEAR(0.8990158003782567, v_out_ptr->d[5], 1e-9);
  EXPECT_NEAR(0.4051874612965523, v_out_ptr->d[6], 1e-9);
  EXPECT_NEAR(-0.0321003416146943, v_out_ptr->d[7], 1e-9);
  EXPECT_NEAR(-0.3548127373805284, v_out_ptr->d[8], 1e-9);
  EXPECT_NEAR(0.6209104789577695, v_out_ptr->d[9], 1e-9);
  EXPECT_NEAR(-0.0013431798539531, v_out_ptr->d[10], 1e-9);
  EXPECT_NEAR(0.0209853398809978, v_out_ptr->d[11], 1e-9);
  EXPECT_NEAR(0.2033025280592912, v_out_ptr->d[12], 1e-9);
  EXPECT_NEAR(-0.6694658102497703, v_out_ptr->d[13], 1e-9);
  EXPECT_NEAR(-0.0788904986690683, v_out_ptr->d[14], 1e-9);
  EXPECT_NEAR(-0.0731939191182861, v_out_ptr->d[15], 1e-9);
  EXPECT_NEAR(0.1599103386963491, v_out_ptr->d[16], 1e-9);
  EXPECT_NEAR(0.6320312893456866, v_out_ptr->d[17], 1e-9);
  EXPECT_NEAR(-0.8005499233290181, v_out_ptr->d[18], 1e-9);
  EXPECT_NEAR(0.1714619029529228, v_out_ptr->d[19], 1e-9);
  EXPECT_NEAR(0.5283695130275454, v_out_ptr->d[20], 1e-9);
  EXPECT_NEAR(0.4655284645448018, v_out_ptr->d[21], 1e-9);
}

TEST(VecDot, Normal19) {
  double d_v0[22] = {
      -0.6541368766407174, 0.6629872788978894,  0.8873936686582209,
      -0.1181124000276468, 0.4813335632845592,  0.2724927306285929,
      0.1034968965923637,  -0.2335063196932647, 0.4208355507635118,
      -0.0869100862740853, 0.8358790326652219,  0.3064598023561576,
      0.0625194078860458,  -0.1963700973316833, 0.1647901519516979,
      0.9700565983100378,  0.4998104254832603,  0.9421569279260320,
      0.2046320651166780,  -0.8572594405026348, 0.2478070487889790,
      -0.2580299438752061};
  const Vec v0 = {22, &d_v0[0], 0, 22};
  double d_v1[22] = {
      0.3057100557920938,  -0.2710814209916268, 0.6067843534633952,
      0.3201890276418580,  -0.3034018756644636, -0.8802476812587017,
      -0.1138143563377840, 0.5914493368129734,  0.6711211581447385,
      -0.8970935748904403, 0.0651687425138585,  -0.0081418433448941,
      -0.3977651894175398, -0.5745985522600319, 0.6184629761330738,
      -0.5167319572529363, 0.4713355550461205,  0.9860661008705156,
      -0.0415012298621105, 0.1780287546238819,  0.2875217509280037,
      -0.2541962834702665};
  const Vec v1 = {22, &d_v1[0], 0, 22};
  double v_dot = VecDot(&v0, &v1);

  EXPECT_NEAR(0.8264910950589662, v_dot, 1e-9);
}

TEST(VecMult, Normal19) {
  double d_v0[22] = {
      -0.5556811814382803, 0.4781099875830956,  0.7604848197352572,
      0.8535932625387470,  0.9613149603025841,  -0.7043461824575523,
      0.0424991672673347,  -0.2493910850010415, -0.2654415832984514,
      0.2248106085525594,  -0.1131650003337357, -0.1155394623297057,
      -0.9669505873633093, 0.5679267859435062,  -0.1453566601998375,
      -0.1087089340624021, -0.0728127821037294, -0.2092145126732488,
      -0.6097977645808785, -0.9291699132315585, 0.7872387238822238,
      0.2490160873932286};
  const Vec v0 = {22, &d_v0[0], 0, 22};
  double d_v1[22] = {
      -0.8932738246259044, -0.2388801813758084, 0.9752280392577988,
      -0.8336621724886562, 0.0061417127481547,  0.1158520780086165,
      -0.7001522075840534, 0.6623956255855166,  -0.4031803430188825,
      0.4850156471025220,  0.1607510811236672,  -0.7407940090886842,
      -0.8262692137124925, -0.0557657226911692, 0.2128042419792808,
      0.3269559155650910,  -0.0108537961644251, -0.3542661210262974,
      0.5614056559931324,  -0.7486509178132050, 0.9727010510355476,
      -0.5914891620041254};
  const Vec v1 = {22, &d_v1[0], 0, 22};
  double d_v_out[22] = {
      -0.2634660286726134, -0.2798636100540746, 0.8491955861321394,
      -0.5244971957438658, -0.9873483562643692, -0.3417818989294295,
      0.0267719226454419,  -0.8102734784155359, 0.2500039738264899,
      -0.2676173902963752, -0.6074911994454428, -0.6288331368900011,
      0.7608778201909396,  0.2830702375594651,  -0.8821809877814477,
      -0.0928706429782951, -0.1443340908928405, -0.6615542103790897,
      -0.9401610539950502, -0.4283516530319897, -0.6943828679416237,
      -0.0386636603920769};
  Vec v_out = {22, &d_v_out[0], 0, 22};
  const Vec *v_out_ptr = VecMult(&v0, &v1, &v_out);

  EXPECT_NEAR(0.4963754542160138, v_out.d[0], 1e-9);
  EXPECT_NEAR(-0.1142110005514354, v_out.d[1], 1e-9);
  EXPECT_NEAR(0.7416461196357353, v_out.d[2], 1e-9);
  EXPECT_NEAR(-0.7116084136697317, v_out.d[3], 1e-9);
  EXPECT_NEAR(0.0059041203466822, v_out.d[4], 1e-9);
  EXPECT_NEAR(-0.0815999688751436, v_out.d[5], 1e-9);
  EXPECT_NEAR(-0.0297558857827083, v_out.d[6], 1e-9);
  EXPECT_NEAR(-0.1651955637647156, v_out.d[7], 1e-9);
  EXPECT_NEAR(0.1070208286057449, v_out.d[8], 1e-9);
  EXPECT_NEAR(0.1090366627826314, v_out.d[9], 1e-9);
  EXPECT_NEAR(-0.0181913961490082, v_out.d[10], 1e-9);
  EXPECT_NEAR(0.0855909415071737, v_out.d[11], 1e-9);
  EXPECT_NEAR(0.7989615015195144, v_out.d[12], 1e-9);
  EXPECT_NEAR(-0.0316708476538126, v_out.d[13], 1e-9);
  EXPECT_NEAR(-0.0309325138904663, v_out.d[14], 1e-9);
  EXPECT_NEAR(-0.0355430290664778, v_out.d[15], 1e-9);
  EXPECT_NEAR(0.0007902950951186, v_out.d[16], 1e-9);
  EXPECT_NEAR(0.0741176138671590, v_out.d[17], 1e-9);
  EXPECT_NEAR(-0.3423439140476738, v_out.d[18], 1e-9);
  EXPECT_NEAR(0.6956239083452224, v_out.d[19], 1e-9);
  EXPECT_NEAR(0.7657479341361224, v_out.d[20], 1e-9);
  EXPECT_NEAR(-0.1472903168577668, v_out.d[21], 1e-9);

  EXPECT_NEAR(0.4963754542160138, v_out_ptr->d[0], 1e-9);
  EXPECT_NEAR(-0.1142110005514354, v_out_ptr->d[1], 1e-9);
  EXPECT_NEAR(0.7416461196357353, v_out_ptr->d[2], 1e-9);
  EXPECT_NEAR(-0.7116084136697317, v_out_ptr->d[3], 1e-9);
  EXPECT_NEAR(0.0059041203466822, v_out_ptr->d[4], 1e-9);
  EXPECT_NEAR(-0.0815999688751436, v_out_ptr->d[5], 1e-9);
  EXPECT_NEAR(-0.0297558857827083, v_out_ptr->d[6], 1e-9);
  EXPECT_NEAR(-0.1651955637647156, v_out_ptr->d[7], 1e-9);
  EXPECT_NEAR(0.1070208286057449, v_out_ptr->d[8], 1e-9);
  EXPECT_NEAR(0.1090366627826314, v_out_ptr->d[9], 1e-9);
  EXPECT_NEAR(-0.0181913961490082, v_out_ptr->d[10], 1e-9);
  EXPECT_NEAR(0.0855909415071737, v_out_ptr->d[11], 1e-9);
  EXPECT_NEAR(0.7989615015195144, v_out_ptr->d[12], 1e-9);
  EXPECT_NEAR(-0.0316708476538126, v_out_ptr->d[13], 1e-9);
  EXPECT_NEAR(-0.0309325138904663, v_out_ptr->d[14], 1e-9);
  EXPECT_NEAR(-0.0355430290664778, v_out_ptr->d[15], 1e-9);
  EXPECT_NEAR(0.0007902950951186, v_out_ptr->d[16], 1e-9);
  EXPECT_NEAR(0.0741176138671590, v_out_ptr->d[17], 1e-9);
  EXPECT_NEAR(-0.3423439140476738, v_out_ptr->d[18], 1e-9);
  EXPECT_NEAR(0.6956239083452224, v_out_ptr->d[19], 1e-9);
  EXPECT_NEAR(0.7657479341361224, v_out_ptr->d[20], 1e-9);
  EXPECT_NEAR(-0.1472903168577668, v_out_ptr->d[21], 1e-9);
}

TEST(MatArrMult, Normal4) {
  double m1[3][10] = {
      {0.0212663982810339, 0.5001061709652783, 0.0136750154016715,
       0.1422545459025129, 0.0772797400261893, 0.2073855639259408,
       0.7286714664230285, 0.1137147695884373, 0.0307588245863429,
       0.2957375838757390},
      {0.6896099937128201, 0.4431475391505540, 0.0276507762181946,
       0.1455553465292179, 0.8121368430402656, 0.6371631180480314,
       0.8327851698836141, 0.7002736874402681, 0.5657936047933997,
       0.6234819817776441},
      {0.4919885028939213, 0.7343043011308045, 0.9412627768308213,
       0.5015038851176992, 0.3846873271753091, 0.2956572263755478,
       0.2431912745588644, 0.2878933786183501, 0.7500730036689808,
       0.2355536029000470}};
  double m2[10][7] = {
      {0.1824361981908923, 0.7559029097499337, 0.5716463008645467,
       0.0133292428161933, 0.2344607101935844, 0.8445763627994631,
       0.8920411605507118},
      {0.8413251433967899, 0.9539804195416104, 0.7579151288966645,
       0.1301343535145895, 0.5969342603542435, 0.5774096802703936,
       0.1968261070651068},
      {0.0062084824708137, 0.8189190354886705, 0.6580080850330124,
       0.8399209041503406, 0.5233105778074014, 0.9477391745062378,
       0.4379387959682177},
      {0.6526126267310454, 0.9660142665372682, 0.9071405471260575,
       0.4612019095149215, 0.6794934350522996, 0.3453486656456631,
       0.5201339064847298},
      {0.5876145911985086, 0.0486893014891411, 0.3675498807514909,
       0.4537754568579218, 0.0073305115536528, 0.7634720856833220,
       0.2277473606805529},
      {0.0620331914017177, 0.9230443431240203, 0.9429943198853293,
       0.4103770208249277, 0.0577137473763381, 0.7486073516080307,
       0.9641833898462868},
      {0.3766969116754463, 0.1553480474146214, 0.8336877805299724,
       0.6317915928156657, 0.6632255548472447, 0.3371387291526093,
       0.1688992964673791},
      {0.5675569547257123, 0.1363854298594471, 0.6983567961262022,
       0.9794836384235838, 0.2823239730150129, 0.0740757685061099,
       0.9330599330403420},
      {0.9704254943310817, 0.7557503233480093, 0.7990213070886386,
       0.7449450439552316, 0.3723870561591852, 0.5707161913855966,
       0.4816201626340684},
      {0.5087371234939781, 0.3529213786371157, 0.9125660734227068,
       0.1182417662646156, 0.5493802808363822, 0.1181749417542938,
       0.5517180190304022}};
  double m_out[3][7];
  double m_out2[3][7];
  Mat mn_m1 = {3, 10, &m1[0][0], 0, 0};
  Mat mn_m2 = {10, 7, &m2[0][0], 0, 0};
  Mat mn_m_out = {3, 7, &m_out2[0][0], 0, 0};
  MatArrMult(&m1[0][0], 3, 10, &m2[0][0], 7, &m_out[0][0]);
  MatMult(&mn_m1, &mn_m2, &mn_m_out);

  EXPECT_NEAR(m_out[0][0], 1.0951588966830257, 1e-9);
  EXPECT_NEAR(m_out2[0][0], 1.0951588966830257, 1e-9);
  EXPECT_NEAR(m_out[0][1], 1.0932990812569610, 1e-9);
  EXPECT_NEAR(m_out2[0][1], 1.0932990812569610, 1e-9);
  EXPECT_NEAR(m_out[0][2], 1.7345606179234032, 1e-9);
  EXPECT_NEAR(m_out2[0][2], 1.7345606179234032, 1e-9);
  EXPECT_NEAR(m_out[0][3], 0.8922648078353622, 1e-9);
  EXPECT_NEAR(m_out2[0][3], 0.8922648078353622, 1e-9);
  EXPECT_NEAR(m_out[0][4], 1.1091739786221202, 1e-9);
  EXPECT_NEAR(m_out2[0][4], 1.1091739786221202, 1e-9);
  EXPECT_NEAR(m_out[0][5], 0.8896565011530245, 1e-9);
  EXPECT_NEAR(m_out2[0][5], 0.8896565011530245, 1e-9);
  EXPECT_NEAR(m_out[0][6], 0.8220952760264167, 1e-9);
  EXPECT_NEAR(m_out2[0][6], 0.8220952760264167, 1e-9);
  EXPECT_NEAR(m_out[1][0], 2.6879544111614595, 1e-9);
  EXPECT_NEAR(m_out2[1][0], 2.6879544111614595, 1e-9);
  EXPECT_NEAR(m_out[1][1], 2.6074742501029626, 1e-9);
  EXPECT_NEAR(m_out2[1][1], 2.6074742501029626, 1e-9);
  EXPECT_NEAR(m_out[1][2], 3.9840301764862582, 1e-9);
  EXPECT_NEAR(m_out2[1][2], 3.9840301764862582, 1e-9);
  EXPECT_NEAR(m_out[1][3], 2.4944804762127419, 1e-9);
  EXPECT_NEAR(m_out2[1][3], 2.4944804762127419, 1e-9);
  EXPECT_NEAR(m_out[1][4], 1.8855680703484310, 1e-9);
  EXPECT_NEAR(m_out2[1][4], 1.8855680703484310, 1e-9);
  EXPECT_NEAR(m_out[1][5], 2.7410328149359731, 1e-9);
  EXPECT_NEAR(m_out2[1][5], 2.7410328149359731, 1e-9);
  EXPECT_NEAR(m_out[1][6], 3.0000432413686031, 1e-9);
  EXPECT_NEAR(m_out2[1][6], 3.0000432413686031, 1e-9);
  EXPECT_NEAR(m_out[2][0], 2.3877953315755902, 1e-9);
  EXPECT_NEAR(m_out2[2][0], 2.3877953315755902, 1e-9);
  EXPECT_NEAR(m_out[2][1], 3.3463638360012427, 1e-9);
  EXPECT_NEAR(m_out2[2][1], 3.3463638360012427, 1e-9);
  EXPECT_NEAR(m_out[2][2], 3.5503520688816805, 1e-9);
  EXPECT_NEAR(m_out2[2][2], 3.5503520688816805, 1e-9);
  EXPECT_NEAR(m_out[2][3], 2.4421379785388080, 1e-9);
  EXPECT_NEAR(m_out2[2][3], 2.4421379785388080, 1e-9);
  EXPECT_NEAR(m_out[2][4], 2.0582040279516871, 1e-9);
  EXPECT_NEAR(m_out2[2][4], 2.0582040279516871, 1e-9);
  EXPECT_NEAR(m_out[2][5], 2.9790412476668657, 1e-9);
  EXPECT_NEAR(m_out2[2][5], 2.9790412476668657, 1e-9);
  EXPECT_NEAR(m_out[2][6], 2.4300542852994891, 1e-9);
  EXPECT_NEAR(m_out2[2][6], 2.4300542852994891, 1e-9);
}

#if !defined(NDEBUG)

TEST(MatArrMultDeath, Death4) {
  double m1[4][4] = {{0.1125868963671079, 0.9674990239094460,
                      0.5543998460464290, 0.9833057743317446},
                     {0.4245785774172883, 0.8817676977773536,
                      0.2576347591534958, 0.5520077662160923},
                     {0.5328452539256002, 0.9226892105952087,
                      0.9011804796077909, 0.8754019148398485},
                     {0.0236967063939544, 0.1614602796004420,
                      0.7085537535017395, 0.0064275726489542}};
  double m_out[4][4] = {{0.0267050818615244, 0.6445465409499394,
                         0.2811262148815696, 0.8180443235296191},
                        {0.9923907580200908, 0.7027180579916781,
                         0.2387933264864496, 0.8256109515538397},
                        {0.9147336863539686, 0.5427479632597990,
                         0.2003874118078430, 0.5350854910894467},
                        {0.5567354092324736, 0.9935225001129042,
                         0.2640007611896548, 0.0379596928484023}};
  ASSERT_DEATH(MatArrMult(&m1[0][0], 4, 4, &m_out[0][0], 4, &m_out[0][0]),
               "MatArrMult: Assertion `m_out != m1 && m_out != m2' failed.");
  ASSERT_DEATH(MatArrMult(&m_out[0][0], 4, 4, &m1[0][0], 4, &m_out[0][0]),
               "MatArrMult: Assertion `m_out != m1 && m_out != m2' failed.");
}

#endif  // !defined(NDEBUG)

TEST(MatArrZero, Normal4) {
  double m[4][9] = {
      {0.3739221657524540, 0.6531386966243239, 0.9918238133360220,
       0.1454009721520160, 0.7769764346118496, 0.1428454710136994,
       0.7575502365981188, 0.4404812450625154, 0.9996209268943256},
      {0.0657622483433286, 0.5442730515228016, 0.7036322843633684,
       0.1114210864043648, 0.2386144155283532, 0.9480395139519537,
       0.2701618763380824, 0.5255902828159678, 0.2907711923697116},
      {0.2712502540383011, 0.0947564413161806, 0.5832899699214481,
       0.7569401094660414, 0.2411542416720936, 0.0838182117692806,
       0.0695171501115106, 0.0866831035785719, 0.6434867482710138},
      {0.2590886635690861, 0.7017609236714308, 0.7671698643566480,
       0.4455869704865372, 0.6159213197703541, 0.4301695543760448,
       0.4261329609695383, 0.3956366304397368, 0.5455373688519676}};
  MatArrZero(4, 9, &m[0][0]);

  EXPECT_NEAR(m[0][0], 0.0, 1e-9);
  EXPECT_NEAR(m[0][1], 0.0, 1e-9);
  EXPECT_NEAR(m[0][2], 0.0, 1e-9);
  EXPECT_NEAR(m[0][3], 0.0, 1e-9);
  EXPECT_NEAR(m[0][4], 0.0, 1e-9);
  EXPECT_NEAR(m[0][5], 0.0, 1e-9);
  EXPECT_NEAR(m[0][6], 0.0, 1e-9);
  EXPECT_NEAR(m[0][7], 0.0, 1e-9);
  EXPECT_NEAR(m[0][8], 0.0, 1e-9);
  EXPECT_NEAR(m[1][0], 0.0, 1e-9);
  EXPECT_NEAR(m[1][1], 0.0, 1e-9);
  EXPECT_NEAR(m[1][2], 0.0, 1e-9);
  EXPECT_NEAR(m[1][3], 0.0, 1e-9);
  EXPECT_NEAR(m[1][4], 0.0, 1e-9);
  EXPECT_NEAR(m[1][5], 0.0, 1e-9);
  EXPECT_NEAR(m[1][6], 0.0, 1e-9);
  EXPECT_NEAR(m[1][7], 0.0, 1e-9);
  EXPECT_NEAR(m[1][8], 0.0, 1e-9);
  EXPECT_NEAR(m[2][0], 0.0, 1e-9);
  EXPECT_NEAR(m[2][1], 0.0, 1e-9);
  EXPECT_NEAR(m[2][2], 0.0, 1e-9);
  EXPECT_NEAR(m[2][3], 0.0, 1e-9);
  EXPECT_NEAR(m[2][4], 0.0, 1e-9);
  EXPECT_NEAR(m[2][5], 0.0, 1e-9);
  EXPECT_NEAR(m[2][6], 0.0, 1e-9);
  EXPECT_NEAR(m[2][7], 0.0, 1e-9);
  EXPECT_NEAR(m[2][8], 0.0, 1e-9);
  EXPECT_NEAR(m[3][0], 0.0, 1e-9);
  EXPECT_NEAR(m[3][1], 0.0, 1e-9);
  EXPECT_NEAR(m[3][2], 0.0, 1e-9);
  EXPECT_NEAR(m[3][3], 0.0, 1e-9);
  EXPECT_NEAR(m[3][4], 0.0, 1e-9);
  EXPECT_NEAR(m[3][5], 0.0, 1e-9);
  EXPECT_NEAR(m[3][6], 0.0, 1e-9);
  EXPECT_NEAR(m[3][7], 0.0, 1e-9);
  EXPECT_NEAR(m[3][8], 0.0, 1e-9);
}

TEST(MatArrCopy, Normal4) {
  double m[7][6] = {
      {0.3365281109558904, 0.0737001483357042, 0.0398775963689120,
       0.3706055041088927, 0.1869349540100423, 0.1025629304343716},
      {0.4552206369647379, 0.3257254821781085, 0.5268159339701488,
       0.5868375401640828, 0.5279214164505385, 0.7268883882505669},
      {0.7738322472304904, 0.2242638438929699, 0.9695845469640529,
       0.5376520994850909, 0.3705577418805630, 0.3212228778465703},
      {0.3421694040481019, 0.3402004546344495, 0.7902552025409523,
       0.6532732298194461, 0.2997954968859535, 0.7976361892824924},
      {0.5076242404996422, 0.9719341817710336, 0.1990571742562754,
       0.1623544331292072, 0.7766251339282274, 0.6577503347050410},
      {0.6624721717107778, 0.1359628706013299, 0.5710178217253654,
       0.0219631997054428, 0.4320626196773696, 0.6579539733616591},
      {0.0678682239037411, 0.6916711237694313, 0.3613529313780218,
       0.5138049870907301, 0.3542492511781959, 0.4998475510543675}};
  double m_out[7][6];
  MatArrCopy(&m[0][0], 7, 6, &m_out[0][0]);
  MatArrCopy(&m[0][0], 7, 6, &m[0][0]);

  EXPECT_NEAR(m_out[0][0], m[0][0], 1e-9);
  EXPECT_NEAR(m_out[0][1], m[0][1], 1e-9);
  EXPECT_NEAR(m_out[0][2], m[0][2], 1e-9);
  EXPECT_NEAR(m_out[0][3], m[0][3], 1e-9);
  EXPECT_NEAR(m_out[0][4], m[0][4], 1e-9);
  EXPECT_NEAR(m_out[0][5], m[0][5], 1e-9);
  EXPECT_NEAR(m_out[1][0], m[1][0], 1e-9);
  EXPECT_NEAR(m_out[1][1], m[1][1], 1e-9);
  EXPECT_NEAR(m_out[1][2], m[1][2], 1e-9);
  EXPECT_NEAR(m_out[1][3], m[1][3], 1e-9);
  EXPECT_NEAR(m_out[1][4], m[1][4], 1e-9);
  EXPECT_NEAR(m_out[1][5], m[1][5], 1e-9);
  EXPECT_NEAR(m_out[2][0], m[2][0], 1e-9);
  EXPECT_NEAR(m_out[2][1], m[2][1], 1e-9);
  EXPECT_NEAR(m_out[2][2], m[2][2], 1e-9);
  EXPECT_NEAR(m_out[2][3], m[2][3], 1e-9);
  EXPECT_NEAR(m_out[2][4], m[2][4], 1e-9);
  EXPECT_NEAR(m_out[2][5], m[2][5], 1e-9);
  EXPECT_NEAR(m_out[3][0], m[3][0], 1e-9);
  EXPECT_NEAR(m_out[3][1], m[3][1], 1e-9);
  EXPECT_NEAR(m_out[3][2], m[3][2], 1e-9);
  EXPECT_NEAR(m_out[3][3], m[3][3], 1e-9);
  EXPECT_NEAR(m_out[3][4], m[3][4], 1e-9);
  EXPECT_NEAR(m_out[3][5], m[3][5], 1e-9);
  EXPECT_NEAR(m_out[4][0], m[4][0], 1e-9);
  EXPECT_NEAR(m_out[4][1], m[4][1], 1e-9);
  EXPECT_NEAR(m_out[4][2], m[4][2], 1e-9);
  EXPECT_NEAR(m_out[4][3], m[4][3], 1e-9);
  EXPECT_NEAR(m_out[4][4], m[4][4], 1e-9);
  EXPECT_NEAR(m_out[4][5], m[4][5], 1e-9);
  EXPECT_NEAR(m_out[5][0], m[5][0], 1e-9);
  EXPECT_NEAR(m_out[5][1], m[5][1], 1e-9);
  EXPECT_NEAR(m_out[5][2], m[5][2], 1e-9);
  EXPECT_NEAR(m_out[5][3], m[5][3], 1e-9);
  EXPECT_NEAR(m_out[5][4], m[5][4], 1e-9);
  EXPECT_NEAR(m_out[5][5], m[5][5], 1e-9);
  EXPECT_NEAR(m_out[6][0], m[6][0], 1e-9);
  EXPECT_NEAR(m_out[6][1], m[6][1], 1e-9);
  EXPECT_NEAR(m_out[6][2], m[6][2], 1e-9);
  EXPECT_NEAR(m_out[6][3], m[6][3], 1e-9);
  EXPECT_NEAR(m_out[6][4], m[6][4], 1e-9);
  EXPECT_NEAR(m_out[6][5], m[6][5], 1e-9);
}

TEST(MatVecMult, Normal4) {
  double m[8][2] = {{0.5787049315788571, 0.9963095435344906},
                    {0.0023668561719622, 0.5763197385719523},
                    {0.9652797887499762, 0.4755331571616471},
                    {0.8273286246649642, 0.3140438611910976},
                    {0.1181605851883888, 0.4345520503338400},
                    {0.5901748880115287, 0.4237742540125820},
                    {0.9878311854535364, 0.8051140615603157},
                    {0.4285865065826227, 0.4492664162917003}};
  double v[2] = {0.6553256183090724, 0.5612429265606003};
  double v_out[8];
  Mat mn_m = {8, 2, &m[0][0], 0, 0};
  Vec n_v = {2, v, 0, 0};
  Vec n_v_out = {8, v_out, 0, 0};
  MatVecMult(&mn_m, &n_v, &n_v_out);

  EXPECT_NEAR(v_out[0], 0.9384118510789773, 1e-9);
  EXPECT_NEAR(v_out[1], 0.3250064381951023, 1e-9);
  EXPECT_NEAR(v_out[2], 0.8994621952058337, 1e-9);
  EXPECT_NEAR(v_out[3], 0.7184245382266448, 1e-9);
  EXPECT_NEAR(v_out[4], 0.3213229230206164, 1e-9);
  EXPECT_NEAR(v_out[5], 0.6245970259196993, 1e-9);
  EXPECT_NEAR(v_out[6], 1.0992156545175256, 1e-9);
  EXPECT_NEAR(v_out[7], 0.5330113157101293, 1e-9);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

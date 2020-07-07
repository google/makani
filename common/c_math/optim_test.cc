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

#include <string>

#include "common/c_math/linalg.h"
#include "common/c_math/optim.h"
#include "common/c_math/util.h"
#include "common/c_math/util/linalg_io.h"
#include "common/macros.h"
#include "lib/util/test_util.h"

using ::test_util::Rand;
using ::test_util::RandNormal;

// Utility Functions for optim_test.cc.

// Generate a random 4x4 orthonormal basis.
// Ref: Alan Genz, Methods for Generating Random Orthogonal Matrices
const Mat *rand_orthonormal_mat_n4(Mat *A) {
  MAT_INIT(4, 4, Q1, {{0}});
  MAT_INIT(4, 4, Q2, {{0}});
  MAT_INIT(4, 4, I, {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}});

  double th1 = Rand(-2 * M_PI, 2 * M_PI);
  double th2 = Rand(-2 * M_PI, 2 * M_PI);
  double th3 = Rand(-2 * M_PI, 2 * M_PI);
  *MatPtr(&Q1, 0, 0) = cos(th1);
  *MatPtr(&Q1, 0, 1) = -sin(th1);
  *MatPtr(&Q1, 1, 0) = sin(th1);
  *MatPtr(&Q1, 1, 1) = cos(th1);
  *MatPtr(&Q1, 2, 2) = cos(th3);
  *MatPtr(&Q1, 2, 3) = -sin(th3);
  *MatPtr(&Q1, 3, 2) = sin(th3);
  *MatPtr(&Q1, 3, 3) = cos(th3);

  double c2 = cos(th2), s2 = sin(th2);
  *MatPtr(&Q2, 0, 0) = c2;
  *MatPtr(&Q2, 1, 1) = c2;
  *MatPtr(&Q2, 2, 2) = c2;
  *MatPtr(&Q2, 3, 3) = c2;
  *MatPtr(&Q2, 0, 2) = -s2;
  *MatPtr(&Q2, 1, 3) = -s2;
  *MatPtr(&Q2, 2, 0) = s2;
  *MatPtr(&Q2, 3, 1) = s2;

  // A should be orthonormal
  MatMult(&Q1, &Q2, A);

  // Confirm that A is orthonormal
  MAT_INIT(4, 4, A_trans, {{0}});
  MAT_INIT(4, 4, I_test, {{0}});
  MatMult(MatTrans(A, &A_trans), A, &I_test);
  for (int32_t j = 0; j < I.nr; ++j) {
    for (int32_t k = 0; k < I.nc; ++k) {
      assert(fabs(*MatPtr(&I, j, k) - *MatPtr(&I_test, j, k)) < 1e-9);
    }
  }
  return A;
}

// Tests.

static double SinFunc(double x, const void * /*context*/) { return sin(x); }
static double CosFunc(double x, const void * /*context*/) { return cos(x); }

TEST(Newton, SinNormal) {
  const double tol = 1e-9;
  for (double x0 = -M_PI / 2.0; x0 < M_PI / 2.0; x0 += 0.1) {
    EXPECT_NEAR(Newton(&SinFunc, &CosFunc, nullptr, x0, -M_PI / 2.0, M_PI / 2.0,
                       tol, 10),
                0.0, tol);
  }
  for (double x0 = M_PI / 2.0; x0 < 3.0 * M_PI / 2.0; x0 += 0.1) {
    EXPECT_NEAR(Newton(&SinFunc, &CosFunc, nullptr, x0, M_PI / 2.0,
                       3.0 * M_PI / 2.0, tol, 10),
                M_PI, tol);
  }
  for (double x0 = -3.0 * M_PI / 2.0; x0 < -M_PI / 2.0; x0 += 0.1) {
    EXPECT_NEAR(Newton(&SinFunc, &CosFunc, nullptr, x0, -3.0 * M_PI / 2.0,
                       -M_PI / 2.0, tol, 10),
                -M_PI, tol);
  }
}

TEST(Newton, SolutionAtBounds) {
  const double tol = 1e-9;
  for (double x0 = 0.0; x0 < M_PI / 2.0; x0 += 0.1) {
    EXPECT_NEAR(
        Newton(&SinFunc, &CosFunc, nullptr, x0, 0.0, M_PI / 2.0, tol, 10), 0.0,
        tol);
  }
  for (double x0 = M_PI / 2.0; x0 < PI; x0 += 0.1) {
    EXPECT_NEAR(
        Newton(&SinFunc, &CosFunc, nullptr, x0, M_PI / 2.0, PI, tol, 10), PI,
        tol);
  }
}

static double SimpleQuadratic(double x, const void * /*context*/) {
  double f_coeffs[] = {1.0, 0.0, -4.0};
  return PolyVal(f_coeffs, x, ARRAYSIZE(f_coeffs));
}

static double SimpleQuadraticDeriv(double x, const void * /*context*/) {
  double df_coeffs[] = {2.0, 0.0};
  return PolyVal(df_coeffs, x, ARRAYSIZE(df_coeffs));
}

TEST(Newton, ZeroDerivative) {
  const double tol = 1e-9;
  EXPECT_NEAR(Newton(&SimpleQuadratic, &SimpleQuadraticDeriv, nullptr, 0.0,
                     -1.9, 5.0, tol, 10),
              2.0, tol);
  for (double x0 = -0.1; x0 < 0.1; x0 += 0.001) {
    EXPECT_NEAR(Newton(&SimpleQuadratic, &SimpleQuadraticDeriv, nullptr, x0,
                       -5.0, 5.0, tol, 10),
                Sign(x0) * 2.0, tol);
  }
}

TEST(LeastSquares, CompareWithLeftDivide) {
  for (int32_t k = 0; k < 222; ++k) {
    MAT_INIT(
        4, 5, A,
        {{RandNormal(), RandNormal(), RandNormal(), RandNormal(), RandNormal()},
         {RandNormal(), RandNormal(), RandNormal(), RandNormal(), RandNormal()},
         {RandNormal(), RandNormal(), RandNormal(), RandNormal(), RandNormal()},
         {RandNormal(), RandNormal(), RandNormal(), RandNormal(),
          RandNormal()}});
    VEC_INIT(4, b, {RandNormal(), RandNormal(), RandNormal(), RandNormal()});
    VEC_INIT(5, x_pinv, {0});
    VEC_INIT(5, x_left_divide, {0});
    VEC_INIT(4, tmp, {0});

    LeastSquares(&A, &b, &x_pinv);
    MatVecLeftDivide(&A, &b, &x_left_divide);
    double norm_pinv = VecNorm(VecSub(&b, MatVecMult(&A, &x_pinv, &tmp), &tmp));
    double norm_left_divide =
        VecNorm(VecSub(&b, MatVecMult(&A, &x_left_divide, &tmp), &tmp));
    EXPECT_NEAR(norm_pinv, norm_left_divide, 1e-9);
  }
}

TEST(LeastSquares, OverConstrainedRankDeficient) {
  MAT_INIT(5, 4, A, {{-0.18, 0.11, -0.29, -0.14},
                     {0.34, -0.55, 0.75, 0.63},
                     {-0.10, 1.87, -1.23, -2.01},
                     {-0.20, -2.86, 1.44, 3.03},
                     {0.02, -0.11, 0.09, 0.12}});
  VEC_INIT(5, b, {1.1, -1.1, 0.0, 0.6, 1.1});
  VEC_INIT(4, x_ans, {-1.475628609649396, -0.534334567864229,
                      -1.528792608323792, 0.386312894012433});
  VEC_INIT(4, x, {0});
  LeastSquares(&A, &b, &x);
  for (int32_t i = 0; i < x.length; ++i) {
    EXPECT_NEAR(VecGet(&x, i), VecGet(&x_ans, i), 1e-9);
  }
}

TEST(LeastSquares, UnderConstrainedRankDeficient) {
  MAT_INIT(4, 5, A, {{-1.29, 0.34, -2.65, -0.08, 1.77},
                     {2.14, -3.44, 1.06, -0.50, 0.86},
                     {-0.24, 2.04, 1.80, 0.42, -2.28},
                     {0.77, -0.42, 1.33, 0.0, -0.77}});
  VEC_INIT(4, b, {0.2, 1.6, -0.8, 0.7});
  VEC_INIT(5, x_ans, {0.161260087781396, -0.305341532506653, 0.026377629456377,
                      -0.047823926617696, 0.125683471924780});
  VEC_INIT(5, x, {0});
  LeastSquares(&A, &b, &x);
  for (int32_t i = 0; i < x.length; ++i) {
    EXPECT_NEAR(VecGet(&x, i), VecGet(&x_ans, i), 1e-9);
  }
}

TEST(BoundedLeastSquares, Normal_0) {
  MAT_INIT(2, 3, A, {{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}});
  VEC_INIT(2, b, {1.0, 2.0});
  VEC_INIT(3, l, {-1.0, -1.0, -1.0});
  VEC_INIT(3, u, {1.0, 1.0, 1.0 / 6.0});
  VEC_INIT(3, x_ans, {-1.0 / 6.0, 1.0 / 3.0, 1.0 / 6.0});
  VEC_INIT(3, x, {0});

  BoundedLeastSquares(&A, &b, &l, &u, &x);

  VEC_INIT(2, tmp, {0});
  double res_norm = VecNorm(VecSub(&b, MatVecMult(&A, &x, &tmp), &tmp));
  double res_norm_ans = VecNorm(VecSub(&b, MatVecMult(&A, &x_ans, &tmp), &tmp));
  EXPECT_NEAR(res_norm, res_norm_ans, 1e-9);
}

TEST(BoundedLeastSquares, Normal_1) {
  MAT_INIT(2, 3, A, {{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}});
  VEC_INIT(2, b, {1.0, 2.0});
  VEC_INIT(3, l, {0.0, 0.0, 0.0});
  VEC_INIT(3, u, {1.0, 1.0, 1.0 / 6.0});
  VEC_INIT(3, x_ans, {0.0, 0.206896551724138, 1.0 / 6.0});
  VEC_INIT(3, x, {0});

  BoundedLeastSquares(&A, &b, &l, &u, &x);
  for (int32_t i = 0; i < 3; ++i) {
    EXPECT_NEAR(x.d[i], x_ans.d[i], 1e-9);
  }
}

TEST(BoundedLeastSquares, Normal_2) {
  MAT_INIT(4, 3, A, {{0.3, -0.4, 0.5},
                     {-0.4, 0.4, -0.4},
                     {-0.2, 0.2, 0.0},
                     {-0.4, -0.1, -0.1}});
  VEC_INIT(4, b, {0.3, 0.3, -0.3, 0.0});
  VEC_INIT(3, l, {-1.0, -1.0, -0.2});
  VEC_INIT(3, u, {0.001, 1.0, 1.0});
  VEC_INIT(3, x_ans, {0.001, -0.350594594594595, -0.2});
  VEC_INIT(3, x, {0});

  BoundedLeastSquares(&A, &b, &l, &u, &x);
  for (int32_t i = 0; i < 3; ++i) EXPECT_NEAR(x.d[i], x_ans.d[i], 1e-9);
}

TEST(BoundedLeastSquares, No_bound1) {
  MAT_INIT(4, 2, A, {{1, 1}, {-1, 1}, {0.001, 1}, {-1, 1}});
  VEC_INIT(4, b, {-1, -1, 1, -3});
  VEC_INIT(2, l, {-10000, -10000});
  VEC_INIT(2, u, {10000, 10000});
  VEC_INIT(2, x_ans, {0.7278674620, -0.8182151014});
  VEC_INIT(2, x, {0});

  BoundedLeastSquares(&A, &b, &l, &u, &x);
  for (int32_t i = 0; i < x.length; ++i) {
    EXPECT_NEAR(x.d[i], x_ans.d[i], 1e-9);
  }
}

TEST(BoundedLeastSquares, ManyElementsNoBounds) {
  MAT_INIT(16, 3, A, {{0}});
  VEC_INIT(16, b, {0});
  VEC_INIT(3, l, {-10000, -10000, -10000});
  VEC_INIT(3, u, {10000, 10000, 10000});
  VEC_INIT(3, x_ans, {-0.4108930811, -0.1096526686, 0.2316469899});
  VEC_INIT(3, x, {0});

  // Populate A and b with a chaotic sequence to remove dependence on
  // language-specfic random function.
  double c = 0.4;
  for (int32_t i = 0; i < b.length; ++i) {
    c = c * c - 2.0;
    *VecPtr(&b, i) = c;
  }
  c = 0.5;
  for (int32_t i = 0; i < A.nr; ++i) {
    for (int32_t j = 0; j < A.nc; ++j) {
      c = c * c - 2.0;
      *MatPtr(&A, i, j) = c;
    }
  }

  BoundedLeastSquares(&A, &b, &l, &u, &x);
  for (int32_t i = 0; i < x.length; ++i) {
    EXPECT_NEAR(x.d[i], x_ans.d[i], 1e-9);
  }
}

TEST(BoundedLeastSquares, OrthonormalA) {
  // If A is orthonormal, then pinv(A) = A', so x is easy to find: x = A'*b
  MAT_INIT(4, 4, A, {{0}});
  A.variable_dim = 1;
  VEC_INIT(4, b, {0, 0, 0, 0});
  VEC_INIT(3, l, {-INFINITY, -INFINITY, -INFINITY});
  VEC_INIT(3, u, {INFINITY, INFINITY, INFINITY});
  VEC_INIT(3, x_ans_unbounded, {0, 0, 0});
  VEC_INIT(3, x_bounded, {0, 0, 0});
  VEC_INIT(3, x_diff, {0, 0, 0});

  for (int32_t i = 0; i < 222; ++i) {
    rand_orthonormal_mat_n4(&A);

    // Truncate A to create a least squares problem
    int32_t r[4] = {1, 1, 1, 1};
    int32_t c[4] = {1, 1, 1, 0};
    MatSlice(&A, r, c, &A);

    // Randomize a b vector
    for (int32_t j = 0; j < b.length; ++j) {
      *VecPtr(&b, j) = Rand(-10, 10);
    }

    // Generate lower and upper limit
    *VecPtr(&l, 0) = Rand(-30, 30);
    *VecPtr(&u, 0) = *VecPtr(&l, 0) + Rand(0, 40);

    MatTransVecMult(&A, &b, &x_ans_unbounded);
    BoundedLeastSquares(&A, &b, &l, &u, &x_bounded);

    // Find the vector pointing from the bounded to unbounded solution
    VecSub(&x_bounded, &x_ans_unbounded, &x_diff);

    bool equiv = true;
    for (int32_t j = 0; j < x_ans_unbounded.length; ++j) {
      if (fabs(VecGet(&x_diff, j)) > 1e-9) {
        equiv = false;
      }
    }
    if (equiv) {
      // unbounded solutions are equivalent
      EXPECT_EQ(1, 1);
    } else {
      // Since only the x bound is active (others are set to INF) and since we
      // know the system to be a convex paraboloid, we know that the vector
      // from the bounded to unbounded solution must be parallel to the x-axis
      EXPECT_NEAR(VecGet(&x_diff, 1), 0.0, 1e-9);
      EXPECT_NEAR(VecGet(&x_diff, 2), 0.0, 1e-9);
    }
  }
}

TEST(ConstrainedLeastSquares, OrthonormalA) {
  // If A is orthonormal, then pinv(A) = A', so x is easy to find: x = A'*b
  MAT_INIT(4, 4, A, {{0}});
  A.variable_dim = 1;
  VEC_INIT(4, b, {0});
  VEC_INIT(1, l, {0});
  VEC_INIT(1, u, {0});
  VEC_INIT(3, x0, {0, 0, 0});
  VEC_INIT(3, x_ans_unconstrained, {0});
  VEC_INIT(3, x_constrained, {0});
  VEC_INIT(3, x_diff, {0});
  MAT_INIT(1, 3, C, {{0}});

  for (int32_t i = 0; i < 222; ++i) {
    rand_orthonormal_mat_n4(&A);

    // Truncate A to create a least squares problem
    int32_t row[4] = {1, 1, 1, 1};
    int32_t col[4] = {1, 1, 1, 0};
    MatSlice(&A, row, col, &A);

    // Randomize a b-vector
    for (int32_t j = 0; j < b.length; ++j) {
      *VecPtr(&b, j) = Rand(-10, 10);
    }

    // Randomize a single constraint
    for (int32_t j = 0; j < C.nc; ++j) {
      *MatPtr(&C, 0, j) = Rand(-10, 10);
    }

    *VecPtr(&l, 0) = Rand(-30, 0);
    *VecPtr(&u, 0) = Rand(0, 30);
    // TODO: When x0 can be outside of the constraints, use
    // this following:
    // *VecPtr(&l, 0) = Rand(-30, 30);
    // if (i < 10)
    //   *VecPtr(&u, 0) = *VecPtr(&l, 0);
    // else
    //   *VecPtr(&u, 0) = *VecPtr(&l, 0) + Rand(0, 40);

    MatTransVecMult(&A, &b, &x_ans_unconstrained);
    ConstrainedLeastSquares(&A, &b, &C, &l, &u, &x0, &x_constrained);

    // Find the vector pointing from the bounded to unbounded
    // solution.
    VecSub(&x_constrained, &x_ans_unconstrained, &x_diff);

    bool equiv = true;
    for (int32_t j = 0; j < x_ans_unconstrained.length; ++j) {
      if (fabs(VecGet(&x_diff, j)) > 1e-9) {
        equiv = false;
      }
    }
    if (equiv) {
      // The unbounded solutions are equivalent.
      EXPECT_EQ(1, 1);
    } else {
      // This is setup so that:
      // 1) There is only one constraint.
      // 2) We know the system to be a convex paraboloid with circular contour
      //    because the A matrix is orthonormal.
      // Therefore, the vector from the bounded to unbounded solution must be
      // perpendicular to the constraint, i.e.
      //
      //   (c'*c)*(x_diff'*x_diff) == (c'*x_diff)*(c'*x_diff)
      //
      // where, c is the first row of the the C matrix.
      VEC_INIT(3, c, {0});
      for (int32_t j = 0; j < C.nc; ++j) {
        *VecPtr(&c, j) = MatGet(&C, 0, j);
      }

      EXPECT_NEAR(VecDot(&c, &c) * VecDot(&x_diff, &x_diff),
                  VecDot(&c, &x_diff) * VecDot(&c, &x_diff), 1e-9);
    }
  }
}

TEST(ConstrainedLeastSquares, CompareToOctave) {
  std::string path =
      test_util::TestRunfilesDir() +
      "/common/c_math/test_data/constrained_least_squares_comparison.dat";
  FILE *f = fopen(path.c_str(), "r");
  assert(f != NULL);

  int32_t num_problems;
  int32_t scanned = fscanf(f, "%d\n", &num_problems);
  EXPECT_EQ(1, scanned);
  EXPECT_LT(0, num_problems);
  for (int32_t i = 0; i < num_problems; ++i) {
    int32_t nx, nb, nc;
    scanned = fscanf(f, "%d\n%d\n%d\n", &nx, &nb, &nc);
    EXPECT_EQ(3, scanned);
    MAT(nb, nx, A);
    VEC(nb, b);
    MAT(nc, nx, C);
    VEC(nc, lower);
    VEC(nc, upper);
    VEC(nx, x0);
    VEC(nx, x_ans);

    MatScan(f, &A);
    VecScan(f, &b);
    MatScan(f, &C);
    VecScan(f, &lower);
    VecScan(f, &upper);
    VecScan(f, &x0);
    VecScan(f, &x_ans);

    VEC(nb, r_ans);
    MatVecGenMult(kNoTrans, 1.0, &A, &x_ans, -1.0, &b, &r_ans);

    VEC(nx, x);
    ConstrainedLeastSquares(&A, &b, &C, &lower, &upper, &x0, &x);

    // Check the solution satisfies the constraints.
    EXPECT_EQ(IsWithinConstraints(&C, &x, &lower, &upper, DBL_TOL), 1);
    // Check that a similar optimal value is achieved.
    VEC(nb, r);
    MatVecGenMult(kNoTrans, 1.0, &A, &x, -1.0, &b, &r);
    EXPECT_LT(VecNorm(&r), 1.001 * VecNorm(&r_ans));
  }
  fclose(f);
}

TEST(BoundedLeastSquares, ColumnA) {
  // If A is a column vector, then pinv(A) = A'/(A'A), so x is easy to find,
  // x = A'/(A'A)*b
  MAT_INIT(8, 1, A, {{0}});
  VEC_INIT(8, b, {0});
  VEC_INIT(1, l, {0});
  VEC_INIT(1, u, {0});
  VEC_INIT(1, x_ans, {0});
  VEC_INIT(1, x, {0});
  VEC_INIT(1, x_constrained, {0});
  VEC_INIT(1, x0, {0});
  MAT_INIT(1, 1, A_mag, {{0}});
  MAT_INIT(1, 8, A_trans, {{0}});
  A_mag.variable_dim = 0;
  A_trans.variable_dim = 0;
  MAT_INIT(1, 1, C, {{1}});
  double rmax = 10;
  for (int32_t i = 0; i < 222; ++i) {
    // Generate A.
    for (int32_t j = 0; j < A.nr; ++j) {
      *MatPtr(&A, j, 0) = Rand(-rmax, rmax);
    }
    // Generate b.
    for (int32_t j = 0; j < b.length; ++j) {
      *VecPtr(&b, j) = Rand(-rmax, rmax);
    }

    // Generate lower and upper limit
    *VecPtr(&l, 0) = Rand(-rmax, rmax);
    *VecPtr(&u, 0) = VecGet(&l, 0) + Rand(0.1, rmax);

    // Generate initial guess
    BoundedLeastSquares(&A, &b, &l, &u, &x);
    *VecPtr(&x0, 0) = (VecGet(&l, 0) + VecGet(&u, 0)) / 2.0;
    ConstrainedLeastSquares(&A, &b, &C, &l, &u, &x0, &x_constrained);
    // TODO: Substitude &x0 for &l when ConstrainedLeastSquares
    // can accept it.

    // x_ans = Saturate(A'b/(A'A), l, u)
    MatMult(MatTrans(&A, &A_trans), &A, &A_mag);
    MatTransVecMult(&A, &b, &x_ans);
    x_ans.d[0] /= MatGet(&A_mag, 0, 0);
    x_ans.d[0] = Saturate(x_ans.d[0], l.d[0], u.d[0]);

    EXPECT_NEAR(x.d[0], x_ans.d[0], 1e-9);
    EXPECT_NEAR(x_constrained.d[0], x_ans.d[0], 1e-9);
  }
}

TEST(ConstrainedLeastSquares, Normal_0) {
  MAT_INIT(2, 3, A, {{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}});
  VEC_INIT(2, b, {1.0, 2.0});
  MAT_INIT(3, 3, C, {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}});
  VEC_INIT(3, lower, {-1.0, -1.0, -1.0});
  VEC_INIT(3, upper, {1.0, 1.0, 1.0 / 6.0});
  VEC_INIT(3, x_ans, {-1.0 / 6.0, 1.0 / 3.0, 1.0 / 6.0});
  VEC_INIT(3, x, {0.0, 0.0, 0.0});

  ConstrainedLeastSquares(&A, &b, &C, &lower, &upper, &x, &x);

  VEC_INIT(2, tmp, {0});
  double res_norm = VecNorm(VecSub(&b, MatVecMult(&A, &x, &tmp), &tmp));
  double res_norm_ans = VecNorm(VecSub(&b, MatVecMult(&A, &x_ans, &tmp), &tmp));
  EXPECT_NEAR(res_norm, res_norm_ans, 1e-9);
}

TEST(ConstrainedLeastSquares, CheckMinimumUnderconstrained) {
  for (int32_t k = 0; k < 222; ++k) {
    MAT_INIT(
        4, 5, A,
        {{RandNormal(), RandNormal(), RandNormal(), RandNormal(), RandNormal()},
         {RandNormal(), RandNormal(), RandNormal(), RandNormal(), RandNormal()},
         {RandNormal(), RandNormal(), RandNormal(), RandNormal(), RandNormal()},
         {RandNormal(), RandNormal(), RandNormal(), RandNormal(),
          RandNormal()}});
    VEC_INIT(4, b, {RandNormal(), RandNormal(), RandNormal(), RandNormal()});
    VEC_INIT(4, r, {0});
    MAT_INIT(
        3, 5, C,
        {{RandNormal(), RandNormal(), RandNormal(), RandNormal(), RandNormal()},
         {RandNormal(), RandNormal(), RandNormal(), RandNormal(), RandNormal()},
         {RandNormal(), RandNormal(), RandNormal(), RandNormal(),
          RandNormal()}});
    VEC_INIT(3, lower, {RandNormal(), RandNormal(), RandNormal()});
    VEC_INIT(3, upper,
             {lower.d[0] + Rand(), lower.d[1] + Rand(), lower.d[2] + Rand()});
    VEC_INIT(3, mid, {0});
    VEC_INIT(5, x0, {0});
    VEC_INIT(5, x, {0});
    VEC_INIT(5, x2, {0});

    MatVecLeftDivide(&C, VecLinComb(0.5, &lower, 0.5, &upper, &mid), &x0);
    ConstrainedLeastSquares(&A, &b, &C, &lower, &upper, &x0, &x);
    ASSERT_TRUE(IsWithinConstraints(&C, &x, &lower, &upper, 1e-4));

    for (int32_t i = 0; i < 222; ++i) {
      do {
        for (int32_t j = 0; j < x2.length; ++j) {
          x2.d[j] = x.d[j] + VecNorm(&x) / 10.0 * (Rand() - 0.5);
        }
      } while (!IsWithinConstraints(&C, &x2, &lower, &upper, 0.0));
      double norm_r = VecNorm(VecSub(MatVecMult(&A, &x, &r), &b, &r));
      double norm_r2 = VecNorm(VecSub(MatVecMult(&A, &x2, &r), &b, &r));
      EXPECT_LE(norm_r, norm_r2);
    }
  }
}

TEST(ConstrainedLeastSquares, OverConstrained) {
  MAT_INIT(6, 3, A, {{0.52, -0.71, -1.12},
                     {-0.02, 1.35, 2.53},
                     {-0.03, -0.22, 1.66},
                     {-0.80, -0.59, 0.31},
                     {1.02, -0.29, -1.26},
                     {-0.13, -0.85, -0.87}});
  VEC_INIT(6, b, {-0.18, 0.79, -1.33, -2.33, -1.45, 0.33});
  MAT_INIT(5, 3, C, {{1.0, 0.0, 0.0},
                     {0.0, 1.0, 0.0},
                     {0.0, 0.0, 1.0},
                     {0.39, 0.45, -0.13},
                     {0.18, -0.48, 0.86}});
  VEC_INIT(5, l, {-1.0, -1.0, -1.0, -1.0, -1.0});
  VEC_INIT(5, u, {1.0, 1.0, 1.0, 1.0, 1.0});
  VEC_INIT(3, x_ans, {-0.167428438915193, 1.0, -0.335329873368754});
  VEC_INIT(3, x, {0.0, 0.0, 0.0});
  ConstrainedLeastSquares(&A, &b, &C, &l, &u, &x, &x);
  for (int32_t i = 0; i < 3; ++i) {
    EXPECT_NEAR(x_ans.d[i], x.d[i], 1e-5);
  }
}

TEST(ConstrainedLeastSquares, CheckMinimumOverconstrained) {
  int32_t num_tests = 0;
  while (num_tests < 222) {
    MAT_INIT(5, 3, A, {{RandNormal(), RandNormal(), RandNormal()},
                       {RandNormal(), RandNormal(), RandNormal()},
                       {RandNormal(), RandNormal(), RandNormal()},
                       {RandNormal(), RandNormal(), RandNormal()},
                       {RandNormal(), RandNormal(), RandNormal()}});
    VEC_INIT(5, b, {RandNormal(), RandNormal(), RandNormal(), RandNormal(),
                    RandNormal()});
    VEC_INIT(5, r, {0});
    MAT_INIT(6, 3, C, {{1.0, 0.0, 0.0},
                       {0.0, 1.0, 0.0},
                       {0.0, 0.0, 1.0},
                       {RandNormal(), RandNormal(), RandNormal()},
                       {RandNormal(), RandNormal(), RandNormal()},
                       {RandNormal(), RandNormal(), RandNormal()}});
    VEC_INIT(6, lower, {RandNormal(), RandNormal(), RandNormal(), RandNormal(),
                        RandNormal(), RandNormal()});
    VEC_INIT(6, upper,
             {lower.d[0] + Rand(), lower.d[1] + Rand(), lower.d[2] + Rand(),
              lower.d[3] + Rand(), lower.d[4] + Rand(), lower.d[5] + Rand()});
    VEC_INIT(6, mid, {0});
    VEC_INIT(3, x0, {0});
    VEC_INIT(3, x, {0});
    VEC_INIT(3, x2, {0});

    MatVecLeftDivide(&C, VecLinComb(0.5, &lower, 0.5, &upper, &mid), &x0);
    if (IsWithinConstraints(&C, &x0, &lower, &upper, 0.0)) {
      ConstrainedLeastSquares(&A, &b, &C, &lower, &upper, &x0, &x);
      ASSERT_TRUE(IsWithinConstraints(&C, &x, &lower, &upper, 1e-4));
      for (int32_t i = 0; i < 222; ++i) {
        do {
          for (int32_t j = 0; j < x2.length; ++j) {
            x2.d[j] = x.d[j] + VecNorm(&x) / 10.0 * (Rand() - 0.5);
          }
        } while (!IsWithinConstraints(&C, &x2, &lower, &upper, 0.0));
        double norm_r = VecNorm(VecSub(MatVecMult(&A, &x, &r), &b, &r));
        double norm_r2 = VecNorm(VecSub(MatVecMult(&A, &x2, &r), &b, &r));
        ASSERT_LE(norm_r, norm_r2);
      }
      num_tests++;
    }
  }
}

TEST(ConstrainedLeastSquares, CompareWithBounded) {
  for (int32_t k = 0; k < 222; ++k) {
    int32_t num_constr = 1 + (rand() % 15);  // NOLINT(runtime/threadsafe_fn)
    int32_t num_data = 1 + (rand() % 15);    // NOLINT(runtime/threadsafe_fn)

    MAT(num_data, num_constr, A);
    VEC(num_data, b);
    MAT(num_constr, num_constr, C);
    VEC(num_constr, lower);
    VEC(num_constr, mid);
    VEC(num_constr, upper);
    VEC(num_constr, x_constr);
    VEC(num_constr, x_bound);
    VEC(num_constr, x0);
    VEC(num_data, r);

    for (int32_t i = 0; i < num_data; ++i) {
      for (int32_t j = 0; j < num_constr; ++j) {
        *MatPtr(&A, i, j) = RandNormal();
      }
      b.d[i] = RandNormal();
    }

    for (int32_t i = 0; i < num_constr; ++i) {
      for (int32_t j = 0; j < num_constr; ++j) {
        if (i == j) {
          *MatPtr(&C, i, j) = 1.0;
        } else {
          *MatPtr(&C, i, j) = 0.0;
        }
      }
      lower.d[i] = -Rand();
      upper.d[i] = lower.d[i] + Rand();
    }

    MatVecLeftDivide(&C, VecLinComb(0.5, &lower, 0.5, &upper, &mid), &x0);
    ConstrainedLeastSquares(&A, &b, &C, &lower, &upper, &x0, &x_constr);
    BoundedLeastSquares(&A, &b, &lower, &upper, &x_bound);
    double norm_bound = VecNorm(VecSub(MatVecMult(&A, &x_bound, &r), &b, &r));
    double norm_constr = VecNorm(VecSub(MatVecMult(&A, &x_constr, &r), &b, &r));

    ASSERT_TRUE(IsWithinConstraints(&C, &x_bound, &lower, &upper, 1e-4));
    ASSERT_TRUE(IsWithinConstraints(&C, &x_constr, &lower, &upper, 1e-4));
    ASSERT_TRUE(fabs(norm_bound - norm_constr) < 1e-4);
  }
}

TEST(ConstrainedLeastSquares, RealisticUse) {
  for (int32_t k = 0; k < 2222; ++k) {
    MAT_INIT(11, 8, C, {{1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                        {0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                        {0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                        {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0},
                        {0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0},
                        {0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0},
                        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0},
                        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0},
                        {1.0, 0.0, 0.0, -1.0, 1.0, 0.0, 0.0, -1.0},
                        {0.0, 1.0, -1.0, 0.0, 0.0, 1.0, -1.0, 0.0},
                        {0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 1.0, -1.0}});
    VEC_INIT(11, lower, {-250.0, -240.0, -230.0, -240.0, -250.0, -260.0, -240.0,
                         -230.0, -10.0, -10.0, -10.0});
    VEC_INIT(
        11, upper,
        {Rand(10000.0, 12000.0), Rand(10000.0, 12000.0), Rand(10000.0, 12000.0),
         Rand(10000.0, 12000.0), Rand(10000.0, 12000.0), Rand(10000.0, 12000.0),
         Rand(10000.0, 12000.0), Rand(10000.0, 12000.0), 10.0, 10.0, 10.0});
    VEC_INIT(4, weights, {1e-2, 1e-4, 1.0, 1.0});
    VEC_INIT(8, thrusts, {0});
    MAT_INIT(
        4, 8, A,
        {{0.1250, 0.1250, 0.1250, 0.1250, 0.1250, 0.1250, 0.1250, 0.1250},
         {-0.5000, 0.5000, -0.5000, 0.5000, -0.5000, 0.5000, -0.5000, 0.5000},
         {0.0951, 0.0951, 0.0951, 0.0951, -0.0951, -0.0951, -0.0951, -0.0951},
         {-0.0618, -0.0206, 0.0206, 0.0618, 0.0618, 0.0206, -0.0206, -0.0618}});
    VEC_INIT(4, b, {Rand(-20000.0, 20000.0), 0.0, Rand(-2000.0, 2000.0),
                    Rand(-2000.0, 2000.0)});
    WeightLeastSquaresInputs(&weights, &A, &b);
    ConstrainedLeastSquares(&A, &b, &C, &lower, &upper, &thrusts, &thrusts);
    ASSERT_TRUE(IsWithinConstraints(&C, &thrusts, &lower, &upper, 1e-4));

    VEC_INIT(8, x, {0});
    VEC_INIT(4, r, {0});
    double norm_opt = VecNorm(VecSub(MatVecMult(&A, &thrusts, &r), &b, &r));

    // Try all 6561 (3^8) possible combinations of increasing, leaving
    // the same, or decreasing each individual rotor's thrust.
    for (int32_t i = 0; i < 6561; ++i) {
      VecCopy(&thrusts, &x);
      int32_t base3_num = i;
      for (int32_t j = 0; j < 8; ++j) {
        if (base3_num % 3 == 0) {
          *VecPtr(&x, j) -= 0.1;
        } else if (base3_num % 3 == 1) {
          *VecPtr(&x, j) += 0.1;
        }

        // Get next ternary digit.
        base3_num /= 3;
      }

      // All thrust vectors that meet the constraints should have a
      // higher error norm than the optimal solution.
      if (IsWithinConstraints(&C, &x, &lower, &upper, 0.0)) {
        double norm_mod = VecNorm(VecSub(MatVecMult(&A, &x, &r), &b, &r));
        ASSERT_LE(norm_opt, norm_mod + DBL_TOL);
      }
    }
  }
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

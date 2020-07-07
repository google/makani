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
#include <stdlib.h>

#include <string>

#include "common/c_math/filter.h"
#include "common/c_math/kalman.h"
#include "common/c_math/linalg.h"
#include "common/c_math/util/linalg_io.h"
#include "common/macros.h"
#include "common/runfiles_dir.h"
#include "lib/util/random_matrix.h"
#include "lib/util/test_util.h"

using ::test_util::Rand;
using ::test_util::RandNormal;

extern "C" {

void ModifiedWeightedGramSchmidt(int32_t n, int32_t m, const double d[],
                                 double W[], double ud[]);

}  // extern "C"

TEST(BoundedKalman1dEstimator, CompareToLpf) {
  BoundedKalman1dEstimatorParams estimator_params;
  estimator_params.fc_min = 1.1;
  estimator_params.fc_max = 1.1;
  estimator_params.Q = 2.22;

  double x = 0.0;
  double P = 0.0;
  double z = 0.22;
  double ts = 0.01;
  double R = 22.0;

  double x_lpf = 0.0;

  double ndz, fc;

  for (int32_t i = 0; i < 2222; ++i) {
    z += Rand(-1.0, 1.0);
    BoundedKalman1dEstimator(z, R, ts, &estimator_params, &ndz, &fc, &x, &P);
    x_lpf = Lpf(z, estimator_params.fc_min, ts, &x_lpf);
    EXPECT_NEAR(x, x_lpf, 1e-9);
  }
}

TEST(BoundedKalman1d, CompareToMatlab) {
  const int32_t num_loops = 10;

  double z[num_loops] = {-0.99, -0.44, -0.19, -0.25, -0.81,
                         -0.99, -0.23, -0.04, -0.30, -0.70};
  double x_ans[num_loops] = {0.000000000000,  -0.264003519930, -0.236250523379,
                             -0.240000908994, -0.362174685609, -0.473011916364,
                             -0.436538899633, -0.384775253610, -0.374983288448,
                             -0.408650211515};
  double P_ans[num_loops] = {3.000000000000, 1.200015999680, 0.750045310732,
                             0.545531399963, 0.428680601576, 0.353083025529,
                             0.300174717986, 0.261077266675, 0.231009988048,
                             0.207170412294};
  double dz_norm_ans[num_loops] = {
      0.000000000000,  -0.196772014310, 0.041368475489, -0.008291027319,
      -0.357253426948, -0.402851217043, 0.158416245700, 0.261454393429,
      0.056376977990,  -0.217593141166};

  BoundedKalman1dParams kalman1d_params;
  kalman1d_params.A = 0.0000;
  kalman1d_params.H = 1.0000;
  kalman1d_params.fc_min = 0.0;
  kalman1d_params.fc_max = INFINITY;
  kalman1d_params.Q = 0.0100;
  kalman1d_params.R = 0.0200;
  double ts = 0.0100;

  double fc[num_loops], dz_norm[num_loops];
  double x_tmp, P_tmp;
  double x[num_loops], P[num_loops];

  for (int32_t i = 0; i < num_loops; ++i) {
    if (i == 0) {
      x_tmp = x_ans[0];
      P_tmp = P_ans[0];
    } else {
      BoundedKalman1d(z[i], ts, &kalman1d_params, &dz_norm[i], &fc[i], &x_tmp,
                      &P_tmp);
    }
    x[i] = x_tmp;
    P[i] = P_tmp;

    EXPECT_NEAR(x[i], x_ans[i], 1e-9);
    EXPECT_NEAR(P[i], P_ans[i], 1e-9);
    EXPECT_NEAR(dz_norm[i], dz_norm_ans[i], 1e-9);
  }
}

namespace {

void WeightedAAT(const Mat *W, const Vec *d, Mat *P) {
  int32_t n = W->nr;
  int32_t m = W->nc;
  for (int32_t i = 0; i < n; ++i) {
    for (int32_t j = 0; j < n; ++j) {
      *MatPtr(P, i, j) = 0.0;
      for (int32_t k = 0; k < m; ++k) {
        *MatPtr(P, i, j) += MatGet(W, i, k) * MatGet(W, j, k) * VecGet(d, k);
      }
    }
  }
}

void RandomUd(Mat *U, Vec *D) {
  int32_t nx = U->nr;
  for (int32_t i = 0; i < nx; ++i) {
    for (int32_t j = 0; j < nx; ++j) {
      if (i == j) {
        *MatPtr(U, i, j) = 1.0;
        *VecPtr(D, i) = fabs(RandNormal());
      } else {
        *MatPtr(U, i, j) = i < j ? RandNormal() : 0.0;
      }
    }
  }
}

}  // namespace

TEST(UdKalmanStoreUpperTri, Recovery) {
  constexpr int32_t max_nx = 8;
  constexpr int32_t kUdDataSize = ((max_nx + 1) * max_nx) / 2;
  double ud_data[kUdDataSize];
  for (int32_t nx = 1; nx < max_nx; ++nx) {
    MAT(nx, nx, P);
    for (int32_t i = 0; i < nx; ++i) {
      for (int32_t j = 0; j < nx; ++j) {
        *MatPtr(&P, i, j) = RandNormal();
      }
    }

    Vec ud;
    ud.length = ((nx + 1) * nx) / 2;
    ud.max_length = ud.length;
    ud.variable_len = 0;
    ud.d = ud_data;

    // Test d == NULL case.
    MAT(nx, nx, U);
    VEC(nx, D);
    UdKalmanStoreUpperTri(&P, NULL, &ud);
    UdKalmanExtractUd(&ud, &U, &D);
    for (int32_t j = 0; j < nx; ++j) {
      for (int32_t i = 0; i < nx; ++i) {
        if (i == j) {
          EXPECT_EQ(1.0, MatGet(&U, i, j));
          EXPECT_EQ(MatGet(&P, i, j), VecGet(&D, i));
        } else if (i < j) {
          EXPECT_EQ(MatGet(&P, i, j), MatGet(&U, i, j));
        } else {
          EXPECT_EQ(0.0, MatGet(&U, i, j));
        }
      }
    }

    // Test case where neither is NULL.
    VEC(nx, rand_D);
    for (int32_t i = 0; i < nx; ++i) *VecPtr(&rand_D, i) = RandNormal();
    UdKalmanStoreUpperTri(&P, &rand_D, &ud);
    UdKalmanExtractUd(&ud, &U, &D);
    for (int32_t j = 0; j < nx; ++j) {
      for (int32_t i = 0; i < nx; ++i) {
        if (i == j) {
          EXPECT_EQ(1.0, MatGet(&U, i, j));
          EXPECT_EQ(VecGet(&rand_D, i), VecGet(&D, i));
        } else if (i < j) {
          EXPECT_EQ(MatGet(&P, i, j), MatGet(&U, i, j));
        } else {
          EXPECT_EQ(0.0, MatGet(&U, i, j));
        }
      }
    }

    // Test case where P is NULL.
    UdKalmanStoreUpperTri(NULL, &rand_D, &ud);
    UdKalmanExtractUd(&ud, &U, &D);
    for (int32_t j = 0; j < nx; ++j) {
      for (int32_t i = 0; i < nx; ++i) {
        if (i == j) {
          EXPECT_EQ(1.0, MatGet(&U, i, j));
          EXPECT_EQ(VecGet(&rand_D, i), VecGet(&D, i));
        } else {
          EXPECT_EQ(0.0, MatGet(&U, i, j));
        }
      }
    }
  }
}

// Also tests UdKalmanExtractUd.
TEST(UdKalmanGain, Recovery) {
  constexpr int32_t max_nx = 16;
  constexpr int32_t kUdDataSize = ((max_nx + 1) * max_nx) / 2;
  double ud_data[kUdDataSize];
  for (int32_t nx = 1; nx < max_nx; ++nx) {
    double R = pow(RandNormal(), 2.0);
    MAT(nx, nx, U);
    VEC(nx, D);
    RandomUd(&U, &D);
    VEC(nx, h);
    for (int32_t i = 0; i < nx; ++i) *VecPtr(&h, i) = RandNormal();

    // Compute standard covariance matrix.
    MAT(nx, nx, P);
    WeightedAAT(&U, &D, &P);

    // Pack result into vector form.
    Vec ud;
    ud.length = ((nx + 1) * nx) / 2;
    ud.variable_len = 0;
    ud.d = ud_data;
    ud.max_length = ARRAYSIZE(ud_data);
    UdKalmanStoreUpperTri(&U, &D, &ud);

    // Standard calculation for the Kalman gain.
    VEC(nx, k);
    MatVecMult(&P, &h, &k);
    double s = VecDot(&k, &h) + R;
    VecScale(&k, 1.0 / s, &k);

    // Calculate using UD update.
    VEC(nx, f);
    VecCopy(&h, &f);
    VEC(nx, ud_k);
    UdKalmanMeasurementUpdate(R, &f, &ud, &ud_k);

    // Compare Kalman gains.
    for (int32_t i = 0; i < nx; ++i) {
      EXPECT_NEAR(VecGet(&k, i), VecGet(&ud_k, i), 1e-14);
    }

    // Calculated standard P update, P = (I - K * H') *P.
    MatVecMult(&P, &h, &f);
    Mat f_mat;
    f_mat.nr = 1;
    f_mat.nc = nx;
    f_mat.d = f.d;
    f_mat.variable_dim = 0;
    f_mat.max_size = nx;
    Mat k_mat;
    k_mat.nr = nx;
    k_mat.nc = 1;
    k_mat.d = k.d;
    k_mat.variable_dim = 0;
    k_mat.max_size = nx;
    MatMult(&k_mat, &f_mat, &U);
    MatSub(&P, &U, &P);

    // Extract U and D.
    // Randomize contents to better test extraction functions.
    for (int32_t i = 0; i < nx; ++i) {
      *VecPtr(&f, i) = RandNormal();
      for (int32_t j = 0; j < nx; ++j) {
        *MatPtr(&U, i, j) = RandNormal();
      }
    }
    UdKalmanExtractUd(&ud, &U, &D);
    MAT(nx, nx, P_ud);
    WeightedAAT(&U, &D, &P_ud);

    // Compare P matrices.
    for (int32_t i = 0; i < nx; ++i) {
      for (int32_t j = 0; j < nx; ++j) {
        EXPECT_NEAR(MatGet(&P, i, j), MatGet(&P_ud, i, j), 1e-13);
      }
    }
  }
}

namespace {

void TestModifiedWeightedGramSchmidt(int32_t n, int32_t m) {
  MAT(n, m, W);
  VEC(m, d);
  for (int32_t i = 0; i < m; ++i) {
    *VecPtr(&d, i) = fabs(RandNormal());
    for (int32_t j = 0; j < n; ++j) {
      *MatPtr(&W, j, i) = RandNormal();
    }
  }
  MAT(n, m, W_copy);
  MatCopy(&W, &W_copy);
  VEC((n + 1) * n / 2, ud);
  ModifiedWeightedGramSchmidt(n, m, d.d, W_copy.d, ud.d);

  MAT(n, n, U);
  VEC(n, D);
  UdKalmanExtractUd(&ud, &U, &D);

  MAT(n, n, P);
  WeightedAAT(&W, &d, &P);
  MAT(n, n, P_ud);
  WeightedAAT(&U, &D, &P_ud);
  for (int32_t i = 0; i < n; ++i) {
    for (int32_t j = 0; j < n; ++j) {
      EXPECT_NEAR(MatGet(&P, i, j), MatGet(&P_ud, i, j), 1e-14);
    }
  }
}

}  // namespace

TEST(ModifiedWeightGramSchmidt, Recovery) {
  TestModifiedWeightedGramSchmidt(4, 4);
  TestModifiedWeightedGramSchmidt(4, 8);
}

namespace {

void TestUdKalmanTimeUpdate(int32_t n, int32_t p) {
  MAT(n, n + p, W);
  VEC(n + p, d);

  MAT(n, n, FU);
  VEC(n, D);
  for (int32_t i = 0; i < n; ++i) {
    *VecPtr(&D, i) = fabs(RandNormal());
    *VecPtr(&d, i) = VecGet(&D, i);
    for (int32_t j = 0; j < n; ++j) {
      *MatPtr(&FU, i, j) = RandNormal();
      *MatPtr(&W, i, j) = MatGet(&FU, i, j);
    }
  }
  MAT(n, p, B);
  VEC(p, q);
  for (int32_t j = 0; j < p; ++j) {
    *VecPtr(&q, j) = fabs(RandNormal());
    *VecPtr(&d, j + n) = VecGet(&q, j);
    for (int32_t i = 0; i < n; ++i) {
      *MatPtr(&B, i, j) = RandNormal();
      *MatPtr(&W, i, j + n) = MatGet(&B, i, j);
    }
  }

  MAT(n, n, P);
  WeightedAAT(&FU, &D, &P);
  MAT(n, n, P2);
  WeightedAAT(&B, &q, &P2);
  MatAdd(&P, &P2, &P);

  VEC(((n + 1) * n) / 2, ud);
  UdKalmanTimeUpdate(&d, &W, &ud);
  UdKalmanExtractUd(&ud, &FU, &D);
  WeightedAAT(&FU, &D, &P2);

  for (int32_t i = 0; i < n; ++i) {
    for (int32_t j = 0; j < n; ++j) {
      EXPECT_NEAR(MatGet(&P, i, j), MatGet(&P2, i, j), 1e-14);
    }
  }
}

}  // namespace

TEST(TestUdKalmanTimeUpdate, Recovery) {
  TestUdKalmanTimeUpdate(4, 1);
  TestUdKalmanTimeUpdate(4, 4);
}

TEST(UdKalmanExtractCovariances, Full) {
  constexpr int32_t max_nx = 8;
  constexpr int32_t kUdDataSize = ((max_nx + 1) * max_nx) / 2;
  double ud_data[kUdDataSize];
  for (int32_t nx = 0; nx < max_nx; ++nx) {
    MAT(nx, nx, U);
    VEC(nx, D);
    RandomUd(&U, &D);

    MAT(nx, nx, P);
    WeightedAAT(&U, &D, &P);

    Vec ud;
    ud.length = ((nx + 1) * nx) / 2;
    ud.variable_len = 0;
    ud.d = ud_data;
    ud.max_length = ARRAYSIZE(ud_data);
    UdKalmanStoreUpperTri(&U, &D, &ud);

    VEC(nx, cov);
    UdKalmanExtractCovariances(ud.d, &cov);
    for (int32_t i = 0; i < nx; ++i) {
      EXPECT_NEAR(MatGet(&P, i, i), VecGet(&cov, i), 1e-14);
    }
  }
}

TEST(UdKalmanTransitionMatrixMultiply, Full) {
  constexpr int32_t kNumStates = 6;
  constexpr int32_t kNumNoises = 4;
  MAT(kNumStates, kNumStates, F);
  MAT(kNumStates, kNumStates, U);
  VEC(kNumStates, d);
  for (int32_t i = 0; i < kNumStates; ++i) {
    for (int32_t j = 0; j < kNumStates; ++j) {
      *MatPtr(&F, i, j) = RandNormal();
      if (i == j) {
        *MatPtr(&U, i, j) = 1.0;
      } else if (i < j) {
        *MatPtr(&U, i, j) = RandNormal();
      } else {
        *MatPtr(&U, i, j) = 0.0;
      }
    }
    *VecPtr(&d, i) = fabs(RandNormal());
  }

  MAT(kNumStates, kNumStates + kNumNoises, W);
  VEC(kNumStates + kNumNoises, D);
  for (int32_t j = 0; j < kNumStates + kNumNoises; ++j) {
    for (int32_t i = 0; i < kNumStates; ++i) {
      *MatPtr(&W, i, j) = RandNormal();
    }
    *VecPtr(&D, j) = fabs(RandNormal());
  }
  MAT_CLONE(kNumStates, kNumStates + kNumNoises, W_clone, W.d);
  VEC_CLONE(kNumStates + kNumNoises, D_clone, D.d);

  VEC(((kNumStates + 1) * kNumStates) / 2, ud);
  UdKalmanStoreUpperTri(&U, &d, &ud);
  UdKalmanTransitionMatrixMultiply(&F, &ud, &W, &D);

  MAT(kNumStates, kNumStates, FU);
  MatMult(&F, &U, &FU);
  for (int32_t i = 0; i < kNumStates; ++i) {
    for (int32_t j = 0; j < kNumStates; ++j) {
      EXPECT_EQ(MatGet(&FU, i, j), MatGet(&W, i, j));
    }
    for (int32_t j = kNumStates; j < kNumStates + kNumNoises; ++j) {
      EXPECT_EQ(MatGet(&W_clone, i, j), MatGet(&W, i, j));
    }
    EXPECT_EQ(VecGet(&d, i), VecGet(&D, i));
  }
  for (int32_t j = kNumStates; j < kNumStates + kNumNoises; ++j) {
    EXPECT_EQ(VecGet(&D_clone, j), VecGet(&D, j));
  }
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  test_util::SetRunfilesDir();
  return RUN_ALL_TESTS();
}

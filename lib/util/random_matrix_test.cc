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

#include <assert.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_rng.h>
#include <stdint.h>

#include "lib/util/random_matrix.h"
#include "lib/util/test_util.h"

TEST(MatArrRandomOrthogonal, IsOrthogonal) {
  const int32_t kMaxDim = 10;
  const int32_t kMaxSize = kMaxDim * kMaxDim;
  double U_data[kMaxSize];
  double error_data[kMaxSize];

  gsl_rng *rng = gsl_rng_alloc(gsl_rng_taus);
  for (int32_t n = 1; n < kMaxDim; ++n) {
    gsl_matrix_view U_view
        = gsl_matrix_view_array(U_data, (size_t)n, (size_t)n);
    gsl_matrix_view error_view
        = gsl_matrix_view_array(error_data, (size_t)n, (size_t)n);
    for (int32_t sample = 0; sample < 222; ++sample) {
      MatArrRandomOrthogonal(n, rng, U_data);
      gsl_matrix_set_identity(&error_view.matrix);
      gsl_blas_dgemm(CblasTrans, CblasNoTrans, 1.0, &U_view.matrix,
                     &U_view.matrix, -1.0, &error_view.matrix);
      for (int32_t i = 0; i < n*n; ++i) {
        EXPECT_NEAR(error_data[i], 0.0, 1e-10);
      }
    }
  }
  gsl_rng_free(rng);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

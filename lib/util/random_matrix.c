/*
 * Copyright 2020 Makani Technologies LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "lib/util/random_matrix.h"

#include <assert.h>
#include <gsl/gsl_linalg.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_randist.h>
#include <gsl/gsl_rng.h>
#include <gsl/gsl_vector.h>
#include <stdint.h>

#include "common/c_math/linalg.h"

void MatArrRandomGaussian(int32_t nr, int32_t nc, const gsl_rng *rng,
                          double *d) {
  for (int32_t i = 0; i < nr; ++i) {
    for (int32_t j = 0; j < nc; ++j) {
      d[i*nc + j] = gsl_ran_gaussian(rng, 1.0);
    }
  }
}

void MatArrRandomOrthogonal(int32_t n, const gsl_rng *rng, double *d) {
  MatArrRandomGaussian(n, n, rng, d);
  gsl_matrix *Q = gsl_matrix_alloc((size_t)n, (size_t)n);
  gsl_matrix *R = gsl_matrix_alloc((size_t)n, (size_t)n);
  gsl_vector *tau = gsl_vector_alloc((size_t)n);

  gsl_matrix_view A = gsl_matrix_view_array(d, (size_t)n, (size_t)n);
  gsl_linalg_QR_decomp(&A.matrix, tau);
  gsl_linalg_QR_unpack(&A.matrix, tau, Q, R);
  gsl_matrix_memcpy(&A.matrix, Q);

  gsl_vector_free(tau);
  gsl_matrix_free(R);
  gsl_matrix_free(Q);
}

void MatArrRandomWithRank(int32_t nr, int32_t nc, int32_t rank,
                          const gsl_rng *rng, double *d) {
  assert(rank <= nr && rank <= nc);

  MatArrZero(nr, nc, d);
  double *u = (double *)malloc((size_t)nr * sizeof(*d));
  double *v = (double *)malloc((size_t)nc * sizeof(*d));
  for (int32_t k = 0; k < rank; ++k) {
    MatArrRandomGaussian(1, nr, rng, u);
    MatArrRandomGaussian(1, nc, rng, v);

    for (int32_t i = 0; i < nr; ++i) {
      for (int32_t j = 0; j < nc; ++j) {
        d[i*nc + j] += u[i] * v[j];
      }
    }
  }
  free(u);
  free(v);
}

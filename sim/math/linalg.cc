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

#include "sim/math/linalg.h"

#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include <gsl/gsl_blas.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_vector.h>

// We use GSL style conventions here to maintain consistency with the
// GSL library.

double gsl_matrix_trace(const gsl_matrix *m) {
  assert(m->size1 == m->size2);

  double tr = 0.0;
  for (uint32_t i = 0; i < m->size1; ++i) tr += gsl_matrix_get(m, i, i);

  return tr;
}

const gsl_vector *gsl_matrix_diag(const gsl_matrix *m, gsl_vector *d) {
  assert(m->size1 >= d->size && m->size2 >= d->size);

  for (uint32_t i = 0; i < d->size; ++i)
    gsl_vector_set(d, i, gsl_matrix_get(m, i, i));

  return d;
}

void gsl_matrix_disp(const gsl_matrix *m) {
  printf("[");
  for (size_t i = 0; i < m->size1; ++i) {
    for (size_t j = 0; j < m->size2; ++j) {
      printf("%f", gsl_matrix_get(m, i, j));
      if (j < m->size2 - 1)
        printf(", ");
      else if (i < m->size1 - 1)
        printf("; ");
    }
  }
  printf("]");
}

void gsl_vector_disp(const gsl_vector *v) {
  printf("[");
  for (size_t i = 0; i < v->size; ++i) {
    printf("%f", gsl_vector_get(v, i));
    if (i < v->size - 1) printf(", ");
  }
  printf("]");
}

const gsl_vector *gsl_vector_saturate(const gsl_vector *x, double low,
                                      double high, gsl_vector *y) {
  gsl_vector_memcpy(y, x);
  for (uint32_t i = 0; i < x->size; ++i) {
    if (gsl_vector_get(y, i) < low) {
      gsl_vector_set(y, i, low);
    } else if (gsl_vector_get(y, i) > high) {
      gsl_vector_set(y, i, high);
    }
  }

  return y;
}

double gsl_vector_norm_bound(const gsl_vector *x, double low) {
  return fmax(gsl_blas_dnrm2(x), low);
}

double gsl_vector_dot(const gsl_vector *x, const gsl_vector *y) {
  double dot_x_y;
  gsl_blas_ddot(x, y, &dot_x_y);
  return dot_x_y;
}

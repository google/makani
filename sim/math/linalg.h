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

#ifndef SIM_MATH_LINALG_H_
#define SIM_MATH_LINALG_H_

#include <gsl/gsl_matrix.h>
#include <gsl/gsl_vector.h>
#include <stdint.h>

// We use GSL style conventions here to maintain consistency with the
// GSL library.  See GSL docs for comment.

#define GSL_VECTOR_STACK_ALLOC(v_, size_)                               \
  double v_##_data[size_];                                              \
  gsl_block v_##_block = {size_, v_##_data};                            \
  gsl_vector v_##_vector = {size_, 1, v_##_block.data, &v_##_block, 0}; \
  gsl_vector *v_ = &v_##_vector;

#define GSL_MATRIX_STACK_ALLOC(m_, size1_, size2_)                \
  double m_##_data[(size1_) * (size2_)];                          \
  gsl_block m_##_block = {(size1_) * (size2_), m_##_data};        \
  gsl_matrix m_##_matrix = {size1_,          size2_,      size2_, \
                            m_##_block.data, &m_##_block, 0};     \
  gsl_matrix *m_ = &m_##_matrix;

double gsl_matrix_trace(const gsl_matrix *m);
const gsl_vector *gsl_matrix_diag(const gsl_matrix *m, gsl_vector *d);
void gsl_matrix_disp(const gsl_matrix *m);
void gsl_vector_disp(const gsl_vector *v);
const gsl_vector *gsl_vector_saturate(const gsl_vector *x, double low,
                                      double high, gsl_vector *y);
double gsl_vector_norm_bound(const gsl_vector *x, double low);
double gsl_vector_dot(const gsl_vector *x, const gsl_vector *y);

#endif  // SIM_MATH_LINALG_H_

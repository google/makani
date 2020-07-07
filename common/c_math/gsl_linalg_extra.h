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

// The GSL extra library provides linear algebra functions using a
// similar interface to the rest of the GSL library.

#ifndef COMMON_C_MATH_GSL_LINALG_EXTRA_H_
#define COMMON_C_MATH_GSL_LINALG_EXTRA_H_

#include <stdint.h>

#include <gsl/gsl_blas_types.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_vector.h>

#ifdef __cplusplus
extern "C" {
#endif

int32_t GslTriangularSolve(CBLAS_SIDE_t side, CBLAS_UPLO_t uplo,
                           CBLAS_TRANSPOSE_t transpose, const gsl_matrix *T,
                           const gsl_matrix *B, gsl_matrix *X);
void GslTrapezoidalToTriangular(const gsl_matrix *R, gsl_matrix *T,
                                gsl_vector *tau);
void GslTrapezoidalToTriangularZTMat(const gsl_matrix *T, const gsl_vector *tau,
                                     const gsl_matrix *X, gsl_matrix *Zt_X);
int32_t GslMatrixDivide(CBLAS_SIDE_t side, const gsl_matrix *A,
                        const gsl_matrix *B, gsl_matrix *X);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // COMMON_C_MATH_GSL_LINALG_EXTRA_H_

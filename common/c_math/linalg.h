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

#ifndef COMMON_C_MATH_LINALG_H_
#define COMMON_C_MATH_LINALG_H_

#include <assert.h>
#include <float.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "common/c_math/linalg_common.h"

#ifdef __cplusplus
extern "C" {
#endif

// Operations on vectors with arbitrary numbers of elements.

#define VEC_MAX_ELEMENTS 32

typedef struct {
  int32_t length;
  double *d;
  int32_t variable_len;
  int32_t max_length;
} Vec;

#if !defined(__cplusplus) && (__STDC_VERSION__ >= 199901L)
#define CHECK_VEC_INIT_SIZE(N_, ...)                                          \
  assert((N_) == sizeof((double[])__VA_ARGS__) / sizeof(double)  /* NOLINT */ \
         || (sizeof((double[])__VA_ARGS__) / sizeof(double) == 1 /* NOLINT */ \
             && ((double[])__VA_ARGS__)[0] == 0.0))
#elif defined(__cplusplus) && !defined(NDEBUG)
#define CHECK_VEC_INIT_SIZE(N_, ...)                               \
  do {                                                             \
    double va_args_[] = __VA_ARGS__;                               \
    assert((N_) == sizeof(va_args_) / sizeof(double)  /* NOLINT */ \
           || (sizeof(va_args_) / sizeof(double) == 1 /* NOLINT */ \
               && fabs(va_args_[0]) <= DBL_EPSILON));              \
  } while (0)
#else
#define CHECK_VEC_INIT_SIZE(N_, ...) \
  do {                               \
  } while (0)
#endif

// Stack-allocate a vector of length n_ called name_.  We actually
// allocate VEC_MAX_ELEMENTS so n_ doesn't have to be a constant.
#define VEC(n_, name_)                    \
  assert((n_) <= VEC_MAX_ELEMENTS);       \
  double name_##_data_[VEC_MAX_ELEMENTS]; \
  Vec name_ = {(n_), name_##_data_, 1, VEC_MAX_ELEMENTS}

// Initialize a vector given an existing data array.
#define VEC_WRAP(array_, vec_)           \
  do {                                   \
    vec_.length = ARRAYSIZE(array_);     \
    vec_.variable_len = 0;               \
    vec_.max_length = ARRAYSIZE(array_); \
    vec_.d = array_;                     \
  } while (0)

// Stack-allocate a vector of length N_ (must be a constant) called
// name_ and initialize it to the third argument.
#define VEC_INIT(N_, name_, ...)          \
  CHECK_VEC_INIT_SIZE((N_), __VA_ARGS__); \
  double name_##_data_[N_] = __VA_ARGS__; \
  Vec name_ = {(N_), name_##_data_, 0, (N_)}

// Stack-allocate a vector of length N_ called name_, and initialize
// the array by copying the elements in vec_ptr_.
#define VEC_CLONE(N_, name_, vec_ptr_)                                       \
  double name_##_data_[N_];                                                  \
  memcpy(&(name_##_data_[0]), vec_ptr_, (N_) * sizeof(double)); /* NOLINT */ \
  Vec name_ = {(N_), &(name_##_data_[0]), 0, (N_)}

#define VEC_DISP(v)                                    \
  do {                                                 \
    printf("\n%s:%u: %s = [", __FILE__, __LINE__, #v); \
    printf("%.12f", (v).d[0]);                         \
    for (int32_t i = 1; i < (v).length; ++i) {         \
      printf(", %.12f", (v).d[i]);                     \
    }                                                  \
    printf("]");                                       \
  } while (0)

bool VecIsSize(const Vec *v, int32_t l);
double *VecPtr(const Vec *v, int32_t i);
double VecGet(const Vec *v, int32_t i);
const Vec *VecZero(Vec *v);
const Vec *VecAxpy(double a, const Vec *v, Vec *v_out);
bool VecResize(int32_t length, Vec *v);
const Vec *VecCopy(const Vec *v_in, Vec *v_out);
const Vec *VecInit(const double a_in[], int32_t length, Vec *v_out);
const Vec *VecScale(const Vec *v_in, double scale, Vec *v_out);
const Vec *VecAdd(const Vec *v0, const Vec *v1, Vec *v_out);
const Vec *VecAdd3(const Vec *v0, const Vec *v1, const Vec *v2, Vec *v_out);
const Vec *VecSub(const Vec *v0, const Vec *v1, Vec *v_out);
const Vec *VecLinComb(double c0, const Vec *v0, double c1, const Vec *v1,
                      Vec *v_out);
const Vec *VecLinComb3(double c0, const Vec *v0, double c1, const Vec *v1,
                       double c2, const Vec *v2, Vec *v_out);
const Vec *VecMult(const Vec *v0, const Vec *v1, Vec *v_out);
double VecDot(const Vec *v0, const Vec *v1);
double VecNorm(const Vec *v);
double VecNormSquared(const Vec *v);
double VecNormBound(const Vec *v, double low);
const Vec *VecNormalize(const Vec *v_in, Vec *v_out);
const Vec *VecSlice(const Vec *v_in, const int32_t inds[], Vec *v_out);
const Vec *VecSliceSet(const Vec *v_in, const int32_t inds[], Vec *v_out);

// Operations on MxN matrices.

#define MAT_MAX_ELEMENTS (16 * 16)

typedef struct {
  int32_t nr, nc;
  double *d;
  int32_t variable_dim;
  int32_t max_size;
} Mat;

#if !defined(__cplusplus) && (__STDC_VERSION__ >= 199901L)
#define CHECK_MAT_INIT_SIZE(M_, N_, ...)                                     \
  assert((M_) * (N_) ==                                                      \
             sizeof((double[][N_])__VA_ARGS__) / sizeof(double) /* NOLINT */ \
         || (sizeof((double[][N_])__VA_ARGS__) / sizeof(double) ==           \
                 (N_) /* NOLINT */                                           \
             && ((double[][N_])__VA_ARGS__)[0][0] == 0.0))
#elif defined(__cplusplus) && !defined(NDEBUG)
#define CHECK_MAT_INIT_SIZE(M_, N_, ...)                                 \
  do {                                                                   \
    double va_args_[][N_] = __VA_ARGS__;                                 \
    assert((M_) * (N_) == sizeof(va_args_) / sizeof(double) /* NOLINT */ \
           || (sizeof(va_args_) / sizeof(double) == (N_)    /* NOLINT */ \
               && fabs(va_args_[0][0]) <= DBL_EPSILON));                 \
  } while (0)
#else
#define CHECK_MAT_INIT_SIZE(M_, N_, ...) \
  do {                                   \
  } while (0)
#endif

// Stack-allocate an m_ by n_ matrix called name_.  We actually
// allocate MAT_MAX_ELEMENTS, so m_ and n_ don't have to be
// constants.
#define MAT(m_, n_, name_)                 \
  assert((m_) * (n_) <= MAT_MAX_ELEMENTS); \
  double name_##_data_[MAT_MAX_ELEMENTS];  \
  Mat name_ = {(m_), (n_), name_##_data_, 1, MAT_MAX_ELEMENTS}

// Stack-allocate an m_ by n_ matrix called name_, and initialize the
// array to the initialization list passed in through __VA_ARGS__.
#define MAT_INIT(M_, N_, name_, ...)            \
  CHECK_MAT_INIT_SIZE((M_), (N_), __VA_ARGS__); \
  double name_##_data_[M_][N_] = __VA_ARGS__;   \
  Mat name_ = {(M_), (N_), &(name_##_data_[0][0]), 0, (M_) * (N_)}

// Stack-allocate an m_ by n_ matrix called name_, and initialize the
// array by copying the elements in mat_ptr_.
#define MAT_CLONE(M_, N_, name_, mat_ptr_)           \
  double name_##_data_[M_][N_];                      \
  memcpy(&(name_##_data_[0][0]), mat_ptr_,           \
         (M_) * (N_) * sizeof(double)); /* NOLINT */ \
  Mat name_ = {(M_), (N_), &(name_##_data_[0][0]), 0, (M_) * (N_)}

bool MatIsSize(const Mat *m, int32_t nr, int32_t nc);
double *MatPtr(const Mat *m, int32_t i, int32_t j);
double MatGet(const Mat *m, int32_t i, int32_t j);
bool MatResize(int32_t nr, int32_t nc, Mat *m);
const Mat *MatInit(const double *d, int32_t nr, int32_t nc, Mat *m);
const Mat *MatScale(const Mat *m_in, double scale, Mat *m_out);
const Mat *MatZero(Mat *m);
const Mat *MatCopy(const Mat *a, Mat *b);
const Mat *MatSubmatSet(const Mat *src, int32_t start_row, int32_t start_col,
                        int32_t num_rows, int32_t num_cols, int32_t dest_row,
                        int32_t dest_col, Mat *dest);
const Mat *MatI(int32_t n, Mat *m);
const Mat *MatMult(const Mat *m1, const Mat *m2, Mat *m_out);
const Vec *MatVecGenMult(TransposeType ta, double alpha, const Mat *a,
                         const Vec *x, double beta, const Vec *y, Vec *z);
const Mat *MatGenMult(TransposeType ta, TransposeType tb, double alpha,
                      const Mat *a, const Mat *b, double beta, const Mat *c,
                      Mat *d);
const Mat *MatMult3(const Mat *m0, const Mat *m1, const Mat *m2, Mat *m_out);
const Vec *MatVecMult(const Mat *m, const Vec *v_in, Vec *v_out);
const Vec *MatTransVecMult(const Mat *m, const Vec *v_in, Vec *v_out);
const Mat *MatTrans(const Mat *m, Mat *m_out);
const Mat *MatAdd(const Mat *m0, const Mat *m1, Mat *m_out);
const Mat *MatSub(const Mat *m0, const Mat *m1, Mat *m_out);
void MatQrDecomp(const Mat *A, Mat *Q, Mat *R);
int32_t MatThinSvDecomp(const Mat *A, Mat *U, Vec *s, Mat *V);
int32_t MatRank(const Mat *A, double tol);
int32_t MatVecLeftDivide(const Mat *A, const Vec *b, Vec *x);
int32_t MatVecRightDivide(const Mat *A, const Vec *b, Vec *x);
int32_t MatMatLeftDivide(const Mat *A, const Mat *B, Mat *X);
int32_t MatMatRightDivide(const Mat *A, const Mat *B, Mat *X);
bool MatIsUpperTriangular(const Mat *r);
bool MatIsLowerTriangular(const Mat *l);
bool MatHasNonnegDiag(const Mat *m);
void MatSqrtSum(const Mat *A, const Mat *B, Mat *sqrt_P);
int32_t MatMatBackSub(const Mat *R, const Mat *b, Mat *x);
int32_t MatMatForwardSub(const Mat *L, const Mat *b, Mat *x);
int32_t MatVecBackSub(const Mat *R, const Vec *b, Vec *x);
int32_t MatVecForwardSub(const Mat *L, const Vec *b, Vec *x);
const Mat *MatSlice(const Mat *m, const int32_t rows[], const int32_t cols[],
                    Mat *m_out);

const double *MatArrMult(const double *m1, int32_t nr1, int32_t nc1,
                         const double *m2, int32_t nc2, double *m_out);
const double *MatArrGemv(TransposeType ta, double alpha, const double *a,
                         int32_t nra, int32_t nca, const double *x, double beta,
                         double *y);
const double *MatArrGemm(TransposeType ta, TransposeType tb, double alpha,
                         const double *a, int32_t nra, int32_t nca,
                         const double *b, int32_t ncb, double beta, double *c);
void MatArrCopy(const double *d_in, int32_t nr, int32_t nc, double *d_out);
void MatArrTrans(const double *m_in, int32_t nr, int32_t nc, double *m_out);
const double *MatArrZero(int32_t nr, int32_t nc, double *m_out);
const double *MatArrI(int32_t nr, double *m_out);
void MatArrQrDecomp(const double *a, int32_t nr, int32_t nc, double *q,
                    double *r);
bool MatArrIsUpperTriangular(const double *r, int32_t nr, int32_t nc);
bool MatArrIsLowerTriangular(const double *l, int32_t nr, int32_t nc);
int32_t MatArrBackSub(const double *r, int32_t nr, int32_t nc, const double *b,
                      int32_t nc_b, double *x);
int32_t MatArrForwardSub(const double *l, int32_t nr, int32_t nc,
                         const double *b, int32_t nc_b, double *x);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // COMMON_C_MATH_LINALG_H_

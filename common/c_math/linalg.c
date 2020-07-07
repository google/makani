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

#include "common/c_math/linalg.h"

#include <assert.h>
#include <float.h>
#include <math.h>
#include <stddef.h>
#include <stdint.h>

#include <gsl/gsl_cblas.h>
#include <gsl/gsl_errno.h>
#include <gsl/gsl_linalg.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_permute_vector.h>
#include <gsl/gsl_vector.h>

#include "common/c_math/gsl_linalg_extra.h"
#include "common/c_math/linalg_common.h"

#if defined(__arm__)
// Turn off any error handling by the GSL library.  The standard way
// of doing this is with gsl_set_error_handler_off; however this
// method does not require bringing in more components of GSL.
void gsl_error(const char *reason, const char *file, int line, int gsl_errno) {
  (void)reason;
  (void)file;
  (void)line;
  (void)gsl_errno;
}
#endif  // defined(__arm__)

// Operations on vectors with any number of elements.

bool VecIsSize(const Vec *v, int32_t l) { return v != NULL && l == v->length; }

double *VecPtr(const Vec *v, int32_t i) {
  assert(0 <= i && i < v->length);
  return &v->d[i];
}

double VecGet(const Vec *v, int32_t i) {
  assert(0 <= i && i < v->length);
  return v->d[i];
}

// v = [0, 0, ... ]
const Vec *VecZero(Vec *v) {
  for (int32_t i = 0; i < v->length; ++i) {
    v->d[i] = 0.0;
  }
  return v;
}

// v_out = a*v + v_out
const Vec *VecAxpy(double a, const Vec *v, Vec *v_out) {
  assert(v->length == v_out->length);
  for (int32_t i = 0; i < v->length; ++i) {
    v_out->d[i] += a * v->d[i];
  }
  return v_out;
}

// Changes the length of a vector.  Returns true on success, false
// otherwise.
bool VecResize(int32_t length, Vec *v) {
  if (v->length == length) {
    return true;
  } else if (v->variable_len && 0 <= length && length <= v->max_length) {
    v->length = length;
    return true;
  }
  assert(false);
  return false;
}

// v_out = v_in
const Vec *VecCopy(const Vec *v_in, Vec *v_out) {
  VecResize(v_in->length, v_out);
  for (int32_t i = 0; i < v_out->length; ++i) {
    v_out->d[i] = v_in->d[i];
  }
  return v_out;
}

// Copy the contents of an array in a Vec.
const Vec *VecInit(const double a_in[], int32_t length, Vec *v_out) {
  VecResize(length, v_out);
  for (int32_t i = 0; i < v_out->length; ++i) {
    v_out->d[i] = a_in[i];
  }
  return v_out;
}

// v_out = scale * v_in
const Vec *VecScale(const Vec *v_in, double scale, Vec *v_out) {
  VecResize(v_in->length, v_out);
  for (int32_t i = 0; i < v_in->length; ++i) {
    v_out->d[i] = scale * v_in->d[i];
  }
  return v_out;
}

// v_out = v0 + v1
const Vec *VecAdd(const Vec *v0, const Vec *v1, Vec *v_out) {
  assert(v0->length == v1->length);
  VecResize(v0->length, v_out);
  for (int32_t i = 0; i < v_out->length; ++i) {
    v_out->d[i] = v0->d[i] + v1->d[i];
  }
  return v_out;
}

// v_out = v0 + v1 + v2
const Vec *VecAdd3(const Vec *v0, const Vec *v1, const Vec *v2, Vec *v_out) {
  assert(v0->length == v1->length && v0->length == v2->length);
  VecResize(v0->length, v_out);
  for (int32_t i = 0; i < v_out->length; ++i) {
    v_out->d[i] = v0->d[i] + v1->d[i] + v2->d[i];
  }
  return v_out;
}

// v_out = v0 - v1
const Vec *VecSub(const Vec *v0, const Vec *v1, Vec *v_out) {
  assert(v0->length == v1->length);
  VecResize(v0->length, v_out);
  for (int32_t i = 0; i < v_out->length; ++i) {
    v_out->d[i] = v0->d[i] - v1->d[i];
  }
  return v_out;
}

// v_out = c0 * v0 + c1 * v1
const Vec *VecLinComb(double c0, const Vec *v0, double c1, const Vec *v1,
                      Vec *v_out) {
  assert(v0->length == v1->length);
  VecResize(v0->length, v_out);
  for (int32_t i = 0; i < v_out->length; ++i) {
    v_out->d[i] = c0 * v0->d[i] + c1 * v1->d[i];
  }
  return v_out;
}

// v_out = c0 * v0 + c1 * v1 + c2 * v2
const Vec *VecLinComb3(double c0, const Vec *v0, double c1, const Vec *v1,
                       double c2, const Vec *v2, Vec *v_out) {
  assert(v0->length == v1->length && v0->length == v2->length);
  VecResize(v0->length, v_out);
  for (int32_t i = 0; i < v_out->length; ++i) {
    v_out->d[i] = c0 * v0->d[i] + c1 * v1->d[i] + c2 * v2->d[i];
  }
  return v_out;
}

// v_out = v0 .* v1
const Vec *VecMult(const Vec *v0, const Vec *v1, Vec *v_out) {
  assert(v0->length == v1->length);
  VecResize(v0->length, v_out);
  for (int32_t i = 0; i < v_out->length; ++i) {
    v_out->d[i] = v0->d[i] * v1->d[i];
  }
  return v_out;
}

// v0^T * v1
double VecDot(const Vec *v0, const Vec *v1) {
  assert(v0->length == v1->length);
  double tmp = 0.0;
  for (int32_t i = 0; i < v0->length; ++i) {
    tmp += v0->d[i] * v1->d[i];
  }
  return tmp;
}

// Computes the 2-norm of a vector.
//
// TODO: This does not yet handle overflow and underflow
// gracefully.
double VecNorm(const Vec *v) { return sqrt(VecNormSquared(v)); }

double VecNormBound(const Vec *v, double low) { return fmax(low, VecNorm(v)); }

double VecNormSquared(const Vec *v) { return VecDot(v, v); }

const Vec *VecNormalize(const Vec *v_in, Vec *v_out) {
  return VecScale(v_in, 1.0 / VecNormBound(v_in, 1e-9), v_out);
}

// Return the elements of v_in where inds != 0. The MATLAB logical indexing
// equivalent is: v_out = v_in(inds).
const Vec *VecSlice(const Vec *v_in, const int32_t inds[], Vec *v_out) {
  int32_t i_out = 0;
  for (int32_t i = 0; i < v_in->length; ++i) {
    if (inds[i]) {
      assert(i_out < v_out->max_length);
      // Don't use VecPtr here because we haven't set v_out->length yet.
      v_out->d[i_out] = VecGet(v_in, i);
      i_out++;
    }
  }

  // Do not resize v_out until the end of the function if you want to
  // be able to reuse and input as an output.
  VecResize(i_out, v_out);
  return v_out;
}

// Assigns elements of a larger vector with those from a smaller one. The
// MATLAB logical indexing equivalent is: v_out(inds) = v_in. The sum(inds != 0)
// must be equal to length(v_in). Warning: there is no check that
// length(inds) == length(v_out).
const Vec *VecSliceSet(const Vec *v_in, const int32_t inds[], Vec *v_out) {
  assert(v_in != v_out);

  int32_t i_in = 0;
  for (int32_t i = 0; i < v_out->length; ++i) {
    if (inds[i]) {
      *VecPtr(v_out, i) = VecGet(v_in, i_in);
      i_in++;
    }
  }

  // Assert that the sum(inds != 0) is equal to length(v_in).
  assert(v_in->length == i_in);

  return v_out;
}

// Operations on arbitrary matrices.

bool MatIsSize(const Mat *m, int32_t nr, int32_t nc) {
  return m != NULL && m->nr == nr && m->nc == nc;
}

double *MatPtr(const Mat *m, int32_t i, int32_t j) {
  assert(i < m->nr && j < m->nc);
  return &m->d[i * m->nc + j];
}

double MatGet(const Mat *m, int32_t i, int32_t j) {
  assert(i < m->nr && j < m->nc);
  return m->d[i * m->nc + j];
}

bool MatResize(int32_t nr, int32_t nc, Mat *m) {
  if (m->nr == nr && m->nc == nc) {
    return true;
  } else if (m->variable_dim && nr * nc <= m->max_size) {
    m->nr = nr;
    m->nc = nc;
    return true;
  }
  assert(false);
  return false;
}

// Copy the contents of an array into a Mat.
const Mat *MatInit(const double *d, int32_t nr, int32_t nc, Mat *m) {
  MatResize(nr, nc, m);
  for (int32_t i = 0; i < m->nr; ++i) {
    for (int32_t j = 0; j < m->nc; ++j) {
      *MatPtr(m, i, j) = d[i * m->nc + j];
    }
  }
  return m;
}

// m_out = scale * m_in
const Mat *MatScale(const Mat *m_in, double scale, Mat *m_out) {
  MatResize(m_in->nr, m_in->nc, m_out);
  for (int32_t i = 0; i < m_out->nr; ++i) {
    for (int32_t j = 0; j < m_out->nc; ++j) {
      *MatPtr(m_out, i, j) = scale * MatGet(m_in, i, j);
    }
  }
  return m_out;
}

const Mat *MatZero(Mat *m) {
  MatArrZero(m->nr, m->nc, m->d);
  return m;
}

const Mat *MatCopy(const Mat *a, Mat *b) {
  if (a != b) {  // No work to do if the Mat pointers are equal.
    MatResize(a->nr, a->nc, b);
    // This is still safe if the data pointers are equal.
    MatArrCopy(a->d, a->nr, a->nc, b->d);
  }
  return b;
}

// Copies the sub-matrix src(start_row + (1:num_rows), start_col + (1:num_cols))
// into dest(dest_row + (1:num_rows), dest_col + (1:num_cols)).
//
// It is not safe for src and dest to share any memory.
//
// Returns:
//   A pointer to dest.
const Mat *MatSubmatSet(const Mat *src, int32_t start_row, int32_t start_col,
                        int32_t num_rows, int32_t num_cols, int32_t dest_row,
                        int32_t dest_col, Mat *dest) {
  assert(src != dest && src->d != dest->d);
  assert(start_row >= 0 && start_col >= 0);
  assert(num_rows >= 0 && num_cols >= 0);
  assert(dest_row >= 0 && dest_col >= 0);
  // Make sure there is sufficient data and sufficient room.
  assert(src->nr >= start_row + num_rows && src->nc >= start_col + num_cols);
  assert(dest->nr >= dest_row + num_rows && dest->nc >= dest_col + num_cols);

  for (int32_t i = 0; i < num_rows; ++i) {
    for (int32_t j = 0; j < num_cols; ++j) {
      *MatPtr(dest, dest_row + i, dest_col + j) =
          MatGet(src, i + start_row, j + start_col);
    }
  }
  return dest;
}

const Mat *MatI(int32_t n, Mat *m) {
  MatResize(n, n, m);
  MatArrI(n, m->d);
  return m;
}

const Mat *MatMult(const Mat *m1, const Mat *m2, Mat *m_out) {
  assert(m1->nc == m2->nr);
  MatResize(m1->nr, m2->nc, m_out);
  MatArrMult(m1->d, m1->nr, m1->nc, m2->d, m2->nc, m_out->d);
  return m_out;
}

// Generalized matrix-vector multiply.
//
//   z = alpha*op(a)*x + beta*y.
//
// Args:
//   ta: Indicates if a should be transposed.
//   alpha, a, x, beta: See formula above.
//   y: Can be NULL if beta = 0.0.
//   z: Resulting vector.  It is safe to reuse y as z.
const Vec *MatVecGenMult(TransposeType ta, double alpha, const Mat *a,
                         const Vec *x, double beta, const Vec *y, Vec *z) {
  assert(a != NULL && x != NULL && z != NULL);
  assert((ta == kNoTrans ? a->nc : a->nr) == x->length);
  const int32_t zl = (ta == kNoTrans) ? a->nr : a->nc;
  assert(y == NULL || y->length == zl);
  assert(y != NULL || beta == 0.0);
  if (y == NULL) beta = 0.0;

  if (y == NULL || beta == 0.0) {
    VecResize(zl, z);
  } else {
    VecCopy(y, z);
  }

  MatArrGemv(ta, alpha, a->d, a->nr, a->nc, x->d, beta, z->d);
  return z;
}

// Generalized matrix-matrix multiply.
//
//   d = alpha*op(a)*op(b) + beta*c.
//
// Args:
//   ta: Indicates if a should be transposed.
//   tb: Indicates if b should be transposed.
//   alpha, a, b, beta: See above formula.
//   c: Can be NULL if beta = 0.0.
//   d: Resulting matrix.  It is safe to reuse c as d.
const Mat *MatGenMult(TransposeType ta, TransposeType tb, double alpha,
                      const Mat *a, const Mat *b, double beta, const Mat *c,
                      Mat *d) {
  assert(a != NULL && b != NULL && d != NULL);
  assert((ta == kNoTrans ? a->nc : a->nr) == (tb == kNoTrans ? b->nr : b->nc));
  const int32_t nr_d = (ta == kNoTrans ? a->nr : a->nc);
  const int32_t nc_d = (tb == kNoTrans ? b->nc : b->nr);
  assert(c == NULL || (c->nr == nr_d && c->nc == nc_d));
  assert(c != NULL || beta == 0.0);
  if (c == NULL) beta = 0.0;

  if (c == NULL || beta == 0.0)
    MatResize(nr_d, nc_d, d);
  else
    MatCopy(c, d);

  MatArrGemm(ta, tb, alpha, a->d, a->nr, a->nc, b->d, nc_d, beta, d->d);
  return d;
}

const Mat *MatMult3(const Mat *m0, const Mat *m1, const Mat *m2, Mat *m_out) {
  assert(m0->nc == m1->nr && m1->nc == m2->nr);
  MatResize(m0->nr, m2->nc, m_out);
  MAT(m0->nr, m1->nc, m0m1);
  return MatMult(MatMult(m0, m1, &m0m1), m2, m_out);
}

const Vec *MatVecMult(const Mat *m, const Vec *v_in, Vec *v_out) {
  assert(m->nc == v_in->length);
  VecResize(m->nr, v_out);
  MatArrMult(m->d, m->nr, m->nc, v_in->d, 1, v_out->d);
  return v_out;
}

const Vec *MatTransVecMult(const Mat *m, const Vec *v_in, Vec *v_out) {
  assert(m->nr == v_in->length);
  VecResize(m->nc, v_out);
  MatArrGemm(kTrans, kNoTrans, 1.0, m->d, m->nr, m->nc, v_in->d, 1, 0.0,
             v_out->d);
  return v_out;
}

const Mat *MatTrans(const Mat *m, Mat *m_out) {
  MatResize(m->nc, m->nr, m_out);
  MatArrTrans(m->d, m->nr, m->nc, m_out->d);
  return m_out;
}

const Mat *MatAdd(const Mat *m0, const Mat *m1, Mat *m_out) {
  assert(m0->nr == m1->nr && m0->nc == m1->nc);
  MatResize(m0->nr, m0->nc, m_out);
  for (int32_t i = 0; i < m0->nr; ++i) {
    for (int32_t j = 0; j < m0->nc; ++j) {
      *MatPtr(m_out, i, j) = MatGet(m0, i, j) + MatGet(m1, i, j);
    }
  }
  return m_out;
}

const Mat *MatSub(const Mat *m0, const Mat *m1, Mat *m_out) {
  assert(m0->nr == m1->nr && m0->nc == m1->nc);
  MatResize(m0->nr, m0->nc, m_out);
  for (int32_t i = 0; i < m0->nr; ++i) {
    for (int32_t j = 0; j < m0->nc; ++j) {
      *MatPtr(m_out, i, j) = MatGet(m0, i, j) - MatGet(m1, i, j);
    }
  }
  return m_out;
}

void MatQrDecomp(const Mat *A, Mat *Q, Mat *R) {
  MatResize(A->nr, A->nc, R);
  double *Qd = NULL;
  if (Q != NULL) {
    MatResize(A->nr, A->nr, Q);
    Qd = Q->d;
  }
  MatArrQrDecomp(A->d, A->nr, A->nc, Qd, R->d);
}

// Calculates the "thin" singular value decomposition of an m x n
// matrix A, defined by:
//
//   A = U * S * V'
//
// where U is an m x n column-orthogonal matrix (i.e. U'*U = I), S is
// an n x n diagonal matrix with non-decreasing elements, and V is an
// n x n orthogonal matrix (i.e. V'*V = V*V' = I).  The "thin" aspect
// is equivalent to MATLAB's "economy" SVD.
//
// Args:
//   A: Input m x n matrix with m >= n.
//   U: Output m x n column-orthogonal matrix.
//   s: Output n length vector of the diagonal elements of S.
//   V: Output n x n orthogonal matrix.
//
// Returns:
//   kLinalgErrorNone on success and kLinalgErrorMaxIter if GSL's
//   solver reached its maximum number of iterations.
int32_t MatThinSvDecomp(const Mat *A, Mat *U, Vec *s, Mat *V) {
  assert(A != NULL && U != NULL && s != NULL && V != NULL);
  assert(A->nc <= VEC_MAX_ELEMENTS);
  assert(A->nr >= A->nc);

  MatCopy(A, U);
  VecResize(A->nc, s);
  MatResize(A->nc, A->nc, V);

  double work_data[VEC_MAX_ELEMENTS];
  gsl_matrix_view U_gsl =
      gsl_matrix_view_array(U->d, (size_t)U->nr, (size_t)U->nc);
  gsl_matrix_view V_gsl =
      gsl_matrix_view_array(V->d, (size_t)V->nr, (size_t)V->nc);
  gsl_vector_view s_gsl = gsl_vector_view_array(s->d, (size_t)s->length);
  gsl_vector_view work = gsl_vector_view_array(work_data, (size_t)s->length);
  int32_t ret = gsl_linalg_SV_decomp(&U_gsl.matrix, &V_gsl.matrix,
                                     &s_gsl.vector, &work.vector);
  // These are the only errors that gsl_linalg_SV_decomp should ever
  // return, assuming we're passing in matrices and vectors of the
  // appropriate sizes.
  assert(ret == GSL_SUCCESS || ret == GSL_EMAXITER);

  if (ret == GSL_EMAXITER) {
    assert(false);
    return kLinalgErrorMaxIter;
  }
  return kLinalgErrorNone;
}

// Calculates the rank of a matrix.  Performs a QR decomposition (with
// pivoting) of the input matrix and finds the number of diagonal
// elements of R with magnitude greater than a specified tolerance.
//
// Args:
//   A: Input matrix.
//   tol: Tolerance for checking linear independence of vectors.
//
// Returns:
//   Rank of the matrix.
int32_t MatRank(const Mat *A, double tol) {
  assert(A != NULL);
  assert(A->nr * A->nc <= MAT_MAX_ELEMENTS);
  assert(A->nr <= VEC_MAX_ELEMENTS && A->nc <= VEC_MAX_ELEMENTS);
  assert(tol >= DBL_EPSILON);

  int32_t min_dim = (A->nr < A->nc) ? A->nr : A->nc;
  if (min_dim == 0) return 0;

  double qr_data[MAT_MAX_ELEMENTS];
  double tau_data[VEC_MAX_ELEMENTS];
  double norm_data[VEC_MAX_ELEMENTS];
  size_t perm_data[VEC_MAX_ELEMENTS];
  int signum;
  gsl_matrix_const_view A_gsl =
      gsl_matrix_const_view_array(A->d, (size_t)A->nr, (size_t)A->nc);
  gsl_matrix_view QR =
      gsl_matrix_view_array(qr_data, (size_t)A->nr, (size_t)A->nc);
  gsl_vector_view tau = gsl_vector_view_array(tau_data, (size_t)min_dim);
  gsl_permutation p = {(size_t)A->nc, perm_data};
  gsl_vector_view norm = gsl_vector_view_array(norm_data, (size_t)A->nc);

  gsl_matrix_memcpy(&QR.matrix, &A_gsl.matrix);
  int32_t ret = gsl_linalg_QRPT_decomp(&QR.matrix, &tau.vector, &p, &signum,
                                       &norm.vector);
  assert(ret == GSL_SUCCESS);
  (void)ret;

  int32_t rank = 0;
  while (rank < min_dim) {
    if (fabs(gsl_matrix_get(&QR.matrix, (size_t)rank, (size_t)rank)) <= tol) {
      break;
    }
    rank++;
  }

  return rank;
}

// Solves A*x = b for x similar to x = A \ b.
int32_t MatVecLeftDivide(const Mat *A, const Vec *b, Vec *x) {
  assert(A != NULL && b != NULL && x != NULL);
  assert(b != x);
  assert(A->nr == b->length && A->nr > 0);
  VecResize(A->nc, x);
  if (A->nc == 0) return kLinalgErrorNone;

  size_t m = (size_t)A->nr, n = (size_t)A->nc;
  gsl_matrix_const_view A_gsl = gsl_matrix_const_view_array(A->d, m, n);
  gsl_matrix_const_view b_gsl = gsl_matrix_const_view_array(b->d, m, 1U);
  gsl_matrix_view x_gsl = gsl_matrix_view_array(x->d, n, 1U);
  int32_t ret =
      GslMatrixDivide(CblasLeft, &A_gsl.matrix, &b_gsl.matrix, &x_gsl.matrix);
  assert(ret == GSL_SUCCESS || ret == GSL_ESING);
  return (ret == GSL_ESING) ? kLinalgErrorSingularMat : kLinalgErrorNone;
}

// Solves x*A = b for x similar to x = b / A.
int32_t MatVecRightDivide(const Mat *A, const Vec *b, Vec *x) {
  assert(A != NULL && b != NULL && x != NULL);
  assert(b != x);
  assert(A->nc == b->length && A->nc > 0);
  VecResize(A->nr, x);
  if (A->nr == 0) return kLinalgErrorNone;

  size_t m = (size_t)A->nr, n = (size_t)A->nc;
  gsl_matrix_const_view A_gsl = gsl_matrix_const_view_array(A->d, m, n);
  gsl_matrix_const_view b_gsl = gsl_matrix_const_view_array(b->d, 1U, n);
  gsl_matrix_view x_gsl = gsl_matrix_view_array(x->d, 1U, m);
  int32_t ret =
      GslMatrixDivide(CblasRight, &A_gsl.matrix, &b_gsl.matrix, &x_gsl.matrix);
  assert(ret == GSL_SUCCESS || ret == GSL_ESING);
  return (ret == GSL_ESING) ? kLinalgErrorSingularMat : kLinalgErrorNone;
}

// Solves A*X = b for X similar to X = A \ B.
int32_t MatMatLeftDivide(const Mat *A, const Mat *B, Mat *X) {
  assert(A != NULL && B != NULL && X != NULL);
  assert(A != X && B != X);
  assert(A->nr == B->nr && A->nr > 0);
  MatResize(A->nc, B->nc, X);
  if (A->nc == 0) return kLinalgErrorNone;

  size_t m = (size_t)A->nr, n = (size_t)A->nc, k = (size_t)B->nc;
  gsl_matrix_const_view A_gsl = gsl_matrix_const_view_array(A->d, m, n);
  gsl_matrix_const_view B_gsl = gsl_matrix_const_view_array(B->d, m, k);
  gsl_matrix_view X_gsl = gsl_matrix_view_array(X->d, n, k);
  int32_t ret =
      GslMatrixDivide(CblasLeft, &A_gsl.matrix, &B_gsl.matrix, &X_gsl.matrix);
  assert(ret == GSL_SUCCESS || ret == GSL_ESING);
  return (ret == GSL_ESING) ? kLinalgErrorSingularMat : kLinalgErrorNone;
}

// Solves X*A = B for X similar to X = B / A.
int32_t MatMatRightDivide(const Mat *A, const Mat *B, Mat *X) {
  assert(A != NULL && B != NULL && X != NULL);
  assert(A != X && B != X);
  assert(A->nc == B->nc && A->nc > 0);
  MatResize(B->nr, A->nr, X);
  if (A->nr == 0) return kLinalgErrorNone;

  size_t m = (size_t)A->nr, n = (size_t)A->nc, k = (size_t)B->nr;
  gsl_matrix_const_view A_gsl = gsl_matrix_const_view_array(A->d, m, n);
  gsl_matrix_const_view B_gsl = gsl_matrix_const_view_array(B->d, k, n);
  gsl_matrix_view X_gsl = gsl_matrix_view_array(X->d, k, m);
  int32_t ret =
      GslMatrixDivide(CblasRight, &A_gsl.matrix, &B_gsl.matrix, &X_gsl.matrix);
  assert(ret == GSL_SUCCESS || ret == GSL_ESING);
  return (ret == GSL_ESING) ? kLinalgErrorSingularMat : kLinalgErrorNone;
}

bool MatIsUpperTriangular(const Mat *r) {
  return MatArrIsUpperTriangular(r->d, r->nr, r->nc);
}

bool MatIsLowerTriangular(const Mat *l) {
  return MatArrIsLowerTriangular(l->d, l->nr, l->nc);
}

bool MatHasNonnegDiag(const Mat *m) {
  assert(m != NULL);
  for (int32_t i = 0; i < m->nc && i < m->nr; ++i) {
    if (MatGet(m, i, i) < 0.0) return false;
  }
  return true;
}

// Calculates the matrix square root of P = A'*A + B'*B by calculating a QR
// decomposition, Q*R = [A ; B].  It is assumed that the matrix [A ; B] is full
// row rank (i.e. has linearly independent columns).
//
// Q is an orthogonal matrix and R "trapezoidal matrix", i.e. R = [sqrt_P ; 0]
// where sqrt_P is upper triangular.  Thus:
//   P = A'*A + B'*B = [A ; B]'*[A ; B] = [sqrT_P ; 0]'*Q'*Q*[sqrt_P ; 0].
//
// Args:
//   A: na-by-nc matrix.
//   B: nb-by-nc matrix.
//   sqrt_P: Upper-triangular matrix with positive diagonal elements such that
//     sqrt_P'*sqrt_P = P.
//
// Note: it is safe to reuse A or B as sqrt_P (assuming shapes match).
void MatSqrtSum(const Mat *A, const Mat *B, Mat *sqrt_P) {
  assert(A != NULL && B != NULL && sqrt_P != NULL);
  assert(A->nc == B->nc);
  assert(A->nr + B->nr >= A->nc);  // This is required for full row rank.
  MatResize(A->nc, A->nc, sqrt_P);

  // Define a temporary matrix, stack = [A ; B].
  MAT(A->nr + B->nr, A->nc, stack);
  MatSubmatSet(A, 0, 0, A->nr, A->nc, 0, 0, &stack);
  MatSubmatSet(B, 0, 0, B->nr, B->nc, A->nr, 0, &stack);
  MatQrDecomp(&stack, NULL, &stack);
  MatSubmatSet(&stack, 0, 0, A->nc, A->nc, 0, 0, sqrt_P);
  // Next, enforce a non-negative diagonal.
  for (int32_t i = 0; i < sqrt_P->nr; ++i) {
    if (MatGet(sqrt_P, i, i) < 0.0) {
      for (int32_t j = i; j < sqrt_P->nc; ++j) {
        *MatPtr(sqrt_P, i, j) *= -1.0;
      }
    }
  }
}

int32_t MatMatBackSub(const Mat *R, const Mat *b, Mat *x) {
  assert(b->nr == R->nr && b->nc == x->nc);
  assert(x->nr == R->nc);
  return MatArrBackSub(R->d, R->nr, R->nc, b->d, b->nc, x->d);
}

int32_t MatMatForwardSub(const Mat *L, const Mat *b, Mat *x) {
  assert(b->nr == L->nr && b->nc == x->nc);
  assert(x->nr == L->nc);
  return MatArrForwardSub(L->d, L->nr, L->nc, b->d, b->nc, x->d);
}

int32_t MatVecBackSub(const Mat *R, const Vec *b, Vec *x) {
  assert(b->length == R->nr);
  assert(x->length == R->nc);
  return MatArrBackSub(R->d, R->nr, R->nc, b->d, 1, x->d);
}

int32_t MatVecForwardSub(const Mat *L, const Vec *b, Vec *x) {
  assert(b->length == L->nr);
  assert(x->length == L->nc);
  return MatArrForwardSub(L->d, L->nr, L->nc, b->d, 1, x->d);
}

// Returns the rows and columns of m where rows != 0 and cols != 0
const Mat *MatSlice(const Mat *m, const int32_t rows[], const int32_t cols[],
                    Mat *m_out) {
  int32_t nr_out = 0, nc_out = 0;
  if (rows == NULL) {
    nr_out = m->nr;
  } else {
    for (int32_t i = 0; i < m->nr; ++i)
      if (rows[i]) nr_out++;
  }
  if (cols == NULL) {
    nc_out = m->nc;
  } else {
    for (int32_t i = 0; i < m->nc; ++i)
      if (cols[i]) nc_out++;
  }

  if (m_out->variable_dim) {
    assert(nr_out * nc_out <= m_out->max_size);
  } else {
    assert(m_out->nr == nr_out && m_out->nc == nc_out);
  }

  int32_t i_out = 0;
  for (int32_t i = 0; i < m->nr; ++i) {
    if (rows == NULL || rows[i]) {
      int32_t j_out = 0;
      for (int32_t j = 0; j < m->nc; ++j) {
        if (cols == NULL || cols[j]) {
          // Don't use MatPtr here because we haven't set m_out->nc yet.
          m_out->d[i_out * nc_out + j_out] = MatGet(m, i, j);
          j_out++;
        }
      }
      i_out++;
    }
  }

  // Do not resize m until the end of the function if you want to be
  // able to reuse the input as the output.  Otherwise, the matrix
  // indexing in MatGet will be off.
  MatResize(nr_out, nc_out, m_out);
  return m_out;
}

const double *MatArrMult(const double *m1, int32_t nr1, int32_t nc1,
                         const double *m2, int32_t nc2, double *m_out) {
  assert(m_out != m1 && m_out != m2);
  return MatArrGemm(kNoTrans, kNoTrans, 1.0, m1, nr1, nc1, m2, nc2, 0.0, m_out);
}

// This is modeled after the BLAS function GEMV, which calculates:
//     y <-- alpha*op(A)*x + beta*y,
// where op either returns the matrix or its transpose.
//
// Args:
//   ta: Whether to transpose A.
//   alpha: See above formula.
//   a: Data for A.
//   nra: Number of rows of A without transpose applied.
//   nca: Number of columns of A without transpose applied.
//   x: Data for x.
//   beta: See above formula.
//   c: Data for y.
const double *MatArrGemv(TransposeType ta, double alpha, const double *a,
                         int32_t nra, int32_t nca, const double *x, double beta,
                         double *y) {
  assert(x != y);
  int32_t ly = ((ta == kNoTrans) ? nra : nca);
  int32_t lx = ((ta == kNoTrans) ? nca : nra);
  double aij, axj;
  for (int32_t i = 0; i < ly; ++i) {  // For each entry of y.
    axj = 0.0;
    for (int32_t j = 0; j < lx; ++j) {  // Walk down the row of op(A).
      aij = (ta == kNoTrans) ? a[i * nca + j] : a[j * nca + i];
      axj += aij * x[j];
    }
    if (beta == 0.0) {
      y[i] = alpha * axj;
    } else {
      y[i] = alpha * axj + beta * y[i];
    }
  }
  return y;
}

// This is modeled after the BLAS function GEMM, which calculates:
//     C <-- alpha*op(A)*op(B) + beta*C,
// where op either returns the matrix or its transpose.
//
// Args:
//   ta: Whether to transpose A.
//   tb: Whether to transpose B.
//   alpha: See above formula.
//   a: Data for A.
//   nra: Number of rows of A without transpose applied.
//   nca: Number of columns of A without transpose applied.
//   b: Data for B.
//   ncc: Number of columns of C.
//   beta: See above formula.
//   c: Data for C.
const double *MatArrGemm(TransposeType ta, TransposeType tb, double alpha,
                         const double *a, int32_t nra, int32_t nca,
                         const double *b, int32_t ncc, double beta, double *c) {
  assert(a != c && b != c);
  int32_t nrc = (ta == kNoTrans) ? nra : nca;
  int32_t nk = (ta == kNoTrans) ? nca : nra;
  double ABij, aik, bkj;
  for (int32_t i = 0; i < nrc; ++i) {
    for (int32_t j = 0; j < ncc; ++j) {
      ABij = 0.0;
      for (int32_t k = 0; k < nk; ++k) {
        aik = (ta == kNoTrans) ? a[i * nca + k] : a[k * nca + i];
        bkj = (tb == kNoTrans) ? b[k * ncc + j] : b[j * nk + k];
        ABij += aik * bkj;
      }
      if (beta == 0.0) {
        c[i * ncc + j] = alpha * ABij;
      } else {
        c[i * ncc + j] = alpha * ABij + beta * c[i * ncc + j];
      }
    }
  }
  return c;
}

const double *MatArrZero(int32_t nr, int32_t nc, double *m_out) {
  for (int32_t i = 0; i < nr; ++i) {
    for (int32_t j = 0; j < nc; ++j) {
      m_out[i * nc + j] = 0.0;
    }
  }
  return m_out;
}

void MatArrCopy(const double *d_in, int32_t nr, int32_t nc, double *d_out) {
  for (int32_t i = 0; i < nr; ++i) {
    for (int32_t j = 0; j < nc; ++j) {
      d_out[i * nc + j] = d_in[i * nc + j];
    }
  }
}

void MatArrTrans(const double *m_in, int32_t nr, int32_t nc, double *m_out) {
  assert(m_out != m_in && m_out != NULL);
  for (int32_t i = 0; i < nr; ++i) {
    for (int32_t j = 0; j < nc; ++j) {
      m_out[j * nr + i] = m_in[i * nc + j];
    }
  }
}

const double *MatArrI(int32_t nr, double *m_out) {
  MatArrZero(nr, nr, m_out);
  for (int32_t i = 0; i < nr; ++i) {
    m_out[i * nr + i] = 1.0;
  }
  return m_out;
}

// QR decomposition.  Decompose a matrix into orthogonal and
// upper-right-triangular components.
//
// You may reuse a as r, and you may skip calculating q by passing a
// NULL pointer.
void MatArrQrDecomp(const double *a, int32_t nr, int32_t nc, double *q,
                    double *r) {
  assert(q != a && r != q && r != NULL);
  assert(nr <= VEC_MAX_ELEMENTS && nc <= VEC_MAX_ELEMENTS);

  double tau_data[VEC_MAX_ELEMENTS];
  gsl_matrix_const_view A =
      gsl_matrix_const_view_array(a, (size_t)nr, (size_t)nc);
  gsl_matrix_view QR = gsl_matrix_view_array(r, (size_t)nr, (size_t)nc);
  gsl_vector_view tau =
      gsl_vector_view_array(tau_data, (nr < nc) ? (size_t)nr : (size_t)nc);
  gsl_matrix_memcpy(&QR.matrix, &A.matrix);
  gsl_linalg_QR_decomp(&QR.matrix, &tau.vector);

  if (q != NULL) {
    gsl_matrix_view Q = gsl_matrix_view_array(q, (size_t)nr, (size_t)nr);
    gsl_matrix_view R = gsl_matrix_view_array(r, (size_t)nr, (size_t)nc);
    gsl_linalg_QR_unpack(&QR.matrix, &tau.vector, &Q.matrix, &R.matrix);
  } else {
    for (int32_t i = 1; i < nr; ++i) {
      for (int32_t j = 0; j < i && j < nc; ++j) {
        r[i * nc + j] = 0.0;
      }
    }
  }
}

bool MatArrIsUpperTriangular(const double *r, int32_t nr, int32_t nc) {
  for (int32_t i = 1; i < nr; ++i) {
    for (int32_t j = 0; j < i && j < nc; ++j) {
      if (fabs(r[i * nc + j]) >= DBL_EPSILON) {
        return false;
      }
    }
  }
  return true;
}

bool MatArrIsLowerTriangular(const double *l, int32_t nr, int32_t nc) {
  for (int32_t i = 0; i < nr; ++i) {
    for (int32_t j = i + 1; j < nc; ++j) {
      if (fabs(l[i * nc + j]) >= DBL_EPSILON) {
        return false;
      }
    }
  }
  return true;
}

// Solves R*X = B using back substitution where R is an upper
// triangular matrix.  If the triangular matrix contains a zero
// diagonal element (i.e. the matrix is singular), we set the
// corresponding x element to zero.
//
// Returns kLinalgErrorNone on success and kLinalgErrorSingularMat
// if the matrix was singular.
//
// Note: Confirmed safe to reuse b as x.
int32_t MatArrBackSub(const double *r, int32_t nr, int32_t nc, const double *b,
                      int32_t nc_b, double *x) {
  assert(r != x);
  assert(nr >= nc);
  assert(nr >= 0 && nc >= 0 && nc_b >= 0);
  assert(MatArrIsUpperTriangular(r, nr, nc));
  (void)nr;

  int32_t err = kLinalgErrorNone;
  // Solve for the k-th column of x.
  for (int32_t k = 0; k < nc_b; ++k) {
    // Iterate over the rows of r.
    for (int32_t i = nc - 1; i >= 0; --i) {
      if (fabs(r[i * nc + i]) <= DBL_EPSILON) {
        err = kLinalgErrorSingularMat;
        x[i * nc_b + k] = 0.0;
      } else {
        double sum = 0.0;
        for (int32_t j = nc - 1; j > i; --j) {
          sum += x[j * nc_b + k] * r[i * nc + j];
        }
        x[i * nc_b + k] = (b[i * nc_b + k] - sum) / r[i * nc + i];
      }
    }
    // TODO: If nr > nc, then the bottom of r is all zeros.
    // Should we test if b has non-zero elements for these rows?
  }
  return err;
}

// Solves L*X = B using forward substitution where L is a lower
// triangular matrix.  If the triangular matrix contains a zero
// diagonal element (i.e. the matrix is singular), we set the
// corresponding x element to zero.
//
// Returns kLinalgErrorNone on success and kLinalgErrorSingularMat
// if the matrix is singular.
//
// Note: Confirmed safe to reuse b as x.
int32_t MatArrForwardSub(const double *l, int32_t nr, int32_t nc,
                         const double *b, int32_t nc_b, double *x) {
  assert(nr <= nc);
  assert(MatArrIsLowerTriangular(l, nr, nc));
  int32_t err = kLinalgErrorNone;
  for (int32_t k = 0; k < nc_b; ++k) {  // Solve for the k-th column of x.
    for (int32_t i = 0; i < nr; ++i) {  // Iterate over the rows of l.
      if (fabs(l[i * nc + i]) <= DBL_EPSILON) {
        err = kLinalgErrorSingularMat;
        x[i * nc_b + k] = 0.0;
      } else {
        double sum = 0.0;
        for (int32_t j = 0; j < i; ++j) {
          sum += x[j * nc_b + k] * l[i * nc + j];
        }
        x[i * nc_b + k] = (b[i * nc_b + k] - sum) / l[i * nc + i];
      }
    }
    for (int32_t i = nr; i < nc; ++i) {
      x[i * nc_b + k] = 0.0;
    }
  }
  return err;
}

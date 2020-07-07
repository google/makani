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

#include "common/c_math/gsl_linalg_extra.h"

#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdint.h>

#include <gsl/gsl_blas.h>
#include <gsl/gsl_blas_types.h>
#include <gsl/gsl_cblas.h>
#include <gsl/gsl_errno.h>
#include <gsl/gsl_linalg.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_permute_vector.h>
#include <gsl/gsl_vector.h>

// TODO: Remove these and allocate arrays in calling
// library.
#define MAX_VECTOR_SIZE (32U)
#define MAX_MATRIX_SIZE (16U * 16U)

// Solves triangular matrix equations of the form:
//
//   op(T) * X = B   or   X * op(T) = B
//
// using forward or back substitution.  For rectangular T
// (i.e. underdetermined or overdetermined systems), this only solves
// the square submatrix of T.  For underdetermined systems, the
// remaining elements of X are set to zeros.  If the triangular matrix
// T is singular, then this will only solve the part of the matrix up
// to the first zero diagonal element.
//
// Args:
//   side: Which side of the multiplication op(T) appears.
//   uplo: Whether T is upper or lower triangular.
//   transpose: Whether op(.) is a transpose or not.
//   T: Triangular matrix (m x n).
//   B: Right hand side matrix (m x k).
//   X: Solution matrix (n x k).
//
// Returns:
//   GSL_ESING if T is singular and GSL_SUCCESS otherwise.
int32_t GslTriangularSolve(CBLAS_SIDE_t side, CBLAS_UPLO_t uplo,
                           CBLAS_TRANSPOSE_t transpose, const gsl_matrix *T,
                           const gsl_matrix *B, gsl_matrix *X) {
  assert(T != NULL && B != NULL && X != NULL);
  assert(T != X && B != X);
  assert(side != CblasLeft ||
         (X->size2 == B->size2 &&
          (transpose == CblasNoTrans ? T->size1 : T->size2) == B->size1 &&
          (transpose == CblasNoTrans ? T->size2 : T->size1) == X->size1));
  assert(side != CblasRight ||
         (X->size1 == B->size1 &&
          (transpose == CblasNoTrans ? T->size1 : T->size2) == X->size2 &&
          (transpose == CblasNoTrans ? T->size2 : T->size1) == B->size2));

  int32_t err = GSL_SUCCESS;
  size_t small_dim = (T->size1 < T->size2) ? T->size1 : T->size2;

  gsl_matrix_set_zero(X);
  // Calling this "rank" here is assuming that the diagonal elements
  // of T are in descending order.
  size_t rank = small_dim;
  for (size_t i = 0U; i < small_dim; ++i) {
    if (fabs(gsl_matrix_get(T, i, i)) <= DBL_EPSILON) {
      rank = i;
      err = GSL_ESING;
      break;
    }
    gsl_vector_const_view row = gsl_matrix_const_row(B, i);
    gsl_matrix_set_row(X, i, &row.vector);
  }
  if (rank == 0U) return err;

  // Make a square sub-matrix of T that has all non-zero elements
  // along the diagonal and a reduced B matrix with the same number of
  // rows.
  gsl_matrix_const_view T_rank =
      gsl_matrix_const_submatrix(T, 0U, 0U, rank, rank);
  gsl_matrix_view X_rank = gsl_matrix_submatrix(X, 0U, 0U, rank, X->size2);

  // Solve the triangular system op(T) * X = B or X * op(T) = B.
  gsl_blas_dtrsm(side, uplo, transpose, CblasNonUnit, 1.0, &T_rank.matrix,
                 &X_rank.matrix);
  return err;
}

// Converts a trapezoidal matrix R with m < n to a pure upper
// triangular matrix with zeros outside of the square portion of the
// matrix using orthogonal transformations from the right:
//
//   R = [T, 0] * Z
//
// Here Z = Z_1 * Z_2 * ... * Z_m with Z_k = I - tau_k * v_k * v_k'
// where v_k is the Householder vector which cancels the
// non-triangular elements of the kth row of T.  This is similar to
// the DTZRZF function from LAPACK.
//
// Args:
//   R: Right trapezoidal matrix with m < n.  This function will
//       happily ignore non-zero components in the lower portion of
//       the R matrix; however these will also appear in the T matrix.
//   T: Upper triangular matrix output (m x n).  The "zero" portion of
//       the T matrix is used to store the Householder vectors that,
//       with tau, form Z.
//   tau: Vector of length m of Householder coefficients.
void GslTrapezoidalToTriangular(const gsl_matrix *R, gsl_matrix *T,
                                gsl_vector *tau) {
  assert(R != NULL && T != NULL && tau != NULL);
  assert(R->size1 == T->size1 && R->size2 == T->size2);
  assert(tau->size == T->size1);
  assert(R->size1 > 0U && R->size1 < R->size2);

  if (T != R) gsl_matrix_memcpy(T, R);

  size_t i_max = R->size1 - 1U;
  for (size_t k = 0U; k < R->size1; ++k) {
    size_t i = i_max - k;

    // hh is used to calculate the Householder vector; however, the
    // Household vector itself is only stored in hh1.
    gsl_vector_view hh = gsl_matrix_subrow(T, i, i_max, R->size2 - i_max);
    gsl_vector_view hh1 =
        gsl_matrix_subrow(T, i, R->size1, R->size2 - R->size1);

    // GSL assumes that the elements you want to cancel are adjacent
    // to the diagonal element, which isn't the case here, so we have
    // to swap elements around.
    gsl_vector_view row = gsl_matrix_row(T, i);
    gsl_vector_swap_elements(&row.vector, i, i_max);
    double tau_k = gsl_linalg_householder_transform(&hh.vector);
    gsl_vector_set(tau, k, tau_k);
    gsl_vector_swap_elements(&row.vector, i, i_max);

    // Apply Householder reflections to the right hand side of the
    // triangular matrix.  We don't apply the reflection to the ith
    // row itself because GSL's Householder transform sets the
    // diagonal element for us.  (Doing this with the
    // gsl_linalg_householder_mh function would require swapping
    // columns because of assumptions GSL makes about the diagonal
    // element being adjacent to the elements that the Householder
    // vector cancels.)
    for (size_t j = 0U; j < i; ++j) {
      gsl_vector_view T_jm =
          gsl_matrix_subrow(T, j, R->size1, R->size2 - R->size1);
      double alpha, T_ji = gsl_matrix_get(T, j, i);
      gsl_blas_ddot(&hh1.vector, &T_jm.vector, &alpha);
      alpha += T_ji;
      gsl_blas_daxpy(-tau_k * alpha, &hh1.vector, &T_jm.vector);
      gsl_matrix_set(T, j, i, -tau_k * alpha + T_ji);
    }
  }
}

// Applies the inverse of the Householder transformations output by
// GslTrapezoidalToTriangular to the matrix X.  More specifically, for
// the matrix Z defined by:
//
//   R = [T, 0] * Z
//
// this function applies Z' to the input matrix X.
//
// Args:
//   T: The R matrix of a QR decomposition after complete
//       orthogonalization.  The elements that should be zero from the
//       QR decomposition are ignored.  The elements that should be
//       zero from the complete orthogonalization must contain the
//       Householder vectors used in the orthogonalization.
//   tau: The factors to multiply the Householder vectors by which is
//       output by the GslTrapezoidalToTriangular function.
//   X: The input matrix to which we apply Z'.
//   Zt_X: Output matrix with Householder transformations applied  Z' * X.
void GslTrapezoidalToTriangularZTMat(const gsl_matrix *T, const gsl_vector *tau,
                                     const gsl_matrix *X, gsl_matrix *Zt_X) {
  assert(X != NULL && tau != NULL && Zt_X != NULL && T != NULL);
  assert(T != X && T != Zt_X);
  assert(T->size1 == tau->size && T->size2 == X->size1);
  assert(X->size1 == Zt_X->size1 && X->size2 == Zt_X->size2);
  assert(T->size1 <= T->size2);

  if (X != Zt_X) gsl_matrix_memcpy(Zt_X, X);

  for (size_t i = 0U; i < T->size1; ++i) {
    // Grab the Householder vector from the columns past the square
    // section of the T matrix.
    gsl_vector_const_view hh1 =
        gsl_matrix_const_subrow(T, i, T->size1, T->size2 - T->size1);

    // Apply Householder reflections to the left hand side of X.
    // (Doing this with the gsl_linalg_householder_hm function would
    // require swapping rows because of assumptions GSL makes about
    // the diagonal element being adjacent to the elements that the
    // Householder vector cancels.)
    //
    // TODO: If we need this non-standard
    // gsl_linalg_householder_hm in other places, we should make a
    // helper function.
    double tau_k = gsl_vector_get(tau, T->size1 - i - 1U);
    for (size_t j = 0U; j < Zt_X->size2; ++j) {
      gsl_vector_view Zt_X_mj =
          gsl_matrix_subcolumn(Zt_X, j, T->size1, T->size2 - T->size1);
      double alpha, Zt_X_ij = gsl_matrix_get(Zt_X, i, j);
      gsl_blas_ddot(&hh1.vector, &Zt_X_mj.vector, &alpha);
      alpha += Zt_X_ij;
      gsl_blas_daxpy(-tau_k * alpha, &hh1.vector, &Zt_X_mj.vector);
      gsl_matrix_set(Zt_X, i, j, -tau_k * alpha + Zt_X_ij);
    }
  }
}

// Left or right matrix divide.  Finds a solution, X, to the equations
// A*X = B or X*A = B for left and right divide respectively.  If the
// system is overdetermined or underdetermined, then this returns a
// least-squares solution that minimizes |A*X - B| or |X*A - B|,
// similar to MATLAB's "\" or "/" operators.  The solution is found
// using QR decomposition with column pivoting followed by back
// substitution.  For matrix left divide, the solution proceeds as:
//
//   A*X = B
//   Q*R*P' * X = B
//   R * P'*X = Q'*B
//
// For matrix right divide, the solution proceeds as:
//
//   X*A = B
//   A'*X' = B'
//   Q*R*P' * X' = B'
//   R * (X*P)' = Q'*B'
//
// Args:
//   side: CblasLeft or CblasRight for left or right divide.
//   A: Left hand side matrix.
//   B: Right hand side matrix.
//   X: Solution matrix.
//
// Returns:
//   GSL_ESING A is rank deficient and thus the R matrix is singular and
//   GSL_SUCCESS otherwise.
int32_t GslMatrixDivide(CBLAS_SIDE_t side, const gsl_matrix *A,
                        const gsl_matrix *B, gsl_matrix *X) {
  assert(A != NULL && B != NULL && X != NULL);
  assert(A != X && B != X);
  assert(side != CblasLeft || (A->size1 == B->size1 && A->size2 == X->size1 &&
                               X->size2 == B->size2));
  assert(side != CblasRight || (A->size2 == B->size2 && A->size1 == X->size2 &&
                                X->size1 == B->size1));
  assert(A->size1 <= MAX_VECTOR_SIZE && A->size2 <= MAX_VECTOR_SIZE);
  assert(A->size1 * A->size2 <= MAX_MATRIX_SIZE);
  assert(B->size1 * B->size2 <= MAX_MATRIX_SIZE);
  assert(X->size1 * X->size2 <= MAX_MATRIX_SIZE);

  int32_t err = GSL_SUCCESS;
  size_t m = (side == CblasLeft) ? A->size1 : A->size2;
  size_t n = (side == CblasLeft) ? A->size2 : A->size1;
  size_t k = (side == CblasLeft) ? B->size2 : B->size1;

  // Allocate workspace variables.
  double workspace[MAX_MATRIX_SIZE];
  double qr_data[MAX_MATRIX_SIZE];
  double tau_data[MAX_VECTOR_SIZE];
  size_t perm_data[MAX_VECTOR_SIZE];
  int signum;

  // Decompose op(A) into Q*R*P'.
  gsl_matrix_view QR = gsl_matrix_view_array(qr_data, m, n);
  gsl_vector_view tau = gsl_vector_view_array(tau_data, m < n ? m : n);
  gsl_permutation p = {n, perm_data};
  gsl_vector_view norm = gsl_vector_view_array(workspace, n);
  if (side == CblasLeft) {
    gsl_matrix_memcpy(&QR.matrix, A);
  } else {
    gsl_matrix_transpose_memcpy(&QR.matrix, A);
  }
  gsl_linalg_QRPT_decomp(&QR.matrix, &tau.vector, &p, &signum, &norm.vector);

  // Calculate Q'*op(B) matrix.
  gsl_matrix_view Qt_opB = gsl_matrix_view_array(workspace, m, k);
  if (side == CblasLeft) {
    gsl_matrix_memcpy(&Qt_opB.matrix, B);
  } else {
    gsl_matrix_transpose_memcpy(&Qt_opB.matrix, B);
  }
  gsl_linalg_QR_QTmat(&QR.matrix, &tau.vector, &Qt_opB.matrix);

  // Determine the rank of the R matrix.
  //
  // TODO: DGELSY uses a different iterative condition
  // number estimation technique to determine the rank of the
  // submatrix R(1:rank, 1:rank).  See http://www.netlib.org/lapack/
  // lapack-3.1.1/html/dgelsy.f.html#DGEQP3.269.
  double R_00 = fabs(gsl_matrix_get(&QR.matrix, 0U, 0U));
  uint32_t rank = 0U;
  if (R_00 > DBL_EPSILON) {
    for (rank = 1U; rank < m && rank < n; ++rank) {
      if (fabs(gsl_matrix_get(&QR.matrix, rank, rank)) < 1e-9 * R_00) {
        err = GSL_ESING;
        break;
      }
    }
  } else {
    gsl_matrix_set_zero(X);
    return GSL_ESING;
  }

  // Solve the triangular system R * Z * P' * op(X) = Q' * op(B) and
  // multiply the solution by Z' to get P' * op(X).
  gsl_matrix_view Z_Pt_opX = gsl_matrix_view_array(X->data, n, k);
  gsl_matrix_set_zero(&Z_Pt_opX.matrix);

  // For underdetermined or rank deficient matrices, use complete
  // orthogonalization to provide the minimal 2-norm solution.
  gsl_vector_view tau_trap = gsl_vector_view_array(tau_data, rank);
  gsl_matrix_view QR_trap = gsl_matrix_submatrix(&QR.matrix, 0U, 0U, rank, n);
  if (rank < n) {
    GslTrapezoidalToTriangular(&QR_trap.matrix, &QR_trap.matrix,
                               &tau_trap.vector);
  }

  // Solve the non-singular square triangular system.
  gsl_matrix_const_view QR_rank =
      gsl_matrix_const_submatrix(&QR.matrix, 0U, 0U, rank, rank);
  gsl_matrix_const_view Qt_opB_rank =
      gsl_matrix_const_submatrix(&Qt_opB.matrix, 0U, 0U, rank, k);
  gsl_matrix_view Z_Pt_opX_rank =
      gsl_matrix_submatrix(&Z_Pt_opX.matrix, 0U, 0U, rank, k);
  gsl_matrix_memcpy(&Z_Pt_opX_rank.matrix, &Qt_opB_rank.matrix);
  gsl_blas_dtrsm(CblasLeft, CblasUpper, CblasNoTrans, CblasNonUnit, 1.0,
                 &QR_rank.matrix, &Z_Pt_opX_rank.matrix);

  // For full-rank square and overdetermined matrices, Z = I.
  if (rank < n) {
    GslTrapezoidalToTriangularZTMat(&QR_trap.matrix, &tau_trap.vector,
                                    &Z_Pt_opX.matrix, &Z_Pt_opX.matrix);
  }

  // Apply the permutation matrix to get op(X).
  for (uint32_t i = 0U; i < k; ++i) {
    gsl_vector_view col = gsl_matrix_column(&Z_Pt_opX.matrix, i);
    gsl_permute_vector_inverse(&p, &col.vector);
  }

  if (side == CblasRight) {
    // We can't transpose within the same memory, hence the
    // unfortunate extra memcpy.
    gsl_matrix_view X_copy = gsl_matrix_view_array(qr_data, k, n);
    gsl_matrix_transpose_memcpy(&X_copy.matrix, &Z_Pt_opX.matrix);
    gsl_matrix_memcpy(X, &X_copy.matrix);
  }

  return err;
}

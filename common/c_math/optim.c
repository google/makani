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

#include "common/c_math/optim.h"

#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "common/c_math/linalg.h"
#include "common/c_math/util.h"

double Newton(double (*f)(double, const void *),
              double (*df)(double, const void *), const void *context,
              double x0, double x_min, double x_max, double f_tol,
              int32_t max_iter) {
  assert(f != NULL && df != NULL);
  assert(x_min <= x0 && x0 <= x_max && x_min < x_max);
  assert(f_tol > DBL_EPSILON);
  assert(max_iter > 0);

  double x = Saturate(x0, x_min, x_max);
  for (int32_t i = 0; i < max_iter; ++i) {
    double f_value = f(x, context);
    if (fabs(f_value) < f_tol) break;

    // Calculate the step to take during this iteration and protect
    // from division by a zero derivative.  x_step is guaranteed to be
    // non-zero here, so that it has a definitive sign for the
    // saturations below.
    double df_value = df(x, context);
    int32_t sign_df;
    if (fabs(df_value) < DBL_EPSILON) {
      sign_df = (i % 2 == 0) ? 1 : -1;
    } else {
      sign_df = Sign(df_value);
    }
    double x_step = sign_df * f_value / fmax(fabs(df_value), DBL_EPSILON);

    // Do not allow steps less than the machine precision.
    x_step = Sign(x_step) * fmax(fabs(x_step), fabs(x) * DBL_EPSILON);

    // Do not allow steps larger than half the solution interval.
    x_step = Sign(x_step) * fmin(fabs(x_step), (x_max - x_min) / 2.0);

    x = Saturate(x - x_step, x_min, x_max);
  }
  return x;
}

void LeastSquares(const Mat *A, const Vec *b, Vec *x) {
  MatVecLeftDivide(A, b, x);
}

bool IsWithinBounds(const Vec *x, const Vec *lower, const Vec *upper,
                    double tol) {
  assert(x->length == lower->length && x->length == upper->length);
  assert(tol >= 0.0);
  for (int32_t i = 0; i < x->length; ++i) {
    if (!(VecGet(lower, i) - tol < VecGet(x, i) &&
          VecGet(x, i) < VecGet(upper, i) + tol)) {
      return false;
    }
  }
  return true;
}

int32_t BoundedLeastSquares(const Mat *A, const Vec *b, const Vec *lower,
                            const Vec *upper, Vec *x) {
  assert(A->nr == b->length && A->nc == x->length &&
         lower->length == x->length && upper->length == x->length);
  for (int32_t i = 0; i < lower->length; ++i) {
    assert(VecGet(lower, i) < VecGet(upper, i));
  }

  MAT(A->nr, A->nc, A_free);
  VEC(b->length, b_free);
  VEC(b->length, x_free);
  VEC(b->length, tmp);
  VEC(x->length, lambda);
  int32_t bound[VEC_MAX_ELEMENTS] = {0};

  int32_t num_iter = 0, num_free = A->nc;
  // TODO: This seems to be a good number for
  // max_iterations, but it's not based on anything.
  const int32_t max_iter = 3 * lower->length;

  while (num_iter < max_iter) {
    // Build matrices for active subset of least squares problem.
    A_free.nc = num_free;
    VecCopy(b, &b_free);
    for (int32_t j = 0, k = 0; j < A->nc; ++j) {
      if (!bound[j]) {
        // A_free = A(:,free).
        for (int32_t i = 0; i < A->nr; ++i) {
          *MatPtr(&A_free, i, k) = MatGet(A, i, j);
        }
        k++;
      } else {
        // b_free = b - A(:,bound)*x(bound).
        for (int32_t i = 0; i < A->nr; ++i) {
          *VecPtr(&b_free, i) -= MatGet(A, i, j) * VecGet(x, j);
        }
      }
    }

    // Solve unconstrained least squares problem in active subset.
    if (num_free > 0) {
      LeastSquares(&A_free, &b_free, &x_free);
    }

    // Check if the new solution is within the lower and upper bounds.
    bool all_in_bounds = true;
    for (int32_t j = 0, k = 0; j < A->nc; ++j) {
      if (!bound[j]) {
        if (VecGet(&x_free, k) < VecGet(lower, j) ||
            VecGet(&x_free, k) > VecGet(upper, j)) {
          all_in_bounds = false;
          break;
        }
        k++;
      }
    }

    // If solution is within bounds, check Lagrange multipliers to see
    // if it is optimal.
    if (all_in_bounds) {
      // Update x.
      for (int32_t j = 0, k = 0; j < A->nc; ++j) {
        if (!bound[j]) {
          x->d[j] = x_free.d[k];
          k++;
        }
      }

      // Calculate Lagrange multipliers.
      VecSub(b, MatVecMult(A, x, &tmp), &tmp);
      MatTransVecMult(A, &tmp, &lambda);

      bool all_lambda_neg = true;
      double max_lambda = -INFINITY;
      int32_t lambda_ind = 0;
      for (int32_t j = 0; j < A->nc; ++j) {
        if (bound[j]) {
          lambda.d[j] *= -bound[j];
          if (lambda.d[j] > 0.0) all_lambda_neg = false;
          if (lambda.d[j] > max_lambda) {
            max_lambda = lambda.d[j];
            lambda_ind = j;
          }
        }
      }
      // You are done if all the Lagrange multipliers are negative.
      if (all_lambda_neg) break;

      if (bound[lambda_ind]) {
        bound[lambda_ind] = 0;
        num_free++;
      }
    } else {
      // Move solution to within bounds and remove constraint from free list.
      // Determine alpha.
      double alpha_tmp, alpha = INFINITY;
      int32_t alpha_ind = 0;
      int32_t bound_type = -1;
      for (int32_t j = 0, k = 0; j < A->nc; ++j) {
        if (!bound[j]) {
          if (x_free.d[k] > x->d[j]) {
            alpha_tmp = (upper->d[j] - x->d[j]) / (x_free.d[k] - x->d[j]);
          } else if (x_free.d[k] < x->d[j]) {
            alpha_tmp = (lower->d[j] - x->d[j]) / (x_free.d[k] - x->d[j]);
          } else {
            alpha_tmp = INFINITY;
          }
          if (alpha_tmp < alpha) {
            alpha = alpha_tmp;
            alpha_ind = j;
            bound_type = x_free.d[k] > x->d[j] ? 1 : -1;
          }
          k++;
        }
      }

      // Update x.
      for (int32_t j = 0, k = 0; j < A->nc; ++j) {
        if (!bound[j]) {
          x->d[j] += alpha * (x_free.d[k] - x->d[j]);
          k++;
        }
      }

      // Remove the element that determined alpha from the free list.
      if (!bound[alpha_ind]) {
        bound[alpha_ind] = bound_type;
        num_free--;
      }
    }

    num_iter++;
  }

  // TODO: Justify the 1e-9 tolerance.
  assert(IsWithinBounds(x, lower, upper, 1e-9));
  return num_iter;
}

bool IsWithinConstraints(const Mat *C, const Vec *x0, const Vec *lower,
                         const Vec *upper, double tol) {
  assert(C->nr == lower->length && C->nr == upper->length &&
         C->nc == x0->length);
  assert(tol >= 0.0);
  for (int32_t i = 0; i < C->nr; ++i) {
    double Cx_i = 0.0;
    for (int32_t j = 0; j < C->nc; ++j) {
      Cx_i += MatGet(C, i, j) * VecGet(x0, j);
    }
    if (!(lower->d[i] - tol < Cx_i && Cx_i < upper->d[i] + tol)) {
      return false;
    }
  }
  return true;
}

// Returns true if the solution x is optimal (all Lagrange multipliers
// are negative).  active_ind is set to the constraint with the
// greatest Lagrange multiplier.
static bool IsOptimalAtConstraints(const Mat *A, const Vec *x, const Vec *b,
                                   const Mat *C, const int32_t active[],
                                   int32_t *active_ind) {
  MAT(C->nr, C->nc, C_active);
  VEC(b->length, r);
  VEC(C->nr, At_r);
  VEC(C->nr, lambdas);

  VecSub(b, MatVecMult(A, x, &r), &r);
  MatTransVecMult(A, &r, &At_r);
  // If there is an error (i.e. if the active constraints are
  // dependent and thus C is singular), then we should know about it
  // because the constraints should never become dependent.  However,
  // if the constraints do become dependent and the asserts are turned
  // off, this should still work.
  int32_t err =
      MatVecRightDivide(MatSlice(C, active, NULL, &C_active), &At_r, &lambdas);
  assert(!err);
  // TODO: This statement suppresses warnings generated when compiling
  // with asserts off.  We should pass this error / warning out to the caller
  // while still making a best effort to return reasonable values.
  (void)err;

  double lambda_max = -INFINITY;
  *active_ind = 0;
  for (int32_t i = 0, j = 0; i < C->nr; ++i) {
    if (active[i]) {
      if (-lambdas.d[j] * active[i] >= lambda_max) {
        lambda_max = -lambdas.d[j] * active[i];
        *active_ind = i;
      }
      j++;
    }
  }

  // TODO: Justify the 1e-9 tolerance.
  return lambda_max <= 1e-9;
}

// For each iteration we find a search direction, p, and non-negative
// step length, alpha, to update our solution:
//
//   x_{k+1} = x_k + alpha * p
//
// The search direction, p, is chosen such that the active set of
// constraints, C_k, is satisfied: C_k * p = 0, or equivalently:
//
//   p = Z_k * q,   C_k * [Z_k, Y_k] = [0, T_k]
//
// where [Z_k, Y_k] is the orthogonal matrix, Q_k, in the QR
// decomposition of C_k^T, and the columns of Z_k form a basis for
// the null space of C_k.
//
// We solve the unconstrained least squares problem for the active
// subset:
//
//   min || A_k * Z_k * q - r_k ||     r_k = b - A * x_k
//    q
//
// Then we find the maximum alpha (0 <= alpha <= 1) such that the
// updated least squares estimate: x_{k+1} = x_k + alpha * p meets all
// the inactive constraints: alpha * C'_k * p >= d - C'_k * x_k
//
// Finally, we check for optimality.  The solution is optimal if all
// the Lagrange multipliers, lambda, are positive:
//
//   C_k^T * lambda = -A^T * (b - A * x_{k+1})
//
int32_t ConstrainedLeastSquares(const Mat *A, const Vec *b, const Mat *C,
                                const Vec *lower, const Vec *upper,
                                const Vec *x0, Vec *x) {
  assert(A->nr == b->length && A->nc == x0->length);
  assert(C->nr == lower->length && C->nr == upper->length &&
         C->nc == x0->length);
  assert(x0->length == x->length ||
         (x->variable_len && x->max_length >= x0->length));
  assert(x != b && x != lower && x != upper);
  assert(IsWithinConstraints(C, x0, lower, upper, 0.0));

  MAT(C->nc, C->nr, Ct);
  MAT(C->nc, C->nc, Q);
  MAT(C->nc, C->nc, Z);
  MAT(MaxInt32(A->nr, C->nr), A->nc, tmp_mat);

  VEC(b->length, r);
  r.variable_len = 0;
  VEC(A->nc, p);
  p.variable_len = 0;
  VEC(MaxInt32(C->nr, C->nc), tmp_vec);

  int32_t num_active = 0;
  int32_t num_iter = 0;
  // Based on many test runs, this appears to be a good number for the
  // maximum number of iterations; however, it's not based on a
  // rigorous mathematical analysis.
  const int32_t max_iter = 3 * C->nr;
  int32_t null_cols[VEC_MAX_ELEMENTS], active[VEC_MAX_ELEMENTS] = {0};
  VecCopy(x0, x);
  MatTrans(C, &Ct);

  while (num_iter < max_iter) {
    // Find the solution update direction, p, by solving the least
    // squares problem in a subspace that doesn't affect the active
    // constraints (i.e. C * p = 0).
    int32_t null_dim = (C->nc > num_active) ? C->nc - num_active : 0;
    if (null_dim == 0) {
      // There are no unconstrained variables, so the update is 0.
      VecZero(&p);
    } else {
      VecSub(b, MatVecMult(A, x, &r), &r);
      if (num_active == 0) {
        // There are no constraints, solve normal least squares.
        LeastSquares(A, &r, &p);
      } else {
        // Build Z matrix which is the basis for update vectors that
        // don't change the constraints.
        MatQrDecomp(MatSlice(&Ct, NULL, active, &tmp_mat), &Q, &tmp_mat);
        Slice(num_active, 1, num_active + null_dim, C->nc, null_cols);
        MatSlice(&Q, NULL, null_cols, &Z);

        // Solve the least squares problems along the unconstrained
        // dimensions.
        LeastSquares(MatMult(A, &Z, &tmp_mat), &r, &tmp_vec);
        MatVecMult(&Z, &tmp_vec, &p);
      }
    }

    // Find maximum alpha (alpha <= 1) such that x_{k+1} = x_k + alpha * p
    // meets all the inactive constraints: alpha * C'_k * p >= d - C'_k * x_k.
    double alpha = INFINITY;
    int32_t alpha_ind = 0;
    int32_t bound = 1;
    for (int32_t i = 0; i < C->nr; ++i) {
      if (!active[i]) {
        Vec C_inactive_i = {C->nc, MatPtr(C, i, 0), 0, 0};
        // Check if constraint is linearly independent from currently
        // active constraints.
        //
        // TODO: What the numerical tolerance should be
        // is likely a more complicated function of Z and C_active_i.
        if (null_dim > 0 &&
            (num_active == 0 ||
             (VecNorm(MatTransVecMult(&Z, &C_inactive_i, &tmp_vec)) >
              10.0 * DBL_EPSILON * VecNorm(&C_inactive_i)))) {
          double C_dot_p = VecDot(&C_inactive_i, &p);
          double C_dot_x = VecDot(&C_inactive_i, x);
          double alpha_tmp;
          if (C_dot_p < 0.0) {
            alpha_tmp = (lower->d[i] - C_dot_x) / C_dot_p;
          } else if (C_dot_p > 0.0) {
            alpha_tmp = (upper->d[i] - C_dot_x) / C_dot_p;
          } else {
            alpha_tmp = INFINITY;
          }

          if (alpha_tmp < alpha) {
            alpha = alpha_tmp;
            alpha_ind = i;
            bound = Sign(C_dot_p);
          }
        }
      }
    }

    // Update x_k.
    VecLinComb(1.0, x, Saturate(alpha, 0.0, 1.0), &p, x);

    // Add or remove constraints.
    if (alpha <= 1.0) {
      assert(!active[alpha_ind]);
      active[alpha_ind] = bound;
      num_active++;
    } else {
      // Check optimality.  The solution is optimal if all the Lagrange
      // multipliers, lambda, are positive.
      int32_t active_ind = 0;
      if (IsOptimalAtConstraints(A, x, b, C, active, &active_ind)) break;
      assert(active[active_ind]);
      active[active_ind] = 0;
      num_active--;
    }

    num_iter++;
  }

  // TODO: Justify the 1e-8 tolerance.  Note that we
  // violated a previous 1e-9 tolerance likely due to the large scale
  // difference between the thrust constraints and the power
  // constraint.
  assert(IsWithinConstraints(C, x, lower, upper, 1e-8));
  return num_iter;
}

void WeightLeastSquaresInputs(const Vec *weights, Mat *A, Vec *b) {
  assert(weights->length == A->nr && weights->length == b->length);
  for (int32_t i = 0; i < weights->length; ++i) {
    // For now, don't allow exactly 0 weights.
    assert(VecGet(weights, i) > 0.0);
    for (int32_t j = 0; j < A->nc; ++j) {
      *MatPtr(A, i, j) *= Sqrt(weights->d[i]);
    }
    b->d[i] *= Sqrt(weights->d[i]);
  }
}

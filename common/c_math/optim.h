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

#ifndef COMMON_C_MATH_OPTIM_H_
#define COMMON_C_MATH_OPTIM_H_

#include <stdbool.h>
#include <stdint.h>

#include "common/c_math/linalg.h"

#ifdef __cplusplus
extern "C" {
#endif

// Uses Newton-Raphson method to find a zero of the given function.
//
//   x_{n+1} = x_n - f(x_n) / f'(x_n)
//
// The Newton-Raphson method has well-known issues with convergence
// outside the region of quadratic convergence, and thus this function
// should only be used on well-behaved functions.
//
// Args:
//   f: Pointer to function.
//   df: Pointer to the derivative of the function.
//   context: Pointer to context needed by the function and its derivative.
//   x0: Initial guess for the root.
//   x_min: Minimum of the interval in which we are looking for the zero.
//   x_max: Maximum of the interval in which we are looking for the zero.
//   f_tol: Tolerance for the solution at which the search terminates.
//   max_iter: Maximum number of iterations to attempt.
//
// Returns:
//   Zero of the function or the final guess when the maximum number
//   of iterations is reached.
double Newton(double (*f)(double, const void *),
              double (*df)(double, const void *), const void *context,
              double x0, double x_min, double x_max, double f_tol,
              int32_t max_iter);

// Solves the least squares problem:
//
//   min || Ax - b ||
//    x
//
// Uses complete orthogonal decomposition to calculate the minimal
// 2-norm least squares solution similar to the LAPACK routine DGELSY.
void LeastSquares(const Mat *A, const Vec *b, Vec *x);

// Returns true if lower < x < upper for each component, false
// otherwise.
bool IsWithinBounds(const Vec *x, const Vec *lower, const Vec *upper,
                    double tol);

// Solves the bounded solution least squares problem:
//
//   min || Ax - b ||
//    x                   l <= x <= u
//
// This uses an "active set" algorithm described in:
//
// [1] P.B. Stark and R. L. Parker, "Bounded-Variable Least-Squares:
//     An Algorithm and Applications",  Computational Statistics
//     10:129-141, 1995.
int32_t BoundedLeastSquares(const Mat *A, const Vec *b, const Vec *lower,
                            const Vec *upper, Vec *x);

// Returns true if lower < C * x0 < upper (to within the tolerance),
// false otherwise.  This uses strict inequalities (< instead of <=)
// because the active set algorithm assumes that none of the
// constraints are active to start.
bool IsWithinConstraints(const Mat *C, const Vec *x0, const Vec *lower,
                         const Vec *upper, double tol);

// Solves the linear inequality constrained least squares problem:
//
//   min || Ax - b ||
//    x                   lower <= Cx <= upper
//
// This uses an "active set" algorithm described in the following
// sources:
//
// [1] P.B. Stark and R. L. Parker, "Bounded-Variable Least-Squares:
//     An Algorithm and Applications", Computational Statistics
//     10:129-141, 1995.
// [2] Ake Bjorck, "Numerical Methods in Scientific Computing, Vol 2"
int32_t ConstrainedLeastSquares(const Mat *A, const Vec *b, const Mat *C,
                                const Vec *lower, const Vec *upper,
                                const Vec *x0, Vec *x);

// Adjusts A and b vectors of a least squares problem by the weights.
void WeightLeastSquaresInputs(const Vec *weights, Mat *A, Vec *b);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // COMMON_C_MATH_OPTIM_H_

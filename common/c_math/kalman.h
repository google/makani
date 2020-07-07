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

#ifndef COMMON_C_MATH_KALMAN_H_
#define COMMON_C_MATH_KALMAN_H_

#include <stdint.h>

#include "common/c_math/linalg.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct { double A, H, Q, R, fc_min, fc_max; } BoundedKalman1dParams;

typedef struct { double Q, fc_min, fc_max; } BoundedKalman1dEstimatorParams;

double BoundedKalman1d(double z, double ts, const BoundedKalman1dParams *params,
                       double *dz_norm, double *fc, double *x, double *P);
double BoundedKalman1dEstimator(double z, double R, double ts,
                                const BoundedKalman1dEstimatorParams *params,
                                double *dz_norm, double *fc, double *x,
                                double *P);

// Returns the array length for storing a UD decomposition for an
// n-by-n matrix.
static inline int32_t UdKalmanArrayLength(int32_t n) {
  return ((n + 1) * n) / 2;
}

// Extract the U matrix and D vector from a vector representing a
// UD-factorization of a covariance.
//
// For example a 3-by-3 covariance is represented by the product:
//
//   P = U D U'
//
//  where U = [1   U_12 U_13   and D = [D_1 0   0
//             0   1    U_23            0   D_2 0
//             0   0    1   ]           0   0   D_3].
//
// This is stored as the vector:
//
//     [D_1, U_12, D_2, U_13, U_23, ... D_n].
//
// of (n+1) * n / 2 elements.
//
// Args:
//   n: Number of states.
//   ud: (n+1)*n/2 element array representation of the U and D matrices as
//       above.
//   U: Matrix of n * n elements to represent the matrix.
//   d: Vector of n elements to represent the diagonal of D.
void UdKalmanExtractUd(const Vec *ud, Mat *U, Vec *d);

// Store the upper triangle of a matrix into a vector.
//
// One of P or d must be non-NULL.  If P and d are both not NULL, u is set to:
//
//     u = [d_1, P_12, d_2, P_13, P_23, d_3, ... d_n].
//
// If only P is not NULL, u is set to:
//
//     u = [P_11, P_12, P_22, P_13, P_23, P_33, ... P_nn].
//
// and if only d is not null, u is set to:
//
//     u = [d_1, 0, d_2, 0, 0, d_3, ... d_n].
//
// Args:
//   P: n-by-n matrix or NULL.
//   d: Vector of length n or NULL.
//   u: Vector of length ((n+1) * n) / 2.
void UdKalmanStoreUpperTri(const Mat *P, const Vec *d, Vec *u);

// Calculate the vectors:
//
//   f = U' * h',
//   g = D * f,
//
// used in the UD Kalman filter measurement update.
//
// See Gibbs, Bruce P. Advanced Kalman Filtering, Least Squares and
// Modeling, 2011. Eq. 10.2-5 on page 394.
//
// Args:
//   ud: Vector representation of the UD factorization.
//   h: Input/output array of n values.  Should store the coefficients
//       of h at input, will store the coefficients of f on output.
//   g: Output array of n values storing the coefficients of g.
void UdKalmanCalcWeightVectors(const Vec *ud, Vec *h, Vec *g);

// Perform the UD factorization and calculate the Kalman gain for a measurement.
//
// See Gibbs, Bruce P. Advanced Kalman Filtering, Least Squares and
// Modeling, 2011. Algorithm on 10.2-12 on page 396.
//
// Args:
//   R: Measurement covariance.
//   f: Array of n values, see UdKalmanCalcWeightVectors.
//   ud: Input/output argument storing the UD factorization.
//   g: Input/ouput array storing n values.  On input should store the values of
//       g (see UdKalmanCalcWeightVectors).  On output stores the Kalman gain.
void UdKalmanCalcGain(double R, const Vec *f, Vec *ud, Vec *g);

// Calculate the Kalman gain associated with a scalar measurement update for
// a UD Kalman filter.  Also, update U and D to reflect the correction.
//
// Args:
//   R: Measurement covariance.
//   h: Vector of n elements storing the measurement matrix.  This will be used
//       as scratch space.
//   ud: Vector of (n+1)*n/2 elements representing the U and D matrices as
//       above.
//   k: Vector of n elements that store the resulting Kalman gain.
void UdKalmanMeasurementUpdate(double R, Vec *h, Vec *ud, Vec *k);

// Perform a propagation (time) update on a UD factorized covariance matrix.
//
//     x[t+1|t] = F * x[t|t] + B w[t].
//
// where w[t] is zero mean with a diagonal covariance matrix Q and x[t|t] has
// a covariance matrix P = UDU'.  The input argument d = [diag(D); diag(Q)].
// The input argument W = [F * U B].
//
// Args:
//   d: Vector of n+p diagonal weights, see above.
//   W: n-by-(n+p) dimension matrix, see above.  This array is used as
//       working space.
//   ud: Output vector of ((n + 1) * n) / 2 elements storing the UD
//       factorization.
void UdKalmanTimeUpdate(const Vec *d, Mat *W, Vec *ud);

// Extract the covariance matrix diagonal from a UD factorization.
void UdKalmanExtractCovariances(const double ud[], Vec *cov);

// Computes F * U where F is a transition matrix.
//
// Args:
//   F: n-by-n matrix.
//   ud: UD factorization for an n-state system.
//   W: n-by-m output matrix where m > n.  The first n columns of W
//       are set to F * U.
//   d: m-by-1 output matrix.  The first n entries of d are set to the
//       diagonal elements of D.
void UdKalmanTransitionMatrixMultiply(const Mat *F, const Vec *ud, Mat *W,
                                      Vec *d);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // COMMON_C_MATH_KALMAN_H_

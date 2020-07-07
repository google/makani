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

#include "common/c_math/kalman.h"

#include <assert.h>
#include <math.h>
#include <stddef.h>
#include <stdint.h>

#include "common/c_math/linalg.h"
#include "common/c_math/util.h"

// One dimensional Kalman filter.
//
// Sample time independent Kalman filter.  The gains A, H, Q, and R are supplied
// in the continuous form and translated to the discrete form at run time.  Note
// that implicit in translating R to its discrete form is the assumption that
// the longer the sample time, the longer the period over which the measurement
// is averaged and the smaller its discrete-time variance.
//
// Arguments
//
// z: Measurement.
// ts: Sample time.
// params: The A, H, R, and Q, gains for the continuous Kalman filter. Also,
//     K_min and K_max define the range of allowable Kalman gains.
//
// Returned Values
//
// dz_norm: Normalized innovation (units of standard deviations).  Pass NULL
//          if this output is unnecessary.
// fc: Current filter pole.  Pass NULL if this output is unnecessary.
// x: State.
// P: State covariance.
//
double BoundedKalman1d(double z, double ts, const BoundedKalman1dParams *params,
                       double *dz_norm, double *fc, double *x, double *P) {
  assert(ts > 0.0);
  assert(params->Q >= 0.0 && params->R >= 0.0);
  assert(*P >= 0.0);
  assert(0.0 <= params->fc_min && params->fc_min <= params->fc_max);

  // Translate continuous filter parameters to discrete filter parameters.
  double A_k = 1.0 + params->A * ts;
  double Q_k = params->Q * ts;
  double R_k = params->R / ts;
  double K_min_k = 2.0 * PI * ts * params->fc_min;
  double K_max_k = 2.0 * PI * ts * params->fc_max;

  // Saturate the K_min/K_max bounds to ensure that the eigenvalue of the
  // filter remains inside the unit circle.  (-1 <= A - K*H*A <= 1)
  double HA_k = params->H * A_k;
  if (HA_k != 0.0) {
    double K_stable_min, K_stable_max;
    if (HA_k > 0.0) {
      K_stable_min = (A_k - 1.0) / HA_k;
      K_stable_max = (A_k + 1.0) / HA_k;
    } else {
      K_stable_max = (A_k - 1.0) / HA_k;
      K_stable_min = (A_k + 1.0) / HA_k;
    }
    K_min_k = Saturate(K_min_k, K_stable_min, K_stable_max);
    K_max_k = Saturate(K_max_k, K_stable_min, K_stable_max);
  }

  // Covariance should never be negative.
  *P = fmax(*P, DBL_TOL);

  *x *= A_k;                       // Propagate state.
  *P = A_k * *P * A_k + Q_k;       // Propagate covariance.
  double dz = z - params->H * *x;  // Find innovation.
  // Find the covariance of the innovation.
  double C = fmax(params->H * *P * params->H + R_k, DBL_TOL);
  // Find and bound the Kalman gain.
  double K_k = Saturate(*P * params->H / C, K_min_k, K_max_k);
  *x += K_k * dz;  // Update state.

  // Update covariance.  Since K_k is not necessarily the Kalman gain, the co-
  // variance update does not always simplify to the familiar, P = P - KHP.
  double temp = (1.0 - K_k * params->H);
  *P = temp * *P * temp + K_k * R_k * K_k;

  // Calculate optional outputs.
  if (fc != NULL) *fc = K_k / (2.0 * PI * ts);

  if (dz_norm != NULL)
    *dz_norm = dz / fmax(Sqrt(C), DBL_TOL);  // Normalize the innovation.

  return *x;
}

// BoundedKalman1dEstimator is a convenience function to reduce the number of
// required parameters when using BoundedKalman1d with A = 0, and H = 1 --
// a common configuration for estimation. When fc_min == fc_max this is a first
// order low-pass filter with uncertainty propagation.
double BoundedKalman1dEstimator(double z, double R, double ts,
                                const BoundedKalman1dEstimatorParams *params,
                                double *dz_norm, double *fc, double *x,
                                double *P) {
  BoundedKalman1dParams kalman1d_params;
  kalman1d_params.A = 0.0;
  kalman1d_params.H = 1.0;
  kalman1d_params.Q = params->Q;
  kalman1d_params.R = R;

  // The fc_max_allowable bound prevents stable chattering, which is stricter
  // than the unit circle bound applied in BoundedKalman1d.
  double fc_max_allowable = 1.0 / (2.0 * PI * ts);
  kalman1d_params.fc_min = Saturate(params->fc_min, 0.0, fc_max_allowable);
  kalman1d_params.fc_max = Saturate(params->fc_max, 0.0, fc_max_allowable);

  BoundedKalman1d(z, ts, &kalman1d_params, dz_norm, fc, x, P);

  return *x;
}

static void UdKalmanExtractUdArr(int32_t n, const double ud[], double U[],
                                 double d[]) {
  int32_t index = 0;
  for (int32_t j = 0; j < n; ++j) {
    for (int32_t i = 0; i < n; ++i) {
      if (i == j) {
        U[n * i + j] = 1.0;
        d[i] = ud[index];
        index++;
      } else if (i < j) {
        U[n * i + j] = ud[index];
        index++;
      } else {
        U[n * i + j] = 0.0;
      }
    }
  }
}

void UdKalmanExtractUd(const Vec *ud, Mat *U, Vec *d) {
  assert(ud != NULL && U != NULL && d != NULL);
  int32_t n = d->length;
  assert(U->nr == n && U->nc == n && ud->length == UdKalmanArrayLength(n));
  UdKalmanExtractUdArr(n, ud->d, U->d, d->d);
}

void UdKalmanStoreUpperTri(const Mat *P, const Vec *d, Vec *u) {
  assert((P != NULL || d != NULL) && u != NULL);
  int32_t n = (P != NULL) ? P->nr : d->length;
  assert((P == NULL || P->nc == n) && (d == NULL || d->length == n) &&
         u->length == UdKalmanArrayLength(n));

  int32_t index = 0;
  for (int32_t j = 0; j < n; ++j) {
    for (int32_t i = 0; i <= j; ++i) {
      if (i == j && d != NULL) {
        *VecPtr(u, index) = VecGet(d, i);
      } else if (P != NULL) {
        *VecPtr(u, index) = MatGet(P, i, j);
      } else {
        *VecPtr(u, index) = 0.0;
      }
      index++;
    }
  }
}

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
//   n: Number of states.
//   ud: Vector representation of the UD factorization.
//   h: Input/output array of n values.  Should store the coefficients
//       of h at input, will store the coefficients of f on output.
//   g: Output array of n values storing the coefficients of g.
static void UdKalmanCalcWeightVectorsImpl(int32_t n, const double ud[],
                                          double h[], double g[]) {
  int32_t index = UdKalmanArrayLength(n) - 1;
  for (int32_t j = n - 1; j >= 0; --j) {
    double d = ud[index];
    index--;
    for (int32_t k = j - 1; k >= 0; --k) {
      h[j] += ud[index] * h[k];
      index--;
    }
    g[j] = fmax(d, 0.0) * h[j];
  }
}

void UdKalmanCalcWeightVectors(const Vec *ud, Vec *h, Vec *g) {
  assert(ud != NULL && h != NULL && g != NULL);
  const int32_t n = h->length;
  assert(n > 0 && VecIsSize(ud, UdKalmanArrayLength(n)) && VecIsSize(g, n));

  UdKalmanCalcWeightVectorsImpl(n, ud->d, h->d, g->d);
}

// Perform the UD factorization and calculate the Kalman gain for a measurement.
//
// See Gibbs, Bruce P. Advanced Kalman Filtering, Least Squares and
// Modeling, 2011. Algorithm on 10.2-12 on page 396.
//
// Args:
//   n: Number of states.
//   R: Measurement covariance.
//   f: Array of n values, see UdKalmanCalcWeightVectors.
//   ud: Input/output argument storing the UD factorization.
//   g: Input/ouput array storing n values.  On input should store the values of
//       g (see UdKalmanCalcWeightVectors).  On output stores the Kalman gain.
static void UdKalmanCalcGainImpl(int32_t n, double R, const double f[],
                                 double ud[], double g[]) {
  double gamma = fmax(R + f[0] * g[0], DBL_EPSILON);
  // Update first diagonal element.
  ud[0] *= R / gamma;

  double alpha = gamma;
  int32_t index = 1;
  for (int32_t j = 1; j < n; ++j) {
    // At the beginning of this function, g[j] should be D[j, j] *
    // f[j], where D[j, j] is a positive number.  As g[j] is not
    // written until the j+1-th iteration we know that g[j] * f[j]
    // should be non-negative.
    double f_j_times_g_j = fmax(g[j] * f[j], 0.0);
    alpha += f_j_times_g_j;

    // Update elements of the column.
    for (int32_t k = 0; k < j; ++k) {
      double u_value = ud[index];
      ud[index] = u_value - (f[j] / gamma) * g[k];
      g[k] += g[j] * u_value;
      index++;
    }

    // Update diagonal element.
    // Division by zero is avoided as gamma >= DBL_EPSILON is
    // maintained by this update and f_j_times_g_j >= 0.
    double gamma_j = gamma + f_j_times_g_j;
    ud[index] *= gamma / gamma_j;
    index++;
    gamma = gamma_j;
  }
  // See eq. 10.2-13 in Gibbs.
  //
  // As we sum gamma >= DBL_EPSILON with other non-negative terms,
  // alpha >= DBL_EPSILON.
  for (int32_t j = 0; j < n; ++j) g[j] /= alpha;
}

void UdKalmanCalcGain(double R, const Vec *f, Vec *ud, Vec *g) {
  assert(R > DBL_EPSILON && f != NULL && ud != NULL && g != NULL);
  const int32_t n = f->length;
  assert(n > 0 && VecIsSize(ud, UdKalmanArrayLength(n)) && VecIsSize(g, n));

  UdKalmanCalcGainImpl(n, R, f->d, ud->d, g->d);
}

void UdKalmanMeasurementUpdate(double R, Vec *h, Vec *ud, Vec *k) {
  assert(R > DBL_EPSILON && h != NULL && ud != NULL && k != NULL);
  const int32_t n = h->length;
  assert(n > 0 && VecIsSize(k, n) && VecIsSize(ud, UdKalmanArrayLength(n)));

  UdKalmanCalcWeightVectorsImpl(n, ud->d, h->d, k->d);
  UdKalmanCalcGainImpl(n, R, h->d, ud->d, k->d);
}

// Simple modified-weighted Gram Schmidt, with output written to UD
// matrix.
//
// See:
//
//   Thornton, Catherine, I. Triangular Covariance Factorizations for
//   Kalman Filtering. Technical Memorandum 33-798, Jet Propulsion
//   Laboratory California Institute of Technology, 1976.
//
// Args:
//   n: Number of vectors to factor.
//   m: Dimension of vectors to factor.
//   d: Array of m weights.
//   W: Array of n*m elements representing a row-major n-by-m matrix.
//   ud: Resulting UD factorization.
static void ModifiedWeightedGramSchmidt(int32_t n, int32_t m, const double d[],
                                        double W[], double ud[]) {
  assert(m >= n);
  int32_t index = UdKalmanArrayLength(n) - 1;
  for (int32_t j = n - 1; j >= 0; --j) {
    double d_j_update = 0.0;
    // Calculate d-weighted norm of j-th column (see eq. 3.28 on pg. 36).
    for (int32_t k = 0; k < m; ++k) {
      d_j_update += W[k + m * j] * W[k + m * j] * d[k];
    }
    ud[index] = d_j_update;
    index--;

    if (d_j_update < DBL_EPSILON) {
      for (int32_t i = j - 1; i >= 0; --i) {
        ud[index] = 0.0;
        index--;
      }
    } else {
      for (int32_t i = j - 1; i >= 0; --i) {
        // Calculate the inner product with i-th vector (see eq. 3.29
        // on pg. 36).
        double u_i_j_update = 0.0;
        for (int32_t k = 0; k < m; ++k) {
          // TODO: The second product here is repeated and could
          // be stored with more working space.
          u_i_j_update += W[k + i * m] * W[k + j * m] * d[k];
        }
        u_i_j_update /= d_j_update;
        // Update other columns (see eq. 3.30 on pg. 36).
        for (int32_t k = 0; k < m; ++k) {
          W[k + i * m] -= u_i_j_update * W[k + j * m];
        }
        ud[index] = u_i_j_update;
        index--;
      }
    }
  }
}

void UdKalmanTimeUpdate(const Vec *d, Mat *W, Vec *ud) {
  assert(d != NULL && W != NULL && ud != NULL);
  int32_t n = W->nr;
  int32_t m = W->nc;
  assert(m > n && VecIsSize(ud, UdKalmanArrayLength(n)) && VecIsSize(d, m));
  ModifiedWeightedGramSchmidt(n, m, d->d, W->d, ud->d);
}

static void UdKalmanExtractCovariancesArr(int32_t n, const double ud[],
                                          double cov[]) {
  for (int32_t i = 0; i < n; ++i) cov[i] = 0.0;
  int32_t index = 0;
  for (int32_t j = 0; j < n; ++j) {
    double d_weight = ud[UdKalmanArrayLength(j + 1) - 1];
    for (int32_t i = 0; i <= j; ++i) {
      if (i == j) {
        cov[i] += d_weight;
      } else {
        cov[i] += d_weight * ud[index] * ud[index];
      }
      index++;
    }
  }
}

void UdKalmanExtractCovariances(const double ud[], Vec *cov) {
  assert(ud != NULL && cov != NULL);
  int32_t n = cov->length;
  UdKalmanExtractCovariancesArr(n, ud, cov->d);
}

void UdKalmanTransitionMatrixMultiply(const Mat *F, const Vec *ud, Mat *W,
                                      Vec *d) {
  assert(F != NULL && ud != NULL && W != NULL && d != NULL);
  assert(F->nr == F->nc && F->nr == W->nr && W->nc > F->nc &&
         d->length == W->nc);
  // Compute F * U and extract the diagonal of D.
  // Iterate over rows of F.
  for (int32_t i = 0; i < F->nr; ++i) {
    // Iterate over columns of U.
    int32_t index = 0;
    for (int32_t j = 0; j < F->nr; ++j) {
      double result = 0.0;
      for (int32_t k = 0; k <= j; ++k) {
        if (j == k) {
          result += MatGet(F, i, k);
        } else {
          result += MatGet(F, i, k) * ud->d[index];
        }
        index++;
      }
      *MatPtr(W, i, j) = result;
    }

    *VecPtr(d, i) = ud->d[UdKalmanArrayLength(i + 1) - 1];
  }
}

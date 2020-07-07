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

#include "sim/math/util.h"

#include <math.h>
#include <stdint.h>
#include <stdlib.h>

#include <glog/logging.h>
#include <gsl/gsl_linalg.h>
#include <gsl/gsl_math.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_vector.h>

#include <limits>

uint32_t NamedRandomNumberGenerator::seed_offset_ = 0U;

double HyperGeom(double a, double b, double c, double z) {
  uint32_t n = 0U;
  double F21 = 0.0, err = 1.0;
  double pa, pb, pc;
  while (fabs(err) > 1.0e-6) {
    pa = RisingFactorial(a, n);
    pb = RisingFactorial(b, n);
    pc = RisingFactorial(c, n);
    err = pa * pb / pc * pow(z, static_cast<double>(n)) / Factorial(n);
    F21 += err;
    n++;
    // TODO: Figure out better way of doing tolerances.
    if (n > 20) break;
  }
  return F21;
}

uint32_t Factorial(uint32_t n) {
  DCHECK_LE(n, 12U) << "n is too large.";

  uint32_t fact = 1U, i;
  for (i = 2U; i <= n; ++i) {
    fact *= i;
  }
  return fact;
}

double RisingFactorial(double q, int32_t n) {
  int32_t i;
  double p = 1.0;
  for (i = 0; i < n; ++i) {
    p *= q + static_cast<double>(i);
  }
  return p;
}

bool IntersectCircles(double d, double r1, double r2, double *x, double *y) {
  DCHECK_GE(d, 0.0) << "Distance between circle centers is negative.";
  DCHECK_GE(r1, 0.0) << "Radius of first circle is negative.";
  DCHECK_GE(r2, 0.0) << "Radius of second circle is negative.";
  DCHECK(nullptr != x);
  DCHECK(nullptr != y);

  if (fabs(d) > 0.0) {
    double c = (-d + r1 - r2) * (-d - r1 + r2) * (-d + r1 + r2) * (d + r1 + r2);
    *x = (d * d + r1 * r1 - r2 * r2) / (2.0 * d);
    *y = sqrt(fmax(c, 0.0)) / (2.0 * d);
    return c >= 0.0;
  } else {
    // Arbitrarily choose (r1, 0.0) as point to return if circles are
    // equivalent.
    *x = r1;
    *y = 0.0;
    return fabs(r1 - r2) <
           (fmin(r1, r2) * std::numeric_limits<double>::epsilon());
  }
}

double gsl_interp_index(const gsl_vector *x, double x_i) {
  DCHECK_LE(x->size, std::numeric_limits<uint32_t>::max())
      << "x exceeds the maximum allowed vector length.";
  return InterpIndex(x->data, static_cast<uint32_t>(x->size), x_i,
                     kInterpOptionDefault, NULL);
}

double gsl_interp1(const gsl_vector *x, const gsl_vector *y, double x_i,
                   InterpOption opt) {
  DCHECK_EQ(x->size, y->size) << "x and y vectors are not the same length.";
  DCHECK_LE(x->size, std::numeric_limits<uint32_t>::max())
      << "x exceeds the maximum allowed vector length.";
  return Interp1(x->data, y->data, static_cast<uint32_t>(x->size), x_i, opt);
}

double gsl_interp2(const gsl_vector *x, const gsl_vector *y,
                   const gsl_matrix *z, double x_i, double y_i,
                   InterpOption opt) {
  double s = gsl_interp_index(x, x_i);
  double t = gsl_interp_index(y, y_i);
  return gsl_interp2_scaled(s, t, z, opt);
}

double gsl_interp2_scaled(double s, double t, const gsl_matrix *z,
                          InterpOption opt) {
  int32_t s0, t0;
  double ds, dt;
  gsl_interp2_indices(s, t, static_cast<int32_t>(z->size1),
                      static_cast<int32_t>(z->size2), opt, &s0, &t0, &ds, &dt);
  return gsl_interp2_scaled_helper(z, s0, t0, ds, dt);
}

void gsl_interp2_indices(double s, double t, int32_t size1, int32_t size2,
                         InterpOption opt, int32_t *s0, int32_t *t0, double *ds,
                         double *dt) {
  *s0 = SaturateInt32(static_cast<int32_t>(floor(s)), 0,
                      static_cast<int32_t>(size1 - 2));
  *t0 = SaturateInt32(static_cast<int32_t>(floor(t)), 0,
                      static_cast<int32_t>(size2 - 2));

  if (opt & kInterpOptionSaturate) {
    s = Saturate(s, 0.0, static_cast<double>(size1 - 1));
    t = Saturate(t, 0.0, static_cast<double>(size2 - 1));
  }

  *ds = s - *s0;
  *dt = t - *t0;
}

double gsl_interp2_scaled_helper(const gsl_matrix *z, int32_t s0, int32_t t0,
                                 double ds, double dt) {
  double z00 = gsl_matrix_get(z, s0, t0);
  double z10 = gsl_matrix_get(z, s0 + 1, t0);
  double z01 = gsl_matrix_get(z, s0, t0 + 1);
  double z11 = gsl_matrix_get(z, s0 + 1, t0 + 1);

  return z00 + (z10 - z00) * ds + (z01 - z00) * dt +
         (z00 - z10 - z01 + z11) * ds * dt;
}

double gsl_interp3(const gsl_vector *x, const gsl_vector *y,
                   const gsl_vector *z, const gsl_vector *mat, double x_i,
                   double y_i, double z_i, InterpOption opt) {
  double u = gsl_interp_index(x, x_i);
  double v = gsl_interp_index(y, y_i);
  double w = gsl_interp_index(z, z_i);

  return gsl_interp3_scaled(u, v, w, mat, static_cast<int32_t>(x->size),
                            static_cast<int32_t>(y->size),
                            static_cast<int32_t>(z->size), opt);
}

double gsl_interp3_scaled(double u, double v, double w, const gsl_vector *z,
                          int32_t size1, int32_t size2, int32_t size3,
                          InterpOption opt) {
  DCHECK_EQ(size1 * size2 * size3, z->size)
      << "z vector is not the correct size.";

  int32_t indices[8];
  double du, dv, dw;
  gsl_interp3_indices(u, v, w, size1, size2, size3, opt, indices, &du, &dv,
                      &dw);
  return gsl_interp3_scaled_helper(z, indices, du, dv, dw);
}

void gsl_interp3_indices(double u, double v, double w, int32_t size1,
                         int32_t size2, int32_t size3, InterpOption opt,
                         int32_t *indices, double *du, double *dv, double *dw) {
  int32_t u0 = SaturateInt32(static_cast<int32_t>(floor(u)), 0, size1 - 2);
  int32_t v0 = SaturateInt32(static_cast<int32_t>(floor(v)), 0, size2 - 2);
  int32_t w0 = SaturateInt32(static_cast<int32_t>(floor(w)), 0, size3 - 2);

  if (opt & kInterpOptionSaturate) {
    u = Saturate(u, 0.0, static_cast<double>(size1 - 1));
    v = Saturate(v, 0.0, static_cast<double>(size2 - 1));
    w = Saturate(w, 0.0, static_cast<double>(size3 - 1));
  }

  indices[0] = u0 * size2 * size3 + v0 * size3 + w0;
  indices[1] = u0 * size2 * size3 + v0 * size3 + (w0 + 1);
  indices[2] = u0 * size2 * size3 + (v0 + 1) * size3 + w0;
  indices[3] = u0 * size2 * size3 + (v0 + 1) * size3 + (w0 + 1);
  indices[4] = (u0 + 1) * size2 * size3 + v0 * size3 + w0;
  indices[5] = (u0 + 1) * size2 * size3 + v0 * size3 + (w0 + 1);
  indices[6] = (u0 + 1) * size2 * size3 + (v0 + 1) * size3 + w0;
  indices[7] = (u0 + 1) * size2 * size3 + (v0 + 1) * size3 + (w0 + 1);

  *du = u - static_cast<double>(u0);
  *dv = v - static_cast<double>(v0);
  *dw = w - static_cast<double>(w0);
}

double gsl_interp3_scaled_helper(const gsl_vector *z, int32_t indices[],
                                 double du, double dv, double dw) {
  double z000 = gsl_vector_get(z, indices[0]);
  double z001 = gsl_vector_get(z, indices[1]);
  double z010 = gsl_vector_get(z, indices[2]);
  double z011 = gsl_vector_get(z, indices[3]);
  double z100 = gsl_vector_get(z, indices[4]);
  double z101 = gsl_vector_get(z, indices[5]);
  double z110 = gsl_vector_get(z, indices[6]);
  double z111 = gsl_vector_get(z, indices[7]);

  return z000 + (z100 - z000) * du + (z010 - z000) * dv + (z001 - z000) * dw +
         (z000 - z100 - z010 + z110) * du * dv +
         (z000 - z100 - z001 + z101) * du * dw +
         (z000 - z010 - z001 + z011) * dv * dw +
         (-z000 + z100 + z010 + z001 - z110 - z101 - z011 + z111) * du * dv *
             dw;
}

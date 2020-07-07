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

#ifndef SIM_MATH_UTIL_H_
#define SIM_MATH_UTIL_H_

#include <stdint.h>

#include <gsl/gsl_matrix.h>
#include <gsl/gsl_vector.h>

#include <functional>
#include <limits>
#include <random>
#include <string>

#include "common/c_math/util.h"

// Generates random numbers using a generator seeded by a string hash. This
// facilitates binding of generators to simulator models.
//
// A static seed offset can be specified using SetSeedOffset, supporting
// deterministic perturbation of the generators used by all models.
class NamedRandomNumberGenerator {
 public:
  explicit NamedRandomNumberGenerator(const std::string &name)
      : generator_(static_cast<uint32_t>(std::hash<std::string>{}(name)) +
                   seed_offset_),
        normal_(0.0, 1.0),
        uniform_real_(0.0, 1.0),
        uniform_int32_(0, std::numeric_limits<int32_t>::max()) {}

  // Returns a random number drawn from a normal distribution with mean=0.0
  // and stddev=1.0.
  double GetNormal() { return normal_(generator_); }

  // Returns a random number drawn from a uniform distribution on [0.0, 1.0).
  double GetUniformReal() { return uniform_real_(generator_); }

  // Returns a random integer drawn from a uniform distribution on
  // [0, std::numeric_limits<int32_t>::max()].
  int32_t GetUniformInt32() { return uniform_int32_(generator_); }

  static void SetSeedOffset(uint32_t offset) { seed_offset_ = offset; }

 private:
  static uint32_t seed_offset_;

  // 32-bit Mersenne Twister generator.
  std::mt19937 generator_;

  std::normal_distribution<double> normal_;
  std::uniform_real_distribution<double> uniform_real_;
  std::uniform_int_distribution<int32_t> uniform_int32_;
};

// The hypergeometric function, {_2}F_1(a, b; c; z)
double HyperGeom(double a, double b, double c, double z);
uint32_t Factorial(uint32_t n);

// The rising factorial, denoted by the Pochhammer symbol (q)_n, is
// defined by:
//
//   (q)_n = 1                     if n == 0
//           q*(q+1)*...*(q+n-1)   if n > 0
//
// Warning: The Pochhammer symbol can denote either rising or falling
// factorial depending on context.
double RisingFactorial(double q, int32_t n);

// Computes the intersection point of two circles in a coordinate
// system centered at the first circle and with the positive x-axis
// along the line between the centers of the first and second circles.
// For both the non-degenerate intersecting and single point
// intersecting cases, this returns the intersection point with
// non-negative y.  For the identical center case, this returns the
// point (r1, 0) where r1 is the radius of the first circle.  For the
// remaining non-intersecting case, this returns the point:
//
//   ((d^2 + r1^2 - r2^2) / (2 * d), 0)
//
// For a derivation of the formulas used here, see:
// http://mathworld.wolfram.com/Circle-CircleIntersection.html
//
// Args:
//   d: Distance, always positive, between circle centers.
//   r1: Radius of first circle.
//   r2: Radius of second circle.
//   x: Output pointer to x coordinate of intersection (i.e. the
//       distance along line between first and second circle).
//   y: Output pointer to y coordinate of intersection (i.e. the
//       perpendicular distance, always positive, from line connecting
//       circle centers).
//
// Returns:
//   True if the circles intersect.
bool IntersectCircles(double d, double r1, double r2, double *x, double *y);

// Finds the fractional index of a value x_i in a GSL vector x
// assuming x is monotonically increasing.
double gsl_interp_index(const gsl_vector *x, double x_i);

// Performs a linear interpolation along a pair of GSL vectors.  The
// function to interpolate on is described by the input GSL vector x
// and the output GSL vector y.  Interp1 linearly interpolates to find
// the value of y for input x_i.  If opt is kInterpOptionDefault,
// extrapolate linearly beyond the end of the vectors.  If opt is
// kInterpOptionSaturate, saturate the output value at the
// ends of the output (y) vector.
//
// Warning: This currently assumes a monotonically increasing x.
double gsl_interp1(const gsl_vector *x, const gsl_vector *y, double xi,
                   InterpOption opt);

// Linear interpolation along a pair of vectors and a matrix.
//
// The function to interpolate on is described by the input vectors x
// and y and the output matrix z, with corresponding entries for its
// value at each (x,y) input pair.  Interp2 bilinearly interpolates to
// find the value of z for inputs x_i and y_i.  If opt is
// kInterpOptionDefault, extrapolate linearly beyond the end of the
// vectors.  If opt is kInterpOptionSaturate, saturate the
// output value at the edges of the output (z) matrix.
//
// Warning: This currently assumes a monotonically increasing x, y.
double gsl_interp2(const gsl_vector *x, const gsl_vector *y,
                   const gsl_matrix *z, double x_i, double y_i,
                   InterpOption opt);
double gsl_interp2_scaled(double s, double t, const gsl_matrix *z,
                          InterpOption opt);
void gsl_interp2_indices(double s, double t, int32_t size1, int32_t size2,
                         InterpOption opt, int32_t *s0, int32_t *t0, double *ds,
                         double *dt);
double gsl_interp2_scaled_helper(const gsl_matrix *z, int32_t s0, int32_t t0,
                                 double ds, double dt);

// Linear interpolation along a set of three vectors and a
// three-dimensional matrix.
//
// The function to interpolate on is described by the input vectors x,
// y, and z and the output matrix mat, with corresponding entries for
// its value at each (x,y,z) input pair.  Interp3 bilinearly
// interpolates to find the value of mat for inputs x_i, y_i, and z_i.
// If opt is kInterpOptionDefault, extrapolate linearly beyond the
// end of the vectors.  If opt is kInterpOptionSaturate,
// saturate the output value at the edges of the output (mat) matrix.
//
// Warning: This currently assumes a monotonically increasing x, y, z.
double gsl_interp3(const gsl_vector *x, const gsl_vector *y,
                   const gsl_vector *z, const gsl_vector *mat, double x_i,
                   double y_i, double z_i, InterpOption opt);
double gsl_interp3_scaled(double u, double v, double w, const gsl_vector *z,
                          int32_t size1, int32_t size2, int32_t size3,
                          InterpOption opt);
void gsl_interp3_indices(double u, double v, double w, int32_t size1,
                         int32_t size2, int32_t size3, InterpOption opt,
                         int32_t *indices, double *du, double *dv, double *dw);
double gsl_interp3_scaled_helper(const gsl_vector *z, int32_t indices[],
                                 double du, double dv, double dw);

#endif  // SIM_MATH_UTIL_H_

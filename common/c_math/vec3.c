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

#include "common/c_math/vec3.h"

#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdbool.h>
#include <stddef.h>

// Define Vec3 zero, identity, X, Y, and Z vectors.
const Vec3 kVec3Zero = {0.0, 0.0, 0.0};
const Vec3 kVec3Ones = {1.0, 1.0, 1.0};
const Vec3 kVec3X = {1.0, 0.0, 0.0};
const Vec3 kVec3Y = {0.0, 1.0, 0.0};
const Vec3 kVec3Z = {0.0, 0.0, 1.0};

// v_out = v0 + v1.
const Vec3 *Vec3Add(const Vec3 *v0, const Vec3 *v1, Vec3 *v_out) {
  v_out->x = v0->x + v1->x;
  v_out->y = v0->y + v1->y;
  v_out->z = v0->z + v1->z;
  return v_out;
}

// v_out = v0 + v1 + v2.
const Vec3 *Vec3Add3(const Vec3 *v0, const Vec3 *v1, const Vec3 *v2,
                     Vec3 *v_out) {
  v_out->x = v0->x + v1->x + v2->x;
  v_out->y = v0->y + v1->y + v2->y;
  v_out->z = v0->z + v1->z + v2->z;
  return v_out;
}

// v_out = v0 - v1.
const Vec3 *Vec3Sub(const Vec3 *v0, const Vec3 *v1, Vec3 *v_out) {
  v_out->x = v0->x - v1->x;
  v_out->y = v0->y - v1->y;
  v_out->z = v0->z - v1->z;
  return v_out;
}

// Scales all elements of a vector by a scalar.
const Vec3 *Vec3Scale(const Vec3 *v_in, double scale, Vec3 *v_out) {
  v_out->x = scale * v_in->x;
  v_out->y = scale * v_in->y;
  v_out->z = scale * v_in->z;
  return v_out;
}

// Returns the component-wise minimum of two vectors.
const Vec3 *Vec3Min(const Vec3 *a, const Vec3 *b, Vec3 *c) {
  c->x = fmin(a->x, b->x);
  c->y = fmin(a->y, b->y);
  c->z = fmin(a->z, b->z);
  return c;
}

// Linear mix between two vectors: v_out = c0 * v0 + c1 * v1.
const Vec3 *Vec3LinComb(double c0, const Vec3 *v0, double c1, const Vec3 *v1,
                        Vec3 *v_out) {
  v_out->x = c0 * v0->x + c1 * v1->x;
  v_out->y = c0 * v0->y + c1 * v1->y;
  v_out->z = c0 * v0->z + c1 * v1->z;
  return v_out;
}

// Linear mix between three vectors: v_out = c0 * v0 + c1 * v1 + c2 * v2.
const Vec3 *Vec3LinComb3(double c0, const Vec3 *v0, double c1, const Vec3 *v1,
                         double c2, const Vec3 *v2, Vec3 *v_out) {
  v_out->x = c0 * v0->x + c1 * v1->x + c2 * v2->x;
  v_out->y = c0 * v0->y + c1 * v1->y + c2 * v2->y;
  v_out->z = c0 * v0->z + c1 * v1->z + c2 * v2->z;
  return v_out;
}

// v_out = a*v0 + v1.
const Vec3 *Vec3Axpy(double a, const Vec3 *v, Vec3 *v_out) {
  v_out->x += a * v->x;
  v_out->y += a * v->y;
  v_out->z += a * v->z;
  return v_out;
}

// v_out = v0 .* v1.
const Vec3 *Vec3Mult(const Vec3 *v0, const Vec3 *v1, Vec3 *v_out) {
  v_out->x = v0->x * v1->x;
  v_out->y = v0->y * v1->y;
  v_out->z = v0->z * v1->z;
  return v_out;
}

// Cross-product of two vectors.
const Vec3 *Vec3Cross(const Vec3 *v0, const Vec3 *v1, Vec3 *v_out) {
  // This temporary variable makes it safe to reuse v0 or v1 in the
  // output.
  Vec3 vtmp = {v0->y * v1->z - v0->z * v1->y, v0->z * v1->x - v0->x * v1->z,
               v0->x * v1->y - v0->y * v1->x};
  *v_out = vtmp;
  return v_out;
}

// v0^T * v1.
double Vec3Dot(const Vec3 *v0, const Vec3 *v1) {
  return v0->x * v1->x + v0->y * v1->y + v0->z * v1->z;
}

// Returns the length of the vector.
double Vec3Norm(const Vec3 *v) {
  assert(v != NULL);
  return hypot(v->x, hypot(v->y, v->z));
}

// Computes the length of the vector, bounded below by "low".
double Vec3NormBound(const Vec3 *v, double low) {
  return fmax(Vec3Norm(v), low);
}

double Vec3NormSquared(const Vec3 *v) { return Vec3Dot(v, v); }

// Rescales v_in so that the norm is 1, when possible, otherwise
// return kVec3Zero.  Infinities are "rounded down" to DBL_MAX, and
// NaN is replaced with zero.  kVec3Zero is returned when the input
// (after these substitutions) is not normalizable.  Care is taken to
// avoid unnecessary underflow or overflow.
const Vec3 *Vec3Normalize(const Vec3 *v_in, Vec3 *v_out) {
  assert(v_in != NULL);
  assert(v_out != NULL);
  assert(isfinite(v_in->x) && isfinite(v_in->y) && isfinite(v_in->z));

  *v_out = *v_in;

  // Turn NaN into zero, and +/- Inf into +/- DBL_MAX.
  if (isnan(v_out->x)) v_out->x = 0.0;
  if (isnan(v_out->y)) v_out->y = 0.0;
  if (isnan(v_out->z)) v_out->z = 0.0;

  if (isinf(v_out->x)) v_out->x = copysign(DBL_MAX, v_out->x);
  if (isinf(v_out->y)) v_out->y = copysign(DBL_MAX, v_out->y);
  if (isinf(v_out->z)) v_out->z = copysign(DBL_MAX, v_out->z);

  double v_max = fmax(fabs(v_out->x), fmax(fabs(v_out->y), fabs(v_out->z)));

  if (v_max > 0.0) {
    // First scale by the largest component, so that the norm is then
    // guaranteed to be representable.  This allows us to normalize
    // vectors like {DBL_MAX, DBL_MAX, DBL_MAX}, whose norm is not
    // representable.
    double denom = sqrt((v_out->x / v_max) * (v_out->x / v_max) +
                        (v_out->y / v_max) * (v_out->y / v_max) +
                        (v_out->z / v_max) * (v_out->z / v_max));

    // Scale the individual vector components.  We avoid using
    // Vec3Scale, as (1.0/v_max) may overflow.  Similarly, we avoid
    // computing (v_max * denom) as that quantity may also not
    // exist.
    v_out->x = (v_out->x / v_max) / denom;
    v_out->y = (v_out->y / v_max) / denom;
    v_out->z = (v_out->z / v_max) / denom;
  } else {
    // Vector is not normalizable.
    *v_out = kVec3Zero;
  }
  return v_out;
}

double Vec3XyNorm(const Vec3 *v) { return hypot(v->x, v->y); }

double Vec3XzNorm(const Vec3 *v) { return hypot(v->x, v->z); }

double Vec3YzNorm(const Vec3 *v) { return hypot(v->y, v->z); }

// Calculate the norm of the difference between two vectors.
double Vec3Distance(const Vec3 *a, const Vec3 *b) {
  Vec3 dx;
  Vec3Sub(a, b, &dx);
  return Vec3Norm(&dx);
}

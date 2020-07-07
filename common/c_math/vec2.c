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

#include "common/c_math/vec2.h"

#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdbool.h>
#include <stddef.h>

// Define Vec2 zero, identity, X vector, and Y vector constants.
const Vec2 kVec2Zero = {0.0, 0.0};
const Vec2 kVec2Ones = {1.0, 1.0};
const Vec2 kVec2X = {1.0, 0.0};
const Vec2 kVec2Y = {0.0, 1.0};

// v_out = v0 + v1.
const Vec2 *Vec2Add(const Vec2 *v0, const Vec2 *v1, Vec2 *v_out) {
  v_out->x = v0->x + v1->x;
  v_out->y = v0->y + v1->y;
  return v_out;
}

// v_out = v0 + v1 + v2.
const Vec2 *Vec2Add3(const Vec2 *v0, const Vec2 *v1, const Vec2 *v2,
                     Vec2 *v_out) {
  v_out->x = v0->x + v1->x + v2->x;
  v_out->y = v0->y + v1->y + v2->y;
  return v_out;
}

// v_out = v0 - v1.
const Vec2 *Vec2Sub(const Vec2 *v0, const Vec2 *v1, Vec2 *v_out) {
  v_out->x = v0->x - v1->x;
  v_out->y = v0->y - v1->y;
  return v_out;
}

// Scales all elements of a vector by a scalar.
const Vec2 *Vec2Scale(const Vec2 *v_in, double scale, Vec2 *v_out) {
  v_out->x = scale * v_in->x;
  v_out->y = scale * v_in->y;
  return v_out;
}

// Linear mix between two vectors: v_out = c0 * v0 + c1 * v1
const Vec2 *Vec2LinComb(double c0, const Vec2 *v0, double c1, const Vec2 *v1,
                        Vec2 *v_out) {
  v_out->x = c0 * v0->x + c1 * v1->x;
  v_out->y = c0 * v0->y + c1 * v1->y;
  return v_out;
}

// Linear mix between three vectors: v_out = c0 * v0 + c1 * v1 + c2 * v2.
const Vec2 *Vec2LinComb3(double c0, const Vec2 *v0, double c1, const Vec2 *v1,
                         double c2, const Vec2 *v2, Vec2 *v_out) {
  v_out->x = c0 * v0->x + c1 * v1->x + c2 * v2->x;
  v_out->y = c0 * v0->y + c1 * v1->y + c2 * v2->y;
  return v_out;
}

// v_out = v0 .* v1.
const Vec2 *Vec2Mult(const Vec2 *v0, const Vec2 *v1, Vec2 *v_out) {
  v_out->x = v0->x * v1->x;
  v_out->y = v0->y * v1->y;
  return v_out;
}

// v0^T * v1.
double Vec2Dot(const Vec2 *v0, const Vec2 *v1) {
  return v0->x * v1->x + v0->y * v1->y;
}

// Returns the length of the vector.
double Vec2Norm(const Vec2 *v) { return hypot(v->x, v->y); }

// Computes the length of the vector, bounded below by "low".
double Vec2NormBound(const Vec2 *v, double low) {
  return fmax(Vec2Norm(v), low);
}

double Vec2NormSquared(const Vec2 *v) { return Vec2Dot(v, v); }

// Rescales v_in so that the norm is 1, when possible, otherwise
// return kVec2Zero.  Infinities are "rounded down" to DBL_MAX, and
// NaN is replaced with zero.  kVec2Zero is returned when the input
// (after these substitutions) is not normalizable.  Care is taken to
// avoid unnecessary underflow or overflow.
const Vec2 *Vec2Normalize(const Vec2 *v_in, Vec2 *v_out) {
  assert(v_in != NULL);
  assert(v_out != NULL);
  assert(isfinite(v_in->x) && isfinite(v_in->y));

  *v_out = *v_in;

  // Turn NaN into zero, and +/- Inf into +/- DBL_MAX.
  if (isnan(v_out->x)) v_out->x = 0.0;
  if (isnan(v_out->y)) v_out->y = 0.0;

  if (isinf(v_out->x)) v_out->x = copysign(DBL_MAX, v_out->x);
  if (isinf(v_out->y)) v_out->y = copysign(DBL_MAX, v_out->y);

  double v_max = fmax(fabs(v_out->x), fabs(v_out->y));

  if (v_max > 0.0) {
    // First scale by the largest component, so that the norm is then
    // guaranteed to be representable.  This allows us to normalize
    // vectors like {DBL_MAX, DBL_MAX}, whose norm is not
    // representable.
    double denom = sqrt((v_out->x / v_max) * (v_out->x / v_max) +
                        (v_out->y / v_max) * (v_out->y / v_max));

    // Scale the individual vector components.  We avoid using
    // Vec2Scale, as (1.0/v_max) may overflow.  Similarly, we avoid
    // computing (v_max * denom) as that quantity may also not
    // exist.
    v_out->x = (v_out->x / v_max) / denom;
    v_out->y = (v_out->y / v_max) / denom;
  } else {
    // Vector is not normalizable.
    assert(false);
    *v_out = kVec2Zero;
  }
  return v_out;
}

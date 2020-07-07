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

#include "common/c_math/vec2f.h"

#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdbool.h>
#include <stddef.h>

// Define Vec2f zero, identity, X vector, and Y vector constants.
const Vec2f kVec2fZero = {0.0f, 0.0f};
const Vec2f kVec2fOnes = {1.0f, 1.0f};
const Vec2f kVec2fX = {1.0f, 0.0f};
const Vec2f kVec2fY = {0.0f, 1.0f};

// v_out = v0 + v1.
const Vec2f *Vec2fAdd(const Vec2f *v0, const Vec2f *v1, Vec2f *v_out) {
  v_out->x = v0->x + v1->x;
  v_out->y = v0->y + v1->y;
  return v_out;
}

// v_out = v0 + v1 + v2.
const Vec2f *Vec2fAdd3(const Vec2f *v0, const Vec2f *v1, const Vec2f *v2,
                       Vec2f *v_out) {
  v_out->x = v0->x + v1->x + v2->x;
  v_out->y = v0->y + v1->y + v2->y;
  return v_out;
}

// v_out = v0 - v1.
const Vec2f *Vec2fSub(const Vec2f *v0, const Vec2f *v1, Vec2f *v_out) {
  v_out->x = v0->x - v1->x;
  v_out->y = v0->y - v1->y;
  return v_out;
}

// Scales all elements of a vector by a scalar.
const Vec2f *Vec2fScale(const Vec2f *v_in, float scale, Vec2f *v_out) {
  v_out->x = scale * v_in->x;
  v_out->y = scale * v_in->y;
  return v_out;
}

// Linear mix between two vectors: v_out = c0 * v0 + c1 * v1
const Vec2f *Vec2fLinComb(float c0, const Vec2f *v0, float c1, const Vec2f *v1,
                          Vec2f *v_out) {
  v_out->x = c0 * v0->x + c1 * v1->x;
  v_out->y = c0 * v0->y + c1 * v1->y;
  return v_out;
}

// Linear mix between three vectors: v_out = c0 * v0 + c1 * v1 + c2 * v2.
const Vec2f *Vec2fLinComb3(float c0, const Vec2f *v0, float c1, const Vec2f *v1,
                           float c2, const Vec2f *v2, Vec2f *v_out) {
  v_out->x = c0 * v0->x + c1 * v1->x + c2 * v2->x;
  v_out->y = c0 * v0->y + c1 * v1->y + c2 * v2->y;
  return v_out;
}

// v_out = v0 .* v1.
const Vec2f *Vec2fMult(const Vec2f *v0, const Vec2f *v1, Vec2f *v_out) {
  v_out->x = v0->x * v1->x;
  v_out->y = v0->y * v1->y;
  return v_out;
}

// v0^T * v1.
float Vec2fDot(const Vec2f *v0, const Vec2f *v1) {
  return v0->x * v1->x + v0->y * v1->y;
}

// Returns the length of the vector.
float Vec2fNorm(const Vec2f *v) { return hypotf(v->x, v->y); }

// Computes the length of the vector, bounded below by "low".
float Vec2fNormBound(const Vec2f *v, float low) {
  return fmaxf(Vec2fNorm(v), low);
}

float Vec2fNormSquared(const Vec2f *v) { return Vec2fDot(v, v); }

// Rescales v_in so that the norm is 1, when possible, otherwise
// return kVec2fZero.  Infinities are "rounded down" to FLT_MAX, and
// NaN is replaced with zero.  kVec2fZero is returned when the input
// (after these substitutions) is not normalizable.  Care is taken to
// avoid unnecessary underflow or overflow.
const Vec2f *Vec2fNormalize(const Vec2f *v_in, Vec2f *v_out) {
  assert(v_in != NULL);
  assert(v_out != NULL);
  assert(isfinite(v_in->x) && isfinite(v_in->y));

  *v_out = *v_in;

  // Turn NaN into zero, and +/- Inf into +/- FLT_MAX.
  if (isnan(v_out->x)) v_out->x = 0.0f;
  if (isnan(v_out->y)) v_out->y = 0.0f;

  if (isinf(v_out->x)) v_out->x = copysignf(FLT_MAX, v_out->x);
  if (isinf(v_out->y)) v_out->y = copysignf(FLT_MAX, v_out->y);

  float v_max = fmaxf(fabsf(v_out->x), fabsf(v_out->y));

  if (v_max > 0.0f) {
    // First scale by the largest component, so that the norm is then
    // guaranteed to be representable.  This allows us to normalize
    // vectors like {FLT_MAX, FLT_MAX}, whose norm is not
    // representable.
    float denom = sqrtf((v_out->x / v_max) * (v_out->x / v_max) +
                        (v_out->y / v_max) * (v_out->y / v_max));

    // Scale the individual vector components.  We avoid using
    // Vec2fScale, as (1.0f/v_max) may overflow.  Similarly, we avoid
    // computing (v_max * denom) as that quantity may also not
    // exist.
    v_out->x = (v_out->x / v_max) / denom;
    v_out->y = (v_out->y / v_max) / denom;
  } else {
    // Vector is not normalizable.
    assert(false);
    *v_out = kVec2fZero;
  }
  return v_out;
}

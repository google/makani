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

#include "common/c_math/vec3f.h"

#include <assert.h>
#include <float.h>
#include <math.h>
#include <stddef.h>

// Define Vec3f zero, identity, X, Y, and Z vectors.
const Vec3f kVec3fZero = {0.0f, 0.0f, 0.0f};
const Vec3f kVec3fOnes = {1.0f, 1.0f, 1.0f};
const Vec3f kVec3fX = {1.0f, 0.0f, 0.0f};
const Vec3f kVec3fY = {0.0f, 1.0f, 0.0f};
const Vec3f kVec3fZ = {0.0f, 0.0f, 1.0f};

// v_out = v0 + v1.
const Vec3f *Vec3fAdd(const Vec3f *v0, const Vec3f *v1, Vec3f *v_out) {
  v_out->x = v0->x + v1->x;
  v_out->y = v0->y + v1->y;
  v_out->z = v0->z + v1->z;
  return v_out;
}

// v_out = v0 + v1 + v2.
const Vec3f *Vec3fAdd3(const Vec3f *v0, const Vec3f *v1, const Vec3f *v2,
                       Vec3f *v_out) {
  v_out->x = v0->x + v1->x + v2->x;
  v_out->y = v0->y + v1->y + v2->y;
  v_out->z = v0->z + v1->z + v2->z;
  return v_out;
}

// v_out = v0 - v1.
const Vec3f *Vec3fSub(const Vec3f *v0, const Vec3f *v1, Vec3f *v_out) {
  v_out->x = v0->x - v1->x;
  v_out->y = v0->y - v1->y;
  v_out->z = v0->z - v1->z;
  return v_out;
}

// Scales all elements of a vector by a scalar.
const Vec3f *Vec3fScale(const Vec3f *v_in, float scale, Vec3f *v_out) {
  v_out->x = scale * v_in->x;
  v_out->y = scale * v_in->y;
  v_out->z = scale * v_in->z;
  return v_out;
}

// Returns the component-wise minimum of two vectors.
const Vec3f *Vec3fMin(const Vec3f *a, const Vec3f *b, Vec3f *c) {
  c->x = fminf(a->x, b->x);
  c->y = fminf(a->y, b->y);
  c->z = fminf(a->z, b->z);
  return c;
}

// Linear mix between two vectors: v_out = c0 * v0 + c1 * v1.
const Vec3f *Vec3fLinComb(float c0, const Vec3f *v0, float c1, const Vec3f *v1,
                          Vec3f *v_out) {
  v_out->x = c0 * v0->x + c1 * v1->x;
  v_out->y = c0 * v0->y + c1 * v1->y;
  v_out->z = c0 * v0->z + c1 * v1->z;
  return v_out;
}

// Linear mix between three vectors: v_out = c0 * v0 + c1 * v1 + c2 * v2.
const Vec3f *Vec3fLinComb3(float c0, const Vec3f *v0, float c1, const Vec3f *v1,
                           float c2, const Vec3f *v2, Vec3f *v_out) {
  v_out->x = c0 * v0->x + c1 * v1->x + c2 * v2->x;
  v_out->y = c0 * v0->y + c1 * v1->y + c2 * v2->y;
  v_out->z = c0 * v0->z + c1 * v1->z + c2 * v2->z;
  return v_out;
}

// v_out = a*v0 + v1.
const Vec3f *Vec3fAxpy(float a, const Vec3f *v, Vec3f *v_out) {
  v_out->x += a * v->x;
  v_out->y += a * v->y;
  v_out->z += a * v->z;
  return v_out;
}

// v_out = v0 .* v1.
const Vec3f *Vec3fMult(const Vec3f *v0, const Vec3f *v1, Vec3f *v_out) {
  v_out->x = v0->x * v1->x;
  v_out->y = v0->y * v1->y;
  v_out->z = v0->z * v1->z;
  return v_out;
}

// Cross-product of two vectors.
const Vec3f *Vec3fCross(const Vec3f *v0, const Vec3f *v1, Vec3f *v_out) {
  // This temporary variable makes it safe to reuse v0 or v1 in the
  // output.
  Vec3f vtmp = {v0->y * v1->z - v0->z * v1->y, v0->z * v1->x - v0->x * v1->z,
                v0->x * v1->y - v0->y * v1->x};
  *v_out = vtmp;
  return v_out;
}

// v0^T * v1.
float Vec3fDot(const Vec3f *v0, const Vec3f *v1) {
  return v0->x * v1->x + v0->y * v1->y + v0->z * v1->z;
}

// Returns the length of the vector.
float Vec3fNorm(const Vec3f *v) {
  assert(v != NULL);
  return hypotf(v->x, hypotf(v->y, v->z));
}

// Computes the length of the vector, bounded below by "low".
float Vec3fNormBound(const Vec3f *v, float low) {
  return fmaxf(Vec3fNorm(v), low);
}

float Vec3fNormSquared(const Vec3f *v) { return Vec3fDot(v, v); }

// Rescales v_in so that the norm is 1, when possible, otherwise
// return kVec3fZero.  Infinities are "rounded down" to FLT_MAX, and
// NaN is replaced with zero.  kVec3fZero is returned when the input
// (after these substitutions) is not normalizable.  Care is taken to
// avoid unnecessary underflow or overflow.
const Vec3f *Vec3fNormalize(const Vec3f *v_in, Vec3f *v_out) {
  assert(v_in != NULL);
  assert(v_out != NULL);
  assert(isfinite(v_in->x) && isfinite(v_in->y) && isfinite(v_in->z));

  *v_out = *v_in;

  // Turn NaN into zero, and +/- Inf into +/- FLT_MAX.
  if (isnan(v_out->x)) v_out->x = 0.0f;
  if (isnan(v_out->y)) v_out->y = 0.0f;
  if (isnan(v_out->z)) v_out->z = 0.0f;

  if (isinf(v_out->x)) v_out->x = copysignf(FLT_MAX, v_out->x);
  if (isinf(v_out->y)) v_out->y = copysignf(FLT_MAX, v_out->y);
  if (isinf(v_out->z)) v_out->z = copysignf(FLT_MAX, v_out->z);

  float v_max = fmaxf(fabsf(v_out->x), fmaxf(fabsf(v_out->y), fabsf(v_out->z)));

  if (v_max > 0.0f) {
    // First scale by the largest component, so that the norm is then
    // guaranteed to be representable.  This allows us to normalize
    // vectors like {FLT_MAX, FLT_MAX, FLT_MAX}, whose norm is not
    // representable.
    float denom = sqrtf((v_out->x / v_max) * (v_out->x / v_max) +
                        (v_out->y / v_max) * (v_out->y / v_max) +
                        (v_out->z / v_max) * (v_out->z / v_max));

    // Scale the individual vector components.  We avoid using
    // Vec3fScale, as (1.0f/v_max) may overflow.  Similarly, we avoid
    // computing (v_max * denom) as that quantity may also not
    // exist.
    v_out->x = (v_out->x / v_max) / denom;
    v_out->y = (v_out->y / v_max) / denom;
    v_out->z = (v_out->z / v_max) / denom;
  } else {
    // Vector is not normalizable.
    *v_out = kVec3fZero;
  }
  return v_out;
}

float Vec3fXyNorm(const Vec3f *v) { return hypotf(v->x, v->y); }

float Vec3fXzNorm(const Vec3f *v) { return hypotf(v->x, v->z); }

float Vec3fYzNorm(const Vec3f *v) { return hypotf(v->y, v->z); }

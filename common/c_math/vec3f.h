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

// The Vec3f library provides functions for performing simple
// arithmetic operations (addition, subtraction, scaling, dot product,
// cross product, norm, etc.) on three-dimensional vectors.

#ifndef COMMON_C_MATH_VEC3F_H_
#define COMMON_C_MATH_VEC3F_H_

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct Vec3f { float x, y, z; } Vec3f;

extern const Vec3f kVec3fZero;
extern const Vec3f kVec3fOnes;
extern const Vec3f kVec3fX;
extern const Vec3f kVec3fY;
extern const Vec3f kVec3fZ;

#define VEC3F_DISP(v)                                                          \
  printf("%s:%u %s = [%.12f, %.12f, %.12f]'\n", __FILE__, __LINE__, #v, (v).x, \
         (v).y, (v).z)

const Vec3f *Vec3fAdd(const Vec3f *v0, const Vec3f *v1, Vec3f *v_out);
const Vec3f *Vec3fAdd3(const Vec3f *v0, const Vec3f *v1, const Vec3f *v2,
                       Vec3f *v_out);
const Vec3f *Vec3fSub(const Vec3f *v0, const Vec3f *v1, Vec3f *v_out);
const Vec3f *Vec3fScale(const Vec3f *v_in, float scale, Vec3f *v_out);
const Vec3f *Vec3fMin(const Vec3f *a, const Vec3f *b, Vec3f *c);
const Vec3f *Vec3fLinComb(float c0, const Vec3f *v0, float c1, const Vec3f *v1,
                          Vec3f *v_out);
const Vec3f *Vec3fLinComb3(float c0, const Vec3f *v0, float c1, const Vec3f *v1,
                           float c2, const Vec3f *v2, Vec3f *v_out);
const Vec3f *Vec3fAxpy(float a, const Vec3f *v, Vec3f *v_out);
const Vec3f *Vec3fMult(const Vec3f *v0, const Vec3f *v1, Vec3f *v_out);
const Vec3f *Vec3fCross(const Vec3f *v0, const Vec3f *v1, Vec3f *v_out);
float Vec3fDot(const Vec3f *v0, const Vec3f *v1);
float Vec3fNorm(const Vec3f *v);
float Vec3fNormBound(const Vec3f *v, float low);
float Vec3fNormSquared(const Vec3f *v);
const Vec3f *Vec3fNormalize(const Vec3f *v_in, Vec3f *v_out);
float Vec3fXyNorm(const Vec3f *v);
float Vec3fXzNorm(const Vec3f *v);
float Vec3fYzNorm(const Vec3f *v);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // COMMON_C_MATH_VEC3F_H_

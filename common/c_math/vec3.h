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

// The Vec3 library provides functions for performing simple
// arithmetic operations (addition, subtraction, scaling, dot product,
// cross product, norm, etc.) on three-dimensional vectors.

#ifndef COMMON_C_MATH_VEC3_H_
#define COMMON_C_MATH_VEC3_H_

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct Vec3 { double x, y, z; } Vec3;

extern const Vec3 kVec3Zero;
extern const Vec3 kVec3Ones;
extern const Vec3 kVec3X;
extern const Vec3 kVec3Y;
extern const Vec3 kVec3Z;

#define VEC3_DISP(v)                                                           \
  printf("%s:%u %s = [%.12f, %.12f, %.12f]'\n", __FILE__, __LINE__, #v, (v).x, \
         (v).y, (v).z)

const Vec3 *Vec3Add(const Vec3 *v0, const Vec3 *v1, Vec3 *v_out);
const Vec3 *Vec3Add3(const Vec3 *v0, const Vec3 *v1, const Vec3 *v2,
                     Vec3 *v_out);
const Vec3 *Vec3Sub(const Vec3 *v0, const Vec3 *v1, Vec3 *v_out);
const Vec3 *Vec3Scale(const Vec3 *v_in, double scale, Vec3 *v_out);
const Vec3 *Vec3Min(const Vec3 *a, const Vec3 *b, Vec3 *c);
const Vec3 *Vec3LinComb(double c0, const Vec3 *v0, double c1, const Vec3 *v1,
                        Vec3 *v_out);
const Vec3 *Vec3LinComb3(double c0, const Vec3 *v0, double c1, const Vec3 *v1,
                         double c2, const Vec3 *v2, Vec3 *v_out);
const Vec3 *Vec3Axpy(double a, const Vec3 *v, Vec3 *v_out);
const Vec3 *Vec3Mult(const Vec3 *v0, const Vec3 *v1, Vec3 *v_out);
const Vec3 *Vec3Cross(const Vec3 *v0, const Vec3 *v1, Vec3 *v_out);
double Vec3Dot(const Vec3 *v0, const Vec3 *v1);
double Vec3Norm(const Vec3 *v);
double Vec3NormBound(const Vec3 *v, double low);
double Vec3NormSquared(const Vec3 *v);
const Vec3 *Vec3Normalize(const Vec3 *v_in, Vec3 *v_out);
double Vec3XyNorm(const Vec3 *v);
double Vec3XzNorm(const Vec3 *v);
double Vec3YzNorm(const Vec3 *v);
double Vec3Distance(const Vec3 *a, const Vec3 *b);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // COMMON_C_MATH_VEC3_H_

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

// The Vec2 library provides functions for performing simple
// arithmetic operations (addition, subtraction, scaling, dot product,
// norm, etc.) on two-dimensional vectors.

#ifndef COMMON_C_MATH_VEC2_H_
#define COMMON_C_MATH_VEC2_H_

#ifdef __cplusplus
extern "C" {
#endif

typedef struct Vec2 { double x, y; } Vec2;

extern const Vec2 kVec2Zero;
extern const Vec2 kVec2Ones;

#define VEC2_DISP(v) \
  printf("%s:%u %s = [%.12lf, %.12lf]'\n", __FILE__, __LINE__, #v, (v).x, (v).y)

const Vec2 *Vec2Add(const Vec2 *v0, const Vec2 *v1, Vec2 *v_out);
const Vec2 *Vec2Add3(const Vec2 *v0, const Vec2 *v1, const Vec2 *v2,
                     Vec2 *v_out);
const Vec2 *Vec2Sub(const Vec2 *v0, const Vec2 *v1, Vec2 *v_out);
const Vec2 *Vec2Scale(const Vec2 *v_in, double scale, Vec2 *v_out);
const Vec2 *Vec2LinComb(double c0, const Vec2 *v0, double c1, const Vec2 *v1,
                        Vec2 *v_out);
const Vec2 *Vec2LinComb3(double c0, const Vec2 *v0, double c1, const Vec2 *v1,
                         double c2, const Vec2 *v2, Vec2 *v_out);
const Vec2 *Vec2Mult(const Vec2 *v0, const Vec2 *v1, Vec2 *v_out);
double Vec2Dot(const Vec2 *v0, const Vec2 *v1);
double Vec2Norm(const Vec2 *v);
double Vec2NormBound(const Vec2 *v, double low);
double Vec2NormSquared(const Vec2 *v);
const Vec2 *Vec2Normalize(const Vec2 *v_in, Vec2 *v_out);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // COMMON_C_MATH_VEC2_H_

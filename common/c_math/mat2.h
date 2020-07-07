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

#ifndef COMMON_C_MATH_MAT2_H_
#define COMMON_C_MATH_MAT2_H_

#include "common/c_math/linalg_common.h"
#include "common/c_math/vec2.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct Mat2 { double d[2][2]; } Mat2;

extern const Mat2 kMat2Zero;
extern const Mat2 kMat2Identity;

#define MAT2_DISP(m)                                                          \
  printf("%s:%u %s = [%.12f, %.12f; %.12f, %.12f]\n", __FILE__, __LINE__, #m, \
         (m).d[0][0], (m).d[0][1], (m).d[1][0], (m).d[1][1]);

const Mat2 *Mat2Scale(const Mat2 *m_in, double scale, Mat2 *m_out);
const Vec2 *Mat2Vec2Axpby(const Mat2 *m, TransposeType t, const Vec2 *v0,
                          double a, const Vec2 *v1, Vec2 *v_out);
const Mat2 *Mat2Abpyc(const Mat2 *m0, TransposeType t0, const Mat2 *m1,
                      TransposeType t1, double a, const Mat2 *m2,
                      TransposeType t2, Mat2 *m_out);
const Mat2 *Mat2Add(const Mat2 *m0, TransposeType t0, const Mat2 *m1,
                    TransposeType t1, Mat2 *m_out);
const Mat2 *Mat2Mult(const Mat2 *m0, TransposeType t0, const Mat2 *m1,
                     TransposeType t1, Mat2 *m_out);
const Vec2 *Mat2Vec2Mult(const Mat2 *m, const Vec2 *v_in, Vec2 *v_out);
const Vec2 *Mat2TransVec2Mult(const Mat2 *m, const Vec2 *v_in, Vec2 *v_out);
double Mat2Det(const Mat2 *m);
const Mat2 *Mat2Inv(const Mat2 *m_in, Mat2 *m_out);
const Vec2 *Mat2Vec2LeftDivide(const Mat2 *m, const Vec2 *v_in, Vec2 *v_out);
double Mat2Trace(const Mat2 *m);
const Vec2 *Mat2Diag(const Mat2 *m, Vec2 *v);
const Mat2 *Mat2Trans(const Mat2 *m_in, Mat2 *m_out);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // COMMON_C_MATH_MAT2_H_

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

#ifndef COMMON_C_MATH_MAT3_H_
#define COMMON_C_MATH_MAT3_H_

#include <stdbool.h>

#include "common/c_math/linalg_common.h"
#include "common/c_math/vec3.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct Mat3 { double d[3][3]; } Mat3;

extern const Mat3 kMat3Zero;
extern const Mat3 kMat3Identity;

#define MAT3_DISP(m)                                                   \
  printf(                                                              \
      "%s:%u <(%s) [%.12f %.12f %.12f] [%.12f %.12f %.12f] "           \
      "[%.12f %.12f %.12f]>\n",                                        \
      __FILE__, __LINE__, #m, (m).d[0][0], (m).d[0][1], (m).d[0][2],   \
      (m).d[1][0], (m).d[1][1], (m).d[1][2], (m).d[2][0], (m).d[2][1], \
      (m).d[2][2]);

const Mat3 *Mat3Scale(const Mat3 *m_in, double scale, Mat3 *m_out);
const Vec3 *Mat3Vec3Axpby(const Mat3 *m, TransposeType t, const Vec3 *v0,
                          double a, const Vec3 *v1, Vec3 *v_out);
const Mat3 *Mat3Abpyc(const Mat3 *m0, TransposeType t0, const Mat3 *m1,
                      TransposeType t1, double a, const Mat3 *m2,
                      TransposeType t2, Mat3 *m_out);
const Mat3 *Mat3Add(const Mat3 *m0, TransposeType t0, const Mat3 *m1,
                    TransposeType t1, Mat3 *m_out);
const Mat3 *Mat3Mult(const Mat3 *m0, TransposeType t0, const Mat3 *m1,
                     TransposeType t1, Mat3 *m_out);
const Vec3 *Mat3Vec3Mult(const Mat3 *m, const Vec3 *v_in, Vec3 *v_out);
const Vec3 *Mat3TransVec3Mult(const Mat3 *m, const Vec3 *v_in, Vec3 *v_out);
const Mat3 *Mat3Mat3Mult(const Mat3 *m1, const Mat3 *m2, Mat3 *m_out);
double Mat3Det(const Mat3 *m);
const Mat3 *Mat3Inv(const Mat3 *m_in, Mat3 *m_out);
const Vec3 *Mat3Vec3LeftDivide(const Mat3 *m, const Vec3 *v_in, Vec3 *v_out);
double Mat3Trace(const Mat3 *m);
const Vec3 *Mat3Diag(const Mat3 *m, Vec3 *v);
const Mat3 *Mat3Trans(const Mat3 *m_in, Mat3 *m_out);
const Mat3 *Mat3Cross(const Vec3 *v, Mat3 *m);

bool Mat3ContainsNaN(const Mat3 *A);
bool Mat3IsOrthogonal(const Mat3 *A, double tol);
bool Mat3IsSpecialOrthogonal(const Mat3 *A, double tol);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // COMMON_C_MATH_MAT3_H_

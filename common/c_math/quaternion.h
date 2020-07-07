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

// The quaternion math library provides functions for doing basic
// quaternion arithmetic (e.g. addition, subtraction, etc.) and also
// functions for using quaternions as a representation of a rotation.
//
// A quaternion may be described by four numbers q0, q1, q2, and q3,
// and may be written as:
//
//   q = q0 + q1*i + q2*j + q3*k
//
// where i^2 = j^2 = k^2 = ijk = -1.  (Unfortunately, there are
// multiple popular conventions for the ordering of quaternion
// elements.  We follow the convention used by MATLAB where the scalar
// component of the quaternion is first and is labeled q0).
//
// When they are used to describe rotations, the unit norm quaternion
// may be written as:
//
//   q_hat = [cos(theta/2),
//            sin(theta/2) u_x,
//            sin(theta/2) u_y,
//            sin(theta/2) u_z]
//
// where [u_x, u_y, u_z] is the axis of rotation and theta is the
// angle of rotation.  The quaternion rotation functions are designed
// to handle non-unit-norm quaternions gracefully by first normalizing
// the quaternion.  However, we also assert out as this is likely an
// indication that there is a bug elsewhere in the code.

#ifndef COMMON_C_MATH_QUATERNION_H_
#define COMMON_C_MATH_QUATERNION_H_

#include <stdbool.h>

#include "common/c_math/geometry.h"
#include "common/c_math/mat3.h"
#include "common/c_math/vec3.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct Quat { double q0, q1, q2, q3; } Quat;

extern const Quat kQuatZero;
extern const Quat kQuatIdentity;

// Displays the quaternion in a MATLAB and Python friendly format for
// debugging purposes.
#define QUAT_DISP(q)                                                           \
  printf("%s:%u %s = [%.12f, %.12f, %.12f, %.12f]'\n", __FILE__, __LINE__, #q, \
         (q).q0, (q).q1, (q).q2, (q).q3)

// Basic quaternion arithmetic.
const Quat *QuatAdd(const Quat *q, const Quat *r, Quat *q_out);
const Quat *QuatSub(const Quat *q, const Quat *r, Quat *q_out);
const Quat *QuatScale(const Quat *q_in, double scale, Quat *q_out);
const Quat *QuatLinComb(double cq, const Quat *q, double cr, const Quat *r,
                        Quat *q_out);
const Quat *QuatLinComb3(double c0, const Quat *q0, double c1, const Quat *q1,
                         double c2, const Quat *q2, Quat *q_out);
const Quat *QuatConj(const Quat *q_in, Quat *q_out);
const Quat *QuatInv(const Quat *q_in, Quat *q_out);
const Quat *QuatMultiply(const Quat *q, const Quat *r, Quat *q_out);
const Quat *QuatDivide(const Quat *q1, const Quat *q2, Quat *q_out);
double QuatMaxAbs(const Quat *q);
double QuatDot(const Quat *q, const Quat *p);
double QuatModSquared(const Quat *q);
double QuatMod(const Quat *q);
bool QuatHasNaN(const Quat *q);
const Quat *QuatNormalize(const Quat *q_in, Quat *q_out);

// Quaternion rotation functions.
const Vec3 *QuatRotate(const Quat *q, const Vec3 *v_in, Vec3 *v_out);
const Mat3 *QuatToDcm(const Quat *q, Mat3 *dcm);
const Quat *DcmToQuat(const Mat3 *dcm, Quat *q);
double QuatToAxisAngle(const Quat *q, Vec3 *axis);
const Quat *AxisAngleToQuat(const Vec3 *axis, double angle, Quat *q);
const Vec3 *QuatToAxis(const Quat *q, Vec3 *axis);
const Quat *AxisToQuat(const Vec3 *axis, Quat *q);
void QuatToAngle(const Quat *q, RotationOrder order, double *r1, double *r2,
                 double *r3);
void AngleToQuat(double r1, double r2, double r3, RotationOrder order,
                 Quat *q_out);
void QuatToMrp(const Quat *q, Vec3 *mrp);
void MrpToQuat(const Vec3 *mrp, Quat *q);

// This function is only located in the quaternion library because it
// uses quaternions internally, and placing it in geometry.h would
// introduce a circular dependency.
void Vec3Vec3ToDcm(const Vec3 *a, const Vec3 *b, Mat3 *dcm_a2b);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // COMMON_C_MATH_QUATERNION_H_

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

#include "common/c_math/quaternion.h"

#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdbool.h>
#include <stddef.h>

#include "common/c_math/geometry.h"
#include "common/c_math/mat3.h"
#include "common/c_math/util.h"
#include "common/c_math/vec3.h"

// Define the zero and identity quaternions.
const Quat kQuatZero = {0.0, 0.0, 0.0, 0.0};
const Quat kQuatIdentity = {1.0, 0.0, 0.0, 0.0};

// q_out = q + r.
const Quat *QuatAdd(const Quat *q, const Quat *r, Quat *q_out) {
  assert(q != NULL && r != NULL && q_out != NULL);

  q_out->q0 = q->q0 + r->q0;
  q_out->q1 = q->q1 + r->q1;
  q_out->q2 = q->q2 + r->q2;
  q_out->q3 = q->q3 + r->q3;
  return q_out;
}

// q_out = q - r.
const Quat *QuatSub(const Quat *q, const Quat *r, Quat *q_out) {
  assert(q != NULL && r != NULL && q_out != NULL);

  q_out->q0 = q->q0 - r->q0;
  q_out->q1 = q->q1 - r->q1;
  q_out->q2 = q->q2 - r->q2;
  q_out->q3 = q->q3 - r->q3;
  return q_out;
}

// q_out = scale * q_in.
const Quat *QuatScale(const Quat *q_in, double scale, Quat *q_out) {
  assert(q_in != NULL && q_out != NULL);

  q_out->q0 = scale * q_in->q0;
  q_out->q1 = scale * q_in->q1;
  q_out->q2 = scale * q_in->q2;
  q_out->q3 = scale * q_in->q3;
  return q_out;
}

// q_out = cq*q + cr*r.
const Quat *QuatLinComb(double cq, const Quat *q, double cr, const Quat *r,
                        Quat *q_out) {
  assert(q != NULL && r != NULL && q_out != NULL);

  q_out->q0 = cq * q->q0 + cr * r->q0;
  q_out->q1 = cq * q->q1 + cr * r->q1;
  q_out->q2 = cq * q->q2 + cr * r->q2;
  q_out->q3 = cq * q->q3 + cr * r->q3;
  return q_out;
}

// q_out = c0*q0 + c1*q1 + c2*q2.
const Quat *QuatLinComb3(double c0, const Quat *q0, double c1, const Quat *q1,
                         double c2, const Quat *q2, Quat *q_out) {
  assert(q0 != NULL && q1 != NULL && q2 != NULL && q_out != NULL);

  q_out->q0 = c0 * q0->q0 + c1 * q1->q0 + c2 * q2->q0;
  q_out->q1 = c0 * q0->q1 + c1 * q1->q1 + c2 * q2->q1;
  q_out->q2 = c0 * q0->q2 + c1 * q1->q2 + c2 * q2->q2;
  q_out->q3 = c0 * q0->q3 + c1 * q1->q3 + c2 * q2->q3;
  return q_out;
}

// q_out = q_in'.
const Quat *QuatConj(const Quat *q_in, Quat *q_out) {
  assert(q_in != NULL && q_out != NULL);

  q_out->q0 = q_in->q0;
  q_out->q1 = -q_in->q1;
  q_out->q2 = -q_in->q2;
  q_out->q3 = -q_in->q3;
  return q_out;
}

// Takes the inverse of a quaternion: q_out = q_in' / |q_in|^2.  Uses
// QuatNormalize to avoid underflow/overflow and to gracefully handle
// cases with Infs, NaNs, etc.  To avoid Infs and NaNs, we define the
// inverse of the zero quaternion to be [DBL_MAX, 0, 0, 0].
const Quat *QuatInv(const Quat *q_in, Quat *q_out) {
  assert(q_in != NULL && q_out != NULL);

  double quat_mod = QuatMod(q_in);
  QuatNormalize(q_in, q_out);
  if (quat_mod > 0.0) {
    QuatScale(QuatConj(q_out, q_out), 1.0 / quat_mod, q_out);
  } else {
    assert(0);
    // We define the inverse of the zero quaternion to be the
    // following:
    q_out->q0 = DBL_MAX;
    q_out->q1 = 0.0;
    q_out->q2 = 0.0;
    q_out->q3 = 0.0;
  }
  return q_out;
}

// Multiplies two quaternions: q_out = q_in * r_in.  Multiplication of
// quaternions, which are written in the form q = q0 + q1*i + q2*j +
// q3*k, is defined by the following relations between i, j, and k:
//
//   i^2 = j^2 = k^2 = i*j*k = -1
//
const Quat *QuatMultiply(const Quat *q_in, const Quat *r_in, Quat *q_out) {
  assert(q_in != NULL && r_in != NULL && q_out != NULL);

  Quat q = *q_in, r = *r_in;
  q_out->q0 = q.q0 * r.q0 - q.q1 * r.q1 - q.q2 * r.q2 - q.q3 * r.q3;
  q_out->q1 = q.q0 * r.q1 + q.q1 * r.q0 + q.q2 * r.q3 - q.q3 * r.q2;
  q_out->q2 = q.q0 * r.q2 + q.q2 * r.q0 + q.q3 * r.q1 - q.q1 * r.q3;
  q_out->q3 = q.q0 * r.q3 + q.q3 * r.q0 + q.q1 * r.q2 - q.q2 * r.q1;

  return q_out;
}

// q_out = (q2)^-1 * q1.
const Quat *QuatDivide(const Quat *q1, const Quat *q2, Quat *q_out) {
  assert(q1 != NULL && q2 != NULL && q_out != NULL);
  Quat q_tmp;
  return QuatMultiply(QuatInv(q2, &q_tmp), q1, q_out);
}

// Returns the quaternion component with the maximum absolute value.
double QuatMaxAbs(const Quat *q) {
  double q_max = 0.0;
  if (fabs(q->q0) > q_max) q_max = fabs(q->q0);
  if (fabs(q->q1) > q_max) q_max = fabs(q->q1);
  if (fabs(q->q2) > q_max) q_max = fabs(q->q2);
  if (fabs(q->q3) > q_max) q_max = fabs(q->q3);
  return q_max;
}

// Computes the dot product of two quaternions regarded as four element vectors.
//
// This is useful when q and p are unit quaternions, due to the formula:
//
//   1 - QuatDot(q, p)^2 = (1 - cos(theta)) / 2.0
//
// where theta is the angle of rotation for QuatDivide(q, p).
double QuatDot(const Quat *q, const Quat *p) {
  assert(q != NULL && p != NULL);
  return q->q0 * p->q0 + q->q1 * p->q1 + q->q2 * p->q2 + q->q3 * p->q3;
}

// Returns the square of the modulus of the quaternion.  This was
// formally called QuatNorm following MATLAB's convention; however it
// was renamed to avoid confusion with the Euclidean norm.
double QuatModSquared(const Quat *q) {
  assert(q != NULL);
  return QuatDot(q, q);
}

// Returns the modulus of the quaternion.
double QuatMod(const Quat *q) {
  assert(q != NULL);
  return hypot(hypot(q->q0, q->q1), hypot(q->q2, q->q3));
}

// Tests if a quaternion has any NaN components.
bool QuatHasNaN(const Quat *q) {
  assert(q != NULL);
  return isnan(q->q0) || isnan(q->q1) || isnan(q->q2) || isnan(q->q3);
}

// Scales a quaternion to have unit norm: q_out = q_in / |q_in|.  This
// handles edge cases such as NaNs, Infs, and zero quaternions
// gracefully, such that the output quaternion should always be a unit
// quaternion.  For example:
//
//             [0, 0, 0, 0] --> [1, 0, 0, 0]
//   DBL_MAX * [1, 1, 1, 1] --> [0.5, 0.5, 0.5, 0.5]
//           [0, Inf, 0, 0] --> [0, 1, 0, 0]
//         [0, NaN, NaN, 0] --> [1, 0, 0, 0]
//
const Quat *QuatNormalize(const Quat *q_in, Quat *q_out) {
  assert(q_in != NULL && q_out != NULL);

  double quat_mod = QuatMod(q_in);

  if (QuatHasNaN(q_in) || quat_mod == 0.0) {
    // Return the identity quaternion for the unnormalizable cases
    // (e.g. zero quaternion or a quaternion with NaNs).
    *q_out = kQuatIdentity;

  } else if (!isfinite(quat_mod) || !isfinite(1.0 / quat_mod)) {
    // If quat_mod overflows, scale everything by the inverse of the
    // largest element.
    double q_max = QuatMaxAbs(q_in);
    if (!isfinite(q_max)) {
      // Convert all infinities to +/- one and all non-infinities to zero.
      q_out->q0 = isfinite(q_in->q0) ? 0.0 : Sign(q_in->q0);
      q_out->q1 = isfinite(q_in->q1) ? 0.0 : Sign(q_in->q1);
      q_out->q2 = isfinite(q_in->q2) ? 0.0 : Sign(q_in->q2);
      q_out->q3 = isfinite(q_in->q3) ? 0.0 : Sign(q_in->q3);
    } else {
      QuatScale(q_in, 1.0 / fmax(q_max, DBL_MIN), q_out);
    }
    quat_mod = QuatMod(q_out);
    QuatScale(q_out, 1.0 / quat_mod, q_out);

  } else {
    // The typical case.  The quaternion components are finite, and
    // both quat_mod and the reciprocal of quat_mod are finite.
    QuatScale(q_in, 1.0 / quat_mod, q_out);
  }

  return q_out;
}

// Rotates a vector by a rotation described by a quaternion.  For a
// unit quaternion, q_hat, this rotation is equivalent to a rotation
// by an angle theta about the vector u
//
//   q_hat = [cos(theta/2),
//            sin(theta/2) u_x,
//            sin(theta/2) u_y,
//            sin(theta/2) u_z]
//
// Mathematically, the vector acts like a "pure" quaternion with zero
// real component and then the rotation is a sequence of quaternion
// multiplications:
//
//   v_out = q^-1 * v_in * q
//
// Practically, it is easiest to first convert the quaternion to a DCM
// and use matrix multiplication.
const Vec3 *QuatRotate(const Quat *q, const Vec3 *v_in, Vec3 *v_out) {
  assert(q != NULL && v_in != NULL && v_out != NULL);
  // TODO: The relatively loose limit of 1e-3 on the
  // difference between |q| and 1.0 is due to how we use the
  // integrated quaternion in the simulator.  We should be able to
  // tighten this limit in the future if we are more careful with how
  // we use quaternions in the simulator.
  assert(fabs(QuatMod(q) - 1.0) < 1e-3);

  Mat3 dcm;
  QuatToDcm(q, &dcm);
  return Mat3Vec3Mult(&dcm, v_in, v_out);
}

// Converts a quaternion representation of a rotation to a direction
// cosine matrix.
//
//         | q0^2+q1^2-q2^2-q3^2   2*(q1*q2 + q0*q3)     2*(q1*q3 - q0*q2) |
//   DCM = | 2*(q1*q2 - q0*q3)    q0^2-q1^2+q2^2-q3^2    2*(q2*q3 + q0*q1) |
//         | 2*(q1*q3 + q0*q2)     2*(q2*q3 - q0*q1)   q0^2-q1^2-q2^2+q3^2 |
//
const Mat3 *QuatToDcm(const Quat *q, Mat3 *dcm) {
  assert(q != NULL && dcm != NULL);
  assert(fabs(QuatMod(q) - 1.0) < 1e-3);

  Quat q_norm;
  QuatNormalize(q, &q_norm);

  dcm->d[0][0] = q_norm.q0 * q_norm.q0 + q_norm.q1 * q_norm.q1 -
                 q_norm.q2 * q_norm.q2 - q_norm.q3 * q_norm.q3;
  dcm->d[0][1] = 2.0 * (q_norm.q1 * q_norm.q2 + q_norm.q0 * q_norm.q3);
  dcm->d[0][2] = 2.0 * (q_norm.q1 * q_norm.q3 - q_norm.q0 * q_norm.q2);
  dcm->d[1][0] = 2.0 * (q_norm.q1 * q_norm.q2 - q_norm.q0 * q_norm.q3);
  dcm->d[1][1] = q_norm.q0 * q_norm.q0 - q_norm.q1 * q_norm.q1 +
                 q_norm.q2 * q_norm.q2 - q_norm.q3 * q_norm.q3;
  dcm->d[1][2] = 2.0 * (q_norm.q2 * q_norm.q3 + q_norm.q0 * q_norm.q1);
  dcm->d[2][0] = 2.0 * (q_norm.q1 * q_norm.q3 + q_norm.q0 * q_norm.q2);
  dcm->d[2][1] = 2.0 * (q_norm.q2 * q_norm.q3 - q_norm.q0 * q_norm.q1);
  dcm->d[2][2] = q_norm.q0 * q_norm.q0 - q_norm.q1 * q_norm.q1 -
                 q_norm.q2 * q_norm.q2 + q_norm.q3 * q_norm.q3;

  return dcm;
}

// Converts a direction cosine matrix representation of a rotation to
// a quaternion representation.  Based on the equation for the DCM
// used in QuatToDcm, we find the following formulas for the
// magnitudes of the individual components of the quaternion:
//
//   dcm_00 + dcm_11 + dcm_22 = 4*q0^2 - 1
//   dcm_00 - dcm_11 - dcm_22 = 4*q1^2 - 1
//   dcm_11 - dcm_00 - dcm_22 = 4*q2^2 - 1
//   dcm_22 - dcm_00 - dcm_11 = 4*q3^2 - 1
//
// We choose the largest quaternion component to avoid division by a
// small number and to avoid catastrophic cancellation in the formulas
// below.  We then use the following additional relationships to
// determine the other components with sign information.
//
//   dcm_01 - dcm_10 = 4 * q0 * q3
//   dcm_01 + dcm_10 = 4 * q1 * q2
//   dcm_20 - dcm_02 = 4 * q0 * q2
//   dcm_20 + dcm_20 = 4 * q1 * q3
//   dcm_12 - dcm_21 = 4 * q0 * q1
//   dcm_12 + dcm_21 = 4 * q2 * q3
//
const Quat *DcmToQuat(const Mat3 *dcm, Quat *q) {
  assert(dcm != NULL && q != NULL);
  assert(Mat3IsSpecialOrthogonal(dcm, DBL_TOL));

  double scale, tr = Mat3Trace(dcm);
  if (tr > dcm->d[0][0] && tr > dcm->d[1][1] && tr > dcm->d[2][2]) {
    scale = Sqrt(tr + 1.0);
    q->q0 = 0.5 * scale;
    if (scale > 0.0) scale = 0.5 / scale;
    q->q1 = (dcm->d[1][2] - dcm->d[2][1]) * scale;
    q->q2 = (dcm->d[2][0] - dcm->d[0][2]) * scale;
    q->q3 = (dcm->d[0][1] - dcm->d[1][0]) * scale;

  } else if (dcm->d[0][0] > dcm->d[1][1] && dcm->d[0][0] > dcm->d[2][2]) {
    scale = Sqrt(dcm->d[0][0] - dcm->d[1][1] - dcm->d[2][2] + 1.0);
    q->q1 = 0.5 * scale;
    if (scale > 0.0) scale = 0.5 / scale;
    q->q0 = (dcm->d[1][2] - dcm->d[2][1]) * scale;
    q->q2 = (dcm->d[0][1] + dcm->d[1][0]) * scale;
    q->q3 = (dcm->d[2][0] + dcm->d[0][2]) * scale;

  } else if (dcm->d[1][1] > dcm->d[2][2]) {
    scale = Sqrt(dcm->d[1][1] - dcm->d[0][0] - dcm->d[2][2] + 1.0);
    q->q2 = 0.5 * scale;
    if (scale > 0.0) scale = 0.5 / scale;
    q->q0 = (dcm->d[2][0] - dcm->d[0][2]) * scale;
    q->q1 = (dcm->d[0][1] + dcm->d[1][0]) * scale;
    q->q3 = (dcm->d[1][2] + dcm->d[2][1]) * scale;

  } else {
    scale = Sqrt(dcm->d[2][2] - dcm->d[0][0] - dcm->d[1][1] + 1.0);
    q->q3 = 0.5 * scale;
    if (scale > 0.0) scale = 0.5 / scale;
    q->q0 = (dcm->d[0][1] - dcm->d[1][0]) * scale;
    q->q1 = (dcm->d[2][0] + dcm->d[0][2]) * scale;
    q->q2 = (dcm->d[1][2] + dcm->d[2][1]) * scale;
  }

  return QuatNormalize(q, q);
}

// Converts a quaternion representation of a rotation to an axis-angle
// representation.  This will always return the smallest magnitude
// angle (-pi, pi] that represents the rotation.  Also, of the two
// equivalent opposite sign axis-angle pairs, this function returns
// the axis that has the same signs as the [q1, q2, q3] vector portion
// of the quaternion.  Finally, the identity quaternion is defined to
// have a zero length axis vector.
double QuatToAxisAngle(const Quat *q, Vec3 *axis) {
  assert(q != NULL && axis != NULL);
  assert(fabs(QuatMod(q) - 1.0) < 1e-3);

  Quat q_norm;
  QuatNormalize(q, &q_norm);
  axis->x = q_norm.q1;
  axis->y = q_norm.q2;
  axis->z = q_norm.q3;

  double angle, sin_half_angle = Vec3Norm(axis);
  if (fabs(sin_half_angle) > 0.0) {
    Vec3Scale(axis, 1.0 / sin_half_angle, axis);
    // We always want the angle to be in the first quadrant, and we
    // also want the numerical stability of atan2.
    if (q_norm.q0 < 0.0) sin_half_angle *= -1.0;
    angle = 2.0 * atan2(sin_half_angle, fabs(q_norm.q0));
  } else {
    *axis = kVec3Zero;
    angle = 0.0;
  }
  return angle;
}

// Converts a rotation described by a rotation axis and an angle to a
// quaternion.
const Quat *AxisAngleToQuat(const Vec3 *axis, double angle, Quat *q) {
  assert(q != NULL && axis != NULL);
  assert(isfinite(angle));

  double axis_len = Vec3Norm(axis);
  // Only allow small axis lengths if the angle is equally small
  // (e.g. as the identity rotation would appear from AxisToQuat).
  assert(axis_len > DBL_EPSILON || angle <= DBL_EPSILON);

  if (axis_len > 0.0) {
    Vec3 tmp;
    Vec3Scale(axis, sin(angle / 2.0) / axis_len, &tmp);
    q->q0 = cos(angle / 2.0);
    q->q1 = tmp.x;
    q->q2 = tmp.y;
    q->q3 = tmp.z;
  } else {
    *q = kQuatIdentity;
  }

  return q;
}

// Converts a quaternion representation of a rotation to an Euler
// vector representation.
const Vec3 *QuatToAxis(const Quat *q, Vec3 *axis) {
  assert(q != NULL && axis != NULL);
  double angle = QuatToAxisAngle(q, axis);
  return Vec3Scale(axis, angle, axis);
}

// Converts a rotation described by a rotation axis, where the length
// of the axis is the rotation angle, to a quaternion (i.e. same as
// AxisAngleToQuat except that the angle of rotation is embedded in
// the magnitude of the axis vector.).
const Quat *AxisToQuat(const Vec3 *axis, Quat *q) {
  assert(axis != NULL && q != NULL);
  assert(Vec3Norm(axis) <= PI);
  return AxisAngleToQuat(axis, Vec3Norm(axis), q);
}

// Converts a quaternion representation of a rotation to an Euler
// angle representation.
//
// Args:
//   q_in: Input quaternion.
//   order: Order of the rotations about different axes (e.g. kRotationOrderZyx
//       is the standard aerospace ordering).
//   r1, r2, r3: Pointers to the first, second, and third Euler angles
//       as described by the order argument.
void QuatToAngle(const Quat *q_in, RotationOrder order, double *r1, double *r2,
                 double *r3) {
  assert(q_in != NULL && r1 != NULL && r2 != NULL && r3 != NULL);
  assert(r1 != r2 && r1 != r3 && r2 != r3);
  assert(fabs(QuatMod(q_in) - 1.0) < 1e-3);

  Quat q;
  QuatNormalize(q_in, &q);
  switch (order) {
    case kRotationOrderZyx:
      *r1 = atan2(2.0 * (q.q1 * q.q2 + q.q0 * q.q3),
                  q.q0 * q.q0 + q.q1 * q.q1 - q.q2 * q.q2 - q.q3 * q.q3);
      *r2 = Asin(-2.0 * (q.q1 * q.q3 - q.q0 * q.q2));
      *r3 = atan2(2.0 * (q.q2 * q.q3 + q.q0 * q.q1),
                  q.q0 * q.q0 - q.q1 * q.q1 - q.q2 * q.q2 + q.q3 * q.q3);
      break;

    case kRotationOrderXyz:
      *r1 = atan2(-2.0 * (q.q2 * q.q3 - q.q0 * q.q1),
                  q.q0 * q.q0 - q.q1 * q.q1 - q.q2 * q.q2 + q.q3 * q.q3);
      *r2 = Asin(2.0 * (q.q1 * q.q3 + q.q0 * q.q2));
      *r3 = atan2(-2.0 * (q.q1 * q.q2 - q.q0 * q.q3),
                  q.q0 * q.q0 + q.q1 * q.q1 - q.q2 * q.q2 - q.q3 * q.q3);
      break;

    case kRotationOrderXyx:
      *r1 = atan2(q.q1 * q.q2 + q.q0 * q.q3, -q.q1 * q.q3 + q.q0 * q.q2);
      *r2 = Acos(q.q0 * q.q0 + q.q1 * q.q1 - q.q2 * q.q2 - q.q3 * q.q3);
      *r3 = atan2(q.q1 * q.q2 - q.q0 * q.q3, q.q1 * q.q3 + q.q0 * q.q2);
      break;

    case kRotationOrderXzy:
      *r1 = atan2(2.0 * (q.q2 * q.q3 + q.q0 * q.q1),
                  q.q0 * q.q0 - q.q1 * q.q1 + q.q2 * q.q2 - q.q3 * q.q3);
      *r2 = Asin(-2.0 * (q.q1 * q.q2 - q.q0 * q.q3));
      *r3 = atan2(2.0 * (q.q1 * q.q3 + q.q0 * q.q2),
                  q.q0 * q.q0 + q.q1 * q.q1 - q.q2 * q.q2 - q.q3 * q.q3);
      break;

    case kRotationOrderXzx:
      *r1 = atan2(q.q1 * q.q3 - q.q0 * q.q2, q.q1 * q.q2 + q.q0 * q.q3);
      *r2 = Acos(q.q0 * q.q0 + q.q1 * q.q1 - q.q2 * q.q2 - q.q3 * q.q3);
      *r3 = atan2(q.q1 * q.q3 + q.q0 * q.q2, -(q.q1 * q.q2 - q.q0 * q.q3));
      break;

    case kRotationOrderYxz:
      *r1 = atan2(2.0 * (q.q1 * q.q3 + q.q0 * q.q2),
                  q.q0 * q.q0 - q.q1 * q.q1 - q.q2 * q.q2 + q.q3 * q.q3);
      *r2 = Asin(-2.0 * (q.q2 * q.q3 - q.q0 * q.q1));
      *r3 = atan2(2.0 * (q.q1 * q.q2 + q.q0 * q.q3),
                  q.q0 * q.q0 - q.q1 * q.q1 + q.q2 * q.q2 - q.q3 * q.q3);
      break;

    case kRotationOrderYxy:
      *r1 = atan2(q.q1 * q.q2 - q.q0 * q.q3, q.q2 * q.q3 + q.q0 * q.q1);
      *r2 = Acos(q.q0 * q.q0 - q.q1 * q.q1 + q.q2 * q.q2 - q.q3 * q.q3);
      *r3 = atan2(q.q1 * q.q2 + q.q0 * q.q3, -q.q2 * q.q3 + q.q0 * q.q1);
      break;

    case kRotationOrderYzx:
      *r1 = atan2(-2.0 * (q.q1 * q.q3 - q.q0 * q.q2),
                  q.q0 * q.q0 + q.q1 * q.q1 - q.q2 * q.q2 - q.q3 * q.q3);
      *r2 = Asin(2.0 * (q.q1 * q.q2 + q.q0 * q.q3));
      *r3 = atan2(-2.0 * (q.q2 * q.q3 - q.q0 * q.q1),
                  q.q0 * q.q0 - q.q1 * q.q1 + q.q2 * q.q2 - q.q3 * q.q3);
      break;

    case kRotationOrderYzy:
      *r1 = atan2(q.q2 * q.q3 + q.q0 * q.q1, -q.q1 * q.q2 + q.q0 * q.q3);
      *r2 = Acos(q.q0 * q.q0 - q.q1 * q.q1 + q.q2 * q.q2 - q.q3 * q.q3);
      *r3 = atan2(q.q2 * q.q3 - q.q0 * q.q1, q.q1 * q.q2 + q.q0 * q.q3);
      break;

    case kRotationOrderZyz:
      *r1 = atan2(q.q2 * q.q3 - q.q0 * q.q1, q.q1 * q.q3 + q.q0 * q.q2);
      *r2 = Acos(q.q0 * q.q0 - q.q1 * q.q1 - q.q2 * q.q2 + q.q3 * q.q3);
      *r3 = atan2(q.q2 * q.q3 + q.q0 * q.q1, -q.q1 * q.q3 + q.q0 * q.q2);
      break;

    case kRotationOrderZxy:
      *r1 = atan2(-2.0 * (q.q1 * q.q2 - q.q0 * q.q3),
                  q.q0 * q.q0 - q.q1 * q.q1 + q.q2 * q.q2 - q.q3 * q.q3);
      *r2 = Asin(2.0 * (q.q2 * q.q3 + q.q0 * q.q1));
      *r3 = atan2(-2.0 * (q.q1 * q.q3 - q.q0 * q.q2),
                  q.q0 * q.q0 - q.q1 * q.q1 - q.q2 * q.q2 + q.q3 * q.q3);
      break;

    case kRotationOrderZxz:
      *r1 = atan2(q.q1 * q.q3 + q.q0 * q.q2, -q.q2 * q.q3 + q.q0 * q.q1);
      *r2 = Acos(q.q0 * q.q0 - q.q1 * q.q1 - q.q2 * q.q2 + q.q3 * q.q3);
      *r3 = atan2(q.q1 * q.q3 - q.q0 * q.q2, q.q2 * q.q3 + q.q0 * q.q1);
      break;

    case kRotationOrderForceSigned:
    case kNumRotationOrders:
    default:
      assert(0);  // Not implemented!
      break;
  }
}

// Converts an Euler angle representation of a rotation to a quaternion
// representation.
//
// Args:
//   r1, r2, r3: First, second, and third Euler angles as described by
//       the order argument.
//   order: Order of the rotations about different axes (e.g. kRotationOrderZyx
//       is the standard aerospace ordering).
//   q_out: Output quaternion.
void AngleToQuat(double r1, double r2, double r3, RotationOrder order,
                 Quat *q_out) {
  Mat3 dcm;
  AngleToDcm(r1, r2, r3, order, &dcm);
  DcmToQuat(&dcm, q_out);
}

// Convert a unit quaternion to a modified Rodrigues parameter.
void QuatToMrp(const Quat *q, Vec3 *mrp) {
  // Prevent a possible divide by zero condition when the quaternion's real
  // component is -1 using knowledge that -q == q.
  if (q->q0 < 0.0) {
    const double s = 1.0 - q->q0;  // Always >= 1.
    mrp->x = -q->q1 / s;
    mrp->y = -q->q2 / s;
    mrp->z = -q->q3 / s;
  } else {
    const double s = 1.0 + q->q0;  // Always >= 1.
    mrp->x = q->q1 / s;
    mrp->y = q->q2 / s;
    mrp->z = q->q3 / s;
  }
}

// Convert a modified Rodrigues parameter to a unit quaternion.
void MrpToQuat(const Vec3 *mrp, Quat *q) {
  const double mrp2 = Vec3Dot(mrp, mrp);
  const double s = 1.0 + mrp2;  // Always >= 1.
  q->q0 = (1.0 - mrp2) / s;
  q->q1 = 2.0 * mrp->x / s;
  q->q2 = 2.0 * mrp->y / s;
  q->q3 = 2.0 * mrp->z / s;
}

// Compute the transformation matrix that rotates vector "a" to be
// parallel to vector "b".
//
// Input arguments:
//
// a:       The first vector.
// b:       The second vector, representing the direction to which we
//          want to rotate the first vector.
//
// Output arguments:
//
// dcm_a2b: The matrix that rotates vector a to point in the direction
//          of vector b.
void Vec3Vec3ToDcm(const Vec3 *a, const Vec3 *b, Mat3 *dcm_a2b) {
  assert(a != NULL);
  assert(b != NULL);
  assert(dcm_a2b != NULL);

  Vec3 axis;
  const double angle = Vec3ToAxisAngle(b, a, &axis);

  Quat q;
  AxisAngleToQuat(&axis, angle, &q);

  QuatToDcm(&q, dcm_a2b);
}

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

#include "common/c_math/geometry.h"

#include <math.h>

#include "common/c_math/mat3.h"
#include "common/c_math/util.h"
#include "common/c_math/vec2.h"
#include "common/c_math/vec3.h"

// Converts a direction cosine matrix (DCM) to a sequence of Euler
// angles (with a specified rotation order).
void DcmToAngle(const Mat3 *dcm, RotationOrder order, double *r1, double *r2,
                double *r3) {
  switch (order) {
    case kRotationOrderForceSigned:
    case kNumRotationOrders:
      assert(false);  // Fall-through intentional.
    case kRotationOrderZyx:
    default:
      *r1 = atan2(dcm->d[0][1], dcm->d[0][0]);
      *r2 = Asin(-dcm->d[0][2]);
      *r3 = atan2(dcm->d[1][2], dcm->d[2][2]);
      break;

    case kRotationOrderXyz:
      *r1 = atan2(-dcm->d[2][1], dcm->d[2][2]);
      *r2 = Asin(dcm->d[2][0]);
      *r3 = atan2(-dcm->d[1][0], dcm->d[0][0]);
      break;

    case kRotationOrderXyx:
      *r1 = atan2(dcm->d[0][1], -dcm->d[0][2]);
      *r2 = Acos(dcm->d[0][0]);
      *r3 = atan2(dcm->d[1][0], dcm->d[2][0]);
      break;

    case kRotationOrderXzy:
      *r1 = atan2(dcm->d[1][2], dcm->d[1][1]);
      *r2 = Asin(-dcm->d[1][0]);
      *r3 = atan2(dcm->d[2][0], dcm->d[0][0]);
      break;

    case kRotationOrderXzx:
      *r1 = atan2(dcm->d[0][2], dcm->d[0][1]);
      *r2 = Acos(dcm->d[0][0]);
      *r3 = atan2(dcm->d[2][0], -dcm->d[1][0]);
      break;

    case kRotationOrderYxz:
      *r1 = atan2(dcm->d[2][0], dcm->d[2][2]);
      *r2 = Asin(-dcm->d[2][1]);
      *r3 = atan2(dcm->d[0][1], dcm->d[1][1]);
      break;

    case kRotationOrderYxy:
      *r1 = atan2(dcm->d[1][0], dcm->d[1][2]);
      *r2 = Acos(dcm->d[1][1]);
      *r3 = atan2(dcm->d[0][1], -dcm->d[2][1]);
      break;

    case kRotationOrderYzx:
      *r1 = atan2(-dcm->d[0][2], dcm->d[0][0]);
      *r2 = Asin(dcm->d[0][1]);
      *r3 = atan2(-dcm->d[2][1], dcm->d[1][1]);
      break;

    case kRotationOrderYzy:
      *r1 = atan2(dcm->d[1][2], -dcm->d[1][0]);
      *r2 = Acos(dcm->d[1][1]);
      *r3 = atan2(dcm->d[2][1], dcm->d[0][1]);
      break;

    case kRotationOrderZyz:
      *r1 = atan2(dcm->d[2][1], dcm->d[2][0]);
      *r2 = Acos(dcm->d[2][2]);
      *r3 = atan2(dcm->d[1][2], -dcm->d[0][2]);
      break;

    case kRotationOrderZxy:
      *r1 = atan2(-dcm->d[1][0], dcm->d[1][1]);
      *r2 = Asin(dcm->d[1][2]);
      *r3 = atan2(-dcm->d[0][2], dcm->d[2][2]);
      break;

    case kRotationOrderZxz:
      *r1 = atan2(dcm->d[2][0], -dcm->d[2][1]);
      *r2 = Acos(dcm->d[2][2]);
      *r3 = atan2(dcm->d[0][2], dcm->d[1][2]);
      break;
  }
}

// Converts a sequence of Euler angles (with a specified rotation
// order) to a direction cosine matrix (DCM).
const Mat3 *AngleToDcm(double r1, double r2, double r3, RotationOrder order,
                       Mat3 *dcm) {
  // TODO: It would be nice if we could use the sincos()
  // function here, which is available as a GNU extension. I believe
  // it corresponds to an instruction in the underlying assembly
  // language.
  const double sin_r1 = sin(r1);
  const double cos_r1 = cos(r1);
  const double sin_r2 = sin(r2);
  const double cos_r2 = cos(r2);
  const double sin_r3 = sin(r3);
  const double cos_r3 = cos(r3);

  switch (order) {
    case kRotationOrderXyz:
      dcm->d[0][0] = cos_r2 * cos_r3;
      dcm->d[0][1] = sin_r1 * sin_r2 * cos_r3 + cos_r1 * sin_r3;
      dcm->d[0][2] = sin_r1 * sin_r3 - cos_r1 * sin_r2 * cos_r3;
      dcm->d[1][0] = -cos_r2 * sin_r3;
      dcm->d[1][1] = cos_r1 * cos_r3 - sin_r1 * sin_r2 * sin_r3;
      dcm->d[1][2] = cos_r1 * sin_r2 * sin_r3 + sin_r1 * cos_r3;
      dcm->d[2][0] = sin_r2;
      dcm->d[2][1] = -sin_r1 * cos_r2;
      dcm->d[2][2] = cos_r1 * cos_r2;
      break;

    case kRotationOrderXyx:
      dcm->d[0][0] = cos_r2;
      dcm->d[0][1] = sin_r1 * sin_r2;
      dcm->d[0][2] = -cos_r1 * sin_r2;
      dcm->d[1][0] = sin_r2 * sin_r3;
      dcm->d[1][1] = cos_r1 * cos_r3 - sin_r1 * cos_r2 * sin_r3;
      dcm->d[1][2] = cos_r1 * cos_r2 * sin_r3 + sin_r1 * cos_r3;
      dcm->d[2][0] = sin_r2 * cos_r3;
      dcm->d[2][1] = -sin_r1 * cos_r2 * cos_r3 - cos_r1 * sin_r3;
      dcm->d[2][2] = cos_r1 * cos_r2 * cos_r3 - sin_r1 * sin_r3;
      break;

    case kRotationOrderXzy:
      dcm->d[0][0] = cos_r2 * cos_r3;
      dcm->d[0][1] = cos_r1 * sin_r2 * cos_r3 + sin_r1 * sin_r3;
      dcm->d[0][2] = sin_r1 * sin_r2 * cos_r3 - cos_r1 * sin_r3;
      dcm->d[1][0] = -sin_r2;
      dcm->d[1][1] = cos_r1 * cos_r2;
      dcm->d[1][2] = sin_r1 * cos_r2;
      dcm->d[2][0] = cos_r2 * sin_r3;
      dcm->d[2][1] = cos_r1 * sin_r2 * sin_r3 - sin_r1 * cos_r3;
      dcm->d[2][2] = sin_r1 * sin_r2 * sin_r3 + cos_r1 * cos_r3;
      break;

    case kRotationOrderXzx:
      dcm->d[0][0] = cos_r2;
      dcm->d[0][1] = cos_r1 * sin_r2;
      dcm->d[0][2] = sin_r1 * sin_r2;
      dcm->d[1][0] = -sin_r2 * cos_r3;
      dcm->d[1][1] = cos_r1 * cos_r2 * cos_r3 - sin_r1 * sin_r3;
      dcm->d[1][2] = sin_r1 * cos_r2 * cos_r3 + cos_r1 * sin_r3;
      dcm->d[2][0] = sin_r2 * sin_r3;
      dcm->d[2][1] = -cos_r1 * cos_r2 * sin_r3 - sin_r1 * cos_r3;
      dcm->d[2][2] = cos_r1 * cos_r3 - sin_r1 * cos_r2 * sin_r3;
      break;

    case kRotationOrderYxz:
      dcm->d[0][0] = cos_r1 * cos_r3 + sin_r1 * sin_r2 * sin_r3;
      dcm->d[0][1] = cos_r2 * sin_r3;
      dcm->d[0][2] = cos_r1 * sin_r2 * sin_r3 - sin_r1 * cos_r3;
      dcm->d[1][0] = sin_r1 * sin_r2 * cos_r3 - cos_r1 * sin_r3;
      dcm->d[1][1] = cos_r2 * cos_r3;
      dcm->d[1][2] = cos_r1 * sin_r2 * cos_r3 + sin_r1 * sin_r3;
      dcm->d[2][0] = sin_r1 * cos_r2;
      dcm->d[2][1] = -sin_r2;
      dcm->d[2][2] = cos_r1 * cos_r2;
      break;

    case kRotationOrderYxy:
      dcm->d[0][0] = cos_r1 * cos_r3 - sin_r1 * cos_r2 * sin_r3;
      dcm->d[0][1] = sin_r2 * sin_r3;
      dcm->d[0][2] = -cos_r1 * cos_r2 * sin_r3 - sin_r1 * cos_r3;
      dcm->d[1][0] = sin_r1 * sin_r2;
      dcm->d[1][1] = cos_r2;
      dcm->d[1][2] = cos_r1 * sin_r2;
      dcm->d[2][0] = sin_r1 * cos_r2 * cos_r3 + cos_r1 * sin_r3;
      dcm->d[2][1] = -sin_r2 * cos_r3;
      dcm->d[2][2] = cos_r1 * cos_r2 * cos_r3 - sin_r1 * sin_r3;
      break;

    case kRotationOrderYzx:
      dcm->d[0][0] = cos_r1 * cos_r2;
      dcm->d[0][1] = sin_r2;
      dcm->d[0][2] = -sin_r1 * cos_r2;
      dcm->d[1][0] = sin_r1 * sin_r3 - cos_r1 * sin_r2 * cos_r3;
      dcm->d[1][1] = cos_r2 * cos_r3;
      dcm->d[1][2] = cos_r1 * sin_r3 + sin_r1 * sin_r2 * cos_r3;
      dcm->d[2][0] = cos_r1 * sin_r2 * sin_r3 + sin_r1 * cos_r3;
      dcm->d[2][1] = -cos_r2 * sin_r3;
      dcm->d[2][2] = cos_r1 * cos_r3 - sin_r1 * sin_r2 * sin_r3;
      break;

    case kRotationOrderYzy:
      dcm->d[0][0] = cos_r1 * cos_r2 * cos_r3 - sin_r1 * sin_r3;
      dcm->d[0][1] = sin_r2 * cos_r3;
      dcm->d[0][2] = -sin_r1 * cos_r2 * cos_r3 - cos_r1 * sin_r3;
      dcm->d[1][0] = -cos_r1 * sin_r2;
      dcm->d[1][1] = cos_r2;
      dcm->d[1][2] = sin_r1 * sin_r2;
      dcm->d[2][0] = cos_r1 * cos_r2 * sin_r3 + sin_r1 * cos_r3;
      dcm->d[2][1] = sin_r2 * sin_r3;
      dcm->d[2][2] = -sin_r1 * cos_r2 * sin_r3 + cos_r1 * cos_r3;
      break;

    case kRotationOrderForceSigned:
    case kNumRotationOrders:
    default:
      assert(false);  // Fall-through intentional.
    case kRotationOrderZyx:
      dcm->d[0][0] = cos_r1 * cos_r2;
      dcm->d[0][1] = sin_r1 * cos_r2;
      dcm->d[0][2] = -sin_r2;
      dcm->d[1][0] = cos_r1 * sin_r2 * sin_r3 - sin_r1 * cos_r3;
      dcm->d[1][1] = sin_r1 * sin_r2 * sin_r3 + cos_r1 * cos_r3;
      dcm->d[1][2] = cos_r2 * sin_r3;
      dcm->d[2][0] = cos_r1 * sin_r2 * cos_r3 + sin_r1 * sin_r3;
      dcm->d[2][1] = sin_r1 * sin_r2 * cos_r3 - cos_r1 * sin_r3;
      dcm->d[2][2] = cos_r2 * cos_r3;
      break;

    case kRotationOrderZyz:
      dcm->d[0][0] = cos_r1 * cos_r2 * cos_r3 - sin_r1 * sin_r3;
      dcm->d[0][1] = sin_r1 * cos_r2 * cos_r3 + cos_r1 * sin_r3;
      dcm->d[0][2] = -sin_r2 * cos_r3;
      dcm->d[1][0] = -cos_r1 * cos_r2 * sin_r3 - sin_r1 * cos_r3;
      dcm->d[1][1] = cos_r1 * cos_r3 - sin_r1 * cos_r2 * sin_r3;
      dcm->d[1][2] = sin_r2 * sin_r3;
      dcm->d[2][0] = cos_r1 * sin_r2;
      dcm->d[2][1] = sin_r1 * sin_r2;
      dcm->d[2][2] = cos_r2;
      break;

    case kRotationOrderZxy:
      dcm->d[0][0] = cos_r1 * cos_r3 - sin_r1 * sin_r2 * sin_r3;
      dcm->d[0][1] = sin_r1 * cos_r3 + cos_r1 * sin_r2 * sin_r3;
      dcm->d[0][2] = -cos_r2 * sin_r3;
      dcm->d[1][0] = -sin_r1 * cos_r2;
      dcm->d[1][1] = cos_r1 * cos_r2;
      dcm->d[1][2] = sin_r2;
      dcm->d[2][0] = cos_r1 * sin_r3 + sin_r1 * sin_r2 * cos_r3;
      dcm->d[2][1] = sin_r1 * sin_r3 - cos_r1 * sin_r2 * cos_r3;
      dcm->d[2][2] = cos_r2 * cos_r3;
      break;

    case kRotationOrderZxz:
      dcm->d[0][0] = cos_r1 * cos_r3 - sin_r1 * cos_r2 * sin_r3;
      dcm->d[0][1] = cos_r1 * cos_r2 * sin_r3 + sin_r1 * cos_r3;
      dcm->d[0][2] = sin_r2 * sin_r3;
      dcm->d[1][0] = -sin_r1 * cos_r2 * cos_r3 - cos_r1 * sin_r3;
      dcm->d[1][1] = cos_r1 * cos_r2 * cos_r3 - sin_r1 * sin_r3;
      dcm->d[1][2] = sin_r2 * cos_r3;
      dcm->d[2][0] = sin_r1 * sin_r2;
      dcm->d[2][1] = -cos_r1 * sin_r2;
      dcm->d[2][2] = cos_r2;
      break;
  }

  return dcm;
}

// Converts Cartesian to spherical coordinates.
void CartToSph(const Vec3 *v, double *azi, double *ele, double *r) {
  *r = Vec3Norm(v);
  *ele = atan2(v->z, Vec3XyNorm(v));
  *azi = atan2(v->y, v->x);
}

// Converts spherical to Cartesian coordinates.
const Vec3 *SphToCart(double azi, double ele, double r, Vec3 *v) {
  double rcosele = r * cos(ele);
  v->x = rcosele * cos(azi);
  v->y = rcosele * sin(azi);
  v->z = r * sin(ele);
  return v;
}

// Converts Cartesian to cylindrical coordinates.
void CartToCyl(const Vec3 *v, double *azi, double *r, double *z) {
  *azi = atan2(v->y, v->x);
  *r = sqrt(v->x * v->x + v->y * v->y);
  *z = v->z;
}

// Converts cylindrical to Cartesian coordinates.
const Vec3 *CylToCart(double azi, double r, double z, Vec3 *v) {
  v->x = r * cos(azi);
  v->y = r * sin(azi);
  v->z = z;
  return v;
}

// Transforms a vector in coordinate system "a" to an equivalent
// vector in coordinate system "b".
//
// dcm_a2b: Direction cosine matrix which transforms vectors expressed
//          in the "a" coordinate system to vectors expressed in the
//          "b" coordinate system.
// R_a_b_origin: Origin of the "b" coordinate system expressed in the
//               "a" coordinate system.
// X_a: Input vector expressed in the "a" coordinate system.
// X_b: Output vector expressed in the "b" coordinate system.
const Vec3 *PoseTransform(const Mat3 *dcm_a2b, const Vec3 *R_a_b_origin,
                          const Vec3 *X_a, Vec3 *X_b) {
  Vec3Sub(X_a, R_a_b_origin, X_b);
  return Mat3Vec3Mult(dcm_a2b, X_b, X_b);
}

// Transforms a vector in coordinate system "b" to an equivalent
// vector in coordinate system "a".
//
// dcm_a2b: Same as above.
// R_a_b_origin: Same as above.
// X_b: Input vector expressed in the "b" coordinate system.
// X_a: Output vector expressed in the "a" coordinate system.
const Vec3 *InversePoseTransform(const Mat3 *dcm_a2b, const Vec3 *R_a_b_origin,
                                 const Vec3 *X_b, Vec3 *X_a) {
  Vec3 tmp;  // Makes it safe to reuse inputs.
  Mat3TransVec3Mult(dcm_a2b, X_b, &tmp);
  return Vec3Add(&tmp, R_a_b_origin, X_a);
}

// Returns the axis and angle of the rotation that rotates v1 to v2.
//
// Input arguments:
//
// v1:    The first vector.
// v2:    The second vector, representing the direction to which we
//        want to rotate the first vector.
//
// Output arguments:
//
// axis:  The axis of rotation required to rotate v1 to v2, or NULL
//        if this output is not required.
//
// Return value: The magnitude (angle) of the rotation [rad].
double Vec3ToAxisAngle(const Vec3 *v1, const Vec3 *v2, Vec3 *axis) {
  Vec3 v1n, v2n;
  Vec3Normalize(v1, &v1n);
  Vec3Normalize(v2, &v2n);

  if (axis != NULL) {
    Vec3Cross(&v1n, &v2n, axis);
    if (Vec3Norm(axis) > 0.0) {
      Vec3Normalize(axis, axis);
    }
  }

  return Acos(Vec3Dot(&v1n, &v2n));
}

// Project a vector onto a plane using the formula:
//
//   v_out = v_in - v_normal * (v_normal'*v_in) / (v_normal'*v_normal);
//
// v_in: Vector to project.
// v_normal: Normal vector to the plane.
// v_out: Projected vector in the plane.
//
// Warning: As ||v_normal|| approaches zero, the divide-by-zero
// protection is encountered and the output v_normal, approaches the
// input, v_in.
//
// TODO: Remove arbitrary 1e-6.
void ProjectVec3ToPlane(const Vec3 *v_in, const Vec3 *v_normal, Vec3 *v_out) {
  double a = -Vec3Dot(v_normal, v_in) / fmax(Vec3NormSquared(v_normal), 1e-6);
  Vec3LinComb(1.0, v_in, a, v_normal, v_out);
}

// Find the smallest magnitude rotation angle (i.e. signed angle)
// between two Vec2s.  This angle will always be between -pi and pi.
double Vec2ToAngle(const Vec2 *v1, const Vec2 *v2) {
  return atan2(v1->x * v2->y - v1->y * v2->x, Vec2Dot(v1, v2));
}

// Find the point at the intersection between a plane and a line using the
// formula:
//
//   p = d*l + l_0
//
// where the plane is given by a point p0 and a normal unit vector n, and the
// line is given by a point l0 and a unit vector l. The scalar d is obtained by:
//
//   d = dot(p_0 - l_0, n) / dot(l, n)
//
// If dot(l, n) == 0, the line and the plane are parallel, indicating that
// either no solution exists or that infinite solutions exist. In this case,
// false is returned and intersect_point is a zero vector. For the cases where
// the plane and the line intersect in a single point, true is returned and
// intersect_point contains the point of intersection.
// See https://en.wikipedia.org/wiki/Line%E2%80%93plane_intersection
bool IntersectLinePlane(const Vec3 *p_0, const Vec3 *n, const Vec3 *l_0,
                        const Vec3 *l, Vec3 *intersect_point) {
  double l_dot_n = Vec3Dot(l, n);
  if (fabs(l_dot_n) < 1e-10) {
    // Line and plane are parallel.
    *intersect_point = kVec3Zero;
    return false;
  }

  double d;
  Vec3 p_0_minus_l_0;
  d = Vec3Dot(Vec3Sub(p_0, l_0, &p_0_minus_l_0), n) / l_dot_n;
  Vec3Add(Vec3Scale(l, d, intersect_point), l_0, intersect_point);
  return true;
}

// Returns the signed angle between two vectors.
//
// Inputs:
//    a: First vector.
//    b: Second vector.
//    n: Normal vector to the plane determining the angle sign.
//
//  Outputs:
//    Signed angle between a and b. If the vector cross(a,b) is in the same
//    direction as n, the angle is positive; otherwise it is negative.
double SignedAngleBetweenVectors(const Vec3 *a, const Vec3 *b, const Vec3 *n) {
  Vec3 a_n, b_n, n_n;  // Unit versions of the input vectors.
  Vec3Normalize(a, &a_n);
  Vec3Normalize(b, &b_n);
  Vec3Normalize(n, &n_n);
  double angle = Acos(Vec3Dot(&a_n, &b_n));
  Vec3 a_b_normal;
  Vec3Cross(&a_n, &b_n, &a_b_normal);
  angle *= Sign(Vec3Dot(&n_n, &a_b_normal));
  return angle;
}

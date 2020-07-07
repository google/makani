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

// Simple 2- and 3-dimension geometry library.  Contains functions for
// coordinate system transformations and vector math.

#ifndef COMMON_C_MATH_GEOMETRY_H_
#define COMMON_C_MATH_GEOMETRY_H_

#include "common/c_math/mat3.h"
#include "common/c_math/vec2.h"
#include "common/c_math/vec3.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  kRotationOrderForceSigned = -1,
  kRotationOrderXyz,
  kRotationOrderXyx,
  kRotationOrderXzy,
  kRotationOrderXzx,
  kRotationOrderYxz,
  kRotationOrderYxy,
  kRotationOrderYzx,
  kRotationOrderYzy,
  kRotationOrderZyx,
  kRotationOrderZyz,
  kRotationOrderZxy,
  kRotationOrderZxz,
  kNumRotationOrders
} RotationOrder;

void DcmToAngle(const Mat3 *dcm, RotationOrder order, double *r1, double *r2,
                double *r3);
const Mat3 *AngleToDcm(double r1, double r2, double r3, RotationOrder order,
                       Mat3 *dcm);

void CartToSph(const Vec3 *v, double *azi, double *ele, double *r);
const Vec3 *SphToCart(double azi, double ele, double r, Vec3 *v);

void CartToCyl(const Vec3 *v, double *azi, double *r, double *z);
const Vec3 *CylToCart(double azi, double r, double z, Vec3 *v);

const Vec3 *PoseTransform(const Mat3 *dcm_a2b, const Vec3 *R_a_b_origin,
                          const Vec3 *X_a, Vec3 *X_b);
const Vec3 *InversePoseTransform(const Mat3 *dcm_a2b, const Vec3 *R_a_b_origin,
                                 const Vec3 *X_b, Vec3 *X_a);

double Vec3ToAxisAngle(const Vec3 *v1, const Vec3 *v2, Vec3 *axis);
void ProjectVec3ToPlane(const Vec3 *v_in, const Vec3 *v_normal, Vec3 *v_out);
double Vec2ToAngle(const Vec2 *v1, const Vec2 *v2);
bool IntersectLinePlane(const Vec3 *p_0, const Vec3 *n, const Vec3 *l_0,
                        const Vec3 *l, Vec3 *intersect_point);
double SignedAngleBetweenVectors(const Vec3 *a, const Vec3 *b, const Vec3 *n);
#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // COMMON_C_MATH_GEOMETRY_H_

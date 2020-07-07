// Copyright 2020 Makani Technologies LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "sim/physics/aero_frame.h"

#include <math.h>

#include "common/c_math/geometry.h"
#include "common/c_math/mat3.h"
#include "common/c_math/vec3.h"

const Mat3 *CalcDcmWToB(double alpha, double beta, Mat3 *dcm_w2b) {
  return AngleToDcm(-beta, alpha, 0.0, kRotationOrderZyx, dcm_w2b);
}

const Vec3 *RotBToW(const Vec3 *Vb, double alpha, double beta, Vec3 *Vw) {
  Mat3 dcm_w2b;
  CalcDcmWToB(alpha, beta, &dcm_w2b);
  return Mat3TransVec3Mult(&dcm_w2b, Vb, Vw);
}

const Vec3 *RotWToB(const Vec3 *Vw, double alpha, double beta, Vec3 *Vb) {
  Mat3 dcm_w2b;
  CalcDcmWToB(alpha, beta, &dcm_w2b);
  return Mat3Vec3Mult(&dcm_w2b, Vw, Vb);
}

const Mat3 *CalcDcmSToB(double alpha, Mat3 *dcm_s2b) {
  return AngleToDcm(0.0, alpha, 0.0, kRotationOrderZyx, dcm_s2b);
}

const Vec3 *RotBToS(const Vec3 *Vb, double alpha, Vec3 *Vs) {
  Vec3 tmp;  // This working space allows Vb == Vs.
  tmp.x = Vb->x * cos(alpha) + Vb->z * sin(alpha);
  tmp.y = Vb->y;
  tmp.z = -Vb->x * sin(alpha) + Vb->z * cos(alpha);
  *Vs = tmp;
  return Vs;
}

const Vec3 *RotSToB(const Vec3 *Vs, double alpha, Vec3 *Vb) {
  Vec3 tmp;  // This working space allows Vb == Vs.
  tmp.x = Vs->x * cos(alpha) - Vs->z * sin(alpha);
  tmp.y = Vs->y;
  tmp.z = Vs->x * sin(alpha) + Vs->z * cos(alpha);
  *Vb = tmp;
  return Vb;
}

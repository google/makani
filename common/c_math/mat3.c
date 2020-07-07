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

#include "common/c_math/mat3.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "common/c_math/linalg_common.h"
#include "common/c_math/vec3.h"

const Mat3 kMat3Zero = {{{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}}};
const Mat3 kMat3Identity = {
    {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}}};

const Mat3 *Mat3Scale(const Mat3 *m_in, double a, Mat3 *m_out) {
  m_out->d[0][0] = a * m_in->d[0][0];
  m_out->d[0][1] = a * m_in->d[0][1];
  m_out->d[0][2] = a * m_in->d[0][2];
  m_out->d[1][0] = a * m_in->d[1][0];
  m_out->d[1][1] = a * m_in->d[1][1];
  m_out->d[1][2] = a * m_in->d[1][2];
  m_out->d[2][0] = a * m_in->d[2][0];
  m_out->d[2][1] = a * m_in->d[2][1];
  m_out->d[2][2] = a * m_in->d[2][2];
  return m_out;
}

// v_out = m*v0 + a*v1
const Vec3 *Mat3Vec3Axpby(const Mat3 *m, TransposeType t, const Vec3 *v0,
                          double a, const Vec3 *v1, Vec3 *v_out) {
  Vec3 v_tmp;
  if (t == kTrans) {
    v_tmp.x = m->d[0][0] * v0->x + m->d[1][0] * v0->y + m->d[2][0] * v0->z;
    v_tmp.y = m->d[0][1] * v0->x + m->d[1][1] * v0->y + m->d[2][1] * v0->z;
    v_tmp.z = m->d[0][2] * v0->x + m->d[1][2] * v0->y + m->d[2][2] * v0->z;
  } else {
    v_tmp.x = m->d[0][0] * v0->x + m->d[0][1] * v0->y + m->d[0][2] * v0->z;
    v_tmp.y = m->d[1][0] * v0->x + m->d[1][1] * v0->y + m->d[1][2] * v0->z;
    v_tmp.z = m->d[2][0] * v0->x + m->d[2][1] * v0->y + m->d[2][2] * v0->z;
  }

  v_out->x = v_tmp.x + a * v1->x;
  v_out->y = v_tmp.y + a * v1->y;
  v_out->z = v_tmp.z + a * v1->z;
  return v_out;
}

// m_out = m0 * m1 + a * m2
//
// The code for this function was automatically generated and just
// pasted in place for efficiency.
const Mat3 *Mat3Abpyc(const Mat3 *m0, TransposeType t0, const Mat3 *m1,
                      TransposeType t1, double y, const Mat3 *m2,
                      TransposeType t2, Mat3 *m_out) {
  Mat3 m_tmp;
  if (t0 == kNoTrans && t1 == kNoTrans && t2 == kNoTrans) {
    m_tmp.d[0][0] = m0->d[0][0] * m1->d[0][0] + m0->d[0][1] * m1->d[1][0] +
                    m0->d[0][2] * m1->d[2][0] + y * m2->d[0][0];
    m_tmp.d[0][1] = m0->d[0][0] * m1->d[0][1] + m0->d[0][1] * m1->d[1][1] +
                    m0->d[0][2] * m1->d[2][1] + y * m2->d[0][1];
    m_tmp.d[0][2] = m0->d[0][0] * m1->d[0][2] + m0->d[0][1] * m1->d[1][2] +
                    m0->d[0][2] * m1->d[2][2] + y * m2->d[0][2];
    m_tmp.d[1][0] = m0->d[1][0] * m1->d[0][0] + m0->d[1][1] * m1->d[1][0] +
                    m0->d[1][2] * m1->d[2][0] + y * m2->d[1][0];
    m_tmp.d[1][1] = m0->d[1][0] * m1->d[0][1] + m0->d[1][1] * m1->d[1][1] +
                    m0->d[1][2] * m1->d[2][1] + y * m2->d[1][1];
    m_tmp.d[1][2] = m0->d[1][0] * m1->d[0][2] + m0->d[1][1] * m1->d[1][2] +
                    m0->d[1][2] * m1->d[2][2] + y * m2->d[1][2];
    m_tmp.d[2][0] = m0->d[2][0] * m1->d[0][0] + m0->d[2][1] * m1->d[1][0] +
                    m0->d[2][2] * m1->d[2][0] + y * m2->d[2][0];
    m_tmp.d[2][1] = m0->d[2][0] * m1->d[0][1] + m0->d[2][1] * m1->d[1][1] +
                    m0->d[2][2] * m1->d[2][1] + y * m2->d[2][1];
    m_tmp.d[2][2] = m0->d[2][0] * m1->d[0][2] + m0->d[2][1] * m1->d[1][2] +
                    m0->d[2][2] * m1->d[2][2] + y * m2->d[2][2];

  } else if (t0 == kNoTrans && t1 == kNoTrans && t2 == kTrans) {
    m_tmp.d[0][0] = m0->d[0][0] * m1->d[0][0] + m0->d[0][1] * m1->d[1][0] +
                    m0->d[0][2] * m1->d[2][0] + y * m2->d[0][0];
    m_tmp.d[0][1] = m0->d[0][0] * m1->d[0][1] + m0->d[0][1] * m1->d[1][1] +
                    m0->d[0][2] * m1->d[2][1] + y * m2->d[1][0];
    m_tmp.d[0][2] = m0->d[0][0] * m1->d[0][2] + m0->d[0][1] * m1->d[1][2] +
                    m0->d[0][2] * m1->d[2][2] + y * m2->d[2][0];
    m_tmp.d[1][0] = m0->d[1][0] * m1->d[0][0] + m0->d[1][1] * m1->d[1][0] +
                    m0->d[1][2] * m1->d[2][0] + y * m2->d[0][1];
    m_tmp.d[1][1] = m0->d[1][0] * m1->d[0][1] + m0->d[1][1] * m1->d[1][1] +
                    m0->d[1][2] * m1->d[2][1] + y * m2->d[1][1];
    m_tmp.d[1][2] = m0->d[1][0] * m1->d[0][2] + m0->d[1][1] * m1->d[1][2] +
                    m0->d[1][2] * m1->d[2][2] + y * m2->d[2][1];
    m_tmp.d[2][0] = m0->d[2][0] * m1->d[0][0] + m0->d[2][1] * m1->d[1][0] +
                    m0->d[2][2] * m1->d[2][0] + y * m2->d[0][2];
    m_tmp.d[2][1] = m0->d[2][0] * m1->d[0][1] + m0->d[2][1] * m1->d[1][1] +
                    m0->d[2][2] * m1->d[2][1] + y * m2->d[1][2];
    m_tmp.d[2][2] = m0->d[2][0] * m1->d[0][2] + m0->d[2][1] * m1->d[1][2] +
                    m0->d[2][2] * m1->d[2][2] + y * m2->d[2][2];

  } else if (t0 == kNoTrans && t1 == kTrans && t2 == kNoTrans) {
    m_tmp.d[0][0] = m0->d[0][0] * m1->d[0][0] + m0->d[0][1] * m1->d[0][1] +
                    m0->d[0][2] * m1->d[0][2] + y * m2->d[0][0];
    m_tmp.d[0][1] = m0->d[0][0] * m1->d[1][0] + m0->d[0][1] * m1->d[1][1] +
                    m0->d[0][2] * m1->d[1][2] + y * m2->d[0][1];
    m_tmp.d[0][2] = m0->d[0][0] * m1->d[2][0] + m0->d[0][1] * m1->d[2][1] +
                    m0->d[0][2] * m1->d[2][2] + y * m2->d[0][2];
    m_tmp.d[1][0] = m0->d[1][0] * m1->d[0][0] + m0->d[1][1] * m1->d[0][1] +
                    m0->d[1][2] * m1->d[0][2] + y * m2->d[1][0];
    m_tmp.d[1][1] = m0->d[1][0] * m1->d[1][0] + m0->d[1][1] * m1->d[1][1] +
                    m0->d[1][2] * m1->d[1][2] + y * m2->d[1][1];
    m_tmp.d[1][2] = m0->d[1][0] * m1->d[2][0] + m0->d[1][1] * m1->d[2][1] +
                    m0->d[1][2] * m1->d[2][2] + y * m2->d[1][2];
    m_tmp.d[2][0] = m0->d[2][0] * m1->d[0][0] + m0->d[2][1] * m1->d[0][1] +
                    m0->d[2][2] * m1->d[0][2] + y * m2->d[2][0];
    m_tmp.d[2][1] = m0->d[2][0] * m1->d[1][0] + m0->d[2][1] * m1->d[1][1] +
                    m0->d[2][2] * m1->d[1][2] + y * m2->d[2][1];
    m_tmp.d[2][2] = m0->d[2][0] * m1->d[2][0] + m0->d[2][1] * m1->d[2][1] +
                    m0->d[2][2] * m1->d[2][2] + y * m2->d[2][2];

  } else if (t0 == kNoTrans && t1 == kTrans && t2 == kTrans) {
    m_tmp.d[0][0] = m0->d[0][0] * m1->d[0][0] + m0->d[0][1] * m1->d[0][1] +
                    m0->d[0][2] * m1->d[0][2] + y * m2->d[0][0];
    m_tmp.d[0][1] = m0->d[0][0] * m1->d[1][0] + m0->d[0][1] * m1->d[1][1] +
                    m0->d[0][2] * m1->d[1][2] + y * m2->d[1][0];
    m_tmp.d[0][2] = m0->d[0][0] * m1->d[2][0] + m0->d[0][1] * m1->d[2][1] +
                    m0->d[0][2] * m1->d[2][2] + y * m2->d[2][0];
    m_tmp.d[1][0] = m0->d[1][0] * m1->d[0][0] + m0->d[1][1] * m1->d[0][1] +
                    m0->d[1][2] * m1->d[0][2] + y * m2->d[0][1];
    m_tmp.d[1][1] = m0->d[1][0] * m1->d[1][0] + m0->d[1][1] * m1->d[1][1] +
                    m0->d[1][2] * m1->d[1][2] + y * m2->d[1][1];
    m_tmp.d[1][2] = m0->d[1][0] * m1->d[2][0] + m0->d[1][1] * m1->d[2][1] +
                    m0->d[1][2] * m1->d[2][2] + y * m2->d[2][1];
    m_tmp.d[2][0] = m0->d[2][0] * m1->d[0][0] + m0->d[2][1] * m1->d[0][1] +
                    m0->d[2][2] * m1->d[0][2] + y * m2->d[0][2];
    m_tmp.d[2][1] = m0->d[2][0] * m1->d[1][0] + m0->d[2][1] * m1->d[1][1] +
                    m0->d[2][2] * m1->d[1][2] + y * m2->d[1][2];
    m_tmp.d[2][2] = m0->d[2][0] * m1->d[2][0] + m0->d[2][1] * m1->d[2][1] +
                    m0->d[2][2] * m1->d[2][2] + y * m2->d[2][2];

  } else if (t0 == kTrans && t1 == kNoTrans && t2 == kNoTrans) {
    m_tmp.d[0][0] = m0->d[0][0] * m1->d[0][0] + m0->d[1][0] * m1->d[1][0] +
                    m0->d[2][0] * m1->d[2][0] + y * m2->d[0][0];
    m_tmp.d[0][1] = m0->d[0][0] * m1->d[0][1] + m0->d[1][0] * m1->d[1][1] +
                    m0->d[2][0] * m1->d[2][1] + y * m2->d[0][1];
    m_tmp.d[0][2] = m0->d[0][0] * m1->d[0][2] + m0->d[1][0] * m1->d[1][2] +
                    m0->d[2][0] * m1->d[2][2] + y * m2->d[0][2];
    m_tmp.d[1][0] = m0->d[0][1] * m1->d[0][0] + m0->d[1][1] * m1->d[1][0] +
                    m0->d[2][1] * m1->d[2][0] + y * m2->d[1][0];
    m_tmp.d[1][1] = m0->d[0][1] * m1->d[0][1] + m0->d[1][1] * m1->d[1][1] +
                    m0->d[2][1] * m1->d[2][1] + y * m2->d[1][1];
    m_tmp.d[1][2] = m0->d[0][1] * m1->d[0][2] + m0->d[1][1] * m1->d[1][2] +
                    m0->d[2][1] * m1->d[2][2] + y * m2->d[1][2];
    m_tmp.d[2][0] = m0->d[0][2] * m1->d[0][0] + m0->d[1][2] * m1->d[1][0] +
                    m0->d[2][2] * m1->d[2][0] + y * m2->d[2][0];
    m_tmp.d[2][1] = m0->d[0][2] * m1->d[0][1] + m0->d[1][2] * m1->d[1][1] +
                    m0->d[2][2] * m1->d[2][1] + y * m2->d[2][1];
    m_tmp.d[2][2] = m0->d[0][2] * m1->d[0][2] + m0->d[1][2] * m1->d[1][2] +
                    m0->d[2][2] * m1->d[2][2] + y * m2->d[2][2];

  } else if (t0 == kTrans && t1 == kNoTrans && t2 == kTrans) {
    m_tmp.d[0][0] = m0->d[0][0] * m1->d[0][0] + m0->d[1][0] * m1->d[1][0] +
                    m0->d[2][0] * m1->d[2][0] + y * m2->d[0][0];
    m_tmp.d[0][1] = m0->d[0][0] * m1->d[0][1] + m0->d[1][0] * m1->d[1][1] +
                    m0->d[2][0] * m1->d[2][1] + y * m2->d[1][0];
    m_tmp.d[0][2] = m0->d[0][0] * m1->d[0][2] + m0->d[1][0] * m1->d[1][2] +
                    m0->d[2][0] * m1->d[2][2] + y * m2->d[2][0];
    m_tmp.d[1][0] = m0->d[0][1] * m1->d[0][0] + m0->d[1][1] * m1->d[1][0] +
                    m0->d[2][1] * m1->d[2][0] + y * m2->d[0][1];
    m_tmp.d[1][1] = m0->d[0][1] * m1->d[0][1] + m0->d[1][1] * m1->d[1][1] +
                    m0->d[2][1] * m1->d[2][1] + y * m2->d[1][1];
    m_tmp.d[1][2] = m0->d[0][1] * m1->d[0][2] + m0->d[1][1] * m1->d[1][2] +
                    m0->d[2][1] * m1->d[2][2] + y * m2->d[2][1];
    m_tmp.d[2][0] = m0->d[0][2] * m1->d[0][0] + m0->d[1][2] * m1->d[1][0] +
                    m0->d[2][2] * m1->d[2][0] + y * m2->d[0][2];
    m_tmp.d[2][1] = m0->d[0][2] * m1->d[0][1] + m0->d[1][2] * m1->d[1][1] +
                    m0->d[2][2] * m1->d[2][1] + y * m2->d[1][2];
    m_tmp.d[2][2] = m0->d[0][2] * m1->d[0][2] + m0->d[1][2] * m1->d[1][2] +
                    m0->d[2][2] * m1->d[2][2] + y * m2->d[2][2];

  } else if (t0 == kTrans && t1 == kTrans && t2 == kNoTrans) {
    m_tmp.d[0][0] = m0->d[0][0] * m1->d[0][0] + m0->d[1][0] * m1->d[0][1] +
                    m0->d[2][0] * m1->d[0][2] + y * m2->d[0][0];
    m_tmp.d[0][1] = m0->d[0][0] * m1->d[1][0] + m0->d[1][0] * m1->d[1][1] +
                    m0->d[2][0] * m1->d[1][2] + y * m2->d[0][1];
    m_tmp.d[0][2] = m0->d[0][0] * m1->d[2][0] + m0->d[1][0] * m1->d[2][1] +
                    m0->d[2][0] * m1->d[2][2] + y * m2->d[0][2];
    m_tmp.d[1][0] = m0->d[0][1] * m1->d[0][0] + m0->d[1][1] * m1->d[0][1] +
                    m0->d[2][1] * m1->d[0][2] + y * m2->d[1][0];
    m_tmp.d[1][1] = m0->d[0][1] * m1->d[1][0] + m0->d[1][1] * m1->d[1][1] +
                    m0->d[2][1] * m1->d[1][2] + y * m2->d[1][1];
    m_tmp.d[1][2] = m0->d[0][1] * m1->d[2][0] + m0->d[1][1] * m1->d[2][1] +
                    m0->d[2][1] * m1->d[2][2] + y * m2->d[1][2];
    m_tmp.d[2][0] = m0->d[0][2] * m1->d[0][0] + m0->d[1][2] * m1->d[0][1] +
                    m0->d[2][2] * m1->d[0][2] + y * m2->d[2][0];
    m_tmp.d[2][1] = m0->d[0][2] * m1->d[1][0] + m0->d[1][2] * m1->d[1][1] +
                    m0->d[2][2] * m1->d[1][2] + y * m2->d[2][1];
    m_tmp.d[2][2] = m0->d[0][2] * m1->d[2][0] + m0->d[1][2] * m1->d[2][1] +
                    m0->d[2][2] * m1->d[2][2] + y * m2->d[2][2];

  } else {
    // Some compilers complain less if you make it obvious that you've
    // exhausted the logical possibilities
    // (t0 == kTrans && t1 == kTrans && t2 == kTrans)
    m_tmp.d[0][0] = m0->d[0][0] * m1->d[0][0] + m0->d[1][0] * m1->d[0][1] +
                    m0->d[2][0] * m1->d[0][2] + y * m2->d[0][0];
    m_tmp.d[0][1] = m0->d[0][0] * m1->d[1][0] + m0->d[1][0] * m1->d[1][1] +
                    m0->d[2][0] * m1->d[1][2] + y * m2->d[1][0];
    m_tmp.d[0][2] = m0->d[0][0] * m1->d[2][0] + m0->d[1][0] * m1->d[2][1] +
                    m0->d[2][0] * m1->d[2][2] + y * m2->d[2][0];
    m_tmp.d[1][0] = m0->d[0][1] * m1->d[0][0] + m0->d[1][1] * m1->d[0][1] +
                    m0->d[2][1] * m1->d[0][2] + y * m2->d[0][1];
    m_tmp.d[1][1] = m0->d[0][1] * m1->d[1][0] + m0->d[1][1] * m1->d[1][1] +
                    m0->d[2][1] * m1->d[1][2] + y * m2->d[1][1];
    m_tmp.d[1][2] = m0->d[0][1] * m1->d[2][0] + m0->d[1][1] * m1->d[2][1] +
                    m0->d[2][1] * m1->d[2][2] + y * m2->d[2][1];
    m_tmp.d[2][0] = m0->d[0][2] * m1->d[0][0] + m0->d[1][2] * m1->d[0][1] +
                    m0->d[2][2] * m1->d[0][2] + y * m2->d[0][2];
    m_tmp.d[2][1] = m0->d[0][2] * m1->d[1][0] + m0->d[1][2] * m1->d[1][1] +
                    m0->d[2][2] * m1->d[1][2] + y * m2->d[1][2];
    m_tmp.d[2][2] = m0->d[0][2] * m1->d[2][0] + m0->d[1][2] * m1->d[2][1] +
                    m0->d[2][2] * m1->d[2][2] + y * m2->d[2][2];
  }
  *m_out = m_tmp;
  return m_out;
}

const Mat3 *Mat3Mult(const Mat3 *m0, TransposeType t0, const Mat3 *m1,
                     TransposeType t1, Mat3 *m_out) {
  return Mat3Abpyc(m0, t0, m1, t1, 0.0, &kMat3Zero, kNoTrans, m_out);
}

const Mat3 *Mat3Add(const Mat3 *m0, TransposeType t0, const Mat3 *m1,
                    TransposeType t1, Mat3 *m_out) {
  return Mat3Abpyc(m0, t0, &kMat3Identity, kNoTrans, 1.0, m1, t1, m_out);
}

const Vec3 *Mat3Vec3Mult(const Mat3 *m, const Vec3 *v_in, Vec3 *v_out) {
  Vec3 v_tmp;
  v_tmp.x = m->d[0][0] * v_in->x + m->d[0][1] * v_in->y + m->d[0][2] * v_in->z;
  v_tmp.y = m->d[1][0] * v_in->x + m->d[1][1] * v_in->y + m->d[1][2] * v_in->z;
  v_tmp.z = m->d[2][0] * v_in->x + m->d[2][1] * v_in->y + m->d[2][2] * v_in->z;

  *v_out = v_tmp;
  return v_out;
}

const Vec3 *Mat3TransVec3Mult(const Mat3 *m, const Vec3 *v_in, Vec3 *v_out) {
  Vec3 v_tmp;
  v_tmp.x = m->d[0][0] * v_in->x + m->d[1][0] * v_in->y + m->d[2][0] * v_in->z;
  v_tmp.y = m->d[0][1] * v_in->x + m->d[1][1] * v_in->y + m->d[2][1] * v_in->z;
  v_tmp.z = m->d[0][2] * v_in->x + m->d[1][2] * v_in->y + m->d[2][2] * v_in->z;

  *v_out = v_tmp;
  return v_out;
}

// v_out = m^-1 * v_in
const Vec3 *Mat3Vec3LeftDivide(const Mat3 *m, const Vec3 *v_in, Vec3 *v_out) {
  Mat3 m_inv;
  // TODO: Use BLAS or LU decomp. (faster?)
  Mat3Inv(m, &m_inv);
  Mat3Vec3Mult(&m_inv, v_in, v_out);
  return v_out;
}

const Mat3 *Mat3Mat3Mult(const Mat3 *m1, const Mat3 *m2, Mat3 *m_out) {
  return Mat3Abpyc(m1, kNoTrans, m2, kNoTrans, 0.0, &kMat3Zero, kNoTrans,
                   m_out);
}

double Mat3Det(const Mat3 *m) {
  return m->d[0][0] * (m->d[1][1] * m->d[2][2] - m->d[1][2] * m->d[2][1]) +
         m->d[0][1] * (m->d[1][2] * m->d[2][0] - m->d[2][2] * m->d[1][0]) +
         m->d[0][2] * (m->d[1][0] * m->d[2][1] - m->d[1][1] * m->d[2][0]);
}

const Mat3 *Mat3Inv(const Mat3 *m_in, Mat3 *m_out) {
  Vec3 v12, v20, v01;
  Vec3 x0 = {m_in->d[0][0], m_in->d[1][0], m_in->d[2][0]};
  Vec3 x1 = {m_in->d[0][1], m_in->d[1][1], m_in->d[2][1]};
  Vec3 x2 = {m_in->d[0][2], m_in->d[1][2], m_in->d[2][2]};
  double det = Mat3Det(m_in);

  Vec3Cross(&x1, &x2, &v12);
  Vec3Cross(&x2, &x0, &v20);
  Vec3Cross(&x0, &x1, &v01);

  m_out->d[0][0] = v12.x;
  m_out->d[0][1] = v12.y;
  m_out->d[0][2] = v12.z;
  m_out->d[1][0] = v20.x;
  m_out->d[1][1] = v20.y;
  m_out->d[1][2] = v20.z;
  m_out->d[2][0] = v01.x;
  m_out->d[2][1] = v01.y;
  m_out->d[2][2] = v01.z;

  if (det == 0.0) {
    // This behavior for singular matrices is based on MATLAB's behavior.
    for (int32_t i = 0; i < 3; ++i) {
      for (int32_t j = 0; j < 3; ++j) {
        m_out->d[i][j] = INFINITY;
      }
    }
  } else {
    Mat3Scale(m_out, 1.0 / det, m_out);
  }
  return m_out;
}

double Mat3Trace(const Mat3 *m) { return m->d[0][0] + m->d[1][1] + m->d[2][2]; }

const Vec3 *Mat3Diag(const Mat3 *m, Vec3 *v) {
  v->x = m->d[0][0];
  v->y = m->d[1][1];
  v->z = m->d[2][2];
  return v;
}

const Mat3 *Mat3Trans(const Mat3 *m_in, Mat3 *m_out) {
  Mat3 m_tmp;
  m_tmp.d[0][0] = m_in->d[0][0];
  m_tmp.d[0][1] = m_in->d[1][0];
  m_tmp.d[0][2] = m_in->d[2][0];
  m_tmp.d[1][0] = m_in->d[0][1];
  m_tmp.d[1][1] = m_in->d[1][1];
  m_tmp.d[1][2] = m_in->d[2][1];
  m_tmp.d[2][0] = m_in->d[0][2];
  m_tmp.d[2][1] = m_in->d[1][2];
  m_tmp.d[2][2] = m_in->d[2][2];
  *m_out = m_tmp;
  return m_out;
}

// Construct a matrix whose action is equivalent to a cross product
// with a given vector.
const Mat3 *Mat3Cross(const Vec3 *v, Mat3 *m) {
  m->d[0][0] = 0.0;
  m->d[0][1] = -v->z;
  m->d[0][2] = v->y;

  m->d[1][0] = v->z;
  m->d[1][1] = 0.0;
  m->d[1][2] = -v->x;

  m->d[2][0] = -v->y;
  m->d[2][1] = v->x;
  m->d[2][2] = 0.0;

  return m;
}

// Checks if any of the elements of A are NaNs.  Returns true if there
// is a NaN, and false otherwise.
bool Mat3ContainsNaN(const Mat3 *A) {
  for (int32_t i = 0; i < 3; ++i) {
    for (int32_t j = 0; j < 3; ++j) {
      if (isnan(A->d[i][j])) return true;
    }
  }
  return false;
}

// Checks if a matrix A is orthogonal (i.e. a member of O(3)) to
// within a given tolerance, tol, which represents the maximum error
// allowed in the dot products.
bool Mat3IsOrthogonal(const Mat3 *A, double tol) {
  Vec3 u = {A->d[0][0], A->d[1][0], A->d[2][0]};
  Vec3 v = {A->d[0][1], A->d[1][1], A->d[2][1]};
  Vec3 w = {A->d[0][2], A->d[1][2], A->d[2][2]};

  // Check that the vectors are unit vectors.
  if (fabs(Vec3Dot(&u, &u) - 1.0) > tol || fabs(Vec3Dot(&v, &v) - 1.0) > tol ||
      fabs(Vec3Dot(&w, &w) - 1.0) > tol) {
    return false;
  }

  // Check that the vectors are orthogonal.
  if (fabs(Vec3Dot(&u, &v)) > tol || fabs(Vec3Dot(&u, &w)) > tol ||
      fabs(Vec3Dot(&v, &w)) > tol) {
    return false;
  }

  // Check that there are no NaNs.
  if (Mat3ContainsNaN(A)) return false;

  return true;
}

// Checks if a matrix A is a rotation matrix (i.e. a member of SO(3))
// to within a given tolerance, tol, which represents the maximum
// error allowed in the dot and cross products.
bool Mat3IsSpecialOrthogonal(const Mat3 *A, double tol) {
  Vec3 u = {A->d[0][0], A->d[1][0], A->d[2][0]};
  Vec3 v = {A->d[0][1], A->d[1][1], A->d[2][1]};
  Vec3 w = {A->d[0][2], A->d[1][2], A->d[2][2]};

  // Check that the first two vectors are orthogonal unit vectors.
  if (fabs(Vec3Dot(&u, &u) - 1.0) > tol || fabs(Vec3Dot(&v, &v) - 1.0) > tol ||
      fabs(Vec3Dot(&u, &v)) > tol) {
    return false;
  }

  // Check that the last vector is the cross product of the first two.
  Vec3 u_cross_v;
  Vec3Cross(&u, &v, &u_cross_v);
  if (fabs(u_cross_v.x - w.x) > tol || fabs(u_cross_v.y - w.y) > tol ||
      fabs(u_cross_v.z - w.z) > tol) {
    return false;
  }

  // Check that there are no NaNs.
  if (Mat3ContainsNaN(A)) return false;

  return true;
}

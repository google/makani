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

#include "common/c_math/mat2.h"

#include <math.h>
#include <stdint.h>

#include "common/c_math/linalg_common.h"
#include "common/c_math/vec2.h"

// Define Mat2 zero and identity constants.
const Mat2 kMat2Zero = {{{0.0, 0.0}, {0.0, 0.0}}};
const Mat2 kMat2Identity = {{{1.0, 0.0}, {0.0, 1.0}}};

// m_out = m_in*a
const Mat2 *Mat2Scale(const Mat2 *m_in, double a, Mat2 *m_out) {
  m_out->d[0][0] = a * m_in->d[0][0];
  m_out->d[1][0] = a * m_in->d[1][0];
  m_out->d[0][1] = a * m_in->d[0][1];
  m_out->d[1][1] = a * m_in->d[1][1];
  return m_out;
}

// v_out = m*v0 + a*v1
const Vec2 *Mat2Vec2Axpby(const Mat2 *m, TransposeType t, const Vec2 *v0,
                          double a, const Vec2 *v1, Vec2 *v_out) {
  Vec2 v_tmp;
  if (t == kTrans) {
    v_tmp.x = m->d[0][0] * v0->x + m->d[1][0] * v0->y + a * v1->x;
    v_tmp.y = m->d[0][1] * v0->x + m->d[1][1] * v0->y + a * v1->y;
  } else {
    v_tmp.x = m->d[0][0] * v0->x + m->d[0][1] * v0->y + a * v1->x;
    v_tmp.y = m->d[1][0] * v0->x + m->d[1][1] * v0->y + a * v1->y;
  }

  *v_out = v_tmp;
  return v_out;
}

// m_out = m0 * m1 + a * m2
const Mat2 *Mat2Abpyc(const Mat2 *m0, TransposeType t0, const Mat2 *m1,
                      TransposeType t1, double a, const Mat2 *m2,
                      TransposeType t2, Mat2 *m_out) {
  Mat2 m_tmp;
  int32_t r0 = 0, c0 = 1, r1 = 0, c1 = 1, r2 = 0, c2 = 1;

  // Swap off-diagonal element indices if the matrix is transposed.
  if (t0 == kTrans) {
    r0 = 1;
    c0 = 0;
  }
  if (t1 == kTrans) {
    r1 = 1;
    c1 = 0;
  }
  if (t2 == kTrans) {
    r2 = 1;
    c2 = 0;
  }
  m_tmp.d[0][0] = m0->d[0][0] * m1->d[0][0] + m0->d[r0][c0] * m1->d[c1][r1] +
                  a * m2->d[0][0];
  m_tmp.d[0][1] = m0->d[0][0] * m1->d[r1][c1] + m0->d[r0][c0] * m1->d[1][1] +
                  a * m2->d[r2][c2];
  m_tmp.d[1][0] = m0->d[c0][r0] * m1->d[0][0] + m0->d[1][1] * m1->d[c1][r1] +
                  a * m2->d[c2][r2];
  m_tmp.d[1][1] = m0->d[c0][r0] * m1->d[r1][c1] + m0->d[1][1] * m1->d[1][1] +
                  a * m2->d[1][1];

  *m_out = m_tmp;
  return m_out;
}

const Mat2 *Mat2Mult(const Mat2 *m0, TransposeType t0, const Mat2 *m1,
                     TransposeType t1, Mat2 *m_out) {
  return Mat2Abpyc(m0, t0, m1, t1, 0.0, &kMat2Zero, kNoTrans, m_out);
}

const Mat2 *Mat2Add(const Mat2 *m0, TransposeType t0, const Mat2 *m1,
                    TransposeType t1, Mat2 *m_out) {
  return Mat2Abpyc(m0, t0, &kMat2Identity, kNoTrans, 1.0, m1, t1, m_out);
}

const Vec2 *Mat2Vec2Mult(const Mat2 *m, const Vec2 *v_in, Vec2 *v_out) {
  return Mat2Vec2Axpby(m, kNoTrans, v_in, 0.0, &kVec2Zero, v_out);
}

const Vec2 *Mat2TransVec2Mult(const Mat2 *m, const Vec2 *v_in, Vec2 *v_out) {
  return Mat2Vec2Axpby(m, kTrans, v_in, 0.0, &kVec2Zero, v_out);
}

// v_out = m^-1 * v_in.
const Vec2 *Mat2Vec2LeftDivide(const Mat2 *m, const Vec2 *v_in, Vec2 *v_out) {
  Mat2 m_inv;
  // TODO: Use BLAS or LU decomposition.
  Mat2Inv(m, &m_inv);
  Mat2Vec2Mult(&m_inv, v_in, v_out);
  return v_out;
}

// The determinant of a matrix.
double Mat2Det(const Mat2 *m) {
  return m->d[0][0] * m->d[1][1] - m->d[0][1] * m->d[1][0];
}

// The inverse of a matrix.
const Mat2 *Mat2Inv(const Mat2 *m_in, Mat2 *m_out) {
  // This temporary variable makes it safe to reuse inputs as outputs
  Mat2 m_tmp;
  m_tmp.d[0][0] = m_in->d[1][1];
  m_tmp.d[0][1] = -m_in->d[0][1];
  m_tmp.d[1][0] = -m_in->d[1][0];
  m_tmp.d[1][1] = m_in->d[0][0];

  double det = Mat2Det(m_in);
  if (det == 0.0) {
    // This behavior for singular matrices is based on MATLAB's behavior.
    m_tmp.d[0][0] = INFINITY;
    m_tmp.d[0][1] = INFINITY;
    m_tmp.d[1][0] = INFINITY;
    m_tmp.d[1][1] = INFINITY;
  } else {
    Mat2Scale(&m_tmp, 1.0 / det, &m_tmp);
  }
  *m_out = m_tmp;
  return m_out;
}

// Get the trace of the matrix.
double Mat2Trace(const Mat2 *m) { return m->d[0][0] + m->d[1][1]; }

// Put the diagonal elements of m into v
const Vec2 *Mat2Diag(const Mat2 *m, Vec2 *v) {
  v->x = m->d[0][0];
  v->y = m->d[1][1];
  return v;
}

// m_out = m_in^T
const Mat2 *Mat2Trans(const Mat2 *m_in, Mat2 *m_out) {
  // This temporary variable makes it safe to reuse inputs as outputs
  Mat2 m_tmp;
  m_tmp.d[0][0] = m_in->d[0][0];
  m_tmp.d[1][0] = m_in->d[0][1];
  m_tmp.d[0][1] = m_in->d[1][0];
  m_tmp.d[1][1] = m_in->d[1][1];
  *m_out = m_tmp;
  return m_out;
}

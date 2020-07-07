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

#include "common/c_math/voting.h"

#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "common/c_math/quaternion.h"
#include "common/c_math/util.h"
#include "common/c_math/vec3.h"

static void PermuteQuatSignClosest3(const Quat *q_a, const Quat *q_b,
                                    const Quat *q_c, Quat *p_a, Quat *p_b,
                                    Quat *p_c);

// Computes a median of three values with fail-safe behavior for NaN
// values.
//
// Args:
//   a, b, c: Values to compute the median of.
//
// Returns:
//   If there are no NaN values in {a, b, c}, the median is returned.
//   If the values are {-INFINITY, INFINITY, NaN}, or {NaN, NaN, NaN}
//   then 0.0 is returned.  Otherwise, the average of the non-NaN
//   values is returned.
double Median3(double a, double b, double c) {
  int32_t num_not_nan = 0;
  double values[3];

  if (!isnan(a)) {
    values[num_not_nan] = a;
    num_not_nan++;
  }

  if (!isnan(b)) {
    values[num_not_nan] = b;
    num_not_nan++;
  }

  if (!isnan(c)) {
    values[num_not_nan] = c;
    num_not_nan++;
  }

  if (num_not_nan == 3) {
    // Sort the values.
    if (values[1] < values[0]) SwapInPlace(&values[1], &values[0]);
    if (values[2] < values[1]) SwapInPlace(&values[2], &values[1]);
    if (values[1] < values[0]) SwapInPlace(&values[1], &values[0]);
    return values[1];
  } else if (num_not_nan == 2) {
    // Handle the case where one value is -INFINITY and the other is INFINITY.
    if (!isfinite(values[0]) && !isfinite(values[1]) &&
        (values[0] > 0) != (values[1] > 0)) {
      return 0.0;
    } else {
      return MeanPair(values[0], values[1]);
    }
  } else if (num_not_nan == 1) {
    return values[0];
  } else {
    return 0.0;
  }
}

// Compute the component-wise median of a Vec3 (see Median3).
const Vec3 *Median3Vec3(const Vec3 *a, const Vec3 *b, const Vec3 *c, Vec3 *m) {
  assert(a != NULL && b != NULL && c != NULL && m != NULL);

  m->x = Median3(a->x, b->x, c->x);
  m->y = Median3(a->y, b->y, c->y);
  m->z = Median3(a->z, b->z, c->z);

  return m;
}

// Find an approximate geometric median of three rotations represented
// by unit quaternions.
//
// The approximate median is found by taking the median of the
// quaternions in R^4, then projecting back to the unit sphere.  As q
// and -q represent the same rotation, an initial step is taken to
// minimize the distance between the quaternions before taking the
// median.
//
// Args:
//   a, b, c: Unit quaternions to find the median of.
//   m: Output quaternion representing the median.
//
// Returns:
//   A pointer to m.
const Quat *Median3Quat(const Quat *a, const Quat *b, const Quat *c, Quat *m) {
  assert(a != NULL && b != NULL && c != NULL && m != NULL);
  int32_t num_not_nan = 0;
  Quat values[3];

  if (!QuatHasNaN(a)) {
    QuatNormalize(a, &values[num_not_nan]);
    num_not_nan++;
  }

  if (!QuatHasNaN(b)) {
    QuatNormalize(b, &values[num_not_nan]);
    num_not_nan++;
  }

  if (!QuatHasNaN(c)) {
    QuatNormalize(c, &values[num_not_nan]);
    num_not_nan++;
  }

  if (num_not_nan == 3) {
    PermuteQuatSignClosest3(&values[0], &values[1], &values[2], &values[0],
                            &values[1], &values[2]);

    m->q0 = Median3(values[0].q0, values[1].q0, values[2].q0);
    m->q1 = Median3(values[0].q1, values[1].q1, values[2].q1);
    m->q2 = Median3(values[0].q2, values[1].q2, values[2].q2);
    m->q3 = Median3(values[0].q3, values[1].q3, values[2].q3);

    QuatNormalize(m, m);
  } else if (num_not_nan == 2) {
    // Take the geometric mean of two quaternions.
    double dot = QuatDot(&values[0], &values[1]);
    QuatLinComb(1.0, &values[0], dot >= 0.0 ? 1.0 : -1.0, &values[1], m);
    QuatNormalize(m, m);
  } else if (num_not_nan == 1) {
    *m = values[0];
  } else {
    *m = kQuatIdentity;
  }

  return m;
}

// Permutes the signs of three quaternions to minimize the sum of the
// Euclidean distances between them.
//
// Args:
//   q_a, q_b, q_c: Quaternion inputs.
//   p_a, p_b, p_c: Quaternion outputs with p_a = +/- q_a, etc.
void PermuteQuatSignClosest3(const Quat *q_a, const Quat *q_b, const Quat *q_c,
                             Quat *p_a, Quat *p_b, Quat *p_c) {
  assert(q_a != NULL && q_b != NULL && q_c != NULL);
  assert(p_a != NULL && p_b != NULL && p_c != NULL);

  // Try each sign permutation in turn.
  double min_dist = INFINITY;
  bool flip_sign_a = false, flip_sign_b = false;
  for (int32_t i = 0; i < 2; ++i) {
    for (int32_t j = 0; j < 2; ++j) {
      Quat v_a, v_b;  // Variables to allow reuse of inputs as outputs.
      QuatScale(q_a, i == 1 ? -1.0 : 1.0, &v_a);
      QuatScale(q_b, j == 1 ? -1.0 : 1.0, &v_b);

      Quat tmp;
      double d = 0.0;
      d += QuatMod(QuatSub(&v_a, &v_b, &tmp));
      d += QuatMod(QuatSub(&v_b, q_c, &tmp));
      d += QuatMod(QuatSub(q_c, &v_a, &tmp));
      if (d < min_dist) {
        flip_sign_a = i;
        flip_sign_b = j;
        min_dist = d;
      }
    }
  }

  QuatScale(q_a, flip_sign_a ? -1.0 : 1.0, p_a);
  QuatScale(q_b, flip_sign_b ? -1.0 : 1.0, p_b);
  *p_c = *q_c;
}

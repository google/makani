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

// This file is the embedded analog of the controls filter library. At the
// moment, it contains a rate limit, first order filter, and anti-windup
// functions.
//
// FirstOrderFilter:
// The FirstOrderFilter functions implement a discrete first order IIR filter
// with a transfer function:
//
//   y(z)    b0 + b1 * z^-1
//   ---- = ----------------
//   u(z)    1  + a1 * z^-1
//
// The filter consists of a parameter structure of type FirstOrderFilterParams
// and a state structure of type FirstOrderFilterState, both of which are
// initialized with a call to FirstOrderFilterInit. Once initialized, the filter
// is updated by calling FirstOrderFilter.
//
// FirstOrderWindup:
// The FirstOrderWindup functions apply anti-windup on top of a
// FirstOrderFilter. The anti-windup parameters are chosen to acheive a modified
// (and generally faster) pole when the filter output exceeds a set of upper and
// lower limits. For the upper limit, the transfer function becomes:
//
//   y'   a1'    b0 + b1 * z^-1
//   -- = --- * ----------------
//   u'   a1     1 + a1' * z^-1
//
// where y' = y - y_lim is the excess above the limit and u' = u - u_lim
// = u - y_lim / k is the excess above the steady state input that would produce
// an output of y_lim.

#ifndef AVIONICS_COMMON_FAST_MATH_FILTER_H_
#define AVIONICS_COMMON_FAST_MATH_FILTER_H_

#include <assert.h>
#include <stdbool.h>
#include <float.h>
#include <math.h>

#include "avionics/common/fast_math/fast_math.h"

typedef struct {
  float b[2];
  float a[2];
  float k_inv;
  float k_aw;
} FirstOrderFilterParams;

typedef struct {
  float x;  // Last input.
  float y;  // Last output.
} FirstOrderFilterState;

// Initializes parameters and state for a discreted IIR first order filter with
// zero frequency gain k, a continuous time pole of s_pole, and a continuous
// time zero of s_zero when sampled at dt. Poles and zeros at the origin are
// treated specially when normalizing the filter; see the implementation for
// details. Any filter anti-windup is also reset.
void FirstOrderFilterInit(float k, float s_zero, float s_pole, float dt,
                          float y0, FirstOrderFilterParams *params,
                          FirstOrderFilterState *state);

// Resets the state in the first order filter such that the initial output of
// the filter is equal to y.
void FirstOrderFilterReset(float y0, const FirstOrderFilterParams *params,
                           FirstOrderFilterState *state);

// Get the last value output by the filter.
static inline float FirstOrderFilterGet(FirstOrderFilterState *state) {
  return state->y;
}

// Update the FirstOrderFilter state with the next sample x and return the
// filter output.
static inline float FirstOrderFilter(float x,
                                     const FirstOrderFilterParams *params,
                                     FirstOrderFilterState *state) {
  // Perform the Direct Form I update assuming that the denominator is monic.
  state->y
      = params->b[0] * x + params->b[1] * state->x - params->a[1] * state->y;
  state->x = x;
  return state->y;
}

// Initializes the anti-windup portion of params so that the transfer function
// y'/u' = (y - y_lim) / (u - u_lim) has a continuous time pole of s_pole. Note
// that the filter portion of params needs to have already been initialized.
void FirstOrderWindupInit(float s_pole, float dt,
                          FirstOrderFilterParams *params);

// Applies anti-windup to the filter state and returns the result. This function
// should be called after updating the filter state with FirstOrderFilter.
static inline float FirstOrderWindup(float y_lower_limit, float y_upper_limit,
                                     const FirstOrderFilterParams *params,
                                     FirstOrderFilterState *state) {
  float dy = state->y - Saturatef(state->y, y_lower_limit, y_upper_limit);
  state->y -= params->k_aw * dy;
  return state->y;
}

// Rate limit input u such that the derivative of the output is in
// [lower_rate_limit, upper_rate_limit]. The variable yz (i.e. y*z^-1) is the
// output from the previous time step and ts is the time step size.
float RateLimitf(float u, float lower_rate_limit, float upper_rate_limit,
                 float ts, float *yz);

// Transform a point s on the real axis of the s-plane to the z-plane using
// Tustin's method with sample time dt.
static inline float TustinSToZ(float s, float dt) {
  float sT = s * dt;
  if (fabsf(sT - 2.0f) > FLT_EPSILON) {
    return (sT > -FLT_MAX)
        ? (2.0f + sT) / (2.0f - sT)
        : -1.0f;  // lim_{s*T -> -inf} ((2+sT)/(2-sT)) = -1
  } else {
    assert(false);
    return FLT_MAX;
  }
}

#endif  // AVIONICS_COMMON_FAST_MATH_FILTER_H_

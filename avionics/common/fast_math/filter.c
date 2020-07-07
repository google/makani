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

#include "avionics/common/fast_math/filter.h"

#include <assert.h>
#include <float.h>
#include <limits.h>
#include <math.h>
#include <stdint.h>

#include "avionics/common/fast_math/fast_math.h"

void FirstOrderFilterInit(float k, float s_zero, float s_pole, float dt,
                          float y0, FirstOrderFilterParams *params,
                          FirstOrderFilterState *state) {
  assert(0.0f < dt);  // Zero and negative time steps are non-physical.
  assert(s_zero < INFINITY);  // Not supported; results in NaN coefficients.
  assert(s_pole < INFINITY);  // Not supported; results in NaN coefficients.

  // Start the divide 1.0f / k before it's needed to mitigate result latency.
  // Note that for k = 0, the numerator is identically zero; inputs are ignored
  // and it doesn't matter what x*z^-1 gets set to.
  float k_inv = fabsf(k) >= FLT_EPSILON ? 1.0f / k : 0.0f;

  // Reset anti-windup.
  params->k_aw = 0.0f;

  // Test if s_zero == s_pole (or is close).
  // FLT_EPSILON in the comparison is scaled by the magnitude of the pole or
  // zero, which ever is larger. The result is clipped to 0.1 FLT_MAX to deal
  // with an infinite pole or zero.
  if (fabsf(s_zero - s_pole)
      > 10.0f * FLT_EPSILON * Minf(Maxf(fabsf(s_zero), fabsf(s_pole)),
                                   0.1f * FLT_MAX)) {
    // The zero and pole are distinct.

    // Use Tustin's method to calculate the discrete poles and zeros, i.e.
    // z = (1 + sT/2) / (1 - sT/2). Prior to normalization, it is assumed that
    // a[0] = b[0] = 1. A direct consequence of this is that a[1] = -z_pole and
    // b[1] = -z_zero resulting in a transfer function which looks like:
    //
    //    b[0] + b[1]*z^-1     z - z_zero
    //   ------------------ = ------------
    //    a[0] + a[1]*z^-1     z - z_pole

    // Calculate the zero.
    float sT = s_zero * dt;
    assert(fabsf(sT - 2.0f) > FLT_EPSILON);
    params->b[1] = (sT > -FLT_MAX)
        ? -(2.0f + sT) / (2.0f - sT)
        : 1.0f;  // lim_{s*T -> -inf} (-(2+sT)/(2-sT)) = 1

    // Calculate the pole.
    sT = s_pole * dt;
    assert(fabsf(sT - 2.0f) > FLT_EPSILON);
    params->a[1] = (sT > -FLT_MAX)
        ? -(2.0f + sT) / (2.0f - sT)
        : 1.0f;  // lim_{s*T -> -inf} (-(2+sT)/(2-sT)) = 1

    // Normalize the filter.
    if (fabsf(1.0f + params->a[1]) < 10.0f * FLT_EPSILON) {
      // The filter is an integrator so normalize it such that
      // int_0^x k dx = k*x.
      params->k_inv = 0.0f;
      k *= dt / (1.0f + params->b[1]);
    } else if (fabsf(1.0f + params->b[1]) < 10.0f * FLT_EPSILON) {
      // The filter has a derivative in the numerator.
      params->k_inv = 0.0f;
      k *= (1.0f + params->a[1]) / dt;
    } else {
      // Normalize to have an asymptotic value of k to the step function u(x).
      params->k_inv = k_inv;
      k *= (1.0f + params->a[1]) / (1.0f + params->b[1]);
    }

    // Apply the gains to the numerator leaving the denominator as
    // 1 - z_pole * z^-1.
    params->b[0] = k;  // b[0]*k = 1*k = k.
    params->b[1] *= k;
    params->a[0] = 1.0f;
  } else {
    // The pole and zero cancel so simply pass the input through with some gain.
    // This branch is primarily intended for debugging.
    params->k_inv = k_inv;
    params->b[0] = k;
    params->b[1] = 0.0f;
    params->a[0] = 1.0f;
    params->a[1] = 0.0f;
  }

  // Set the filter state to output y0.
  FirstOrderFilterReset(y0, params, state);
}

// Resets the state in a single pole filter such that the initial output of the
// filter is equal to y.
void FirstOrderFilterReset(float y, const FirstOrderFilterParams *params,
                           FirstOrderFilterState *state) {
  state->x = y * params->k_inv;
  state->y = y;
}

// The anti-windup filter is implemented by subtracting k_aw*(y - y_lim) from a
// filter output. The resulting transfer function of the excess, i.e. y' = y -
// y_lim and u' = u - y_lim / k, becomes:
//
//   y'   (1 - k_aw) * (b[0] + b[1]*z^-1)
//   -- = -------------------------------
//   u'     a[0] + a[1]*(1 - k_aw)*z^-1
//
// Assuming that the denominator is monic and matching poles, we have a[1]*(1 -
// k_aw) = z_aw_pole or k_aw = 1 - z_aw_pole / a[1]. For situations where a[1]
// is near zero, the filter is already extremely fast and it's reasonable to
// simply saturate the output by setting k_aw = 1.
void FirstOrderWindupInit(float s_pole, float dt,
                          FirstOrderFilterParams *params) {
  float z_aw_pole = TustinSToZ(s_pole, dt);
  params->k_aw = (fabsf(params->a[1]) > 10.0f * FLT_EPSILON)
      ? 1.0f + z_aw_pole / params->a[1]
      : 1.0f;
}

// Rate limit input u such that the derivative of the output is in
// [lower_rate_limit, upper_rate_limit].
float RateLimitf(float u, float lower_rate_limit, float upper_rate_limit,
                 float ts, float *yz) {
  assert(upper_rate_limit >= lower_rate_limit);

  *yz = *yz + Saturatef(u - *yz, lower_rate_limit*ts, upper_rate_limit*ts);
  return *yz;
}

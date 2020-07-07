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

#ifndef AVIONICS_COMMON_FAST_MATH_FAST_MATH_H_
#define AVIONICS_COMMON_FAST_MATH_FAST_MATH_H_

#include <assert.h>
#include <stdint.h>
#include <stddef.h>

#define PI_F 3.1415927f

// Evaluates the piecewise linear function:
//
//       { low             x <= low
//   x = { x       low  <  x <  high
//       { high    high <= x
static inline float Saturatef(float x, float low, float high) {
  assert(low <= high);
  return x < low ? low : (x > high ? high : x);
}

// Returns the maximum of [x, y]. Unlike fmaxf, there is no checking to
// determine whether one or both arguments are NaN making this implementation
// substantially faster.
static inline float Maxf(float x, float y) {
  return x > y ? x : y;
}

// Returns the minimum of [x, y]. Unlike fminf, there is no checking to
// determine whether one or both arguments are NaN making this implementation
// substantially faster.
static inline float Minf(float x, float y) {
  return x < y ? x : y;
}

// Swap the values of the two floats x and y.
static inline void Swapf(float *x, float *y) {
  float tmp = *x;
  *x = *y;
  *y = tmp;
}

// Returns the sign of x:
//
//       { -1    x < 0
//   x = {  0    x = 0
//       {  1    x > 0
static inline float Signf(float x) {
  return (float)((0.0f < x) - (x < 0.0f));
}

// Same as Signf, except that an integer is returned.
static inline int32_t ISignf(float x) {
  return (0.0f < x) - (x < 0.0f);
}

// Returns the integer value that is nearest to x, with halfway cases rounded
// away from zero. No bounds checking is performed.
static inline int32_t IRoundf(float x) {
  return (int32_t)(x >= 0.0f ? x + 0.5f : x - 0.5f);
}

// Wraps angle into the range [-pi, pi). This function is cheaper than
// ModAngle for small angles, but becomes more expensive as |angle| increases.
// angle is required to be within (-6*pi, 6*pi).
static inline float WrapAngle(float angle) {
  assert(-6.0f * PI_F <= angle && angle < 6.0f * PI_F);

  while (angle >= PI_F) {
    angle -= 2.0f * PI_F;
  }
  while (angle < -PI_F) {
    angle += 2.0f * PI_F;
  }
  return angle;
}

// Wraps angle into the range [-pi, pi]. Unlike WrapAngle, the cost of ModAngle
// is mostly independent of the magnitude of angle.
static inline float ModAngle(float angle) {
  float revs = angle * (1.0f / (2.0f * PI_F));
  angle -= (float)((int32_t)revs) * (2.0f * PI_F);

  if (angle > PI_F) {
    return angle - 2.0f * PI_F;
  } else if (angle < -PI_F) {
    return angle + 2.0f * PI_F;
  } else {
    return angle;
  }
}
// Returns the index and fractional remainder of a value in a uniform
// lookup table.  The table's indices are assumed to be between 0 and
// n - 1.  This is essentially modff except that it returns the
// integral component as an integer rather than a float and it
// saturates the index between 0 and n - 2.
static inline int32_t UniformInterp1Index(float xi, int32_t n, float *frac) {
  assert(frac != NULL);

  int32_t index = (int32_t)xi;
  if (index < 0) {
    assert(0);
    index = 0;
  } else if (index > n - 2) {
    assert(index < n);
    index = n - 2;
  }

  *frac = xi - (float)index;
  return index;
}

// Linear interpolation of a uniformly sampled function y.  The x
// values corresponding to the y values are assumed to be: 0, 1, ...,
// n - 1.
static inline float UniformInterp1(const float y[], int32_t n, float xi) {
  assert(y != NULL);
  assert(n >= 2);

  float frac;
  int32_t index = UniformInterp1Index(xi, n, &frac);

  return y[index] + frac * (y[index + 1] - y[index]);
}

// Cross fades two input signals y0 and y1 using saturated linear interpolation
// based on a third input control value x, i.e.
//
//                { y0                   x <= x_low
//   Crossfadef = { y_interp    x_low  < x <= x_high
//                { y1          x_high < x
//
//   y_interp(x) = (1 - f(x))*y0 + f(x)*y1
//
//   f(x) = (x - x_low) / (x_high - x_low)
//
// This is the single precision analog of Crossfade.
static inline float Crossfadef(float y0, float y1,
                               float x, float x_low, float x_high) {
  float dx = x_high - x_low;
  assert(dx >= 0.0f);

  float dx0 = x - x_low;
  float dx1 = x_high - x;

  if (dx0 <= 0.0f) {  // Return the low value.
    return y0;
  } else if (dx1 <= 0.0f) {  // Return the high value.
    return y1;
  } else {  // Return an interpolated value.
    // If dx == 0.0f, i.e. x_low == x_high, we have dx0 == -dx1. Thus, one of
    // the first two branches is always taken and we cannot divide by zero. As
    // an added benefit versus other implementations, we make no assumptions on
    // the magnitude of x_low and x_high; if both values are smaller than, e.g.
    // FLT_EPSILON, the interpolation is still done correctly.
    return (dx1 * y0 + dx0 * y1) / dx;
  }
}

// Calculates the atan2 function using a lookup table.  This exploits
// the symmetry of the atan2 function by mapping the inputs to the
// first octant.  Returns a value in the range [-pi, pi].
//
// NOTE: For speed reasons, this does not handle all the
// edge cases the same as the standard atan2.  The differences are
// documented in fast_math_test.cc.
float Atan2Lookup(float y, float x);

// Calculates the sine function using a lookup table.  This exploits
// the symmetry of the sine function by mapping the input angle to the
// first quadrant.  Returns values in the range [-1, 1].
float SinLookup(float angle);

// Calculates the sine and cosine functions simultaneously using a
// lookup table.  This exploits the symmetry of the sine and cosine
// functions by mapping the input angle to the first quadrant.
// Returns values in the range [-1, 1].
void SinCosLookup(float angle, float *sin_val, float *cos_val);

#endif  // AVIONICS_COMMON_FAST_MATH_FAST_MATH_H_

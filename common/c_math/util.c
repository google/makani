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

#include "common/c_math/util.h"

#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "common/c_math/cal_params.h"
#include "common/c_math/mat3.h"
#include "common/c_math/vec2.h"
#include "common/c_math/vec3.h"

int32_t MinInt32(int32_t x, int32_t y) { return (x < y) ? x : y; }
int32_t MaxInt32(int32_t x, int32_t y) { return (x > y) ? x : y; }

uint32_t MinUint32(uint32_t x, uint32_t y) { return (x < y) ? x : y; }
uint32_t MaxUint32(uint32_t x, uint32_t y) { return (x > y) ? x : y; }

int64_t MinInt64(int64_t x, int64_t y) { return (x < y) ? x : y; }
int64_t MaxInt64(int64_t x, int64_t y) { return (x > y) ? x : y; }

uint64_t MinUint64(uint64_t x, uint64_t y) { return (x < y) ? x : y; }
uint64_t MaxUint64(uint64_t x, uint64_t y) { return (x > y) ? x : y; }

uint32_t MaxUnsignedValue(int32_t bits) {
  if (bits <= 0) {
    return 0U;
  } else if (bits < 32) {
    return (1U << bits) - 1U;
  } else {
    return UINT32_MAX;
  }
}

int32_t MinSignedValue(int32_t bits) {
  if (bits <= 0) {
    return 0;
  } else if (bits < 32) {
    return -(1 << (bits - 1));
  } else {
    return INT32_MIN;
  }
}

int32_t MaxSignedValue(int32_t bits) {
  if (bits <= 0) {
    return 0;
  } else if (bits < 32) {
    return (1 << (bits - 1)) - 1;
  } else {
    return INT32_MAX;
  }
}

int32_t Sign(double x) { return (0.0 < x) - (x < 0.0); }
int32_t SignInt32(int32_t x) { return (0 < x) - (x < 0); }

bool IsApproximatelyEqual(double x, double y) {
  return fabs(x - y) <= (fabs(x) < fabs(y) ? fabs(y) : fabs(x)) * DBL_EPSILON;
}

bool IsApproximatelyEqualVec3(const Vec3 *u, const Vec3 *v) {
  assert(u != NULL && v != NULL);
  return IsApproximatelyEqual(u->x, v->x) && IsApproximatelyEqual(u->y, v->y) &&
         IsApproximatelyEqual(u->z, v->z);
}

static int32_t GreaterThan(const void *a, const void *b) {
  return *(const double *)a > *(const double *)b;
}

static int32_t LessThan(const void *a, const void *b) {
  return *(const double *)a < *(const double *)b;
}

static int32_t GreaterThanInt32(const void *a, const void *b) {
  return *(const int32_t *)a > *(const int32_t *)b;
}

static int32_t GreaterThanInt64(const void *a, const void *b) {
  return *(const int64_t *)a > *(const int64_t *)b;
}

static int32_t GreaterThanUint32(const void *a, const void *b) {
  return *(const uint32_t *)a > *(const uint32_t *)b;
}

static const void *MaxArrayAny(const void *x, int32_t n, int32_t size,
                               int32_t (*comp)(const void *, const void *),
                               int32_t *maxind) {
  assert(x != NULL);
  assert(n > 0);
  assert(size > 0);
  assert(comp != NULL);

  const uint8_t *curr = x, *max = x;

  if (maxind != NULL) {
    *maxind = 0;
  }

  for (int32_t i = 1; i < n; ++i) {
    curr += size;
    if (comp(curr, max) > 0) {
      max = curr;
      if (maxind != NULL) {
        *maxind = i;
      }
    }
  }
  return max;
}

double MaxArray(const double *x, int32_t n, int32_t *maxind) {
  return *(const double *)MaxArrayAny(x, n, (int32_t)sizeof(*x), &GreaterThan,
                                      maxind);
}

double MinArray(const double *x, int32_t n, int32_t *minind) {
  return *(const double *)MaxArrayAny(x, n, (int32_t)sizeof(*x), &LessThan,
                                      minind);
}

int32_t MaxArrayInt32(const int32_t *x, int32_t n, int32_t *maxind) {
  return *(const int32_t *)MaxArrayAny(x, n, (int32_t)sizeof(*x),
                                       &GreaterThanInt32, maxind);
}

int64_t MaxArrayInt64(const int64_t *x, int32_t n, int32_t *maxind) {
  return *(const int64_t *)MaxArrayAny(x, n, (int32_t)sizeof(*x),
                                       &GreaterThanInt64, maxind);
}

uint32_t MaxArrayUint32(const uint32_t *x, int32_t n, int32_t *maxind) {
  return *(const uint32_t *)MaxArrayAny(x, n, (int32_t)sizeof(*x),
                                        &GreaterThanUint32, maxind);
}

double VarArray(const double *x, int32_t n) {
  if (n == 0) return 0.0;
  assert(x != NULL);

  // The simplest "one-pass" formula for computing variance is
  // numerically unstable.  A two-pass formula is still more numerically
  // stable, but computationally expensive.
  // See: N. Highham, Accuracy and Stability of Numerical Algorithms, 2nd Ed.
  //        Philadelphia: SIAM, 2002, ch. 1, sec. 1.9. pp. 12-13.
  double m = 0.0, q = 0.0;
  for (int32_t i = 0; i < n; ++i) {
    double delta = x[i] - m;
    q += (i * (delta * delta)) / (i + 1);
    m += delta / (i + 1);
  }
  assert(q >= 0.0);
  return fmax(q / n, 0.0);
}

double MeanPair(double x, double y) {
  if (Sign(x) == Sign(y) && isfinite(x) && isfinite(y)) {
    return x + (y - x) / 2.0;
  } else {
    // If a and b are finite with mismatched signs, a + b cannot
    // overflow.  When infinite values are present the standard formula
    // is used.
    return (x + y) / 2.0;
  }
}

double MeanArray(const double *x, int32_t n) {
  assert(x != NULL && n > 0);
  double sum = x[0];
  for (int32_t i = 1; i < n; ++i) {
    sum += x[i];
  }
  return sum / n;
}

void SwapInPlace(double *x, double *y) {
  assert(x != NULL && y != NULL);
  double tmp = *x;
  *x = *y;
  *y = tmp;
}

void SwapInPlacef(float *x, float *y) {
  assert(x != NULL && y != NULL);
  float tmp = *x;
  *x = *y;
  *y = tmp;
}

double Saturate(double x, double low, double high) {
  assert(low <= high);
  return fmin(fmax(x, low), high);
}

bool IsSaturated(double x, double low, double high) {
  assert(low <= high);
  return x <= low || high <= x;
}

const Vec2 *SaturateVec2(const Vec2 *x, const Vec2 *low, const Vec2 *high,
                         Vec2 *y) {
  y->x = Saturate(x->x, low->x, high->x);
  y->y = Saturate(x->y, low->y, high->y);
  return y;
}

const Vec3 *SaturateVec3(const Vec3 *x, const Vec3 *low, const Vec3 *high,
                         Vec3 *y) {
  y->x = Saturate(x->x, low->x, high->x);
  y->y = Saturate(x->y, low->y, high->y);
  y->z = Saturate(x->z, low->z, high->z);
  return y;
}

const Vec3 *SaturateVec3ByScalar(const Vec3 *x, double low, double high,
                                 Vec3 *y) {
  y->x = Saturate(x->x, low, high);
  y->y = Saturate(x->y, low, high);
  y->z = Saturate(x->z, low, high);
  return y;
}

const Vec *SaturateVec(const Vec *x, const Vec *low, const Vec *high, Vec *y) {
  assert(x->length == y->length);
  for (int32_t i = 0; i < x->length; ++i) {
    y->d[i] = Saturate(x->d[i], low->d[i], high->d[i]);
  }
  return y;
}

const double *SaturateArrayByScalar(const double *x, int32_t n, double low,
                                    double high, double *y) {
  for (int32_t i = 0; i < n; ++i) {
    y[i] = Saturate(x[i], low, high);
  }
  return y;
}

int32_t SaturateInt32(int32_t x, int32_t low, int32_t high) {
  assert(low <= high);
  return MinInt32(MaxInt32(x, low), high);
}

uint32_t SaturateUint32(uint32_t x, uint32_t low, uint32_t high) {
  assert(low <= high);
  return MinUint32(MaxUint32(x, low), high);
}

int64_t SaturateInt64(int64_t x, int64_t low, int64_t high) {
  assert(low <= high);
  return MinInt64(MaxInt64(x, low), high);
}

uint64_t SaturateUint64(uint64_t x, uint64_t low, uint64_t high) {
  assert(low <= high);
  return MinUint64(MaxUint64(x, low), high);
}

int32_t SaturateSigned(int32_t x, int32_t bits) {
  return SaturateInt32(x, MinSignedValue(bits), MaxSignedValue(bits));
}

uint32_t SaturateUnsigned(uint32_t x, int32_t bits) {
  return SaturateUint32(x, 0U, MaxUnsignedValue(bits));
}

double SaturateWrapped(double x, double range_start, double range_end,
                       double wrap_left, double wrap_right) {
  // All inputs must be between wrap_left and wrap_right.
  assert(x >= wrap_left && x <= wrap_right);
  assert(range_start >= wrap_left && range_start <= wrap_right);
  assert(range_end >= wrap_left && range_end <= wrap_right);
  assert(range_start != range_end);

  double wrap_range = wrap_right - wrap_left;

  double x_sat;
  if (range_end > range_start) {
    x_sat = Saturate(x, range_start, range_end);
  } else {
    // Range crosses wrap, so unwrap range_end.
    if (x < range_end) {
      // x is in range, both x and range_end need to be unwrapped.
      x_sat = Saturate(x + wrap_range, range_start, range_end + wrap_range);
    } else {
      // x may be in range without unwrap.
      x_sat = Saturate(x, range_start, range_end + wrap_range);
    }
  }

  // Find shortest distance from x to range_start and range_end.
  double x_start_dist =
      fmin(fabs(x - range_start), fabs(x + wrap_range - range_start));
  double x_end_dist =
      fmin(fabs(x - range_end), fabs(x + wrap_range - range_end));

  // Saturation may not pick closest end of range.
  // If saturated, check that it picked closest end.
  if (x_sat == range_start && x_end_dist < x_start_dist) {
    x_sat = range_end;
  } else if (x_sat == range_end && x_start_dist <= x_end_dist) {
    x_sat = range_start;
  }

  return Wrap(x_sat, wrap_left, wrap_right);
}

const Vec3 *FabsVec3(const Vec3 *v_in, Vec3 *v_out) {
  assert(v_in != NULL && v_out != NULL);

  v_out->x = fabs(v_in->x);
  v_out->y = fabs(v_in->y);
  v_out->z = fabs(v_in->z);
  return v_out;
}

double Mix(double x0, double x1, double c) {
  double c_sat = Saturate(c, 0.0, 1.0);
  return (1.0 - c_sat) * x0 + c_sat * x1;
}

double Crossfade(double y0, double y1, double x, double x_low, double x_high) {
  assert(x_low <= x_high);
  return Mix(y0, y1, (x - x_low) / fmax(x_high - x_low, DBL_MIN));
}

const Vec2 *CrossfadeVec2(const Vec2 *y0, const Vec2 *y1, double x,
                          double x_low, double x_high, Vec2 *y_out) {
  assert(y0 != NULL && y1 != NULL && y_out != NULL);
  y_out->x = Crossfade(y0->x, y1->x, x, x_low, x_high);
  y_out->y = Crossfade(y0->y, y1->y, x, x_low, x_high);
  return y_out;
}

const Vec3 *CrossfadeVec3(const Vec3 *y0, const Vec3 *y1, double x,
                          double x_low, double x_high, Vec3 *y_out) {
  assert(y0 != NULL && y1 != NULL && y_out != NULL);
  y_out->x = Crossfade(y0->x, y1->x, x, x_low, x_high);
  y_out->y = Crossfade(y0->y, y1->y, x, x_low, x_high);
  y_out->z = Crossfade(y0->z, y1->z, x, x_low, x_high);
  return y_out;
}

const Mat3 *CrossfadeMat3(const Mat3 *y0, const Mat3 *y1, double x,
                          double x_low, double x_high, Mat3 *y_out) {
  assert(y0 != NULL && y1 != NULL && y_out != NULL);
  CrossfadeArray(&y0->d[0][0], &y1->d[0][0], 9, x, x_low, x_high,
                 &y_out->d[0][0]);
  return y_out;
}

const double *CrossfadeArray(const double *y0, const double *y1, int32_t n,
                             double x, double x_low, double x_high,
                             double *y_out) {
  assert(y0 != NULL && y1 != NULL && y_out != NULL);
  assert(n > 0);

  for (int32_t i = 0; i < n; ++i) {
    y_out[i] = Crossfade(y0[i], y1[i], x, x_low, x_high);
  }
  return y_out;
}

double InterpIndex(const double x[], int32_t n, double x_i, InterpOption opt,
                   int32_t *ind) {
  assert(x != NULL);
  assert(n >= 2);

  // Finds the highest index less than n - 1 for which x_i >=
  // x[index].
  int32_t index = n - 2;
  for (int32_t i = 1; i < n - 1; ++i) {
    assert(x[i] > x[i - 1]);
    if (x_i < x[i]) {
      index = i - 1;
      break;
    }
  }

  double x_low = x[index];
  double x_high = x[index + 1];
  double fractional_ind = (double)index + (x_i - x_low) / (x_high - x_low);

  if (opt & kInterpOptionSaturate) {
    fractional_ind = Saturate(fractional_ind, 0.0, (double)(n - 1));
  }

  if (ind != NULL) *ind = index;

  return fractional_ind;
}

double Interp1(const double x[], const double y[], int32_t n, double x_i,
               InterpOption opt) {
  assert(x != NULL && y != NULL);

  int32_t s0;
  double s = InterpIndex(x, n, x_i, opt, &s0);
  double z0 = y[s0];
  double z1 = y[s0 + 1];

  return z0 + (z1 - z0) * (s - (double)s0);
}

double Interp1WarpY(const double x[], const double y[], int32_t n, double x_i,
                    InterpOption opt, double (*warp_func)(double),
                    double (*unwarp_func)(double)) {
  assert(x != NULL && y != NULL);
  assert(warp_func != NULL && unwarp_func != NULL);
  assert(n >= 2);

  int32_t s0;
  double s = InterpIndex(x, n, x_i, opt, &s0);
  double z0 = warp_func(y[s0]);
  double z1 = warp_func(y[s0 + 1]);

  return unwarp_func(z0 + (z1 - z0) * (s - (double)s0));
}

double Interp2(const double x[], const double y[], int32_t nx, int32_t ny,
               const double *z, double x_i, double y_i, InterpOption opt) {
  // Find the rows above and below the target coordinates.  Here s0 is the
  // index of the row above the target.
  int32_t s0;
  double s = InterpIndex(y, ny, y_i, opt, &s0);

  // For the rows directly above and below the datapoint, interpolate
  // "horizontally" to find the value at the target x-coordinate.
  double z0 = Interp1(x, z + s0 * nx, nx, x_i, opt);
  double z1 = Interp1(x, z + (s0 + 1) * nx, nx, x_i, opt);

  // Interpolate "vertically" between z0 and z1.
  return z0 + (z1 - z0) * (s - (double)s0);
}

// Interpolates a periodic function defined by a lookup table.  The input x_i is
// wrapped into the lookup table's domain.
double CircularInterp1(const double x[], const double y[], int32_t n,
                       double x_i) {
  assert(n > 0);
  assert(y != NULL && y[0] == y[n - 1]);
  assert(x != NULL);
  x_i = Wrap(x_i, x[0], x[n - 1]);
  return Interp1(x, y, n, x_i, kInterpOptionSaturate);
}

void Interp1Vec3(const double x[], const Vec3 y[], int32_t n, double x_i,
                 InterpOption opt, Vec3 *y_out) {
  int32_t s0;
  double s = InterpIndex(x, n, x_i, opt, &s0);

  Vec3LinComb(1.0 - (s - (double)s0), &y[s0], s - (double)s0, &y[s0 + 1],
              y_out);
}

double Sigmoid(double x, double width) {
  return 0.5 + 1.0 / PI * atan2(x, width / 6.31 / 2.0);
}

void PolyFit2(const double x[3], const double y[3], double coeff[3]) {
  Mat3 v = {{{x[0] * x[0], x[0], 1.0},
             {x[1] * x[1], x[1], 1.0},
             {x[2] * x[2], x[2], 1.0}}};
  Vec3 p = {y[0], y[1], y[2]};
  Mat3Vec3Mult(Mat3Inv(&v, &v), &p, &p);
  coeff[0] = p.x;
  coeff[1] = p.y;
  coeff[2] = p.z;
}

double PolyVal(const double c[], double x, int32_t n) {
  assert(c != NULL);
  assert(n > 0);
  double b = 0.0;
  for (int32_t i = 0; i < n; ++i) {
    b = c[i] + x * b;
  }
  return b;
}

void PolyDer(const double c[], int32_t n, double dc[]) {
  assert(c != NULL && dc != NULL);
  assert(n > 0);
  for (int32_t i = 0; i < n; ++i) {
    dc[i] = c[i] * (n - i);
  }
}

// TODO: Change bias sign convention so that it's easier to
// just read off the bias value (both *Cal and *EncoderCal).
double ApplyCal(double raw_val, const CalParams *cal) {
  return cal->scale * (raw_val + (double)cal->bias_count) + cal->bias;
}

float ApplyCal32(float raw_val, const CalParams32 *cal) {
  return cal->scale * (raw_val + (float)cal->bias_count) + cal->bias;
}

// TODO: Consider including saturation as a part of InvertCal/CalParams.
double InvertCal(double cal_val, const CalParams *cal) {
  return (cal_val - cal->bias) / cal->scale - (double)cal->bias_count;
}

float InvertCal32(float cal_val, const CalParams32 *cal) {
  return (cal_val - cal->bias) / cal->scale - (float)cal->bias_count;
}

double ApplyEncoderCal(int32_t raw_val, const EncoderCalParams *ecal) {
  assert(0 <= raw_val && raw_val <= ecal->encoder_counts);
  assert(ecal->cal.scale != 0.0 && ecal->encoder_counts > 0);

  double cal_val_range = fabs(ecal->cal.scale * ecal->encoder_counts);
  double cal_val = fmod(ApplyCal(raw_val, &ecal->cal), cal_val_range);

  if (cal_val < ecal->cal_val_center - cal_val_range / 2.0) {
    cal_val += cal_val_range;
  }
  if (cal_val > ecal->cal_val_center + cal_val_range / 2.0) {
    cal_val -= cal_val_range;
  }
  return cal_val;
}

int32_t InvertEncoderCal(double cal_val, const EncoderCalParams *ecal) {
  assert(ecal->encoder_counts > 0);

  int32_t raw_int =
      (int32_t)InvertCal(cal_val, &ecal->cal) % ecal->encoder_counts;
  if (raw_int < 0) {
    raw_int += ecal->encoder_counts;
  }
  return raw_int;
}

double Wrap(double x, double left, double right) {
  assert(left < right);  // Respect the number line.

  double wrap0 = fmod(x - left, right - left);
  if (wrap0 >= 0.0)
    return wrap0 + left;
  else
    return wrap0 + right;
}

int32_t WrapInt32(int32_t x, int32_t left, int32_t right) {
  assert(left < right);  // Respect the number line.

  int32_t wrap0 = (x - left) % (right - left);
  if (wrap0 >= 0) {
    wrap0 += left;
  } else {
    wrap0 += right;
  }
  return wrap0;
}

bool InsideRange(double x, double min, double max) {
  return min <= x && x < max;
}

bool InsideRangeWrapped(double x, double left, double right, double min,
                        double max) {
  // All values must be within [left, right).
  assert(left <= min && left <= max && min <= right && max <= right);
  assert(left <= x && x < right);
  assert(left < right);

  if (min < max) {
    // Min less than max -- valid range does not wrap.
    return min <= x && x < max;
  } else if (min > max) {
    // Max less than min -- valid range wraps from right to left.
    return min <= x || x < max;
  } else {
    // If min and max are equal, accept all values.
    return true;
  }
}

double Asin(double x) { return asin(Saturate(x, -1.0, 1.0)); }

double Acos(double x) { return acos(Saturate(x, -1.0, 1.0)); }

double Sqrt(double x) { return sqrt(fmax(x, 0.0)); }

double Square(double x) { return x * x; }

double ThirdPower(double x) { return x * x * x; }

double FourthPower(double x) {
  x *= x;
  x *= x;
  return x;
}

double Exp10(double x) {
  // The numeric literal here is the natural logarithm of 10; defined in
  // <math.h> as M_LN10.
  return exp(x * 2.30258509299404568402);
}

const int32_t *Slice(int32_t start, int32_t incr, int32_t end, int32_t length,
                     int32_t *arr) {
  assert(start >= 0 && end >= 0 && length >= 0);
  assert(end <= length);
  memset(arr, 0, sizeof(int32_t) * (size_t)length);
  for (int32_t i = start; i < end; i += incr) {
    arr[i] = 1;
  }
  return arr;
}

void SplitVec3Arr(const Vec3 vs[], int32_t n, double xs[], double ys[],
                  double zs[]) {
  for (int32_t i = 0; i < n; ++i) {
    xs[i] = vs[i].x;
    ys[i] = vs[i].y;
    zs[i] = vs[i].z;
  }
}

void JoinVec3Arr(const double xs[], const double ys[], const double zs[],
                 int32_t n, Vec3 vs[]) {
  for (int32_t i = 0; i < n; ++i) {
    vs[i].x = xs[i];
    vs[i].y = ys[i];
    vs[i].z = zs[i];
  }
}

double DegToRad(double deg) { return deg / 180.0 * PI; }

double RadToDeg(double rad) { return rad / PI * 180.0; }

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

#include "common/c_math/filter.h"

#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "common/c_math/util.h"
#include "common/c_math/vec3.h"
#include "common/macros.h"

double RunningVar(double x, int32_t n, double x_buf[], int32_t *ind) {
  assert(ind != NULL && x_buf != NULL);
  assert(n > 0);
  assert(0 <= *ind && *ind < n);

  // Update the buffer and index.
  x_buf[*ind] = x;
  *ind = (*ind + 1) % n;

  double var = VarArray(x_buf, n);
  assert(var >= 0.0);
  return fmax(var, 0.0);
}

// If 2*pi*ts*fc == 1, then no filtering occurs (y_out = u).
// If 2*pi*ts*fc > 1, then chattering or instability occurs.
double Lpf(double u, double fc, double ts, double *y_z1) {
  assert(2.0 * PI * ts * fc <= 1.0);
  assert(fc >= 0.0);
  assert(y_z1 != NULL);

  *y_z1 += Saturate(2.0 * PI * ts * fc, 0.0, 1.0) * (u - *y_z1);
  return *y_z1;
}

const Vec3 *LpfVec3(const Vec3 *u, double fc, double ts, Vec3 *y_z1) {
  Lpf(u->x, fc, ts, &y_z1->x);
  Lpf(u->y, fc, ts, &y_z1->y);
  Lpf(u->z, fc, ts, &y_z1->z);
  return y_z1;
}

// TODO: Not a perfect complement to Lpf because of
// approximation.
double Hpf(double u, double fc, double ts, double *y_z1, double *u_z1) {
  assert(fc >= 0.0);
  assert(2.0 * PI * ts * fc <= 1.0);  // See comment in Lpf.
  assert(y_z1 != NULL && u_z1 != NULL && y_z1 != u_z1);

  *y_z1 = (1.0 - 2.0 * PI * ts * fc) * *y_z1 + (u - *u_z1);
  *u_z1 = u;
  return *y_z1;
}

double Diff(double u, double ts, double *u_z1) {
  assert(ts > DBL_TOL);
  assert(u_z1 != NULL);

  double du = (u - *u_z1) / ts;
  *u_z1 = u;
  return du;
}

const Vec3 *DiffVec3(const Vec3 *u, double ts, Vec3 *du, Vec3 *u_z1) {
  assert(u != NULL && du != NULL && u_z1 != NULL && du != u_z1);

  du->x = Diff(u->x, ts, &u_z1->x);
  du->y = Diff(u->y, ts, &u_z1->y);
  du->z = Diff(u->z, ts, &u_z1->z);
  return du;
}

double DiffCircular(double u, double wrap_range, double ts, double *u_z1) {
  assert(wrap_range > 0.0);
  assert(ts > DBL_TOL);
  assert(u_z1 != NULL);
  assert(fabs(u - *u_z1) <= wrap_range);

  double du = Wrap(u - *u_z1, -0.5 * wrap_range, 0.5 * wrap_range) / ts;
  *u_z1 = u;
  return du;
}

double RateLimit(double u, double low, double high, double ts, double *y_z1) {
  assert(high >= low);
  assert(ts > 0.0);
  assert(y_z1 != NULL);

  *y_z1 += Saturate(u - *y_z1, low * ts, high * ts);
  return *y_z1;
}

int32_t RateLimitInt32(int32_t u, int32_t low, int32_t high, double ts,
                       int32_t *y_z1) {
  assert(high >= low);
  assert(ts > 0.0);
  assert(y_z1 != NULL);

  *y_z1 += SaturateInt32(u - *y_z1, (int32_t)floor((double)low * ts),
                         (int32_t)ceil((double)high * ts));
  return *y_z1;
}

const Vec3 *RateLimitVec3(const Vec3 *u, const Vec3 *low, const Vec3 *high,
                          double ts, Vec3 *y_z1) {
  assert(u != NULL && low != NULL && high != NULL && y_z1 != NULL);
  assert(y_z1 != u && y_z1 != low && y_z1 != high);

  RateLimit(u->x, low->x, high->x, ts, &y_z1->x);
  RateLimit(u->y, low->y, high->y, ts, &y_z1->y);
  RateLimit(u->z, low->z, high->z, ts, &y_z1->z);
  return y_z1;
}

double RateLimitCircular(double u, double low_rate, double high_rate,
                         double low_wrap, double high_wrap, double ts,
                         double *y_z1) {
  assert(low_rate <= high_rate);
  assert(low_wrap <= high_wrap);
  assert(ts > 0.0);
  assert(fabs(low_rate * ts) <= high_wrap - low_wrap);
  assert(fabs(high_rate * ts) <= high_wrap - low_wrap);
  assert(y_z1 != NULL && low_wrap <= *y_z1 && *y_z1 < high_wrap);

  double increment = Saturate(Wrap(u - *y_z1, low_wrap, high_wrap),
                              low_rate * ts, high_rate * ts);
  *y_z1 = Wrap(*y_z1 + increment, low_wrap, high_wrap);
  return *y_z1;
}

double Filter(double u, int32_t n, const double a[], const double b[],
              double z[]) {
  assert(n > 0);
  assert(a != NULL && b != NULL && z != NULL);
  assert(z != a && z != b);
  assert(fabs(a[0] - 1.0) < DBL_TOL);

  double y = 0.0;
  double z_tmp = u;
  for (int32_t i = 1; i < n; ++i) {
    z_tmp -= a[i] * z[i - 1];
    y += b[i] * z[i - 1];
  }
  y += b[0] * z_tmp;

  // Shift delay line.
  for (int32_t i = n - 2; i > 0; --i) {
    z[i] = z[i - 1];
  }
  z[0] = z_tmp;
  return y;
}

double FilterCircularBuffer(double u, int32_t n, const double a[],
                            const double b[], double z[], int32_t *ind) {
  assert(n > 0);
  assert(a != NULL && b != NULL && z != NULL && ind != NULL);
  assert(z != a && z != b);
  assert(0 <= *ind && *ind < n - 1);
  assert(fabs(a[0] - 1.0) < DBL_TOL);

  double y = 0.0;
  double z_tmp = u;
  for (int32_t i = 1; i < n; ++i) {
    int32_t z_index = (*ind + i - 1) % (n - 1);
    z_tmp -= a[i] * z[z_index];
    y += b[i] * z[z_index];
  }
  y += b[0] * z_tmp;

  // Shift delay line.
  *ind = (n + *ind - 2) % (n - 1);
  z[*ind] = z_tmp;

  return y;
}

double HoldMax(double u, double hold_time, double ts, HoldData *hold_data) {
  assert(hold_time >= 0.0 && ts > DBL_EPSILON);
  assert(hold_data != NULL);

  if (u >= hold_data->max || hold_data->timer <= 0.0) {
    hold_data->max = u;
    hold_data->timer = hold_time;
  } else {
    hold_data->timer -= ts;
  }
  return hold_data->max;
}

bool LatchOn(int32_t u, double hold_time, double ts, int32_t *counter) {
  *counter = MaxInt32(0, *counter - 1);
  if (u != 0) {
    *counter = (int32_t)round(hold_time / ts);
  } else if (*counter == 0) {
    return false;
  }
  return true;
}

double Zoh(double t, double u, double ts, double *t_z1, double *u_z1) {
  assert(t_z1 != NULL && u_z1 != NULL && t_z1 != u_z1);

  if (t - *t_z1 > ts * (1.0 - DBL_TOL)) {
    *t_z1 = t;
    *u_z1 = u;
  }
  return *u_z1;
}

double Backlash(double u, double width, double *y_z1) {
  double half_width = 0.5 * width;
  double y;

  if (u > *y_z1 + half_width) {
    y = u - half_width;
  } else if (u < *y_z1 - half_width) {
    y = u + half_width;
  } else {
    y = *y_z1;
  }

  *y_z1 = y;
  return y;
}

double Delay(double u, int32_t n, double buf[], int32_t *ind) {
  assert(n > 0);
  assert(0 <= *ind && *ind < n);

  *ind = (*ind + 1) % n;
  double u_out = buf[*ind];
  buf[*ind] = u;
  return u_out;
}

double Integrator(double u, double low, double high, double ts,
                  IntegratorMode mode, double *int_u) {
  assert(low <= high);
  assert(ts > 0.0);
  assert(int_u != NULL);

  switch (mode) {
    // Intentionally fall through to integration case for default.
    default:
      assert(false);
    case kIntegratorModeIntegrate:
      *int_u = Saturate(*int_u + ts * u, low, high);
      break;

    case kIntegratorModeReset:
      *int_u = 0.0;
      break;

    case kIntegratorModeHold:
      break;
  }
  return *int_u;
}

double Pid(double error, double deriv_error, double ts, IntegratorMode mode,
           const PidParams *params, double *int_output) {
  Integrator(params->ki * error, params->int_output_min, params->int_output_max,
             ts, mode, int_output);
  return params->kp * error + params->kd * deriv_error + *int_output;
}

double PidAntiWindup(double error, double deriv_error, double tracking_error,
                     double ts, IntegratorMode mode, const PidParams *params,
                     double *int_output) {
  // Set tracking gain.  The tracking gain is based on the
  // recommendations in Ch. 10 of Feedback System by Astrom and
  // Murray, 2008.
  double kt;
  if (fabs(params->kp) < DBL_EPSILON) {
    // For purely integral controllers, use the plain Pid function and
    // pass the tracking error in with the proportional error.
    kt = 0.0;
    assert(false);
  } else if (fabs(params->kd) < DBL_EPSILON) {
    kt = params->ki / params->kp;
  } else {
    kt = Sqrt(params->ki / params->kd);
  }

  Integrator(params->ki * error + kt * tracking_error, params->int_output_min,
             params->int_output_max, ts, mode, int_output);
  return params->kp * error + params->kd * deriv_error + *int_output;
}

void CrossfadePidParams(const PidParams *p0, const PidParams *p1, double x,
                        double x_low, double x_high, PidParams *p_out) {
  assert(p0 != NULL && p1 != NULL && p_out != NULL);

  p_out->kp = Crossfade(p0->kp, p1->kp, x, x_low, x_high);
  p_out->ki = Crossfade(p0->ki, p1->ki, x, x_low, x_high);
  p_out->kd = Crossfade(p0->kd, p1->kd, x, x_low, x_high);
  p_out->int_output_min =
      Crossfade(p0->int_output_min, p1->int_output_min, x, x_low, x_high);
  p_out->int_output_max =
      Crossfade(p0->int_output_max, p1->int_output_max, x, x_low, x_high);
}

void SecondOrderFilterCoeff(double fc, double zeta, double ts, FilterType type,
                            double a[], double b[]) {
  assert(a != NULL && b != NULL && a != b);
  assert(ts > DBL_TOL);
  assert(fc >= 0.0 && zeta > 0.0);

  double w = 2.0 * PI * fc;
  double w2 = w * w;
  double ts2 = ts * ts;

  double a0 = 4.0 / ts2 + w2 + (4.0 * zeta * w) / ts;
  assert(fabs(a0) > DBL_TOL);
  a0 = fmax(a0, DBL_TOL);
  a[0] = 1.0;
  a[1] = (2.0 * w2 - 8.0 / ts2) / a0;
  a[2] = (4.0 / ts2 + w2 - (4.0 * zeta * w) / ts) / a0;

  double w2_a0 = w2 / a0;
  switch (type) {
    case kFilterTypeLowPass:
      // w^2 / (s^2 + 2*zeta*w*s + w^2).
      b[0] = w2_a0;
      b[1] = 2.0 * w2_a0;
      b[2] = w2_a0;
      break;
    case kFilterTypeHighPass:
      // s^2 / (s^2 + 2*zeta*w*s + w^2).
      b[0] = 4.0 / ts2 / a0;
      b[1] = -8.0 / ts2 / a0;
      b[2] = 4.0 / ts2 / a0;
      break;
    case kFilterTypeBandPass:
      // 2*zeta*w*s / (s^2 + 2*zeta*w*s + w^2).
      b[0] = 4.0 * zeta * w / a0 / ts;
      b[1] = 0.0;
      b[2] = -4.0 * zeta * w / a0 / ts;
      break;
    case kFilterTypeBandStop:
      // (s^2 + w^2) / (s^2 + 2*zeta*w*s + w^2).
      b[0] = w2_a0 + 4.0 / ts2 / a0;
      b[1] = 2.0 * w2_a0 - 8.0 / ts2 / a0;
      b[2] = w2_a0 + 4.0 / ts2 / a0;
      break;
    case kFilterTypeDiffAndLowPass:
      // s*w^2 / (s^2 + 2*zeta*w*s + w^2).
      b[0] = 2.0 * w2_a0 / ts;
      b[1] = 0.0;
      b[2] = -2.0 * w2_a0 / ts;
      break;
    default:
      assert(false);
      break;
  }
}

// The signal flow diagram for a second-order digital filter in
// Direct Form II is:
//
//                        w[n]
// x[n] --->(+)------------+-----[b0]----(+)---> y[n]
//           ^             |              ^
//           |            z1              |
//           |             |              |
//          (+)<---[-a1]---+---->[b1]--->(+)
//           ^             |              ^
//           |            z1              |
//           |             |              |
//          (+)<---[-a2]---+---->[b2]--->(+)
//
// which has the difference equations:
//
//  y[n] = b0 w[n] + b1 w[n-1] + b2 w[n-2]
//  w[n] =    x[n] - a1 w[n-1] - a2 w[n-2]
//
// In steady-state, we find w[n] == w[n-1] == w[n-2] == w
// with w = x / (1 + a1 + a2), where x is the steady-state input.
//
// Reference:
//   https://en.wikipedia.org/wiki/Digital_biquad_filter#Direct_form_2
void Lpf2Init(double u0, double fc, double zeta, double ts, double z[]) {
  double a[3], b[3];
  SecondOrderFilterCoeff(fc, zeta, ts, kFilterTypeLowPass, a, b);
  z[0] = u0 / (a[0] + a[1] + a[2]);
  z[1] = z[0];
}

void Lpf2Vec3Init(const Vec3 *u0, double fc, double zeta, double ts, Vec3 z[]) {
  double zx[2], zy[2], zz[2];
  Lpf2Init(u0->x, fc, zeta, ts, zx);
  Lpf2Init(u0->y, fc, zeta, ts, zy);
  Lpf2Init(u0->z, fc, zeta, ts, zz);
  JoinVec3Arr(zx, zy, zz, ARRAYSIZE(zx), z);
}

double Lpf2(double u, double fc, double zeta, double ts, double z[]) {
  double a[3], b[3];
  SecondOrderFilterCoeff(fc, zeta, ts, kFilterTypeLowPass, a, b);
  return Filter(u, ARRAYSIZE(a), a, b, z);
}

double Hpf2(double u, double fc, double zeta, double ts, double z[]) {
  double a[3], b[3];
  SecondOrderFilterCoeff(fc, zeta, ts, kFilterTypeHighPass, a, b);
  return Filter(u, ARRAYSIZE(a), a, b, z);
}

double BandPass2(double u, double fc, double zeta, double ts, double z[]) {
  double a[3], b[3];
  SecondOrderFilterCoeff(fc, zeta, ts, kFilterTypeBandPass, a, b);
  return Filter(u, ARRAYSIZE(a), a, b, z);
}

double DiffLpf2(double u, double fc, double zeta, double ts, double z[]) {
  double a[3], b[3];
  SecondOrderFilterCoeff(fc, zeta, ts, kFilterTypeDiffAndLowPass, a, b);
  return Filter(u, ARRAYSIZE(a), a, b, z);
}

static const Vec3 *SecondOrderFilterVec3(const Vec3 *u, double fc, double zeta,
                                         double ts, FilterType type, Vec3 *y,
                                         Vec3 z[]) {
  assert(u != NULL && y != NULL && z != NULL && u != z && y != z);

  double a[3], b[3];
  SecondOrderFilterCoeff(fc, zeta, ts, type, a, b);

  double zx[2], zy[2], zz[2];
  SplitVec3Arr(z, ARRAYSIZE(zx), zx, zy, zz);
  y->x = Filter(u->x, ARRAYSIZE(a), a, b, zx);
  y->y = Filter(u->y, ARRAYSIZE(a), a, b, zy);
  y->z = Filter(u->z, ARRAYSIZE(a), a, b, zz);
  JoinVec3Arr(zx, zy, zz, ARRAYSIZE(zx), z);
  return y;
}

const Vec3 *Lpf2Vec3(const Vec3 *u, double fc, double zeta, double ts, Vec3 *y,
                     Vec3 z[]) {
  return SecondOrderFilterVec3(u, fc, zeta, ts, kFilterTypeLowPass, y, z);
}

const Vec3 *Hpf2Vec3(const Vec3 *u, double fc, double zeta, double ts, Vec3 *y,
                     Vec3 z[]) {
  return SecondOrderFilterVec3(u, fc, zeta, ts, kFilterTypeHighPass, y, z);
}

const Vec3 *BandPass2Vec3(const Vec3 *u, double fc, double zeta, double ts,
                          Vec3 *y, Vec3 z[]) {
  return SecondOrderFilterVec3(u, fc, zeta, ts, kFilterTypeBandPass, y, z);
}

const Vec3 *DiffLpf2Vec3(const Vec3 *u, double fc, double zeta, double ts,
                         Vec3 *y, Vec3 z[]) {
  return SecondOrderFilterVec3(u, fc, zeta, ts, kFilterTypeDiffAndLowPass, y,
                               z);
}

double PeakDetector(double u, double fc_down, double ts, double *y_z1) {
  assert(y_z1 != NULL);

  if (u > *y_z1) {
    *y_z1 = u;
  } else {
    Lpf(u, fc_down, ts, y_z1);
  }
  return *y_z1;
}

const Vec3 *PeakDetectorVec3(const Vec3 *u, double fc, double ts, Vec3 *y_z1) {
  assert(u != NULL && y_z1 != NULL);

  PeakDetector(u->x, fc, ts, &y_z1->x);
  PeakDetector(u->y, fc, ts, &y_z1->y);
  PeakDetector(u->z, fc, ts, &y_z1->z);
  return y_z1;
}

void InitCircularAveragingBuffer(double *array, int32_t size,
                                 CircularAveragingBuffer *buffer) {
  assert(array != NULL);
  assert(buffer != NULL);
  buffer->array = array;
  buffer->size = size;
  buffer->sum = 0.0;
  buffer->next_idx = 0;
  buffer->full = false;
  for (int32_t i = 0; i < size; ++i) {
    *(buffer->array + i) = 0.0;
  }
}

double UpdateCircularAveragingBuffer(double new_val,
                                     CircularAveragingBuffer *buffer) {
  if (!buffer->full) {
    *(buffer->array + buffer->next_idx) = new_val;
    buffer->sum += new_val;
    buffer->next_idx = (buffer->next_idx + 1) % buffer->size;
    // Check if we've populated the entire buffer.
    if (buffer->next_idx == 0) {
      buffer->full = true;
      return buffer->sum / (double)buffer->size;
    }
    return buffer->sum / (double)buffer->next_idx;
  } else {
    double old_val = *(buffer->array + buffer->next_idx);
    *(buffer->array + buffer->next_idx) = new_val;
    buffer->next_idx = (buffer->next_idx + 1) % buffer->size;
    buffer->sum = buffer->sum - old_val + new_val;
    return buffer->sum / (double)buffer->size;
  }
}

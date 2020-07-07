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

#ifndef COMMON_C_MATH_FILTER_H_
#define COMMON_C_MATH_FILTER_H_

#include <stdbool.h>
#include <stdint.h>

#include "common/c_math/vec3.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct { double max, timer; } HoldData;

typedef enum {
  kFilterTypeLowPass,
  kFilterTypeHighPass,
  kFilterTypeBandPass,
  kFilterTypeBandStop,
  kFilterTypeDiffAndLowPass
} FilterType;

typedef enum {
  kIntegratorModeIntegrate,
  kIntegratorModeReset,
  kIntegratorModeHold
} IntegratorMode;

typedef struct {
  double kp, ki, kd;
  double int_output_min, int_output_max;
} PidParams;

typedef struct {
  double *array;
  int32_t size;
  double sum;
  int32_t next_idx;
  bool full;
} CircularAveragingBuffer;

// Calculates a running variance of values in a circular buffer.
//
// Args:
//   x: Value to be written.
//   n: Length of the buffer.
//   x_buf: Circular buffer.
//   ind: Input/output parameter indicating index to write x.
//       Cyclically incremented by this function.
//
// Returns:
//   The variance of the elements of the updated buffer (note that the
//   variance is normalized by n rather than n-1, i.e. equivalent to
//   calling var(x_buf, 1) in MATLAB rather than var(x_buf).
double RunningVar(double x, int32_t n, double x_buf[], int32_t *ind);

// Single-pole low pass filter.  This function calculates the filter
// coefficients online, so it may be used with varying cutoff
// frequencies.
//
// Args:
//   u: Input signal value.
//   fc: Cutoff frequency [Hz].
//   ts: Sample time [s].
//   y_z1: State of the filter, i.e. last output value.
//
// Returns:
//   Filtered signal.
double Lpf(double u, double fc, double ts, double *y_z1);

// Single-pole low pass filter on a Vec3 signal (see Lpf).
const Vec3 *LpfVec3(const Vec3 *u, double fc, double ts, Vec3 *y_z1);

// Single-pole high pass filter.  This function calculates the filter
// coefficients online, so it may be used with varying cutoff
// frequencies.
//
// Args:
//   u: Input signal value.
//   fc: Cutoff frequency [Hz].
//   ts: Sample time [s].
//   y_z1: State of the filter, i.e. last output value.
//
// Returns:
//   Filtered signal.
double Hpf(double u, double fc, double ts, double *y_z1, double *u_z1);

// Derivative of a signal.
//
// Args:
//   u: Input signal value.
//   ts: Sample time [s].
//   u_z1: State of the filter, i.e. last input value.
//
// Returns:
//   Derivative of u.
double Diff(double u, double ts, double *u_z1);

// Derivative of a Vec3 signal (see Diff).
const Vec3 *DiffVec3(const Vec3 *u, double ts, Vec3 *du, Vec3 *u_z1);

// Differentiates a variable with circular topology.  Prevents
// spurious results when crossing over the wrapping threshold.  Note
// that if the input can jump greater than half of the wrap range in a
// single loop, then the derivative will be incorrect.
//
// Args:
//   u: Input signal value.
//   wrap_range: Difference between the maximum and minimum possible
//       values of the input.
//   ts: Sample time [s].
//   u_z1: State of the filter, i.e. last input value.
//
// Returns:
//   Derivative of u.
double DiffCircular(double u, double wrap_range, double ts, double *u_z1);

// Rate limits a signal between a low and a high rate.
//
// Args:
//   u: Input signal value.
//   low: The signal cannot decrease faster than low [#/s].
//   high: The signal cannot increase faster than high [#/s].
//   ts: Sample time [s].
//   y_z1: State of the filter, i.e. last output value.
//
// Returns:
//   Rate-limited signal.
double RateLimit(double u, double low, double high, double ts, double *y_z1);

// Rate limits an integer signal between a low and a high rate (see
// RateLimit).
int32_t RateLimitInt32(int32_t u, int32_t low, int32_t high, double ts,
                       int32_t *y_z1);

// Rate limits a Vec3 signal between a low and a high rate (see
// RateLimit).
const Vec3 *RateLimitVec3(const Vec3 *u, const Vec3 *low, const Vec3 *high,
                          double ts, Vec3 *y_z1);

// Rate limits a signal with a circular topology between a low and a
// high rate.
//
// Args:
//   u: Input signal value.
//   low_rate: The signal cannot decrease faster than low [#/s].
//   high_rate: The signal cannot increase faster than high [#/s].
//   low_wrap: Minimum value of the range.
//   high_wrap: Maximum value of the range.
//   ts: Sample time [s].
//   y_z1: State of the filter, i.e. last output value.
//
// Returns:
//   Rate-limited signal.
double RateLimitCircular(double u, double low_rate, double high_rate,
                         double low_wrap, double high_wrap, double ts,
                         double *y_z1);

// Linear filter in Direct Form II.
//
// Here is some example Python code to calculate a and b vectors for a
// 10 rad/s, single-pole low pass filter in a 100 Hz loop:
//
//   import scipy.signal as signal
//   (b, a) = signal.bilinear([10.0], [1.0, 10.0], 100.0)
//
// Args:
//   u: Input signal value.
//   n: Length of the a and b coefficient vectors.  Note that this is
//       one more than the order of the filter.
//   a: Filter denominator coefficients.  Note that the first
//       coefficient of a assumed to be 1.
//   b: Filter numerator coefficients.
//   z: Delay line, which must have length n - 1.  Most recent value
//       is stored in z[0].
//
// Returns:
//   Filtered signal.
double Filter(double u, int32_t n, const double a[], const double b[],
              double z[]);

// Linear filter in Direct Form II implemented with a circular buffer.
// ind stores the most recent value of the delay line.
double FilterCircularBuffer(double u, int32_t n, const double a[],
                            const double b[], double z[], int32_t *ind);

// Holds the maximum input value for hold_time.  Resets to current
// value after hold_time.
//
// Args:
//   u: Input signal value.
//   hold_time: Time [s] to hold the maximum value.
//   ts: Sample time [s].
//   hold_data: State of the filter that contains the maximum value
//       and a countdown timer.
//
// Returns:
//   Held input value.
double HoldMax(double u, double hold_time, double ts, HoldData *hold_data);

// Latches a non-zero signal on for specified hold time.
//
// Args:
//   u: Input signal to latch when non-zero.
//   hold_time: Time [s] to latch the input signal.
//   ts: Sample time [s].
//   counter: State of the filter that contains a countdown timer.
//
// Returns:
//   True for hold_time after u is non-zero; otherwise returns false.
bool LatchOn(int32_t u, double hold_time, double ts, int32_t *counter);

// Zero-order hold.  Converts a signal with any sample rate to one
// with sample rate ts.
//
// Args:
//   t: Current time [s].
//   u: Input signal value.
//   ts: Sample time [s].
//   t_z1: Last time [s] when this function was called.
//   u_z1: Last input value when this function was called.
//
// Returns:
//   Sampled signal.
double Zoh(double t, double u, double ts, double *t_z1, double *u_z1);

// Adds backlash to a signal.
//
// Args:
//   u: Input signal value.
//   width: Width of the backlash.  When switching directions, the
//       output does not change until the input has changed by this
//       width.
//   y_z1: State of the filter, i.e. last output value.
//
// Returns:
//   Signal with backlash.
double Backlash(double u, double width, double *y_z1);

// Delays a signal by n samples.
//
// Args:
//   u: Input signal value.
//   n: Number of samples to delay the signal by.
//   buf: Buffer of size n to store the delayed signal.
//   ind: Index of the most recently added value.
//
// Returns:
//   Delayed signal.
double Delay(double u, int32_t n, double buf[], int32_t *ind);

// General purpose integrator with saturations and hold and reset
// commands.
//
// Args:
//   u: Value to be integrated.
//   low: Integrator is saturated from below by this.
//   high: Integrator is saturated from above by this.
//   ts: Sample time.
//   mode: Either integrate, hold, or reset the integrator.
//   int_u: In/out argument that holds the integrator state.
//
// Returns:
//   Returns a copy of the integrator state.
double Integrator(double u, double low, double high, double ts,
                  IntegratorMode mode, double *int_u);

// Runs a simple proportional-integral-derivative (PID) loop.
//
// The derivative of the error is an input (rather than being
// calculated internally) for two reasons: 1) This allows using
// measured derivative (e.g. from a rate gyro) 2) Derivative terms
// should be low-pass-filtered in any case.
//
// Args:
//   error: Error signal to integrate and apply proportional gain to.
//   deriv_error: Derivative of the error signal.
//   ts: Sample time.
//   int_mode: Command to integrate, hold, or reset the integrator.
//   params: Parameter structure that include P, I, and D gains as
//       well as the saturation limits.
//   int_output: In-out parameter that holds the integrated state.
//
// Returns:
//   The output of the PID loop, which is the P, I, and D gains
//   multiplied by the error, error derivative, and integrated error,
//   respectively.
double Pid(double error, double deriv_error, double ts, IntegratorMode int_mode,
           const PidParams *params, double *int_output);

// Runs a proportional-integral-derivative (PID) loop with
// back-calculation anti-windup (see Astrom and Murray, Feedback
// Systems).
//
// Args:
//   error: Error signal to integrate and apply proportional gain to.
//   deriv_error: Derivative of the error signal.
//   tracking_error: This is the difference between the measured or
//       modeled actuator output and the output of the PID loop.
//   ts: Sample time.
//   int_mode: Command to integrate, hold, or reset the integrator.
//   params: Parameter structure that include P, I, and D gains as
//       well as the saturation limits.
//   int_output: In-out parameter that holds the integrated state.
//
// Returns:
//   The output of the PID loop, which is the P, I, and D gains
//   multiplied by the error, error derivative, and integrated error,
//   respectively.
double PidAntiWindup(double error, double deriv_error, double tracking_error,
                     double ts, IntegratorMode mode, const PidParams *params,
                     double *int_output);

// Linear interpolation between two PID parameter sets, p0 and p1,
// based on a parameter x and the limits x_low and x_high.
//
//           x <= x_low     p_out = p0
//   x_low < x < x_high     linear interpolation between p0 and p1
//           x >= x_high    p_out = p1
void CrossfadePidParams(const PidParams *p0, const PidParams *p1, double x,
                        double x_low, double x_high, PidParams *p_out);

// Calculates the second order filter coefficients using the Tustin
// method on a standard continuous second order transfer function.
//
// Args:
//   fc: Cutoff frequency [Hz].
//   zeta: Damping ratio [#].
//   ts: Sample time [s].
//   filter_type: Enum that sets whether the filter is a low pass,
//       high pass, band pass, band stop, or differentiated low pass
//       filter.
//   a: Denominator coefficients.  This array must have a length of 3.
//   b: Numerator coefficients.  This array must have a length of 3.
void SecondOrderFilterCoeff(double fc, double zeta, double ts, FilterType type,
                            double a[], double b[]);

// Initializes the internal state of a second-order filter implemented
// in Direct Form II.  The argument u0 is the steady-state input value.
void Lpf2Init(double u0, double fc, double zeta, double ts, double z[]);
void Lpf2Vec3Init(const Vec3 *u0, double fc, double zeta, double ts, Vec3 z[]);

// Second-order low pass filter.
double Lpf2(double u, double fc, double zeta, double ts, double z[]);

// Second-order high pass filter.
double Hpf2(double u, double fc, double zeta, double ts, double z[]);

// Second-order band pass filter.
double BandPass2(double u, double fc, double zeta, double ts, double z[]);

// Second-order low pass filter of a differentiated signal.
double DiffLpf2(double u, double fc, double zeta, double ts, double z[]);

// Second-order low pass filter on a Vec3 signal.  See Lpf2.
const Vec3 *Lpf2Vec3(const Vec3 *u, double fc, double zeta, double ts, Vec3 *y,
                     Vec3 z[]);

// Second-order high pass filter on a Vec3 signal.  See Hpf2.
const Vec3 *Hpf2Vec3(const Vec3 *u, double fc, double zeta, double ts, Vec3 *y,
                     Vec3 z[]);

// Second-order band pass filter on a Vec3 signal.
const Vec3 *BandPass2Vec3(const Vec3 *u, double fc, double zeta, double ts,
                          Vec3 *y, Vec3 z[]);

// Second-order low pass filter of a differentiated Vec3 signal.  See
// DiffLpf2.
const Vec3 *DiffLpf2Vec3(const Vec3 *u, double fc, double zeta, double ts,
                         Vec3 *y, Vec3 z[]);

// Peak detection filter.  Increases in the input signal are
// unfiltered while decreases are low pass filtered.
//
// Args:
//   u: Input signal value.
//   fc_down: Low pass filter frequency [Hz] for decreasing signals.
//   ts: Sample time [s].
//   y_z1: State of the filter, i.e. last output value.
//
// Returns:
//   Filtered signal.
double PeakDetector(double u, double fc_down, double ts, double *y_z1);

// Peak detection filter for a Vec3 signal.  See PeakDetector.
const Vec3 *PeakDetectorVec3(const Vec3 *u, double fc, double ts, Vec3 *y_z1);

// Initializes a CircularAveragingBuffer with the given array and size.
void InitCircularAveragingBuffer(double *array, int32_t size,
                                 CircularAveragingBuffer *buffer);

// Updates the circular averaging buffer with the provided new value, and
// returns the new average of the buffer.
double UpdateCircularAveragingBuffer(double new_val,
                                     CircularAveragingBuffer *buffer);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // COMMON_C_MATH_FILTER_H_

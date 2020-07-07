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

#ifndef COMMON_C_MATH_UTIL_H_
#define COMMON_C_MATH_UTIL_H_

#include <stdbool.h>
#include <stdint.h>

#include "common/c_math/cal_params.h"
#include "common/c_math/linalg.h"
#include "common/c_math/mat3.h"
#include "common/c_math/quaternion.h"
#include "common/c_math/vec2.h"
#include "common/c_math/vec3.h"

// This definition is equal to M_PI in math.h.
#define PI 3.14159265358979323846

// This is the standard "tolerance" we use for floating point
// operations.  Use this number to protect operations from
// divide-by-zero, etc.
#define DBL_TOL 1e-9

// Bit-masked options for interpolation functions.  By default, we use
// linear interpolation that is linearly extrapolated outside the
// domain of the input vectors.  The kInterpOptionSaturate
// saturates the output at the limits of the input vectors.
typedef enum {
  kInterpOptionDefault = 1 << 0,
  kInterpOptionSaturate = 1 << 1
} InterpOption;

typedef struct {
  double scale;
  double bias;
  int32_t bias_count;
} CalParams;

typedef struct {
  CalParams cal;
  int32_t encoder_counts;
  double cal_val_center;
} EncoderCalParams;

#ifdef __cplusplus
extern "C" {
#endif

int32_t MinInt32(int32_t x, int32_t y);
int32_t MaxInt32(int32_t x, int32_t y);
uint32_t MinUint32(uint32_t x, uint32_t y);
uint32_t MaxUint32(uint32_t x, uint32_t y);
int64_t MinInt64(int64_t x, int64_t y);
int64_t MaxInt64(int64_t x, int64_t y);
uint64_t MinUint64(uint64_t x, uint64_t y);
uint64_t MaxUint64(uint64_t x, uint64_t y);
uint32_t MaxUnsignedValue(int32_t bits);
int32_t MinSignedValue(int32_t bits);
int32_t MaxSignedValue(int32_t bits);
int32_t Sign(double x);
int32_t SignInt32(int32_t x);

// Returns true if x and y are equal to within floating point
// tolerance.
bool IsApproximatelyEqual(double x, double y);
bool IsApproximatelyEqualVec3(const Vec3 *u, const Vec3 *v);

// Returns the maximum value of an array of doubles of length n.  The
// index of the maximum value is returned in maxind (if it is not
// NULL).
double MaxArray(const double *x, int32_t n, int32_t *maxind);

// Returns the minimum value of an array of doubles of length n.  The
// index of the minimum value is returned in minind (if it is not
// NULL).
double MinArray(const double *x, int32_t n, int32_t *minind);

// Returns the maximum value of an array of integers of length n.  The
// index of the maximum value is returned in maxind (if it is not
// NULL).
int32_t MaxArrayInt32(const int32_t *x, int32_t n, int32_t *maxind);
int64_t MaxArrayInt64(const int64_t *x, int32_t n, int32_t *maxind);

// Returns the maximum value of an array of integers of length n.  The
// index of the maximum value is returned in maxind (if it is not
// NULL).
uint32_t MaxArrayUint32(const uint32_t *x, int32_t n, int32_t *maxind);

// Finds the variance of a list of doubles.
//
// Args:
//   x_buf: List of doubles.
//   n: Length of the list.
//
// Returns:
//   Variance of the list normalized by n instead of n - 1.
double VarArray(const double *x, int32_t n);

// Takes the mean of a pair of doubles paying attention to possible overflow.
//
// Args:
//   x: First number in average.
//   y: Second number in average.
//
// Returns:
//   The value (x + y)/2.0 is calculated so as to avoid overflow.
double MeanPair(double x, double y);

// Returns the mean of an array of doubles of length n.
double MeanArray(const double *x, int32_t n);

// Swaps two double values in place.
//
// Args:
//   x: First value to swap.
//   y: Second value to swap.
void SwapInPlace(double *x, double *y);

// Swaps two float values in place.
//
// Args:
//   x: First value to swap.
//   y: Second value to swap.
void SwapInPlacef(float *x, float *y);

// Returns a value saturated between low and high.
double Saturate(double x, double low, double high);

// Returns true if x is at or outside the low and high bounds.
bool IsSaturated(double x, double low, double high);

const Vec2 *SaturateVec2(const Vec2 *x, const Vec2 *low, const Vec2 *high,
                         Vec2 *y);
const Vec3 *SaturateVec3(const Vec3 *x, const Vec3 *low, const Vec3 *high,
                         Vec3 *y);
const Vec3 *SaturateVec3ByScalar(const Vec3 *x, double low, double high,
                                 Vec3 *y);
const Vec *SaturateVec(const Vec *x, const Vec *low, const Vec *high, Vec *y);
const double *SaturateArrayByScalar(const double *x, int32_t n, double low,
                                    double high, double *y);
int32_t SaturateInt32(int32_t x, int32_t low, int32_t high);
uint32_t SaturateUint32(uint32_t x, uint32_t low, uint32_t high);
int64_t SaturateInt64(int64_t x, int64_t low, int64_t high);
uint64_t SaturateUint64(uint64_t x, uint64_t low, uint64_t high);
int32_t SaturateSigned(int32_t x, int32_t bits);
uint32_t SaturateUnsigned(uint32_t x, int32_t bits);

// Saturates into a range that may be wrapped.
// All inputs must be between wrap_left and wrap_right.
// Output snaps to closest end of range.
//
// Args:
//   x: Value to saturate.
//   range_start: Beginning value for allowed range.
//   range_end: End value for allowed range.
//   wrap_left: Left of wrapping interval.
//   wrap_right: Right of wrapping interval.
double SaturateWrapped(double x, double range_start, double range_end,
                       double wrap_left, double wrap_right);

const Vec3 *FabsVec3(const Vec3 *v_in, Vec3 *v_out);

// Implements the mixing function:
//
//          / x0                       c <= 0
//   f(c) = | (1 - c) * x0 + c * x1    0 < c < 1
//          \ x1                       c >= 1
double Mix(double x0, double x1, double c);

// Crossfades two input signals using saturated linear interpolation
// based on a third input control value.
//
// Args:
//   y0: Signal to use when x <= x_low.
//   y1: Signal to use when x > x_high.
//   x: Control value used to set the fraction of each signal.
//   x_low: The threshold for x below which y0 is used completely.
//   x_high: The threshold for x above which y1 is used completely.
//
// Returns:
//   If x <= x_low, returns y0.  If x > x_high, returns y1.
//   Otherwise, returns a linear combination of y0 and y1 based on the
//   fraction of the distance of x between x_low and x_high.
double Crossfade(double y0, double y1, double x, double x_low, double x_high);

const Vec2 *CrossfadeVec2(const Vec2 *y0, const Vec2 *y1, double x,
                          double x_low, double x_high, Vec2 *y_out);
const Vec3 *CrossfadeVec3(const Vec3 *y0, const Vec3 *y1, double x,
                          double x_low, double x_high, Vec3 *y_out);
const Mat3 *CrossfadeMat3(const Mat3 *y0, const Mat3 *y1, double x,
                          double x_low, double x_high, Mat3 *y_out);
const double *CrossfadeArray(const double *y0, const double *y1, int32_t n,
                             double x, double x_low, double x_high,
                             double *y_out);

// Finds the fractional index of a value x_i in a vector x assuming x
// is monotonically increasing.
//
// Args:
//   x: Monotonically increasing array of values.
//   n: Length of the array.
//   x_i: The value to find the fractional index of.
//   opt: Enum that describes how to handle values outside of the
//       domain of x (e.g. extrapolate or saturate).  See the
//       definition of InterpOption for details.
//   ind: Ignored if NULL, otherwise returns the last index of x
//       before x_i.
//
// Returns: The fractional index of x_i in x.
double InterpIndex(const double x[], int32_t n, double x_i, InterpOption opt,
                   int32_t *ind);

// Performs a linear interpolation along a pair of vectors.  The
// function to interpolate on is described by the input vector x and
// the output vector y.  Interp1 linearly interpolates to find the
// value of y for input x_i.  If opt is kInterpOptionDefault,
// extrapolate linearly beyond the end of the vectors.  If opt is
// kInterpOptionSaturate, saturate the output value at the
// ends of the output (y) vector.
//
// Warning: This currently assumes a monotonically increasing x.
double Interp1(const double x[], const double y[], int32_t n, double x_i,
               InterpOption opt);

// Performs a "warped" interpolation along a pair of vectors.  The
// function to interpolate on is described by the input vector x and
// the output vector y.  Interp1 interpolates using the supplied
// "warping" function such that:
//
//   Unwarp(Interp1(x, Warp(y), n, x_i, ...))
//
// where Interp1 is the traditional linear interpolation function.
// The InterpOption values have the same meaning here as in the linear
// Interp1 function.
double Interp1WarpY(const double x[], const double y[], int32_t n, double x_i,
                    InterpOption opt, double (*warp_func)(double),
                    double (*unwarp_func)(double));

// Performs bilinear interpolation on a two-dimensional data table z.
// The coordinates of the columns are listed in array x, and the
// coordinates of the rows are listed in array y.  These arrays must
// each be monotonically increasing, but need not be evenly spaced.
//
// For information on bilinear interpolation, please see:
// https://en.wikipedia.org/wiki/Bilinear_interpolation
//
// Input arguments:
//
// x:   Array containing x-coordinates of data points
// y:   Array containing y-coordinates of data points
// nx:  Length of array x
// ny:  Length of array y
// z:   Data table (array of size [ny][nx], cast to const double *).
// xi:  x coordinate to interpolate
// yi:  y coordinate to interpolate
// opt: Extrapolation mode; see Interp1.
//
// Return value:
//
// The interpolated value.
double Interp2(const double x[], const double y[], int32_t nx, int32_t ny,
               const double *z, double x_i, double y_i, InterpOption opt);

// Interpolation on circular topology of interval [left, right].
// The first and last elements of the lookup tables must be at the edge
// of the interval and those y-values must be equivalent; i.e. the
// lookup tables must be of this form:
//
//   x = [x_left, x_1, ..., x_{n-1}, x_right]
//   y = [y_left, y_1, ..., y_{n-1}, y_right]
//
double CircularInterp1(const double x[], const double y[], int32_t n,
                       double x_i);
void Interp1Vec3(const double x[], const Vec3 y[], int32_t n, double x_i,
                 InterpOption opt, Vec3 *y_out);

// Sigmoid function based on atan2. (Sigmoids are a class of functions
// which resemble the integral of a bell-shaped curve.)  It is defined
// such that:
//
//   Sigmoid(-Inf, 1.0) = 0.0
//   Sigmoid(0.0, 1.0) = 0.5
//   Sigmoid(Inf, 1.0) = 1.0
//
// The "width" is the x distance between the points where y = 0.05 and
// y = 0.95, which is where the 6.31 magic number comes from.
//
// TODO: Replace with more standard sigmoid function,
// 1/(1+exp(-t)), after we switch to C.
double Sigmoid(double x, double width);

// Fits x and y data to 2nd order polynomial, similarly to MATLAB's
// polyfit function.
void PolyFit2(const double x[], const double y[], double coeff[]);

// Nth order polynomial evaluated using Horner's method.
//
// Note: The input n is the length of the c array, so a 2nd order
// polynomial requires n = 3.  Also, c[0] is the coefficient of the
// *highest* power as in the MATLAB polyval function.
double PolyVal(const double c[], double x, int32_t n);

// Coefficients of the derivative of a polynomial.  Here, n is the
// length of the array of derivatives, which should be one less than
// the length of the array of coefficients.  c[0] is the coefficient
// of the *highest* power.
void PolyDer(const double c[], int32_t n, double dc[]);

double ApplyCal(double raw_val, const CalParams *cal);
float ApplyCal32(float raw_val, const CalParams32 *cal);
double InvertCal(double cal_val, const CalParams *cal);
float InvertCal32(float cal_val, const CalParams32 *cal);

double ApplyEncoderCal(int32_t raw_val, const EncoderCalParams *ecal);
int32_t InvertEncoderCal(double cal_val, const EncoderCalParams *ecal);

// Generalized wrapping function to wrap x to the interval [left,
// right).
double Wrap(double x, double left, double right);
int32_t WrapInt32(int32_t x, int32_t left, int32_t right);

// Tests if min <= x < max.
bool InsideRange(double x, double min, double max);
// Tests if min <= x < max, wrapping from right to left.  Thus, if min >= max,
// the range wraps from right to left.  If min == max, all valid x will pass.
bool InsideRangeWrapped(double x, double left, double right, double min,
                        double max);

// Safe version of the asin function.
double Asin(double x);

// Safe version of the acos function.
double Acos(double x);

// Safe version of sqrt function.
double Sqrt(double x);

// Squares a number.
double Square(double x);

// Compute the third power of a number.
double ThirdPower(double x);

// Compute the fourth power of a number.
double FourthPower(double x);

// Compute 10^x.
double Exp10(double x);

// Returns an array with ones at the indices specified by the slice
// (similar to Python slices).
const int32_t *Slice(int32_t start, int32_t incr, int32_t end, int32_t length,
                     int32_t *arr);

void SplitVec3Arr(const Vec3 vs[], int32_t n, double xs[], double ys[],
                  double zs[]);
void JoinVec3Arr(const double xs[], const double ys[], const double zs[],
                 int32_t n, Vec3 vs[]);

// Conversions between degrees and radians.
//
// NOTE: Using "inline" with these functions appears to generate
// spurious warnings. This looks related to
// https://gcc.gnu.org/bugzilla/show_bug.cgi?id=54113.
double DegToRad(double deg);
double RadToDeg(double rad);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // COMMON_C_MATH_UTIL_H_

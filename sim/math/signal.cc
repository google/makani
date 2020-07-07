// Copyright 2020 Makani Technologies LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "sim/math/signal.h"

#include <glog/logging.h>
#include <math.h>
#include <stdint.h>

#include <vector>

#include "common/c_math/vec3.h"
#include "sim/math/util.h"

namespace sim {
namespace signal {

template <>
const double &Add<double>(const double &x, const double &y, double *result) {
  *result = x + y;
  return *result;
}

template <>
const double &GetAddIdentity<double>() {
  static const double double_add_identity = 0.0;
  return double_add_identity;
}

template <>
const double &Mult<double>(const double &x, const double &y, double *result) {
  *result = x * y;
  return *result;
}

template <>
const double &GetMultIdentity<double>() {
  static const double double_mult_identity = 1.0;
  return double_mult_identity;
}

template <>
const double &Scale<double>(const double &x, double y, double *result) {
  *result = x * y;
  return *result;
}

template <>
const double &Quantize<double>(const double &x, const double &quantization,
                               double *result) {
  DCHECK_GE(quantization, 0.0) << "Quantization must be non-negative.";
  *result = fabs(quantization) <= 0.0 ? x : x - fmod(x, quantization);
  return *result;
}

template <>
const double &GetRandNormal<double>(NamedRandomNumberGenerator *rng,
                                    double *result) {
  *result = rng->GetNormal();
  return *result;
}

template <>
const double &Saturate<double>(const double &value, const double &bound_low,
                               const double &bound_high, double *result) {
  *result = ::Saturate(value, bound_low, bound_high);
  return *result;
}

template <>
uint32_t GetSize<double>() {
  return 1U;
}

template <>
void ToVector<double>(const double &value, std::vector<double> *vec) {
  vec->resize(1);
  (*vec)[0] = value;
}

template <>
const double &FromVector<double>(const std::vector<double> &vec,
                                 double *value) {
  DCHECK_EQ(1U, vec.size());
  *value = vec[0];
  return *value;
}

template <>
bool IsNear<double>(const double &expected, const double &actual, double tol) {
  return tol > fabs(expected - actual);
}

// Component-wise Vec3 operations.
template <>
const Vec3 &Add<Vec3>(const Vec3 &x, const Vec3 &y, Vec3 *result) {
  Vec3Add(&x, &y, result);
  return *result;
}

template <>
const Vec3 &GetAddIdentity<Vec3>() {
  return kVec3Zero;
}

template <>
const Vec3 &Mult<Vec3>(const Vec3 &x, const Vec3 &y, Vec3 *result) {
  Vec3Mult(&x, &y, result);
  return *result;
}

template <>
const Vec3 &GetMultIdentity<Vec3>() {
  return kVec3Ones;
}

template <>
const Vec3 &Scale<Vec3>(const Vec3 &x, double y, Vec3 *result) {
  Vec3Scale(&x, y, result);
  return *result;
}

template <>
const Vec3 &Quantize<Vec3>(const Vec3 &x, const Vec3 &quantization,
                           Vec3 *result) {
  Quantize(x.x, quantization.x, &result->x);
  Quantize(x.y, quantization.y, &result->y);
  Quantize(x.z, quantization.z, &result->z);
  return *result;
}

template <>
const Vec3 &GetRandNormal<Vec3>(NamedRandomNumberGenerator *rng, Vec3 *value) {
  value->x = rng->GetNormal();
  value->y = rng->GetNormal();
  value->z = rng->GetNormal();
  return *value;
}

template <>
const Vec3 &Saturate<Vec3>(const Vec3 &value, const Vec3 &bound_low,
                           const Vec3 &bound_high, Vec3 *result) {
  SaturateVec3(&value, &bound_low, &bound_high, result);
  return *result;
}

template <>
uint32_t GetSize<Vec3>() {
  return 3U;
}

template <>
void ToVector<Vec3>(const Vec3 &value, std::vector<double> *vec) {
  vec->resize(3U);
  (*vec)[0] = value.x;
  (*vec)[1] = value.y;
  (*vec)[2] = value.z;
}

template <>
const Vec3 &FromVector<Vec3>(const std::vector<double> &vec, Vec3 *value) {
  DCHECK_EQ(3U, vec.size());
  value->x = vec[0];
  value->y = vec[1];
  value->z = vec[2];
  return *value;
}

template <>
bool IsNear<Vec3>(const Vec3 &expected, const Vec3 &actual, double tol) {
  return tol > fabs(expected.x - actual.x) &&
         tol > fabs(expected.y - actual.y) && tol > fabs(expected.z - actual.z);
}

}  // namespace signal
}  // namespace sim

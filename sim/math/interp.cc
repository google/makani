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

#include "sim/math/interp.h"

#include <float.h>
#include <math.h>

#include "common/c_math/mat3.h"
#include "common/c_math/util.h"
#include "common/c_math/vec3.h"

namespace sim {

TimeseriesInterpolator::TimeseriesInterpolator(const std::vector<double> &t)
    : t_(t), interp_accel_(nullptr) {
  interp_accel_ = gsl_interp_accel_alloc();
}

TimeseriesInterpolator::~TimeseriesInterpolator() {
  gsl_interp_accel_free(interp_accel_);
}

TimeseriesInterpolator::InterpParams TimeseriesInterpolator::GetInterpParams(
    double t) {
  if (t <= t_[0]) {
    return TimeseriesInterpolator::InterpParams({0}, {1.0});
  } else if (t >= *t_.rbegin()) {
    return TimeseriesInterpolator::InterpParams(
        {static_cast<int32_t>(t_.size()) - 1}, {1.0});
  } else {
    int32_t i = static_cast<int32_t>(
        gsl_interp_accel_find(interp_accel_, t_.data(), t_.size(), t));
    double c = (t_[i + 1] - t) / (t_[i + 1] - t_[i]);
    return TimeseriesInterpolator::InterpParams({i, i + 1}, {c, 1.0 - c});
  }
}

double TimeseriesInterpolator::WrapInterpInternal(
    const std::vector<double> &values, const std::vector<double> &weights,
    double lower, double upper) {
  double out = values[0];
  for (size_t i = 0U; i < values.size(); ++i) {
    out += weights[i] * Wrap(values[i] - values[0], lower, upper);
  }
  return Wrap(out, lower, upper);
}

double TimeseriesInterpolator::Interp(double t,
                                      const std::vector<double> &data) {
  TimeseriesInterpolator::InterpParams params = GetInterpParams(t);
  double out = 0.0;
  for (int32_t i = 0; i < static_cast<int32_t>(params.indices.size()); ++i) {
    out += params.weights[i] * data[params.indices[i]];
  }
  return out;
}

Vec3 TimeseriesInterpolator::InterpVec3(double t,
                                        const std::vector<Vec3> &data) {
  TimeseriesInterpolator::InterpParams params = GetInterpParams(t);
  Vec3 out = kVec3Zero;
  for (int32_t i = 0; i < static_cast<int32_t>(params.indices.size()); ++i) {
    Vec3Axpy(params.weights[i], &data[params.indices[i]], &out);
  }
  return out;
}

double TimeseriesInterpolator::WrapInterp(double t,
                                          const std::vector<double> &data,
                                          double lower, double upper) {
  TimeseriesInterpolator::InterpParams params = GetInterpParams(t);
  const size_t sz = params.indices.size();
  std::vector<double> values(sz);
  for (size_t i = 0; i < sz; ++i) {
    values[i] = data[params.indices[i]];
  }
  return WrapInterpInternal(values, params.weights, lower, upper);
}

Vec3 TimeseriesInterpolator::WrapInterpVec3(double t,
                                            const std::vector<Vec3> &data,
                                            double lower, double upper) {
  TimeseriesInterpolator::InterpParams params = GetInterpParams(t);
  const size_t sz = params.indices.size();
  std::vector<double> x(sz), y(sz), z(sz);
  for (size_t i = 0; i < sz; ++i) {
    const Vec3 &v = data[params.indices[i]];
    x[i] = v.x;
    y[i] = v.y;
    z[i] = v.z;
  }

  Vec3 out = {WrapInterpInternal(x, params.weights, lower, upper),
              WrapInterpInternal(y, params.weights, lower, upper),
              WrapInterpInternal(z, params.weights, lower, upper)};
  return out;
}

}  // namespace sim

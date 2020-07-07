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

#ifndef SIM_MATH_INTERP_H_
#define SIM_MATH_INTERP_H_

#include <gsl/gsl_interp.h>
#include <math.h>
#include <vector>

#include "common/c_math/vec3.h"
#include "common/macros.h"

namespace sim {

// A TimeseriesInterpolator performs linear interpolation at timepoints within
// the bounds of t, and 0th order extrapolation past the endpoints.
class TimeseriesInterpolator {
 public:
  explicit TimeseriesInterpolator(const std::vector<double> &t);
  ~TimeseriesInterpolator();

  double Interp(double t, const std::vector<double> &data);
  Vec3 InterpVec3(double t, const std::vector<Vec3> &data);

  double WrapInterp(double t, const std::vector<double> &data, double lower,
                    double upper);
  Vec3 WrapInterpVec3(double t, const std::vector<Vec3> &data, double lower,
                      double upper);

 private:
  double WrapInterpInternal(const std::vector<double> &values,
                            const std::vector<double> &weights, double lower,
                            double upper);

  struct InterpParams {
    InterpParams(const std::vector<int32_t> &indices__,
                 const std::vector<double> &weights__)
        : indices(indices__), weights(weights__) {}
    ~InterpParams() {}

    std::vector<int32_t> indices;
    std::vector<double> weights;
  };

  const std::vector<double> &t_;
  gsl_interp_accel *interp_accel_;

  InterpParams GetInterpParams(double t);

  DISALLOW_COPY_AND_ASSIGN(TimeseriesInterpolator);
};

}  // namespace sim

#endif  // SIM_MATH_INTERP_H_

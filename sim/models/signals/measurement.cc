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

#include "sim/models/signals/measurement.h"

#include <functional>
#include <limits>
#include <string>
#include <vector>

#include "common/c_math/util.h"
#include "common/c_math/vec3.h"
#include "control/system_types.h"
#include "sim/faults/faults.h"
#include "sim/math/signal.h"
#include "sim/math/util.h"
#include "sim/state.h"

// Builds a new Measurement<double>.
//
// Args:
//   parent_full_name: The full name of this model's parent.
//   name__: Name of this measurement.
//   ts: Sample time.
//   params: Parameters describing the measurement.
//   tracked: A state storing the ideal value of this measurement.
//   faults: The schedule the measurement should check for faults.
template <>
Measurement<double>::Measurement(const std::string &parent_full_name,
                                 const std::string &name__, double ts,
                                 const std::vector<SensorModelParams> &params,
                                 const DiscreteState<double> &tracked,
                                 FaultSchedule *faults)
    : Measurement(parent_full_name, name__, ts, 0.0, params[0].noise_level,
                  params[0].bias, params[0].scale, params[0].bound_low,
                  params[0].bound_high, params[0].quantization,
                  [&tracked](double *v) { *v = tracked.val(); }, faults) {
  CHECK_EQ(1U, params.size());
}

// Same as above but with default values for params.
template <>
Measurement<double>::Measurement(const std::string &parent_full_name,
                                 const std::string &name__, double ts,
                                 const DiscreteState<double> &tracked,
                                 FaultSchedule *faults)
    : Measurement(parent_full_name, name__, ts,
                  {{0.0, 0.0, 1.0, -std::numeric_limits<double>::infinity(),
                    std::numeric_limits<double>::infinity(), 0.0}},
                  tracked, faults) {}

// Same as above but with default values for ts as well.
template <>
Measurement<double>::Measurement(const std::string &parent_full_name,
                                 const std::string &name__,
                                 const DiscreteState<double> &tracked,
                                 FaultSchedule *faults)
    : Measurement(parent_full_name, name__, 0.0, tracked, faults) {}

// Builds a new Measurement<Vec3>.
//
// Args:
//   parent_full_name: The full name of this model's parent.
//   name__: Name of this measurement.
//   ts: Sample time.
//   params: Parameters describing the measurement.
//   tracked: A state storing the ideal value of this measurement.
//   faults: The schedule the measurement should check for faults.
template <>
Measurement<Vec3>::Measurement(const std::string &parent_full_name,
                               const std::string &name__, double ts,
                               const std::vector<SensorModelParams> &params,
                               const DiscreteState<Vec3> &tracked,
                               FaultSchedule *faults)
    : Measurement(
          parent_full_name, name__, ts, {0.0, 0.0, 0.0},
          {params[0].noise_level, params[1].noise_level, params[2].noise_level},
          {params[0].bias, params[1].bias, params[2].bias},
          {params[0].scale, params[1].scale, params[2].scale},
          {params[0].bound_low, params[1].bound_low, params[2].bound_low},
          {params[0].bound_high, params[1].bound_high, params[2].bound_high},
          {params[0].quantization, params[1].quantization,
           params[2].quantization},
          [&tracked](Vec3 *v) { *v = tracked.val(); }, faults) {
  CHECK_EQ(3U, params.size());
}

// Builds a new Measurement<Vec3>.
//
// Args:
//   parent_full_name: The full name of this model's parent.
//   name__: Name of this measurement.
//   ts: Sample time.
//   tracked: A state storing the ideal value of this measurement.
//   faults: The schedule the measurement should check for faults.
template <>
Measurement<Vec3>::Measurement(const std::string &parent_full_name,
                               const std::string &name__, double ts,
                               const DiscreteState<Vec3> &tracked,
                               FaultSchedule *faults)
    : Measurement(parent_full_name, name__, ts,
                  {{0.0, 0.0, 1.0, -std::numeric_limits<double>::infinity(),
                    std::numeric_limits<double>::infinity(), 0.0},
                   {0.0, 0.0, 1.0, -std::numeric_limits<double>::infinity(),
                    std::numeric_limits<double>::infinity(), 0.0},
                   {0.0, 0.0, 1.0, -std::numeric_limits<double>::infinity(),
                    std::numeric_limits<double>::infinity(), 0.0}},
                  tracked, faults) {}

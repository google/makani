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

#include <gtest/gtest.h>
#include <stdint.h>

#include <functional>
#include <limits>
#include <vector>

#include "common/c_math/vec3.h"
#include "lib/util/test_util.h"
#include "sim/faults/faults.h"
#include "sim/math/signal.h"
#include "sim/math/util.h"
#include "sim/models/signals/measurement.h"
#include "sim/sim_types.h"
#include "sim/state.h"

namespace {

template <typename T>
std::string ToString(const T &val);

template <>
std::string ToString(const double &val) {
  return std::to_string(val);
}

template <>
std::string ToString(const Vec3 &val) {
  return "(" + std::to_string(val.x) + ", " + std::to_string(val.y) + ", " +
         std::to_string(val.z) + ")";
}

}  // namespace

template <typename T>
class MeasurementTest : public ::testing::Test {
 public:
  MeasurementTest() : rng_("MeasurementTest") {}

 protected:
  double CheckRecorded(double t, double t_stop, double ts, const T &scale,
                       const T &offset, T *ext_value,
                       Measurement<T> *measurement);
  double CheckBiasDrift(double t, double t_stop, double ts, const T &bound_low,
                        const T &bound_high, const T &bias_drift_mean,
                        T *ext_value, Measurement<T> *measurement);
  void TestBasicFaults(const T &rescale, const T &fixed_val,
                       const T &offset_val, const T &drift_mean,
                       const T &bound_low, const T &bound_high,
                       const T &quantization);

  NamedRandomNumberGenerator rng_;
};

template <typename T>
double MeasurementTest<T>::CheckRecorded(double t, double t_stop, double ts,
                                         const T &scale, const T &offset,
                                         T *ext_value,
                                         Measurement<T> *measurement) {
  while (t < t_stop + (ts / 2.0)) {
    sim::signal::GetRandNormal<T>(&rng_, ext_value);
    measurement->DiscreteUpdate(t);

    T expected;
    sim::signal::Mult<T>(*ext_value, scale, &expected);
    sim::signal::Add<T>(offset, expected, &expected);
    EXPECT_TRUE(sim::signal::IsNear(expected, measurement->recorded(), 1e-9))
        << "expected: " << ToString(expected)
        << "\nactual: " << ToString(measurement->recorded()) << "\n";

    t += ts;
  }
  return t;
}

template <typename T>
double MeasurementTest<T>::CheckBiasDrift(double t, double t_stop, double ts,
                                          const T &bound_low,
                                          const T &bound_high,
                                          const T &drift_mean, T *ext_value,
                                          Measurement<T> *measurement) {
  double t0 = t - ts;
  while (t < t_stop) {
    sim::signal::GetRandNormal<T>(&rng_, ext_value);
    measurement->DiscreteUpdate(t);

    std::vector<double> expected_values(sim::signal::GetSize<T>());
    T expected;
    sim::signal::Scale<T>(drift_mean, t - t0, &expected);
    sim::signal::Add<T>(*ext_value, expected, &expected);
    sim::signal::Saturate<T>(expected, bound_low, bound_high, &expected);
    EXPECT_TRUE(sim::signal::IsNear(expected, measurement->recorded(), 1e-9));

    t += ts;
  }
  return t;
}

template <class T>
void MeasurementTest<T>::TestBasicFaults(const T &rescale, const T &fixed_val,
                                         const T &offset_val,
                                         const T &drift_mean,
                                         const T &bound_low,
                                         const T &bound_high,
                                         const T &quantization) {
  double ts = 1.0;
  T ext_value = sim::signal::GetAddIdentity<T>();

  SimOption saved_sim_opt = GetSimParams()->sim_opt;
  GetSimParamsUnsafe()->sim_opt =
      (SimOption)(saved_sim_opt | kSimOptImperfectSensors);

  std::function<void(T *)> val_func = [&ext_value](T *v) { *v = ext_value; };

  FaultSchedule::FaultFunc fault_func = [ts, rescale, drift_mean, fixed_val,
                                         offset_val](
      double t, SimFaultType type, std::vector<double> *params) -> bool {
    if (t > 10.0 && t < 20.0 + (ts / 2.0) &&
        type == kSimFaultMeasurementRescale) {
      sim::signal::ToVector(rescale, params);
      return true;
    } else if (t > 30.0 && t < 40.0 + (ts / 2.0) &&
               type == kSimFaultMeasurementHoldCurrent) {
      return true;
    } else if (t > 50.0 && t < 60.0 + (ts / 2.0) &&
               type == kSimFaultMeasurementFixValue) {
      sim::signal::ToVector(fixed_val, params);
      return true;
    } else if (t > 70.0 && t < 80.0 + (ts / 2.0) &&
               type == kSimFaultMeasurementBiasOffset) {
      sim::signal::ToVector(offset_val, params);
      return true;
    } else if (t > 90.0 && t < 200.0 + (ts / 2.0) &&
               type == kSimFaultMeasurementBiasDriftMean) {
      sim::signal::ToVector(drift_mean, params);
      return true;
    } else {
      return false;
    }
  };

  T value_0 = sim::signal::GetAddIdentity<T>();
  T value_1 = sim::signal::GetMultIdentity<T>();
  FaultSchedule empty_schedule(nullptr);
  Measurement<T> measurement("Parent", "value", ts, value_0, value_0, value_0,
                             value_1, bound_low, bound_high, quantization,
                             val_func, &empty_schedule);
  measurement.set_fault_func(fault_func);

  // Test scaling.
  double t = 0.0;

  // Check pass through.
  t = CheckRecorded(t, 10.0, ts, value_1, value_0, &ext_value, &measurement);

  // Check rescaling fault.
  t = CheckRecorded(t, 20.0, ts, rescale, value_0, &ext_value, &measurement);

  // Check pass through.
  t = CheckRecorded(t, 30.0, ts, value_1, value_0, &ext_value, &measurement);

  // Check held value.
  T held_value = measurement.recorded();
  t = CheckRecorded(t, 40.0, ts, value_0, held_value, &ext_value, &measurement);

  // Check pass through.
  t = CheckRecorded(t, 50.0, ts, value_1, value_0, &ext_value, &measurement);

  // Check fixed value.
  t = CheckRecorded(t, 60.0, ts, value_0, fixed_val, &ext_value, &measurement);

  // Check pass through.
  t = CheckRecorded(t, 70.0, ts, value_1, value_0, &ext_value, &measurement);

  // Check offset.
  t = CheckRecorded(t, 80.0, ts, value_1, offset_val, &ext_value, &measurement);

  // Check pass through.
  t = CheckRecorded(t, 90.0, ts, value_1, value_0, &ext_value, &measurement);

  t = CheckBiasDrift(t, 200.0, ts, bound_low, bound_high, drift_mean,
                     &ext_value, &measurement);

  GetSimParamsUnsafe()->sim_opt = saved_sim_opt;
}

class MeasurementTestDouble : public MeasurementTest<double> {};

TEST_F(MeasurementTestDouble, BasicFaults) {
  TestBasicFaults(0.0625, 2.0, 7.0, 0.5, -10.0, 20.0, 0.0);
}

TEST_F(MeasurementTestDouble, Quantization) {
  std::function<void(double *)> val_func = [](double *v) { *v = 3.9; };
  FaultSchedule empty_schedule(nullptr);

  double ts = 0.1;
  double inf = std::numeric_limits<double>::infinity();
  Measurement<double> measurement("Parent", "QuantizationTest", ts, 0.0, 0.0,
                                  0.0, 1.0, -inf, inf, 1.1, val_func,
                                  &empty_schedule);
  measurement.DiscreteUpdate(ts);
  EXPECT_NEAR(3.3, measurement.recorded(),
              3.9 * std::numeric_limits<double>::epsilon());
}

class MeasurementTestVec3 : public MeasurementTest<Vec3> {};

TEST_F(MeasurementTestVec3, BasicFaults) {
  Vec3 rescale = {0.0625, 0.125, 0.5};
  Vec3 fixed_val = {1.0, 2.0, 3.0};
  Vec3 offset_val = {5.0, 6.0, 7.0};
  Vec3 drift_mean = {0.5, 0.25, 0.125};
  Vec3 bound_low = {-10.0, -20.0, -30.0};
  Vec3 bound_high = {30.0, 20.0, 10.0};
  Vec3 quantization = {0.0, 0.0, 0.0};
  TestBasicFaults(rescale, fixed_val, offset_val, drift_mean, bound_low,
                  bound_high, quantization);
}

TEST_F(MeasurementTestVec3, Quantization) {
  std::function<void(Vec3 *)> val_func = [](Vec3 *v) { *v = {1.4, 2.3, 3.9}; };
  FaultSchedule empty_schedule(nullptr);

  double ts = 0.1;
  double inf = std::numeric_limits<double>::infinity();
  Measurement<Vec3> measurement("Parent", "QuantizationTest", ts, kVec3Zero,
                                kVec3Zero, kVec3Zero, kVec3Ones,
                                {-inf, -inf, -inf}, {inf, inf, inf},
                                {1.1, 1.1, 1.1}, val_func, &empty_schedule);
  measurement.DiscreteUpdate(ts);
  EXPECT_NEAR_VEC3(Vec3({1.1, 2.2, 3.3}), measurement.recorded(),
                   3.9 * std::numeric_limits<double>::epsilon());
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

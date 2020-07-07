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

#ifndef SIM_MODELS_SIGNALS_MEASUREMENT_H_
#define SIM_MODELS_SIGNALS_MEASUREMENT_H_

#include <glog/logging.h>
#include <math.h>
#include <stdint.h>

#include <array>
#include <string>
#include <utility>
#include <vector>

#include "common/c_math/vec3.h"
#include "sim/faults/faults.h"
#include "sim/math/signal.h"
#include "sim/math/util.h"
#include "sim/models/model.h"
#include "sim/sim_params.h"
#include "sim/sim_types.h"
#include "sim/state.h"

template <typename T>
class MeasurementTest;

// Class for representing imperfect measurements.  Applies a simple sensor model
// including bias, scaling, additive noise, saturation to an externally provided
// value (generally from a parent model), and quantization.  Also applies
// generic faults as defined in sim/faults/faults.h.
//
// For example of how to use this class see sensors/model/imu.{cc,h}.
template <typename T>
class Measurement : public Model {
 public:
  Measurement(const std::string &parent_full_name, const std::string &name__,
              double ts, const T &init, const T &noise_level, const T &bias__,
              const T &scale__, const T &bound_low__, const T &bound_high__,
              const T &quantization__, const std::function<void(T *)> &val_func,
              FaultSchedule *faults);
  Measurement(const std::string &parent_full_name, const std::string &name__,
              double ts, const std::vector<SensorModelParams> &params,
              const DiscreteState<T> &tracked, FaultSchedule *faults);
  Measurement(const std::string &parent_full_name, const std::string &name__,
              double ts, const DiscreteState<T> &tracked,
              FaultSchedule *faults);
  Measurement(const std::string &parent_full_name, const std::string &name__,
              const DiscreteState<T> &tracked, FaultSchedule *faults);

  virtual ~Measurement() __attribute__((noinline)) {}

  // Returns the current (imperfect) value reported by the measurement model.
  const T &recorded() const { return recorded_.val(); }

  // Return the time when the measurement was last updated.
  double last_sample_time() const { return t_z1(); }

 protected:
  void DiscreteStepHelper(double t) override;

 private:
  // Sets function queried to indicate faults (used for testing).
  void set_fault_func(const FaultSchedule::FaultFunc &f) { fault_func_ = f; }

  // Connections to the fault subsystem and the value to be tracked.
  std::function<void(T *)> val_func_;
  FaultSchedule::FaultFunc fault_func_;

  // Parameters.
  const T default_scale_;
  const T default_noise_level_;
  const T bound_low_;
  const T bound_high_;
  const T quantization_;

  // Discrete state.
  DiscreteState<T> random_walk_bias_;
  DiscreteState<T> bias_offset_;
  DiscreteState<T> scale_;
  DiscreteState<T> noise_level_;
  DiscreteState<T> recorded_;

  NamedRandomNumberGenerator rng_;

  const T &scale() const { return scale_.val(); }
  const T &noise_level() const { return noise_level_.val(); }

  std::vector<std::pair<SimFaultType, uint32_t>> GetFaultSpecs();
  T ApplySensorModel(const T &x);
  void ApplyImperfectSensorFaults(double t);
  void UpdateRandomWalkBias(double t, const T &bias_drift_mean,
                            const T &bias_drift_rate);

  friend class MeasurementTest<T>;
};

template <typename T>
Measurement<T>::Measurement(const std::string &parent_full_name,
                            const std::string &name__, double ts, const T &init,
                            const T &noise_level__, const T &bias__,
                            const T &scale__, const T &bound_low__,
                            const T &bound_high__, const T &quantization__,
                            const std::function<void(T *)> &val_func,
                            FaultSchedule *faults)
    : Model(parent_full_name, name__, ts),
      val_func_(val_func),
      fault_func_(faults->ClaimFaultFunc(full_name(), GetFaultSpecs())),
      default_scale_(scale__),
      default_noise_level_(noise_level__),
      bound_low_(bound_low__),
      bound_high_(bound_high__),
      quantization_(quantization__),
      random_walk_bias_(new_discrete_state(), "random_walk_bias", 0.0, bias__),
      bias_offset_(new_discrete_state(), "bias_offset", 0.0,
                   sim::signal::GetAddIdentity<T>()),
      scale_(new_discrete_state(), "scale", 0.0, scale__),
      noise_level_(new_discrete_state(), "noise_level", 0.0, noise_level__),
      recorded_(new_discrete_state(), "recorded", 0.0, init),
      rng_(full_name()) {
  SetupDone();
}

// Returns specifications of faults that will be simulated given the current
// simulator options.
template <typename T>
std::vector<std::pair<SimFaultType, uint32_t>> Measurement<T>::GetFaultSpecs() {
  std::vector<std::pair<SimFaultType, uint32_t>> specs;

  const uint32_t size = sim::signal::GetSize<T>();
  // Parameters: Values to hold.
  specs.emplace_back(kSimFaultMeasurementFixValue, size);
  specs.emplace_back(kSimFaultMeasurementHoldCurrent, 0U);

  if (*g_sim.sim_opt & kSimOptImperfectSensors) {
    // Parameters: Values to multiply the default noise scales by.
    specs.emplace_back(kSimFaultMeasurementNoiseRescale, size);
    // Parameters: Values to multiply the measurements by.
    specs.emplace_back(kSimFaultMeasurementRescale, size);
    // Currently, bias drift integration is based on the sampling rate
    // of the sensor and is disabled if the sensor has a sampling rate
    // of zero.
    if (ts_ > 0.0) {
      // Parameters: Mean drift in bias [Unit / s].
      specs.emplace_back(kSimFaultMeasurementBiasDriftMean, size);
      // Parameters: Bias random walk rate [Unit / s^(1/2)].
      specs.emplace_back(kSimFaultMeasurementBiasDriftRate, size);
      // Parameters: Bias offset [Unit].
      specs.emplace_back(kSimFaultMeasurementBiasOffset, size);
    }
  }

  return specs;
}

// Callback for updating discrete states.
template <typename T>
void Measurement<T>::DiscreteStepHelper(double t) {
  T value;
  val_func_(&value);

  std::vector<double> parameters;
  if (fault_func_(t, kSimFaultMeasurementHoldCurrent, nullptr)) {
    recorded_.DiscreteUpdate(t, recorded_.val());
  } else if (fault_func_(t, kSimFaultMeasurementFixValue, &parameters)) {
    T params;
    recorded_.DiscreteUpdate(t, sim::signal::FromVector(parameters, &params));
  } else if (*g_sim.sim_opt & kSimOptImperfectSensors) {
    ApplyImperfectSensorFaults(t);
    recorded_.DiscreteUpdate(t, ApplySensorModel(value));
  } else {
    recorded_.DiscreteUpdate(t, value);
  }
}

// Applies scaling and bias to a measurement, then adds noise.
template <typename T>
T Measurement<T>::ApplySensorModel(const T &measurement) {
  T v;
  sim::signal::Mult<T>(scale(), measurement, &v);
  sim::signal::Add<T>(random_walk_bias_.val(), v, &v);
  sim::signal::Add<T>(bias_offset_.val(), v, &v);

  T noise;
  sim::signal::GetRandNormal<T>(&rng_, &noise);
  sim::signal::Mult<T>(noise_level(), noise, &noise);
  sim::signal::Add<T>(v, noise, &v);

  sim::signal::Saturate<T>(v, bound_low_, bound_high_, &v);

  sim::signal::Quantize<T>(v, quantization_, &v);

  return v;
}

// Updates the scaling, noise scaling, and bias when the corresponding
// faults are present.
template <typename T>
void Measurement<T>::ApplyImperfectSensorFaults(double t) {
  std::vector<double> params_vector;
  T params;

  params = sim::signal::GetMultIdentity<T>();
  if (fault_func_(t, kSimFaultMeasurementRescale, &params_vector)) {
    sim::signal::FromVector<T>(params_vector, &params);
  }
  scale_.DiscreteUpdate(t,
                        sim::signal::Mult<T>(default_scale_, params, &params));

  params = sim::signal::GetMultIdentity<T>();
  if (fault_func_(t, kSimFaultMeasurementNoiseRescale, &params_vector)) {
    sim::signal::FromVector<T>(params_vector, &params);
  }
  noise_level_.DiscreteUpdate(
      t, sim::signal::Mult<T>(default_noise_level_, params, &params));

  T bias_offset = sim::signal::GetAddIdentity<T>();
  if (fault_func_(t, kSimFaultMeasurementBiasOffset, &params_vector)) {
    sim::signal::FromVector<T>(params_vector, &bias_offset);
  }
  bias_offset_.DiscreteUpdate(t, bias_offset);

  if (ts_ > 0.0) {
    T bias_drift_mean = sim::signal::GetAddIdentity<T>();
    if (fault_func_(t, kSimFaultMeasurementBiasDriftMean, &params_vector)) {
      sim::signal::FromVector<T>(params_vector, &bias_drift_mean);
    }

    T bias_drift_rate;
    bias_drift_rate = sim::signal::GetAddIdentity<T>();
    if (fault_func_(t, kSimFaultMeasurementBiasDriftRate, &params_vector)) {
      sim::signal::FromVector(params_vector, &bias_drift_rate);
    }

    UpdateRandomWalkBias(t, bias_drift_mean, bias_drift_rate);
  }
}

// Updates the bias based on a simple random walk model.
template <typename T>
void Measurement<T>::UpdateRandomWalkBias(double t, const T &bias_drift_mean,
                                          const T &bias_drift_rate) {
  DCHECK_LT(0.0, ts_);
  T bias__;
  sim::signal::Scale(bias_drift_mean, ts_, &bias__);
  sim::signal::Add<T>(random_walk_bias_.val(), bias__, &bias__);

  T noise;
  sim::signal::Mult(bias_drift_rate,
                    sim::signal::GetRandNormal<T>(&rng_, &noise), &noise);
  sim::signal::Scale<T>(noise, sqrt(ts_), &noise);

  sim::signal::Add<T>(bias__, noise, &bias__);
  random_walk_bias_.DiscreteUpdate(t, bias__);
}

#endif  // SIM_MODELS_SIGNALS_MEASUREMENT_H_

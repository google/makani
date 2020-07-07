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

#ifndef SIM_MODELS_SIGNALS_DELAYED_SIGNAL_H_
#define SIM_MODELS_SIGNALS_DELAYED_SIGNAL_H_

#include <glog/logging.h>

#include <string>
#include <vector>

#include "sim/math/signal.h"
#include "sim/models/model.h"

// Base class for delaying a signal.
template <typename T>
class DelayedSignal : public Model {
 public:
  DelayedSignal(const std::string &parent_full_name, const std::string &name__,
                double ts, double delay__, const T &init);
  virtual ~DelayedSignal() __attribute__((noinline)) {}

  // Returns the delayed signal.
  const T &output() const { return output_.val(); }

  // Sets the function which returns the value to be delayed.
  void set_val_func(const std::function<void(T *)> &val_func) {
    val_func_ = val_func;
  }

 protected:
  const T &states(int32_t idx) const { return states_[idx].val(); }

 private:
  void DiscreteStepHelper(double t) override;

  // Interpolates a value between elements of the states_ buffer. This method
  // determines the implementation of fractional delays.
  //
  // Args:
  //   prev: Left index of the interpolation interval.
  //   next: Right index of the interpolation interval.
  //   c: Fraction through the interval at which a sample is chosen.
  //   value: Output value. If c=0.0, then the value at `prev` is be returned.
  //       If c=1.0, then the value at `next` should be returned.
  virtual void InterpolateValue(int32_t prev, int32_t next, double c,
                                T *value) = 0;

  int32_t head() const { return head_.val(); }
  double update_times(int32_t idx) const { return update_times_[idx].val(); }

  // Connection to other models.
  std::function<void(T *)> val_func_;

  // Parameters.
  const double delay_;

  // Discrete state.
  DiscreteState<T> output_;
  // Index of most recent values in circular buffers.
  DiscreteState<int32_t> head_;
  // Circular buffer of current and past values and times at which
  // they were recorded.
  std::vector<DiscreteState<T>> states_;
  std::vector<DiscreteState<double>> update_times_;
};

template <typename T>
DelayedSignal<T>::DelayedSignal(const std::string &parent_full_name,
                                const std::string &name__, double ts,
                                double delay__, const T &init)
    : Model(parent_full_name, name__, ts),
      val_func_([this](T * /*v*/) {
        CHECK(false) << "Value function not set for " << full_name() << ".";
      }),
      delay_(delay__),
      output_(new_discrete_state(), "output", 0.0, init),
      head_(new_discrete_state(), "head", 0.0, 0),
      states_(),
      update_times_() {
  CHECK_LT(0.0, ts) << "Delayed signals must have a positive sample period.";
  CHECK_LE(0.0, delay_) << "Delays must be non-negative.";

  const uint32_t num_delayed_states = static_cast<uint32_t>(delay_ / ts_) + 2U;
  states_.reserve(num_delayed_states);
  update_times_.reserve(num_delayed_states);
  for (uint32_t i = 0U; i < num_delayed_states; ++i) {
    states_.emplace_back(new_discrete_state(),
                         "states[" + std::to_string(i) + "]", 0.0, init);
    update_times_.emplace_back(new_discrete_state(),
                               "update_times[" + std::to_string(i) + "]", 0.0,
                               -DBL_MAX);
  }

  SetupDone();
}

// Callback for updating discrete states.
template <typename T>
void DelayedSignal<T>::DiscreteStepHelper(double t) {
  // Retrieve the new value.
  T new_value;
  val_func_(&new_value);

  // Update the circular buffer.
  head_.DiscreteUpdate(t, (head() + 1) % static_cast<int32_t>(states_.size()));
  states_[head()].DiscreteUpdate(t, new_value);
  update_times_[head()].DiscreteUpdate(t, t);

  T output_value;
  if (delay_ <= 0.0) {
    output_value = new_value;
  } else {
    const double output_time = t - delay_;

    // Find the indices whose update times straddle output_time.
    // Start at the tail.
    int32_t prev = (head() + 1) % static_cast<int32_t>(states_.size());
    // If the tail position is newer that the output time, fail.
    DCHECK_LE(update_times(prev), output_time) << "Delay buffer too short.";

    int32_t index = prev;
    while (prev != head()) {
      index = (prev + 1) % static_cast<int32_t>(states_.size());
      if (update_times(index) >= output_time) break;
      prev = index;
    }
    // This should never occur.
    DCHECK_NE(prev, index) << "Couldn't find sample.";

    // Interpolate between the values found.
    double coeff = (output_time - update_times(prev)) /
                   (update_times(index) - update_times(prev));
    coeff = Saturate(coeff, 0.0, 1.0);
    InterpolateValue(prev, index, coeff, &output_value);
  }

  output_.DiscreteUpdate(t, output_value);
}

// Implements fractional delays by holding the recorded value at t1 across the
// interval [t1, t2), where t1 and t2 are recorded times. This effectively makes
// the delay time into the smallest integer multiple of the model sample time
// that is >= the specified delay time.
//
// For sample usage of this class, see sensors/model/sensors/gps.{cc,h}.
template <typename T>
class DelayedSteppingSignal : public DelayedSignal<T> {
 public:
  DelayedSteppingSignal(const std::string &parent_full_name,
                        const std::string &name__, double ts, double delay__,
                        const T &init)
      : DelayedSignal<T>(parent_full_name, name__, ts, delay__, init) {}

 private:
  void InterpolateValue(int32_t prev, int32_t next, double c,
                        T *value) override {
    // Choose `next` only when we've fully progressed through the interval (up
    // to a small tolerance).
    *value = this->states(c < 1.0 - 1e-8 ? prev : next);
  }
};

// Implements fractional delays using linear interpolation.
//
// For sample usage of this class, see sensors/model/sensors/imu.{cc,h}.
template <typename T>
class DelayedInterpolatingSignal : public DelayedSignal<T> {
 public:
  DelayedInterpolatingSignal(const std::string &parent_full_name,
                             const std::string &name__, double ts,
                             double delay__, const T &init)
      : DelayedSignal<T>(parent_full_name, name__, ts, delay__, init) {}

 private:
  void InterpolateValue(int32_t prev, int32_t next, double c,
                        T *value) override {
    T recent_value;
    sim::signal::Scale<T>(this->states(next), c, &recent_value);
    sim::signal::Scale<T>(this->states(prev), 1.0 - c, value);
    sim::signal::Add<T>(recent_value, *value, value);
  }
};

#endif  // SIM_MODELS_SIGNALS_DELAYED_SIGNAL_H_

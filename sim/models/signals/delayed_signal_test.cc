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
#include <stdio.h>

#include <functional>
#include <limits>
#include <vector>

#include "sim/math/signal.h"
#include "sim/models/signals/delayed_signal.h"
#include "sim/state.h"

template <typename T>
class DelayedInterpolatingSignalTest : public ::testing::Test {
 protected:
  void TestDelay(double base_ts, double ts, double delay);
};

template <typename T>
void DelayedInterpolatingSignalTest<T>::TestDelay(double base_ts, double ts,
                                                  double delay) {
  DelayedInterpolatingSignal<T> delayed("test", "delayed", ts, delay,
                                        sim::signal::GetAddIdentity<T>());
  T value;
  delayed.set_val_func([&value](T *v) { *v = value; });

  double t_z1 = -std::numeric_limits<double>::infinity();
  for (int32_t i = 0; i < 1000; ++i) {
    double t = base_ts * i;
    double t_cmp = t * (1.0 + std::numeric_limits<double>::epsilon());
    if (t_cmp < t_z1 + ts) continue;
    value = sim::signal::GetMultIdentity<T>();
    sim::signal::Scale<T>(value, t, &value);

    delayed.DiscreteUpdate(t);

    if (t > delay) {
      T expected = sim::signal::GetMultIdentity<T>();
      sim::signal::Scale<T>(expected, t - delay, &expected);
      EXPECT_TRUE(sim::signal::IsNear<T>(expected, delayed.output(), 1e-9));
    }
    t_z1 = t_cmp - fmod(t_cmp, ts);
  }
}

class DelayedInterpolatingSignalTestVec3
    : public DelayedInterpolatingSignalTest<Vec3> {};

TEST_F(DelayedInterpolatingSignalTestVec3, Basics) {
  for (double base_ts : {0.004, 0.01}) {
    for (double ts : {0.01, 0.02, 0.025, 0.7, 1.0}) {
      TestDelay(base_ts, ts, 0.0);
      TestDelay(base_ts, ts, 0.25);
      TestDelay(base_ts, ts, 1.25);
      TestDelay(base_ts, ts, 0.5);
      TestDelay(base_ts, ts, 1.0);
      TestDelay(base_ts, ts, 2.0);
      TestDelay(base_ts, ts, 2.25);
    }
  }

  TestDelay(0.004, 0.004, 0.06);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

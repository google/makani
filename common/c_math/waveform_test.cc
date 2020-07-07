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
#include <math.h>
#include <stdint.h>
#include <stdlib.h>

#include "common/c_math/waveform.h"

TEST(PulseTrain, Normal) {
  for (double t = 0.0; t < 10; t += 0.01) {
    double y = 2.0 * PulseTrain(t, 2.0, 0.1, 1.0);
    if (t <= 2.0) {
      EXPECT_NEAR(y, 0.0, 1e-9);
    } else if (fmod(t, 1.0) < 0.1) {
      EXPECT_NEAR(y, 2.0, 1e-9);
    } else {
      EXPECT_NEAR(y, 0.0, 1e-9);
    }
  }
}

TEST(Fourier, TriangleWave) {
  const int32_t N = 100;
  double a0 = 0.0;
  double a[N], b[N];
  for (int32_t i = 0; i < N; ++i) {
    a[i] = 0.0;
    if (i % 2 == 0) {
      b[i] = 8.0 / (M_PI * M_PI * (i + 1) * (i + 1)) * pow(-1.0, i / 2);
    } else {
      b[i] = 0.0;
    }
  }

  const double period = 1.0;
  for (double t = -2.0 * period; t < 2.0 * period; t += 0.1) {
    EXPECT_NEAR(Fourier(t, period, a0, a, b, N), TriangleWave(t, period), 1e-4);
  }
}

TEST(LinearChirp, ConstantFrequencyIsSinWave) {
  double t_start = 22.0, t_end = 222.0;
  double freq = 1.0;

  for (double t = t_start - 1.0; t < t_end + 1.0; t += 0.2) {
    double y = LinearChirp(t, t_start, freq, t_end, freq);
    if (t < t_start || t > t_end) {
      EXPECT_EQ(y, 0.0);
    } else {
      EXPECT_NEAR(y, sin(2.0 * M_PI * freq * t), 1e-9);
    }
  }
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

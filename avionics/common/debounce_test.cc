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

#include "avionics/common/debounce.h"

namespace {

void SimpleTest(bool value) {
  for (int32_t threshold = 0; threshold < 256; ++threshold) {
    DebounceState state;
    DebounceStateInit(!value, &state);

    for (int32_t cycle = 0; cycle < threshold; ++cycle) {
      EXPECT_EQ(!value, Debounce(value, threshold, threshold, &state));
    }

    EXPECT_EQ(value, Debounce(value, threshold, threshold, &state));
  }
}

void ComplexTest(bool value) {
  for (int32_t threshold = 0; threshold < 256; ++threshold) {
    DebounceState state;
    DebounceStateInit(!value, &state);

    for (int32_t max_cycles = 0; max_cycles <= threshold; ++max_cycles) {
      EXPECT_EQ(!value, Debounce(!value, threshold, threshold, &state));
      for (int32_t cycle = 0; cycle < max_cycles; ++cycle) {
        EXPECT_EQ(!value, Debounce(value, threshold, threshold, &state));
      }
    }
    EXPECT_EQ(value, Debounce(value, threshold, threshold, &state));
  }
}

}  // namespace

TEST(TestDebounce, SimpleHigh) {
  SimpleTest(true);
}

TEST(TestDebounce, SimpleLow) {
  SimpleTest(false);
}

TEST(TestDebounce, ComplexHigh) {
  ComplexTest(true);
}

TEST(TestDebounce, ComplexLow) {
  ComplexTest(false);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

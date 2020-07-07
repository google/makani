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

#include "avionics/linux/aio_interface.h"
#include "avionics/network/aio_node.h"

#define ON_PERIODIC_TEST_COUNT 10

class AioInterfaceFixture : public AioInterface {
 public:
  AioInterfaceFixture(void) : AioInterface(1), on_periodic_count_(0) {}
  ~AioInterfaceFixture(void) {}

  int32_t on_periodic_count_;

  void OnPeriodic(void) override {
    ++on_periodic_count_;
    if (on_periodic_count_ >= ON_PERIODIC_TEST_COUNT) {
      Stop();
    }
  }
};

TEST(AioInterface, OnPeriodic) {
  AioInterfaceFixture aio;

  EXPECT_TRUE(aio.Run(kAioNodeControllerA));
  EXPECT_EQ(ON_PERIODIC_TEST_COUNT, aio.on_periodic_count_);
}

// TODO: Add tests.

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

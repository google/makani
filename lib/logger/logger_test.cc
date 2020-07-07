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

#include "lib/logger/logger.h"

#include <gtest/gtest.h>

#include <chrono>  // NOLINT(build/c++11)
#include <csignal>
#include <future>  // NOLINT(build/c++11)
#include <iostream>
#include <memory>
#include <thread>  // NOLINT(build/c++11)

TEST(LoggerTest, Default) {
  auto logger = logger::Logger::GetInstance();

  auto future = std::async(std::launch::async, [&logger] { logger.Start(); });

  auto status = future.wait_for(std::chrono::seconds(2));
  EXPECT_NE(status, std::future_status::ready);

  logger.HandleStopSignal(SIGTERM);

  status = future.wait_for(std::chrono::seconds(2));
  EXPECT_EQ(status, std::future_status::ready);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

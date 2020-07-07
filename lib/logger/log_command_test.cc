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

#include <cstdlib>
#include <string>

const std::string kLogCommand("lib/logger/log_command");

TEST(LogCommand, StartFlagNoSystem) {
  int rc = system((kLogCommand + " --start").c_str());
  EXPECT_EQ(WEXITSTATUS(rc), EXIT_FAILURE);
}

TEST(LogCommand, StartFlagInvalidSystem) {
  int rc = system((kLogCommand + " --start --system M#3").c_str());
  EXPECT_EQ(WEXITSTATUS(rc), EXIT_FAILURE);
}

TEST(LogCommand, SaveFlagInvalidTag) {
  int rc = system((kLogCommand + " --save --tag_name M#3").c_str());
  EXPECT_EQ(WEXITSTATUS(rc), EXIT_FAILURE);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

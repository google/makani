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

#include "common/macros.h"
#include "lib/json_load/json_load_or_die.h"
#include "lib/util/test_util.h"

std::string TestDataPath() {
  return test_util::TestRunfilesDir() + "/lib/json_load/test_data/simple.json";
}

namespace json_load {

TEST(LoadFileOrDieTest, Success) {
  json_t *f = LoadFileOrDie(TestDataPath());
  EXPECT_NE(nullptr, f);
  json_decref(f);
}

TEST(LoadFileOrDieTest, Death) {
  EXPECT_DEATH(LoadFileOrDie("not_a_real_file"), "");
}

class JsonLoadOrDieTest : public ::testing::Test {
 protected:
  JsonLoadOrDieTest() : file_(LoadFileOrDie(TestDataPath())) {}
  ~JsonLoadOrDieTest() { json_decref(file_); }

  json_t *file_;
};

typedef JsonLoadOrDieTest JsonLoadOrDieDeathTest;

TEST_F(JsonLoadOrDieTest, LoadFieldOrDie_Success) {
  json_t *field = LoadFieldOrDie(file_, "some_float");
  EXPECT_NE(nullptr, field);
  EXPECT_TRUE(json_is_real(field));
}

TEST_F(JsonLoadOrDieTest, LoadInt32OrDie_Success) {
  int32_t x = LoadInt32OrDie(file_, "some_int");
  EXPECT_EQ(2, x);
}

TEST_F(JsonLoadOrDieDeathTest, LoadInt32OrDie_Death) {
  EXPECT_DEATH(LoadInt32OrDie(file_, "nonexistent_field"), "");
  EXPECT_DEATH(LoadInt32OrDie(file_, "some_float"), "");
}

}  // namespace json_load

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

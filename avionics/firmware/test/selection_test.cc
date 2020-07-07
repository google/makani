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

#include "avionics/firmware/test/selection.h"

TEST(IsTestSelected, Delimiters) {
  EXPECT_TRUE(IsTestSelected("Suite.Test", "Suite", "Test"));

  // Test comma and space delimiters.
  EXPECT_TRUE(IsTestSelected(", Suite.Test1, Suite.Test2", "Suite", "Test1"));
  EXPECT_TRUE(IsTestSelected("Suite.Test1, Suite.Test2", "Suite", "Test1"));
  EXPECT_TRUE(IsTestSelected("Suite.Test1, Suite.Test2", "Suite", "Test2"));
  EXPECT_FALSE(IsTestSelected("Suite.Test1, Suite.Test2", "Suite", "Test3"));
  EXPECT_TRUE(IsTestSelected(" , Suite.Test1, Suite.Test2", "Suite", "Test1"));
  EXPECT_TRUE(IsTestSelected("Suite.Test1 , Suite.Test2", "Suite", "Test1"));
  EXPECT_TRUE(IsTestSelected("Suite.Test1 , Suite.Test2", "Suite", "Test2"));
  EXPECT_FALSE(IsTestSelected("Suite.Test1 , Suite.Test2", "Suite", "Test3"));
  EXPECT_TRUE(IsTestSelected(" ,Suite.Test1, Suite.Test2", "Suite", "Test1"));
  EXPECT_TRUE(IsTestSelected("Suite.Test1 ,Suite.Test2", "Suite", "Test1"));
  EXPECT_TRUE(IsTestSelected("Suite.Test1 ,Suite.Test2", "Suite", "Test2"));
  EXPECT_FALSE(IsTestSelected("Suite.Test1 ,Suite.Test2", "Suite", "Test3"));

  // Test comma delimiters.
  EXPECT_TRUE(IsTestSelected(",Suite.Test1,Suite.Test2", "Suite", "Test1"));
  EXPECT_TRUE(IsTestSelected("Suite.Test1,Suite.Test2", "Suite", "Test1"));
  EXPECT_TRUE(IsTestSelected("Suite.Test1,Suite.Test2", "Suite", "Test2"));
  EXPECT_FALSE(IsTestSelected("Suite.Test1,Suite.Test2", "Suite", "Test3"));
  EXPECT_TRUE(IsTestSelected(",,Suite.Test1,Suite.Test2", "Suite", "Test1"));
  EXPECT_TRUE(IsTestSelected("Suite.Test1,,Suite.Test2", "Suite", "Test1"));
  EXPECT_TRUE(IsTestSelected("Suite.Test1,,Suite.Test2", "Suite", "Test2"));
  EXPECT_FALSE(IsTestSelected("Suite.Test1,,Suite.Test2", "Suite", "Test3"));

  // Test space delimiters.
  EXPECT_TRUE(IsTestSelected(" Suite.Test1 Suite.Test2", "Suite", "Test1"));
  EXPECT_TRUE(IsTestSelected("Suite.Test1 Suite.Test2", "Suite", "Test1"));
  EXPECT_TRUE(IsTestSelected("Suite.Test1 Suite.Test2", "Suite", "Test2"));
  EXPECT_FALSE(IsTestSelected("Suite.Test1 Suite.Test2", "Suite", "Test3"));
  EXPECT_TRUE(IsTestSelected("  Suite.Test1 Suite.Test2", "Suite", "Test1"));
  EXPECT_TRUE(IsTestSelected("Suite.Test1  Suite.Test2", "Suite", "Test1"));
  EXPECT_TRUE(IsTestSelected("Suite.Test1  Suite.Test2", "Suite", "Test2"));
  EXPECT_FALSE(IsTestSelected("Suite.Test1  Suite.Test2", "Suite", "Test3"));

  // Invalid delimiters.
  EXPECT_FALSE(IsTestSelected("Suite.Test1;Suite.Test2", "Suite", "Test1"));
  EXPECT_FALSE(IsTestSelected("Suite.Test1;Suite.Test2", "Suite", "Test2"));
  EXPECT_FALSE(IsTestSelected("Suite.Test1:Suite.Test2", "Suite", "Test1"));
  EXPECT_FALSE(IsTestSelected("Suite.Test1:Suite.Test2", "Suite", "Test2"));
  EXPECT_FALSE(IsTestSelected("Suite.Test1.Suite.Test2", "Suite", "Test1"));
  EXPECT_FALSE(IsTestSelected("Suite.Test1.Suite.Test2", "Suite", "Test2"));
}

TEST(IsTestSelected, SuiteSelection) {
  EXPECT_TRUE(IsTestSelected("Suite.Test", "Suite", "Test"));

  // Test Suite only syntax.
  EXPECT_TRUE(IsTestSelected("Suite", "Suite", "Test"));
  EXPECT_TRUE(IsTestSelected("SuiteA, SuiteB", "SuiteA", "Test"));
  EXPECT_TRUE(IsTestSelected("SuiteA, SuiteB", "SuiteB", "Test"));
  EXPECT_FALSE(IsTestSelected("SuiteA, SuiteB", "SuiteC", "Test"));

  // Test Suite. syntax.
  EXPECT_TRUE(IsTestSelected("Suite.", "Suite", "Test"));
  EXPECT_TRUE(IsTestSelected("SuiteA., SuiteB.", "SuiteA", "Test"));
  EXPECT_TRUE(IsTestSelected("SuiteA., SuiteB.", "SuiteB", "Test"));
  EXPECT_FALSE(IsTestSelected("SuiteA., SuiteB.", "SuiteC", "Test"));
  EXPECT_FALSE(IsTestSelected("SuiteA., SuiteB.", "Suite", "SuiteA"));
}

TEST(IsTestSelected, TestSelection) {
  EXPECT_TRUE(IsTestSelected("Suite.Test", "Suite", "Test"));

  // Test Test only syntax.
  EXPECT_TRUE(IsTestSelected("Test", "Suite", "Test"));
  EXPECT_TRUE(IsTestSelected("Test1, Test2", "Suite", "Test1"));
  EXPECT_TRUE(IsTestSelected("Test1, Test2", "Suite", "Test2"));
  EXPECT_FALSE(IsTestSelected("Test1, Test2", "Suite", "Test3"));

  // Test .Test syntax.
  EXPECT_TRUE(IsTestSelected(".Test", "Suite", "Test"));
  EXPECT_TRUE(IsTestSelected(".Test1, .Test2", "Suite", "Test1"));
  EXPECT_TRUE(IsTestSelected(".Test1, .Test2", "Suite", "Test2"));
  EXPECT_FALSE(IsTestSelected(".Test1, .Test2", "Suite", "Test3"));
  EXPECT_FALSE(IsTestSelected(".Test1, .Test2", "Test1", "Test"));
}

TEST(IsTestSelected, Wildcards) {
  EXPECT_TRUE(IsTestSelected("Suite.Tes?", "Suite", "Test"));
  EXPECT_TRUE(IsTestSelected("Suite.*e*", "Suite", "Test"));

  EXPECT_TRUE(IsTestSelected("?uite.Test", "Suite", "Test"));
  EXPECT_TRUE(IsTestSelected("*e.Test", "Suite", "Test"));

  EXPECT_TRUE(IsTestSelected("*", "Suite", "Test"));
  EXPECT_TRUE(IsTestSelected("*.", "Suite", "Test"));
  EXPECT_TRUE(IsTestSelected(".*", "Suite", "Test"));
  EXPECT_TRUE(IsTestSelected("*.*", "Suite", "Test"));
}

TEST(IsTestSelected, BadInputs) {
  EXPECT_FALSE(IsTestSelected("", "Suite", "Test"));
  EXPECT_FALSE(IsTestSelected(" ", "Suite", "Test"));
  EXPECT_FALSE(IsTestSelected(",", "Suite", "Test"));
  EXPECT_FALSE(IsTestSelected(".", "Suite", "Test"));
  EXPECT_FALSE(IsTestSelected(" .", "Suite", "Test"));
  EXPECT_FALSE(IsTestSelected(". ", "Suite", "Test"));

  // Test zero length fields (not necessary, but supported).
  EXPECT_TRUE(IsTestSelected(".Test", "", "Test"));
  EXPECT_TRUE(IsTestSelected("Test", "", "Test"));
  EXPECT_TRUE(IsTestSelected("Suite.", "Suite", ""));
  EXPECT_TRUE(IsTestSelected("Suite", "Suite", ""));
  EXPECT_TRUE(IsTestSelected(".", "", ""));
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

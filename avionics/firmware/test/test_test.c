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

#include "avionics/firmware/test/test_test.h"

#include "avionics/firmware/cpu/swi.h"
#include "avionics/firmware/cpu/vim.h"
#include "avionics/firmware/test/test.h"

#define TEST_RUNTIME_US 100000

static void TestSetupFunc(void) {
  VimEnableIrq();
}

static void TestTeardownFunc(void) {
  VimUnregister(kVimChannelSoftware);
}

// Misc.

static void TestEmptyFunction(void) {
}

// Test ASSERT_X() macros.

static void TestAssertPass(void) {
  // Equal.
  ASSERT_EQ(0, 0);
  ASSERT_EQ(10, 10);
  ASSERT_EQ(-15, -15);
  // Not equal.
  ASSERT_NE(1, 0);
  ASSERT_NE(123, 532);
  // Less than or equal.
  ASSERT_LE(231, 231);
  ASSERT_LE(-23, -15);
  // Less than.
  ASSERT_LT(-23, -22);
  ASSERT_LT(-23, 23);
  // Greater than or equal.
  ASSERT_GE(33, 33);
  ASSERT_GE(235, -23);
  // Greater than.
  ASSERT_GT(435, 22);
  ASSERT_GT(23, 22);
  // True.
  ASSERT_TRUE(true);
  // False.
  ASSERT_FALSE(false);
  // Near.
  ASSERT_NEAR(123.0f, 123.1f, 0.2f);
  ASSERT_NEAR(123.0f, 122.9f, 0.2f);
  ASSERT_NEAR(-123.0f, -123.1f, 0.2f);
  ASSERT_NEAR(-123.0f, -122.9f, 0.2f);
}

static void TestAssertEqFail(void) {
  ASSERT_EQ(1, 10);
  EXPECT_FALSE("Should not execute.");
}

static void TestAssertNeFail(void) {
  ASSERT_NE(1, 1);
  EXPECT_FALSE("Should not execute.");
}

static void TestAssertLeFail(void) {
  ASSERT_LE(2, 1);
  EXPECT_FALSE("Should not execute.");
}

static void TestAssertLtFail(void) {
  ASSERT_LT(1, 1);
  EXPECT_FALSE("Should not execute.");
}

static void TestAssertGeFail(void) {
  ASSERT_GE(1, 2);
  EXPECT_FALSE("Should not execute.");
}

static void TestAssertGtFail(void) {
  ASSERT_GT(1, 1);
  EXPECT_FALSE("Should not execute.");
}

static void TestAssertTrueFail(void) {
  ASSERT_TRUE(false);
  EXPECT_FALSE("Should not execute.");
}

static void TestAssertFalseFail(void) {
  ASSERT_FALSE(true);
  EXPECT_FALSE("Should not execute.");
}

static void TestAssertNearFail(void) {
  ASSERT_NEAR(10.0f, 10.2f, 0.1f);
  EXPECT_FALSE("Should not execute.");
}

// Test EXPECT_X() macros.

static void TestExpectPass(void) {
  // Equal.
  EXPECT_EQ(0, 0);
  EXPECT_EQ(10, 10);
  EXPECT_EQ(-15, -15);
  // Not equal.
  EXPECT_NE(1, 0);
  EXPECT_NE(123, 532);
  // Less than or equal.
  EXPECT_LE(231, 231);
  EXPECT_LE(-23, -15);
  // Less than.
  EXPECT_LT(-23, -22);
  EXPECT_LT(-23, 23);
  // Greater than or equal.
  EXPECT_GE(33, 33);
  EXPECT_GE(235, -23);
  // Greater than.
  EXPECT_GT(435, 22);
  EXPECT_GT(23, 22);
  // True.
  EXPECT_TRUE(true);
  // False.
  EXPECT_FALSE(false);
  // Near.
  EXPECT_NEAR(123.0f, 123.1f, 0.2f);
  EXPECT_NEAR(123.0f, 122.9f, 0.2f);
  EXPECT_NEAR(-123.0f, -123.1f, 0.2f);
  EXPECT_NEAR(-123.0f, -122.9f, 0.2f);
}

static void TestExpectEqFail(void) {
  EXPECT_EQ(1, 10);
}

static void TestExpectNeFail(void) {
  EXPECT_NE(1, 1);
}

static void TestExpectLeFail(void) {
  EXPECT_LE(2, 1);
}

static void TestExpectLtFail(void) {
  EXPECT_LT(1, 1);
}

static void TestExpectGeFail(void) {
  EXPECT_GE(1, 2);
}

static void TestExpectGtFail(void) {
  EXPECT_GT(1, 1);
}

static void TestExpectTrueFail(void) {
  EXPECT_TRUE(false);
}

static void TestExpectFalseFail(void) {
  EXPECT_FALSE(true);
}

static void TestExpectNearFail(void) {
  EXPECT_NEAR(10.0f, 10.2f, 0.1f);
}

// Test timeout.

static void TestTimeout(void) {
  while (1) {}
}

// Test interrupt handling.

static void SwiInterruptAssert(void) {
  SwiClearFlag1();
  ASSERT_TRUE(false);
}

static void TestInterruptAssert(void) {
  VimRegisterIrq(kVimChannelSoftware, SwiInterruptAssert);
  VimEnableInterrupt(kVimChannelSoftware);
  SwiClearFlag1();
  SwiTriggerInterrupt1(0x01);
}

static void SwiInterruptExpect(void) {
  SwiClearFlag1();
  EXPECT_TRUE(false);
}

static void TestInterruptExpect(void) {
  VimRegisterIrq(kVimChannelSoftware, SwiInterruptExpect);
  VimEnableInterrupt(kVimChannelSoftware);
  SwiClearFlag1();
  SwiTriggerInterrupt1(0x01);
}

static void SwiInterruptTimeout(void) {
  // If we do not clear the software interrupt flag, this interrupt will fire
  // again during the context restore and re-enter the infinite loop.
  SwiClearFlag1();
  while (1) {}
}

static void TestInterruptTimeout(void) {
  VimRegisterIrq(kVimChannelSoftware, SwiInterruptTimeout);
  VimEnableInterrupt(kVimChannelSoftware);
  SwiClearFlag1();
  SwiTriggerInterrupt1(0x01);
  EXPECT_FALSE("Should not execute.");
}

static const TestConfig kTestTests[] = {
  // Misc.
  TEST_CONFIG_EXPECT_SUCCESS(TestEmptyFunction, TEST_RUNTIME_US),
  // Test ASSERT_X() macros.
  TEST_CONFIG_EXPECT_SUCCESS(TestAssertPass, TEST_RUNTIME_US),
  TEST_CONFIG_EXPECT_ASSERT(TestAssertEqFail, TEST_RUNTIME_US),
  TEST_CONFIG_EXPECT_ASSERT(TestAssertNeFail, TEST_RUNTIME_US),
  TEST_CONFIG_EXPECT_ASSERT(TestAssertLeFail, TEST_RUNTIME_US),
  TEST_CONFIG_EXPECT_ASSERT(TestAssertLtFail, TEST_RUNTIME_US),
  TEST_CONFIG_EXPECT_ASSERT(TestAssertGeFail, TEST_RUNTIME_US),
  TEST_CONFIG_EXPECT_ASSERT(TestAssertGtFail, TEST_RUNTIME_US),
  TEST_CONFIG_EXPECT_ASSERT(TestAssertTrueFail, TEST_RUNTIME_US),
  TEST_CONFIG_EXPECT_ASSERT(TestAssertFalseFail, TEST_RUNTIME_US),
  TEST_CONFIG_EXPECT_ASSERT(TestAssertNearFail, TEST_RUNTIME_US),
  // Test EXPECT_X() macros.
  TEST_CONFIG_EXPECT_SUCCESS(TestExpectPass, TEST_RUNTIME_US),
  TEST_CONFIG_EXPECT_FAILURE(TestExpectEqFail, TEST_RUNTIME_US),
  TEST_CONFIG_EXPECT_FAILURE(TestExpectNeFail, TEST_RUNTIME_US),
  TEST_CONFIG_EXPECT_FAILURE(TestExpectLeFail, TEST_RUNTIME_US),
  TEST_CONFIG_EXPECT_FAILURE(TestExpectLtFail, TEST_RUNTIME_US),
  TEST_CONFIG_EXPECT_FAILURE(TestExpectGeFail, TEST_RUNTIME_US),
  TEST_CONFIG_EXPECT_FAILURE(TestExpectGtFail, TEST_RUNTIME_US),
  TEST_CONFIG_EXPECT_FAILURE(TestExpectTrueFail, TEST_RUNTIME_US),
  TEST_CONFIG_EXPECT_FAILURE(TestExpectFalseFail, TEST_RUNTIME_US),
  TEST_CONFIG_EXPECT_FAILURE(TestExpectNearFail, TEST_RUNTIME_US),
  // Test timeout.
  TEST_CONFIG_EXPECT_TIMEOUT(TestTimeout, TEST_RUNTIME_US),
  // Test interrupt handling.
  TEST_CONFIG_EXPECT_ASSERT(TestInterruptAssert, TEST_RUNTIME_US),
  TEST_CONFIG_EXPECT_FAILURE(TestInterruptExpect, TEST_RUNTIME_US),
  TEST_CONFIG_EXPECT_TIMEOUT(TestInterruptTimeout, TEST_RUNTIME_US)
};

const TestSuite kTestTest =
    TEST_SUITE_INIT(kTestTests, TestSetupFunc, TestTeardownFunc);

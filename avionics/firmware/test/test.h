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

#ifndef AVIONICS_FIRMWARE_TEST_TEST_H_
#define AVIONICS_FIRMWARE_TEST_TEST_H_

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "avionics/firmware/test/types.h"
#include "common/macros.h"

// Helper macro for ASSERT/EXPECT macros.
#define TEST_FUNC(func, condition)                              \
  func(STR(condition), __FILE__, __func__, __LINE__, condition)

// ASSERT functions abort the execution of current test.
#define ASSERT_EQ(a, b) TEST_FUNC(TestAssert, (a) == (b))
#define ASSERT_NE(a, b) TEST_FUNC(TestAssert, (a) != (b))
#define ASSERT_LE(a, b) TEST_FUNC(TestAssert, (a) <= (b))
#define ASSERT_LT(a, b) TEST_FUNC(TestAssert, (a) < (b))
#define ASSERT_GE(a, b) TEST_FUNC(TestAssert, (a) >= (b))
#define ASSERT_GT(a, b) TEST_FUNC(TestAssert, (a) > (b))

#define ASSERT_TRUE(a)  TEST_FUNC(TestAssert, (a))
#define ASSERT_FALSE(a) TEST_FUNC(TestAssert, !(a))

#define ASSERT_NEAR(a, b, tol) TEST_FUNC(TestAssert, fabsf((a) - (b)) < (tol))

#define EXPECT_EQ(a, b) TEST_FUNC(TestExpect, (a) == (b))
#define EXPECT_NE(a, b) TEST_FUNC(TestExpect, (a) != (b))
#define EXPECT_LE(a, b) TEST_FUNC(TestExpect, (a) <= (b))
#define EXPECT_LT(a, b) TEST_FUNC(TestExpect, (a) < (b))
#define EXPECT_GE(a, b) TEST_FUNC(TestExpect, (a) >= (b))
#define EXPECT_GT(a, b) TEST_FUNC(TestExpect, (a) > (b))

#define EXPECT_TRUE(a)  TEST_FUNC(TestExpect, (a))
#define EXPECT_FALSE(a) TEST_FUNC(TestExpect, !(a))

#define EXPECT_NEAR(a, b, tol) TEST_FUNC(TestExpect, fabsf((a) - (b)) < (tol))

void TestInit(void);

// Called by ASSERT/EXPECT macros.
void TestAssert(const char *condition, const char *file, const char *func,
                int32_t line, bool pass);
bool TestExpect(const char *condition, const char *file, const char *func,
                int32_t line, bool pass);

// Helper function to measure timing statistics.
void TestTiming(TestFunction func, int32_t count, float expected_avg_usec,
                float expected_max_usec);
void TestTimingWithDelay(TestFunction func, int32_t count, float delay_usec,
                         float expected_avg_usec, float expected_max_usec);

// For automatic execution.
bool RunTestExpectSuccess(const char *suite_name, const char *test_name,
                          TestSetupFunction setup_func, TestFunction func,
                          TestTeardownFunction teardown_func,
                          int32_t timeout_usec);
bool RunTestExpectFailure(const char *suite_name, const char *test_name,
                          TestSetupFunction setup_func, TestFunction func,
                          TestTeardownFunction teardown_func,
                          int32_t timeout_usec);
bool RunTestExpectAssert(const char *suite_name, const char *test_name,
                         TestSetupFunction setup_func, TestFunction func,
                         TestTeardownFunction teardown_func,
                         int32_t timeout_usec);
bool RunTestExpectTimeout(const char *suite_name, const char *test_name,
                          TestSetupFunction setup_func, TestFunction func,
                          TestTeardownFunction teardown_func,
                          int32_t timeout_usec);
bool RunTest(const char *suite_name, const TestConfig *test,
             TestSetupFunction setup_func, TestTeardownFunction teardown_func);
bool RunTests(int32_t num_suites, const TestSuite **suites);

#endif  // AVIONICS_FIRMWARE_TEST_TEST_H_

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

#ifndef AVIONICS_FIRMWARE_TEST_TYPES_H_
#define AVIONICS_FIRMWARE_TEST_TYPES_H_

#include <stdbool.h>
#include <stdint.h>

#include "common/macros.h"

// Helper macros for initializing an array of TestConfig tests.
#define TEST_CONFIG_EXPECT_SUCCESS(func, timeout_usec)  \
  {STR(func), func, false, false, false, timeout_usec}
#define TEST_CONFIG_EXPECT_ASSERT(func, timeout_usec)   \
  {STR(func), func, true, false, false, timeout_usec}
#define TEST_CONFIG_EXPECT_FAILURE(func, timeout_usec)  \
  {STR(func), func, false, true, false, timeout_usec}
#define TEST_CONFIG_EXPECT_TIMEOUT(func, timeout_usec)  \
  {STR(func), func, false, false, true, timeout_usec}
#define TEST_CONFIG_INIT(func, timeout_usec) \
  TEST_CONFIG_EXPECT_SUCCESS(func, timeout_usec)

// Helper macro for initializing a TestSuite.
#define TEST_SUITE_INIT(test_array, setup_fn, teardown_fn)            \
    {.name = STR(test_array),                                         \
     .test = test_array,                                              \
     .num_tests = ARRAYSIZE(test_array),                              \
     .setup_func = setup_fn,                                          \
     .teardown_func = teardown_fn}

typedef void (* const TestFunction)(void);
typedef void (* const TestSetupFunction)(void);
typedef void (* const TestTeardownFunction)(void);

typedef struct {
  const char *name;
  TestFunction func;
  bool expect_assert;
  bool expect_failure;
  bool expect_timeout;
  int32_t timeout_usec;  // Maximum allowed execution time before abort.
} TestConfig;

typedef struct {
  const char *name;
  const TestConfig *test;
  int32_t num_tests;
  TestSetupFunction setup_func;
  TestTeardownFunction teardown_func;
} TestSuite;

#endif  // AVIONICS_FIRMWARE_TEST_TYPES_H_

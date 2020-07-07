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

#include "avionics/firmware/test/test.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "avionics/firmware/cpu/clock.h"
#include "avionics/firmware/cpu/rti.h"
#include "avionics/firmware/cpu/vim.h"
#include "avionics/firmware/startup/cpu_tms570.h"
#include "avionics/firmware/test/output.h"
#include "common/macros.h"

#define MAXIMUM_RUNTIME_USEC                            \
  ((int32_t)CLOCK32_CYCLES_TO_USEC_F((float)INT32_MAX))

// Status of current test.
static bool g_expect_assert = false;
static bool g_expect_failure = false;
static volatile bool g_got_timeout_abort = false;
static volatile bool g_got_assert_failure = false;
static volatile bool g_got_expect_failure = false;

// Called upon expiration of RTI compare interrupt 3.
static void TimeoutInterrupt(void) {
  // Disable this callback.
  RtiDisablePeriodicInterrupt3();
  RtiClearPeriodicFlag3();

  // Resume processing to last StartupCpuSaveContext.
  g_got_timeout_abort = true;
  StartupCpuRestoreContext();
}

static void ProcessTest(TestSetupFunction setup_func, TestFunction func,
                        TestTeardownFunction teardown_func,
                        int32_t timeout_usec, int32_t *runtime_usec) {
  ASSERT_GT(timeout_usec, 0);
  ASSERT_LT(timeout_usec, MAXIMUM_RUNTIME_USEC);

  // Schedule timeout interrupt.
  uint32_t now = RtiGetPeriodicTimer();
  RtiEnablePeriodicInterrupt3(now + timeout_usec, INT32_MAX);

  // Setup function can timeout (don't trust).
  if (setup_func != NULL) {
    setup_func();
  }

  // Time run-time of function in cycles (no peripheral delay).
  uint32_t start_cycles = Clock32GetCycles();
  func();
  uint32_t stop_cycles = Clock32GetCycles();

  // Teardown function can timeout (don't trust).
  if (teardown_func != NULL) {
    teardown_func();
  }

  // Disable timeout interrupt.
  RtiDisablePeriodicInterrupt3();

  // Compute runtime.
  float run_cycles = (float)CLOCK32_SUBTRACT(stop_cycles, start_cycles);
  float run_usec = CLOCK32_CYCLES_TO_USEC_F(run_cycles);
  *runtime_usec = (int32_t)run_usec;
}

static bool ProcessResult(const TestConfig *test) {
  bool pass = true;

  // Reset global variables in case we call EXPECT_FALSE below.
  g_expect_assert = false;
  g_expect_failure = false;

  // Report assert failure.
  if (test->expect_assert && !g_got_assert_failure) {
    EXPECT_FALSE("Test function did not assert!");
    pass = false;
  }

  // Report expect failure.
  if (test->expect_failure && !g_got_expect_failure) {
    EXPECT_FALSE("Test function did not fail!");
    pass = false;
  }

  // Report timeout failure.
  if (test->expect_timeout && !g_got_timeout_abort) {
    EXPECT_FALSE("Test function did not timeout!");
    pass = false;
  } else if (!test->expect_timeout && g_got_timeout_abort) {
    EXPECT_FALSE("Test function timed out!");
    pass = false;
  }
  return pass;
}

// Application functions.

void TestInit(void) {
  TestOutputInit();
  RtiEnablePeriodicTimer();
  RtiDisablePeriodicInterrupt3();
  VimRegisterFiq(kVimChannelRtiCompare3, TimeoutInterrupt);
  VimEnableInterrupt(kVimChannelRtiCompare3);
}

bool TestExpect(const char *condition, const char *file, const char *func,
                int32_t line, bool pass) {
  g_got_expect_failure |= !pass;
  if (pass != !g_expect_failure) {
    // Prevent timeout from occurring during output.
    uint32_t vim_state = VimDisableInterrupt(kVimChannelRtiCompare3);
    TestOutputExpectCondition(condition, file, func, line, pass);
    VimRestoreInterrupt(kVimChannelRtiCompare3, vim_state);
  }
  return pass;
}

void TestAssert(const char *condition, const char *file, const char *func,
                int32_t line, bool pass) {
  g_got_assert_failure |= !pass;
  if (!pass) {
    // Prevent TimeoutInterrupt from occurring during restore.
    RtiDisablePeriodicInterrupt3();

    // Report result.
    if (pass != !g_expect_assert) {
      TestOutputAssertCondition(condition, file, func, line, pass);
    }

    // Resume processing to last StartupCpuSaveContext.
    StartupCpuRestoreContext();
  }
}

void TestTiming(TestFunction func, int32_t count, float expected_avg_usec,
                float expected_max_usec) {
  TestTimingWithDelay(func, count, 0.0f, expected_avg_usec, expected_max_usec);
}

void TestTimingWithDelay(TestFunction func, int32_t count, float delay_usec,
                         float expected_avg_usec, float expected_max_usec) {
  ASSERT_NE(func, NULL);
  ASSERT_GT(count, 0);
  ASSERT_GE(delay_usec, 0.0f);

  uint32_t max_cycles = 0U;
  uint32_t start_cycles = Clock32GetCycles();
  uint32_t last_cycles = start_cycles;
  int32_t delay_cycles = (int32_t)CLOCK32_USEC_TO_CYCLES(delay_usec);
  for (int32_t i = 0; i < count; ++i) {
    func();

    // Account for overhead in thresholds.
    uint32_t now_cycles = Clock32GetCycles();
    uint32_t delta_cycles = now_cycles - last_cycles;
    if (delta_cycles > max_cycles) {
      max_cycles = delta_cycles;
    }
    last_cycles = now_cycles;

    // Delay for hardware to respond.
    while (CLOCK32_SUBTRACT(now_cycles, last_cycles) < delay_cycles) {
      now_cycles = Clock32GetCycles();
    }
    last_cycles = now_cycles;
  }
  int32_t total_cycles = last_cycles - start_cycles - delay_cycles * count;
  float avg_cycles = (float)total_cycles / (float)count;
  float avg_usec = CLOCK32_CYCLES_TO_USEC_F(avg_cycles);
  float max_usec = CLOCK32_CYCLES_TO_USEC_F((float)max_cycles);

  printf("Test average run time was %.1f usec (expected <= %.1f usec).\n",
         avg_usec, expected_avg_usec);
  printf("Test maximum run time was %.1f usec (expected <= %.1f usec).\n",
         max_usec, expected_max_usec);

  EXPECT_LE(avg_usec, expected_avg_usec);
  EXPECT_LE(max_usec, expected_max_usec);
}

bool RunTest(const char *suite_name, const TestConfig *test,
             TestSetupFunction setup_func, TestTeardownFunction teardown_func) {
  assert(suite_name != NULL);
  assert(test != NULL);

  // Initialize globals.
  g_expect_assert = test->expect_assert;
  g_expect_failure = test->expect_failure;
  g_got_timeout_abort = false;   // Set by TimeoutInterrupt().
  g_got_assert_failure = false;  // Set by ASSERT_X() macro failure.
  g_got_expect_failure = false;  // Set by EXPECT_X() macro failure.

  // Set default runtime. If the test asserts or times out, we report
  // this value in the result.
  int32_t runtime_usec = -1;

  // Report test start.
  TestOutputStart(suite_name, test);

  // Store context to enable TestAssert() functionality. Upon failure,
  // TestAssert() will restore the context and begin executing code
  // immediately following StartupCpuSaveContext().
  bool restored = StartupCpuSaveContext();

  // Do not re-execute function upon StartupCpuRestoreContext().
  if (!restored) {
    ProcessTest(setup_func, test->func, teardown_func, test->timeout_usec,
                &runtime_usec);
  }

  bool pass = ProcessResult(test);

  // Report result.
  TestOutputResult(suite_name, test, pass, runtime_usec);
  return pass;
}

bool RunTestExpectSuccess(const char *suite_name, const char *test_name,
                          TestSetupFunction setup_func, TestFunction func,
                          TestTeardownFunction teardown_func,
                          int32_t timeout_usec) {
  const TestConfig test = {
    .name = test_name,
    .func = func,
    .expect_assert = false,
    .expect_failure = false,
    .expect_timeout = false,
    .timeout_usec = timeout_usec
  };
  return RunTest(suite_name, &test, setup_func, teardown_func);
}

bool RunTestExpectFailure(const char *suite_name, const char *test_name,
                          TestSetupFunction setup_func, TestFunction func,
                          TestTeardownFunction teardown_func,
                          int32_t timeout_usec) {
  const TestConfig test = {
    .name = test_name,
    .func = func,
    .expect_assert = false,
    .expect_failure = true,
    .expect_timeout = false,
    .timeout_usec = timeout_usec
  };
  return RunTest(suite_name, &test, setup_func, teardown_func);
}

bool RunTestExpectAssert(const char *suite_name, const char *test_name,
                         TestSetupFunction setup_func, TestFunction func,
                         TestTeardownFunction teardown_func,
                         int32_t timeout_usec) {
  const TestConfig test = {
    .name = test_name,
    .func = func,
    .expect_assert = true,
    .expect_failure = false,
    .expect_timeout = false,
    .timeout_usec = timeout_usec
  };
  return RunTest(suite_name, &test, setup_func, teardown_func);
}

bool RunTestExpectTimeout(const char *suite_name, const char *test_name,
                          TestSetupFunction setup_func, TestFunction func,
                          TestTeardownFunction teardown_func,
                          int32_t timeout_usec) {
  const TestConfig test = {
    .name = test_name,
    .func = func,
    .expect_assert = false,
    .expect_failure = false,
    .expect_timeout = true,
    .timeout_usec = timeout_usec
  };
  return RunTest(suite_name, &test, setup_func, teardown_func);
}

bool RunTests(int32_t num_suites, const TestSuite **suites) {
  assert(suites != NULL);
  TestOutputInit();

  bool fail = false;
  for (int32_t s = 0; s < num_suites; ++s) {
    const TestSuite *suite = suites[s];
    for (int32_t t = 0; t < suite->num_tests; ++t) {
      fail |= !RunTest(suite->name, &suite->test[t], suite->setup_func,
                       suite->teardown_func);
    }
  }
  return !fail;
}

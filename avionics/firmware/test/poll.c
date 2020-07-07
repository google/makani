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

#include "avionics/firmware/test/poll.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "avionics/common/avionics_messages.h"
#include "avionics/common/cvt_avionics_messages.h"
#include "avionics/firmware/identity/identity.h"
#include "avionics/firmware/test/output.h"
#include "avionics/firmware/test/selection.h"
#include "avionics/firmware/test/test.h"

// Roll over test index to next suite index when necessary.
static bool CarryTestIndex(int32_t num_suites, const TestSuite **suites,
                           int32_t *cur_suite, int32_t *cur_test) {
  if (0 <= *cur_suite && *cur_suite < num_suites) {
    if (*cur_test >= suites[*cur_suite]->num_tests) {
      *cur_test = 0;
      ++(*cur_suite);
      return CarryTestIndex(num_suites, suites, cur_suite, cur_test);
    }
    return true;
  }
  return false;
}

static bool GetNextTestIndex(int32_t num_suites, const TestSuite **suites,
                             int32_t *cur_suite, int32_t *cur_test) {
  ++(*cur_test);
  return CarryTestIndex(num_suites, suites, cur_suite, cur_test);
}

static bool GetNextSelectedTest(const char *selection, int32_t num_suites,
                                const TestSuite **suites, int32_t *cur_suite,
                                int32_t *cur_test) {
  assert(selection != NULL);
  assert(suites != NULL);
  assert(cur_suite != NULL);
  assert(cur_test != NULL);

  int32_t next_suite = *cur_suite;
  int32_t next_test = *cur_test;
  while (GetNextTestIndex(num_suites, suites, &next_suite, &next_test)) {
    assert(0 <= next_suite && next_suite < num_suites);
    const TestSuite *suite = suites[next_suite];
    assert(0 <= next_test && next_test < suite->num_tests);
    const TestConfig *test = &suite->test[next_test];
    if (IsTestSelected(selection, suite->name, test->name)) {
      *cur_suite = next_suite;
      *cur_test = next_test;
      return true;
    }
  }
  return false;
}

void TestPoll(int32_t num_suites, const TestSuite **suites) {
  assert(suites != NULL);

  static TestExecuteMessage current_exec;
  static int32_t current_suite = -1;
  static int32_t current_test = -1;

  // Reset test routine on new TestExecuteMessage.
  TestExecuteMessage exec;
  if (CvtGetTestExecuteMessage(kAioNodeOperator, &exec, NULL, NULL)
      && exec.node == AppConfigGetAioNode()) {
    // Initialize execute command.
    current_exec = exec;
    current_suite = 0;
    current_test = -1;
    TestOutputInit();
  }

  // Execute one test function for each iteration.
  if (GetNextSelectedTest(current_exec.list, num_suites, suites,
                          &current_suite, &current_test)) {
    const TestSuite *suite = suites[current_suite];
    const TestConfig *test = &suite->test[current_test];
    RunTest(suite->name, test, suite->setup_func, suite->teardown_func);
  } else {
    TestOutputStatus();
  }
}

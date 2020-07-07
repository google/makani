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

#include "avionics/firmware/test/output.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "avionics/common/avionics_messages.h"
#include "avionics/common/build_info.h"
#include "avionics/firmware/cpu/mode.h"
#include "avionics/firmware/identity/identity.h"
#include "avionics/firmware/network/net_send.h"
#include "common/macros.h"

#define MAX_TEST_FAILURE_MESSAGES 10

// Initialization functions.

static void InitTestStatusMessage(TestStatusMessage *status) {
  memset(status, 0, sizeof(*status));
  GetBuildInfo(&status->build_info);
  strlcpy(status->node, AioNodeToString(AppConfigGetAioNode()),
          ARRAYSIZE(status->node));
  status->busy = true;
}

static void InitTestResult(const char *suite_name, const char *test_name,
                           int32_t index, TestResult *result) {
  memset(result, 0, sizeof(*result));
  strlcpy(result->suite, suite_name, ARRAYSIZE(result->suite));
  strlcpy(result->test, test_name, ARRAYSIZE(result->test));
  result->index = index;
}

// Network send functions.

static void SendTestStartMessage(const char *suite_name, const TestConfig *test,
                                 int32_t index) {
  static TestStartMessage msg;
  memset(&msg, 0, sizeof(msg));
  strlcpy(msg.suite, suite_name, ARRAYSIZE(msg.suite));
  strlcpy(msg.test, test->name, ARRAYSIZE(msg.test));
  msg.index = index;
  msg.timeout_usec = test->timeout_usec;
  NetSendAioTestStartMessage(&msg);
  printf("Running test %ld: %s.%s.\n", index, suite_name, test->name);
}

static void SendTestFailureMessage(int32_t index, const char *condition,
                                   const char *file, const char *func,
                                   int32_t line, bool pass) {
  static TestFailureMessage msg;
  memset(&msg, 0, sizeof(msg));
  msg.index = index;
  strlcpy(msg.condition, condition, ARRAYSIZE(msg.condition));
  strlcpy(msg.file, file, ARRAYSIZE(msg.file));
  strlcpy(msg.func, func, ARRAYSIZE(msg.func));
  msg.line = line;
  msg.value = pass;
  NetSendAioTestFailureMessage(&msg);
  printf("Test %ld WARNING: Expected %s%s in (%s %s:%ld).\n",
         index, pass ? "!" : "", condition, func, file, line);
}

// Increment failures.

static void IncrementFailures(TestStatusMessage *status) {
  if (status->result.failures < INT32_MAX) {
    ++status->result.failures;
  }
  if (status->num_failures < INT32_MAX) {
    ++status->num_failures;
  }
}

// Test status since TestOutputInit().
static TestStatusMessage g_status;

// Application functions.

void TestOutputInit(void) {
  InitTestStatusMessage(&g_status);
}

void TestOutputStart(const char *suite_name, const TestConfig *test) {
  int32_t index = g_status.result.index + 1;
  InitTestResult(suite_name, test->name, index, &g_status.result);
  SendTestStartMessage(suite_name, test, index);
}

void TestOutputAssertCondition(const char *condition, const char *file,
                               const char *func, int32_t line, bool pass) {
  if (g_status.result.failures < MAX_TEST_FAILURE_MESSAGES
      && !IsInterruptMode()) {
    SendTestFailureMessage(g_status.result.index, condition, file, func, line,
                           pass);
  }
  IncrementFailures(&g_status);
}

void TestOutputExpectCondition(const char *condition, const char *file,
                               const char *func, int32_t line, bool pass) {
  if (g_status.result.failures < MAX_TEST_FAILURE_MESSAGES
      && !IsInterruptMode()) {
    SendTestFailureMessage(g_status.result.index, condition, file, func, line,
                           pass);
  }
  IncrementFailures(&g_status);
}

void TestOutputResult(const char *suite_name, const TestConfig *test,
                      bool pass, int32_t runtime_usec) {
  g_status.result.runtime_usec = runtime_usec;
  if (pass) {
    printf("Test %ld: %s.%s PASSED (runtime=%ld usec).\n",
           g_status.result.index, suite_name, test->name, runtime_usec);
  } else {
    printf("Test %ld: %s.%s FAILED (runtime=%ld usec, failures=%ld).\n",
           g_status.result.index, suite_name, test->name, runtime_usec,
           g_status.result.failures);
  }
  NetSendAioTestStatusMessage(&g_status);
}

void TestOutputStatus(void) {
  g_status.busy = false;
  NetSendAioTestStatusMessage(&g_status);
}

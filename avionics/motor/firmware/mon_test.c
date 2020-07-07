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

#include "avionics/motor/firmware/mon_test.h"

#include "avionics/firmware/cpu/clock.h"
#include "avionics/firmware/serial/board_serial.h"
#include "avionics/firmware/test/test.h"
#include "avionics/motor/firmware/mon.h"

#define MOTOR_MON_POLL_CALLS 10000
#define MOTOR_MON_PERIOD_US  10
#define MOTOR_MON_RUNTIME_US (MOTOR_MON_POLL_CALLS * MOTOR_MON_PERIOD_US)

static void MotorMonTestSetup(void) {
  // Call GetBoardSerialParams() before the test starts to ensure that
  // old serial parameters are read, converted, and cached before the test
  // runs.
  GetBoardSerialParams();
  MotorMonInit();
}

static void MotorMonTestTeardown(void) {
}

static void CallMotorMonPoll(void) {
  MotorMonPoll();
}

static void TestMotorMonPollTiming(void) {
  TestTimingWithDelay(CallMotorMonPoll, MOTOR_MON_POLL_CALLS,
                      MOTOR_MON_PERIOD_US, 5.0f, 6.5f);
}

static const TestConfig kMotorMonTests[] = {
  TEST_CONFIG_INIT(TestMotorMonPollTiming, 2 * MOTOR_MON_RUNTIME_US)
};

const TestSuite kMotorMonTest = TEST_SUITE_INIT(kMotorMonTests,
                                                MotorMonTestSetup,
                                                MotorMonTestTeardown);

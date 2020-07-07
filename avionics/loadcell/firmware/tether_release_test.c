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

#include "avionics/loadcell/firmware/tether_release_test.h"

#include <stdbool.h>
#include <stdint.h>

#include "avionics/common/safety_codes.h"
#include "avionics/firmware/cpu/clock.h"
#include "avionics/firmware/test/test.h"
#include "avionics/loadcell/firmware/tether_release.h"

#define ARM_DELAY_CYCLES CLOCK32_USEC_TO_CYCLES(4)
#define FIRE_DELAY_CYCLES CLOCK32_MSEC_TO_CYCLES(10)
#define FIRE_TIME_CYCLES CLOCK32_MSEC_TO_CYCLES(3500)
#define SELFTEST_RUNTIME_CYCLES CLOCK32_MSEC_TO_CYCLES(1000)
#define MAX_TEST_RUNTIME_US 5000000

static void TestArmSignal(void) {
  TetherReleaseInit();

  EXPECT_FALSE(TetherReleaseArmed());
  EXPECT_FALSE(TetherReleaseFiring());

  TetherReleaseArm();
  Clock32WaitCycles(ARM_DELAY_CYCLES);

  EXPECT_TRUE(TetherReleaseArmed());
  EXPECT_FALSE(TetherReleaseFiring());

  TetherReleaseDisarm();
  Clock32WaitCycles(ARM_DELAY_CYCLES);

  EXPECT_FALSE(TetherReleaseArmed());
  EXPECT_FALSE(TetherReleaseFiring());
}

static void TestFireNoArm(void) {
  TetherReleaseInit();

  EXPECT_FALSE(TetherReleaseArmed());
  EXPECT_FALSE(TetherReleaseFiring());

  uint32_t start_time = Clock32GetCycles();
  // Allow time for the tether to fire.
  while (CLOCK32_LT(Clock32GetCycles(),
                    start_time + FIRE_DELAY_CYCLES)) {
    EXPECT_FALSE(TetherReleaseFire(TETHER_RELEASE_SAFETY_CODE));
  }
  EXPECT_FALSE(TetherReleaseArmed());
  EXPECT_TRUE(TetherReleaseFiring());

  // Delay up to 3500 ms for the tether to stop firing.
  while (CLOCK32_LT(Clock32GetCycles(),
                    start_time + FIRE_TIME_CYCLES)) {
    if (TetherReleaseFire(TETHER_RELEASE_SAFETY_CODE)) {
      break;
    }
    EXPECT_TRUE(TetherReleaseFiring());
  }

  Clock32WaitCycles(FIRE_DELAY_CYCLES);

  EXPECT_FALSE(TetherReleaseArmed());
  EXPECT_FALSE(TetherReleaseFiring());
}

static void TestFireSignal(void) {
  TetherReleaseInit();

  EXPECT_FALSE(TetherReleaseArmed());
  EXPECT_FALSE(TetherReleaseFiring());

  TetherReleaseArm();
  Clock32WaitCycles(ARM_DELAY_CYCLES);

  EXPECT_TRUE(TetherReleaseArmed());
  EXPECT_FALSE(TetherReleaseFiring());

  uint32_t start_time = Clock32GetCycles();
  // Allow time for the tether to fire.
  while (CLOCK32_LT(Clock32GetCycles(),
                    start_time + FIRE_DELAY_CYCLES)) {
    EXPECT_FALSE(TetherReleaseFire(TETHER_RELEASE_SAFETY_CODE));
  }
  EXPECT_TRUE(TetherReleaseArmed());
  EXPECT_TRUE(TetherReleaseFiring());

  // Verify tether release doesn't disarm during fire.
  TetherReleaseDisarm();
  Clock32WaitCycles(ARM_DELAY_CYCLES);
  EXPECT_TRUE(TetherReleaseArmed());

  // Delay up to 3500 ms for the tether to stop firing.
  while (CLOCK32_LT(Clock32GetCycles(),
                    start_time + FIRE_TIME_CYCLES)) {
    if (TetherReleaseFire(TETHER_RELEASE_SAFETY_CODE)) {
      break;
    }
    EXPECT_TRUE(TetherReleaseFiring());
  }

  Clock32WaitCycles(FIRE_DELAY_CYCLES);

  EXPECT_TRUE(TetherReleaseArmed());
  EXPECT_FALSE(TetherReleaseFiring());

  TetherReleaseDisarm();
  Clock32WaitCycles(ARM_DELAY_CYCLES);

  EXPECT_FALSE(TetherReleaseArmed());
}

static void TestFireInvalidCode(void) {
  TetherReleaseInit();

  EXPECT_FALSE(TetherReleaseArmed());
  EXPECT_FALSE(TetherReleaseFiring());

  uint32_t start_time = Clock32GetCycles();
  // Allow time for the tether to fire.
  while (CLOCK32_LT(Clock32GetCycles(),
                    start_time + FIRE_DELAY_CYCLES)) {
    EXPECT_FALSE(TetherReleaseFire(0x12345678));
  }

  EXPECT_FALSE(TetherReleaseArmed());
  EXPECT_FALSE(TetherReleaseFiring());
}

static void RunSelftest(TetherReleaseSelftestResult expected_result,
                        float v_test, float v_arm, float v_release,
                        float v_test_fire, float v_arm_fire,
                        float v_release_fire) {
  TetherReleaseInit();

  EXPECT_FALSE(TetherReleaseArmed());
  EXPECT_FALSE(TetherReleaseFiring());

  uint32_t start_time = Clock32GetCycles();
  TetherReleaseSelftestResult result;
  while (CLOCK32_LT(Clock32GetCycles(),
                    start_time + SELFTEST_RUNTIME_CYCLES)) {
    if (TetherReleaseFiring()) {
      result = TetherReleaseSelftest(v_test_fire, v_arm_fire, v_release_fire);
    } else {
      result = TetherReleaseSelftest(v_test, v_arm, v_release);
    }
    if (result != kTetherReleaseSelftestRunning) {
      break;
    }
  }

  EXPECT_EQ(result, expected_result);

  EXPECT_FALSE(TetherReleaseArmed());
  EXPECT_FALSE(TetherReleaseFiring());
}

static void TestSelftestPass(void) {
  RunSelftest(kTetherReleaseSelftestPassed,
              28.0f, 7.0f, 7.0f,
              28.0f, 0.1f, 0.0f);
}

static void TestSelftestLowBattery(void) {
  RunSelftest(kTetherReleaseSelftestLowBattery,
              20.0f, 5.0f, 5.0f,
              20.0f, 0.1f, 0.0f);
}

static void TestSelftestBatteryDisconnected(void) {
  RunSelftest(kTetherReleaseSelftestBatteryDisconnected,
              0.0f, 0.0f, 0.0f,
              0.0f, 0.0f, 0.0f);
}

static void TestSelftestReleaseFailShort(void) {
  RunSelftest(kTetherReleaseSelftestReleaseCircuitFailedShort,
              28.0f, 0.1f, 0.0f,
              28.0f, 0.1f, 0.0f);
}

static void TestSelftestReleaseFailOpen(void) {
  RunSelftest(kTetherReleaseSelftestReleaseCircuitFailedOpen,
              28.0f, 7.0f, 7.0f,
              28.0f, 7.0f, 7.0f);
}

static void TestSelftestReleaseDisconnected(void) {
  RunSelftest(kTetherReleaseSelftestReleaseDisconnected,
              28.0f, 6.0f, 5.0f,
              28.0f, 1.5f, 0.0f);
}

static const TestConfig kTetherReleaseTests[] = {
  TEST_CONFIG_INIT(TestArmSignal, MAX_TEST_RUNTIME_US),
  TEST_CONFIG_INIT(TestFireNoArm, MAX_TEST_RUNTIME_US),
  TEST_CONFIG_INIT(TestFireSignal, MAX_TEST_RUNTIME_US),
  TEST_CONFIG_INIT(TestFireInvalidCode, MAX_TEST_RUNTIME_US),
  TEST_CONFIG_INIT(TestSelftestPass, MAX_TEST_RUNTIME_US),
  TEST_CONFIG_INIT(TestSelftestLowBattery, MAX_TEST_RUNTIME_US),
  TEST_CONFIG_INIT(TestSelftestBatteryDisconnected, MAX_TEST_RUNTIME_US),
  TEST_CONFIG_INIT(TestSelftestReleaseFailShort, MAX_TEST_RUNTIME_US),
  TEST_CONFIG_INIT(TestSelftestReleaseFailOpen, MAX_TEST_RUNTIME_US),
  TEST_CONFIG_INIT(TestSelftestReleaseDisconnected, MAX_TEST_RUNTIME_US),
};

const TestSuite kTetherReleaseTest = TEST_SUITE_INIT(kTetherReleaseTests,
                                                     NULL, NULL);

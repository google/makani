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

#include "avionics/loadcell/firmware/tether_release.h"

#include <assert.h>
#include <stdint.h>

#include "avionics/common/safety_codes.h"
#include "avionics/firmware/cpu/clock.h"
#include "avionics/firmware/cpu/io.h"

#define FIRE_HALF_PERIOD_CYCLES CLOCK32_USEC_TO_CYCLES(200)
#define FIRE_TIME_CYCLES CLOCK32_MSEC_TO_CYCLES(3000)

#define FIRE_MON_PIN kIoGioPinA6
#define TEST_PIN kIoN2het1Pin4
#define ARM_PIN kIoN2het1Pin22
#define FIRE_PIN kIoN2het1Pin14

void TetherReleaseInit(void) {
  IoConfigureAsInput(FIRE_MON_PIN);
  IoConfigureAsInputPullDown(ARM_PIN);
  IoConfigureAsOutputPushPull(TEST_PIN, 0);
  IoConfigureAsOutputPushPull(FIRE_PIN, 0);
}

bool TetherReleaseArm(void) {
  // Don't allow arm during test sequence.  This could induce a release.
  if (IoGetValue(TEST_PIN)) {
    return false;
  }
  IoConfigureAsOutputPushPull(ARM_PIN, 1);
  return true;
}

bool TetherReleaseDisarm(void) {
  // Don't allow disarm while firing.  The relays aren't rated to break on load.
  if (TetherReleaseFiring()) {
    return false;
  }
  IoConfigureAsInputPullDown(ARM_PIN);
  return true;
}

bool TetherReleaseArmed(void) {
  return IoGetValue(ARM_PIN);
}

bool TetherReleaseFiring(void) {
  return IoGetValue(FIRE_MON_PIN);
}

// Poll this function for as long as you'd like to fire the tether.
static void TetherReleaseFirePoll(uint32_t safety_code) {
  static uint32_t pulse_time = 0;
  if (safety_code == TETHER_RELEASE_SAFETY_CODE) {
    uint32_t now = Clock32GetCycles();
    if (now - pulse_time >= FIRE_HALF_PERIOD_CYCLES) {
      pulse_time = now;
      IoConfigureAsOutputPushPull(FIRE_PIN, !IoGetValue(FIRE_PIN));
    }
  }
}

// Poll this function to fire the tether.  Returns true when finished.
// This function will fire the tether until the tether release monitor is active
// continuously for FIRE_TIME_CYCLES.
bool TetherReleaseFire(uint32_t safety_code) {
  static uint32_t end_time = 0;
  uint32_t now = Clock32GetCycles();
  if (!TetherReleaseFiring()) {
    end_time = now + FIRE_TIME_CYCLES;
  } else if (CLOCK32_GE(now, end_time)) {
    IoSetValue(FIRE_PIN, 0);
    return true;
  }
  TetherReleaseFirePoll(safety_code);
  return false;
}

// CP1A-12V datasheet specifies 10ms max switching time.  Time constant for the
// measurement circuits is ~10ms.  We use 100ms to provide significant margin
// over both.
#define SWITCH_DELAY_CYCLES CLOCK32_MSEC_TO_CYCLES(100)
#define V_NO_BATTERY 5.0f
#define V_LOW_BATTERY 24.0f
#define V_ARM_FAIL_SHORT 5.0f
#define V_RELEASE_FAIL_OPEN 0.5f
#define V_ARM_NO_RELEASE_TEST 1.0f
#define V_ARM_NO_RELEASE_ARMED 18.0f

// This test switches the test relay on, triggers the firing circuit, checks
// voltage measurements against reference values, waits for the firing circuit
// to turn off, and switches the test relay back off.  Delays are set to ensure
// no accidental firing of the tether release.  The arming circuit is guarded
// during this test.
TetherReleaseSelftestResult TetherReleaseSelftest(float v_test, float v_arm,
                                                  float v_release) {
  static enum {
    kStateInit,
    kStateTestOnWait,
    kStateFireOnWait,
    kStateFireOn,
    kStateFireOffWait,
    kStateFireOff,
  } state = kStateInit;
  static uint32_t start_cycles;
  static TetherReleaseSelftestResult result;

  if (state != kStateInit && TetherReleaseArmed()) {
    assert(false);
    return kTetherReleaseSelftestRunning;
  }
  switch (state) {
    case kStateInit:
      if (!TetherReleaseArmed()) {
        IoSetValue(TEST_PIN, true);
        state = kStateTestOnWait;
        start_cycles = Clock32GetCycles();
        result = kTetherReleaseSelftestPassed;
      }
      break;
    case kStateTestOnWait:
      if (CLOCK32_GE(Clock32GetCycles(), start_cycles + SWITCH_DELAY_CYCLES)) {
        // Test for no battery, low battery, and failed on release.
        if (v_test < V_NO_BATTERY) {
          result = kTetherReleaseSelftestBatteryDisconnected;
        } else if (v_test < V_LOW_BATTERY) {
          result = kTetherReleaseSelftestLowBattery;
        } else if (v_arm < V_ARM_FAIL_SHORT) {
          result = kTetherReleaseSelftestReleaseCircuitFailedShort;
        }
        if (result != kTetherReleaseSelftestPassed) {
          // Skip remainder of test if already failed.
          state = kStateFireOff;
        } else {
          state = kStateFireOnWait;
        }
      }
      break;
    case kStateFireOnWait:
      TetherReleaseFirePoll(TETHER_RELEASE_SAFETY_CODE);
      if (TetherReleaseFiring()) {
        state = kStateFireOn;
        start_cycles = Clock32GetCycles();
      }
      break;
    case kStateFireOn:
      TetherReleaseFirePoll(TETHER_RELEASE_SAFETY_CODE);
      if (CLOCK32_GE(Clock32GetCycles(), start_cycles + SWITCH_DELAY_CYCLES)) {
        if (v_release > V_RELEASE_FAIL_OPEN) {
          result = kTetherReleaseSelftestReleaseCircuitFailedOpen;
        } else if (v_arm > V_ARM_NO_RELEASE_TEST) {
          result = kTetherReleaseSelftestReleaseDisconnected;
        }
        state = kStateFireOffWait;
      }
      break;
    case kStateFireOffWait:
      if (!TetherReleaseFiring()) {
        state = kStateFireOff;
        start_cycles = Clock32GetCycles();
      }
      break;
    case kStateFireOff:
      if (CLOCK32_GE(Clock32GetCycles(), start_cycles + SWITCH_DELAY_CYCLES)) {
        IoSetValue(TEST_PIN, false);
        state = kStateInit;
        return result;
      }
      break;
    default:
      assert(false);
      break;
  }
  return kTetherReleaseSelftestRunning;
}

TetherReleaseSelftestResult TetherReleaseArmStatusCheck(float v_arm,
                                                        float v_release) {
  if (TetherReleaseFiring()) {
    if (v_release > V_RELEASE_FAIL_OPEN) {
      return kTetherReleaseSelftestReleaseCircuitFailedOpen;
    } else if (v_arm > V_ARM_NO_RELEASE_ARMED) {
      return kTetherReleaseSelftestReleaseDisconnected;
    }
  } else {
    if (v_arm < V_NO_BATTERY) {
      return kTetherReleaseSelftestBatteryDisconnected;
    } else if (v_arm < V_LOW_BATTERY) {
      return kTetherReleaseSelftestLowBattery;
    }
  }
  return kTetherReleaseSelftestPassed;
}

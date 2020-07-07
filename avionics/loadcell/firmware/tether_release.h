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

#ifndef AVIONICS_LOADCELL_FIRMWARE_TETHER_RELEASE_H_
#define AVIONICS_LOADCELL_FIRMWARE_TETHER_RELEASE_H_

#include <stdbool.h>
#include <stdint.h>

typedef enum {
  kTetherReleaseSelftestRunning,
  kTetherReleaseSelftestPassed,
  kTetherReleaseSelftestLowBattery,
  kTetherReleaseSelftestBatteryDisconnected,
  kTetherReleaseSelftestReleaseCircuitFailedShort,
  kTetherReleaseSelftestReleaseCircuitFailedOpen,
  kTetherReleaseSelftestReleaseDisconnected,
} TetherReleaseSelftestResult;

void TetherReleaseInit(void);
bool TetherReleaseArm(void);
bool TetherReleaseDisarm(void);
bool TetherReleaseArmed(void);
bool TetherReleaseFiring(void);
bool TetherReleaseFire(uint32_t safety_code);
TetherReleaseSelftestResult TetherReleaseSelftest(float v_test, float v_arm,
                                                  float v_release);
TetherReleaseSelftestResult TetherReleaseArmStatusCheck(float v_arm,
                                                        float v_release);

#endif  // AVIONICS_LOADCELL_FIRMWARE_TETHER_RELEASE_H_

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

#include "avionics/firmware/cpu/watchdog.h"

#include <assert.h>
#include <math.h>
#include <stdint.h>

#include "avionics/firmware/cpu/clock.h"
#include "avionics/firmware/cpu/registers.h"

void WatchdogInit(int32_t preload_us) {
  // Set preload.
  double f_rticlk = ClockDomainGetFreq(kClockDomainRticlk);
  double dwdprld = ceil(f_rticlk * (preload_us / 1e6) / 8192 - 1);
  assert(0 <= dwdprld && dwdprld <= 4095);
  RTI.DWDPRLD.DWDPRLD = dwdprld;

  // Start down-counter.
  RTI.DWDCTRL.DWDCTRL = 0xA98559DA;
}

void WatchdogPoll(void) {
  RTI.WDKEY.WDKEY = 0xE51A;
  RTI.WDKEY.WDKEY = 0xA35C;
}

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

#ifndef AVIONICS_FIRMWARE_UTIL_TIMER_H_
#define AVIONICS_FIRMWARE_UTIL_TIMER_H_

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

#include "avionics/firmware/cpu/clock.h"

typedef struct {
  uint32_t expire_cycles;
  bool flag;
} Timer;

void TimerStart(uint32_t cycles, Timer *timer);

static inline void TimerStartMsec(int32_t msec, Timer *timer) {
  assert(msec >= 0);
  TimerStart(CLOCK32_MSEC_TO_CYCLES(msec), timer);
}

static inline void TimerStartUsec(int32_t usec, Timer *timer) {
  assert(usec >= 0);
  TimerStart(CLOCK32_MSEC_TO_CYCLES(usec), timer);
}

bool TimerExpired(Timer *timer);

#endif  // AVIONICS_FIRMWARE_UTIL_TIMER_H_

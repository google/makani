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

#include "avionics/firmware/util/timer.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

#include "avionics/firmware/cpu/clock.h"

void TimerStart(uint32_t cycles, Timer *timer) {
  assert(cycles < 0x80000000UL);
  timer->flag = false;
  timer->expire_cycles = Clock32GetCycles() + cycles;
}

bool TimerExpired(Timer *timer) {
  if (CLOCK32_GE(Clock32GetCycles(), timer->expire_cycles)) {
    timer->flag = true;
  }
  return timer->flag;
}

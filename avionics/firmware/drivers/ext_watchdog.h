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

#ifndef AVIONICS_FIRMWARE_DRIVERS_EXT_WATCHDOG_H_
#define AVIONICS_FIRMWARE_DRIVERS_EXT_WATCHDOG_H_

#include <stdbool.h>
#include <stdint.h>

#include "avionics/firmware/cpu/clock.h"

// Hardware requires that we kick the watchdog every 6ms to 100ms.
// TODO: Determine optimal kick interval over temperature and resistor
// variation.
#define EXT_WATCHDOG_KICK_CYCLES CLOCK32_MSEC_TO_CYCLES(15)

void ExtWatchdogInit(void);
void ExtWatchdogKick(void);
bool ExtWatchdogPoll(void);
void ExtWatchdogWaitCycles(uint32_t cycles);
void ExtWatchdogWaitUsec(uint32_t usec);
void ExtWatchdogWaitMsec(uint32_t msec);

#endif  // AVIONICS_FIRMWARE_DRIVERS_EXT_WATCHDOG_H_

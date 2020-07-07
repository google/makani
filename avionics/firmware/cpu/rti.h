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

#ifndef AVIONICS_FIRMWARE_CPU_RTI_H_
#define AVIONICS_FIRMWARE_CPU_RTI_H_

#include <stdint.h>

// Starts the real-time interrupt (RTI) timer. Called from startup code.
void StartupRtiInit(void);

// Returns number of microseconds since RtiInit. Expect this counter to
// drift relative to the actual time. Counter rolls over at 63 bits.
int64_t RtiGetUptime(void);

// Control periodic timer. This timer operates independent of RtiGetUptime.
void RtiEnablePeriodicTimer(void);
void RtiDisablePeriodicTimer(void);
void RtiSetPeriodicTimer(uint32_t time_usec);
uint32_t RtiGetPeriodicTimer(void);  // Rolls over every 71.583 minutes.

// Schedule a periodic interrupt with initial compare value offset_usec and
// periodic increment period_usec.
void RtiEnablePeriodicInterrupt0(uint32_t offset_usec, uint32_t period_usec);
void RtiEnablePeriodicInterrupt1(uint32_t offset_usec, uint32_t period_usec);
void RtiEnablePeriodicInterrupt2(uint32_t offset_usec, uint32_t period_usec);
void RtiEnablePeriodicInterrupt3(uint32_t offset_usec, uint32_t period_usec);

// Disable periodic interrupt from firing.
void RtiDisablePeriodicInterrupt0(void);
void RtiDisablePeriodicInterrupt1(void);
void RtiDisablePeriodicInterrupt2(void);
void RtiDisablePeriodicInterrupt3(void);

// Clear periodic interrupt flag. Call after servicing the periodic function.
void RtiClearPeriodicFlag0(void);
void RtiClearPeriodicFlag1(void);
void RtiClearPeriodicFlag2(void);
void RtiClearPeriodicFlag3(void);

#endif  // AVIONICS_FIRMWARE_CPU_RTI_H_

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

#ifndef AVIONICS_FIRMWARE_STARTUP_CLOCK_TMS570_H_
#define AVIONICS_FIRMWARE_STARTUP_CLOCK_TMS570_H_

#include <stdint.h>

void StartupClockSelectOscIn(void);
void StartupClockSelectPll1(void);

// This function sets the REFCLKDIV and PLLMUL parameters given arguments
// nr and nf, and dividers ODPLL and PLLDIV to their maximum value. Call
// this function before StartupClockSetPll1Od() and StartupClockSetPll1R().
void StartupClockSetPll1NrNf(int32_t nr, int32_t nf);

// This function sets the REFCLKDIV2 and PLLMUL2 parameters given arguments
// nr and nf, and dividers ODPLL2 and PLLDIV2 to their maximum value. Call
// this function before StartupClockSetPll2Od() and StartupClockSetPll2R().
void StartupClockSetPll2NrNf(int32_t nr, int32_t nf);

// This function sets the ODPLL divider in steps until it reaches the
// desired value. */
void StartupClockSetPll1Od(int32_t od);

// This function sets the PLLDIV divider in steps until it reaches the
// desired value.
void StartupClockSetPll1R(int32_t r);

// This function sets the ODPLL2 divider in steps until it reaches the
// desired value.
void StartupClockSetPll2Od(int32_t od);

// This function sets the PLLDIV2 divider in steps until it reaches the
// desired value.
void StartupClockSetPll2R(int32_t r);

// This function sets the RTICLK divider.
void StartupClockSetRtidiv(int32_t rtidiv);

#endif  // AVIONICS_FIRMWARE_STARTUP_CLOCK_TMS570_H_

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

#ifndef AVIONICS_FIRMWARE_OUTPUT_SLOW_STATUS_H_
#define AVIONICS_FIRMWARE_OUTPUT_SLOW_STATUS_H_

#include <stdbool.h>

#include "avionics/common/avionics_messages.h"

// Slow status message frequency: 1000ms.
// TODO: Remove microsecond usages.
#define SLOW_STATUS_PERIOD_US 1000000
#define SLOW_STATUS_PERIOD_CYCLES CLOCK32_MSEC_TO_CYCLES(1000)

void OutputInitSlowStatusMessage(void);
void OutputInitCoreSwitchSlowStatusMessage(void);
void OutputInitBootloaderSlowStatusMessage(void);
bool OutputSendSlowStatusMessage(int64_t now);
bool OutputSendCoreSwitchSlowStatusMessage(int64_t now);
bool OutputSendBootloaderSlowStatusMessage(void);
const SlowStatusMessage *OutputGetSlowStatusMessage(void);
const CoreSwitchSlowStatusMessage *OutputGetCoreSwitchSlowStatusMessage(void);

#endif  // AVIONICS_FIRMWARE_OUTPUT_SLOW_STATUS_H_

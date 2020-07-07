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

#ifndef GS_MONITOR_INDICATORS_INDICATORS_M600_STATUS_H_
#define GS_MONITOR_INDICATORS_INDICATORS_M600_STATUS_H_

#include <stdint.h>

#include "gs/monitor/widgets/indicator.h"

void UpdateNodeStatus(Indicator *ind, int32_t init);
void UpdateSystemAssertStatus(Indicator *ind, int32_t init);
void UpdateSystemBuildInfoStatus(Indicator *ind, int32_t init);
void UpdateSystemMonitorStatus(Indicator *ind, int32_t init);
void UpdateCommsStatusPoF(Indicator *ind, int32_t init);
void UpdateCommsStatusEoP(Indicator *ind, int32_t init);
void UpdateCommsStatusWiFi(Indicator *ind, int32_t init);
void UpdateCommsStatusJoystick(Indicator *ind, int32_t init);
void UpdateCommsStatusLongRange(Indicator *ind, int32_t init);

#endif  // GS_MONITOR_INDICATORS_INDICATORS_M600_STATUS_H_

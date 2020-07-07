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

#ifndef GS_MONITOR_FILTERS_FILTERS_POWER_H_
#define GS_MONITOR_FILTERS_FILTERS_POWER_H_

#include "avionics/common/avionics_messages.h"
#include "gs/monitor/monitor_filter.h"
#include "system/labels.h"

#ifdef __cplusplus
extern "C" {
#endif

void FilterFmmProxy(const MotorStatusMessage motor_status[kNumMotors],
                    FmmFilterData *fmm);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // GS_MONITOR_FILTERS_FILTERS_POWER_H_

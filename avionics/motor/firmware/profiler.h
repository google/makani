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

#ifndef AVIONICS_MOTOR_FIRMWARE_PROFILER_H_
#define AVIONICS_MOTOR_FIRMWARE_PROFILER_H_

#include "avionics/common/motor_profiler_types.h"

void ProfilerInit(void);
void ProfilerIsrTic(void);
void ProfilerIsrToc(void);
void ProfilerLoopTic(void);
void ProfilerLoopToc(void);
void ProfilerNetPollCount(void);
void ProfilerNetPollCountSample(void);
ProfilerOutput *ProfilerGetOutput(void);

#endif  // AVIONICS_MOTOR_FIRMWARE_PROFILER_H_

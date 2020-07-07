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

#ifndef AVIONICS_PLATFORM_FIRMWARE_OUTPUT_H_
#define AVIONICS_PLATFORM_FIRMWARE_OUTPUT_H_

#include <stdint.h>

#include "avionics/firmware/monitors/aio_types.h"
#include "avionics/firmware/monitors/ground_io_types.h"

void PlatformOutputInit(void);
AioModuleMonitorData *PlatformOutputGetAioModuleMonitors(void);
GroundIoMonitorData *PlatformOutputGetGroundIoMonitors(void);
void PlatformOutputPoll(int64_t now);
void PlatformOutputSendStatus(void);
void PlatformOutputSendMonitorStatus(void);
void PlatformOutputSendWeather(int64_t now);

#endif  // AVIONICS_PLATFORM_FIRMWARE_OUTPUT_H_

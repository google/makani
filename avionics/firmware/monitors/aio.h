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

#ifndef AVIONICS_FIRMWARE_MONITORS_AIO_H_
#define AVIONICS_FIRMWARE_MONITORS_AIO_H_

#include <stdbool.h>

#include "avionics/firmware/monitors/aio_types.h"
#include "avionics/firmware/serial/aio_serial_params.h"

void AioMonitorInit(void);
bool AioMonitorPoll(AioHardware rev, AioModuleMonitorData *monitors);
bool AioMonitorPollStack(AioHardware aio_hardware_revision,
                         AioModuleMonitorData *aio_monitors,
                         bool (* const carrier_poll_function)(void));

#endif  // AVIONICS_FIRMWARE_MONITORS_AIO_H_

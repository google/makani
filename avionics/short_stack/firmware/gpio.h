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

#ifndef AVIONICS_SHORT_STACK_FIRMWARE_GPIO_H_
#define AVIONICS_SHORT_STACK_FIRMWARE_GPIO_H_

#include "avionics/firmware/monitors/short_stack_types.h"
#include "avionics/firmware/serial/short_stack_serial_params.h"

void ShortStackGpioSetCtrlPins(ShortStackHardware rev,
                               ShortStackGpioOutputPin pin_requested,
                               ShortStackMonitorData *mon);
void ShortStackGpioClearCtrlPins(ShortStackHardware rev,
                                 ShortStackMonitorData *mon);
void ShortStackGpioInit(ShortStackHardware rev);
bool ShortStackGpioPollInputPin(ShortStackHardware rev,
                                ShortStackGpioInputPin pin_func);

#endif  // AVIONICS_SHORT_STACK_FIRMWARE_GPIO_H_

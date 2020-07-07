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

#ifndef AVIONICS_SERVO_FIRMWARE_R22_DEF_H_
#define AVIONICS_SERVO_FIRMWARE_R22_DEF_H_

#include "avionics/firmware/cpu/sci.h"

#define R22_BREAK_TIME_US                10000
#define R22_DEFAULT_BAUD_RATE            9600    // Fixed by hardware.
#define R22_DESIRED_BAUD_RATE            115200  // 9600 <= baud <= 115200.
#define R22_POWER_ON_TIME_US             500000
#define R22_SCI                          kSci2Interrupt
#define R22_STATUS_BITS                  32
#define R22_SWAP_OPERATING_MODES_TIME_US 500000
#define R22_TRANSFER_TIMEOUT_US          100000

#endif  // AVIONICS_SERVO_FIRMWARE_R22_DEF_H_

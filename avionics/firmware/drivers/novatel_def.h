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

#ifndef AVIONICS_FIRMWARE_DRIVERS_NOVATEL_DEF_H_
#define AVIONICS_FIRMWARE_DRIVERS_NOVATEL_DEF_H_

#include "avionics/firmware/cpu/clock.h"

// NovAtel timing.
// Reset recovery time must be greater than 20 us.
#define NOVATEL_RESET_CYCLES CLOCK32_USEC_TO_CYCLES(100)
#define NOVATEL_REPLY_TIMEOUT_CYCLES CLOCK32_MSEC_TO_CYCLES(1000)

// NovAtel communications.
#define NOVATEL_DEFAULT_BAUD 9600
#define NOVATEL_COM1_BAUD    460800  // 300 to 921600.
#define NOVATEL_COM2_BAUD    115200  // 300 to 230400.

#endif  // AVIONICS_FIRMWARE_DRIVERS_NOVATEL_DEF_H_

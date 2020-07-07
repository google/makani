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

#ifndef AVIONICS_FIRMWARE_DRIVERS_SEPTENTRIO_DEF_H_
#define AVIONICS_FIRMWARE_DRIVERS_SEPTENTRIO_DEF_H_

// Septentrio timing.
#define SEPTENTRIO_RESET_US          100
#define SEPTENTRIO_RESET_RECOVERY_US 10000000
#define SEPTENTRIO_REPLY_TIMEOUT_US  5000000
#define SEPTENTRIO_REPLY_DELAY_US    100000

// Septentrio communications.
#define SEPTENTRIO_DEFAULT_BAUD 115200
#define SEPTENTRIO_DESIRED_BAUD 460800

#endif  // AVIONICS_FIRMWARE_DRIVERS_SEPTENTRIO_DEF_H_

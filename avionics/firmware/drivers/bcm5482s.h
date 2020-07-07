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

#ifndef AVIONICS_FIRMWARE_DRIVERS_BCM5482S_H_
#define AVIONICS_FIRMWARE_DRIVERS_BCM5482S_H_

#include <stdbool.h>

void Bcm5482SInit(void);

// This state machine must run within a single state in the BCM53284 state
// machine--they can't interlace, or they'll fight over communications to the
// underlying bcm driver.
void Bcm5482SPoll(void);

bool Bcm5482SReady(void);

#endif  // AVIONICS_FIRMWARE_DRIVERS_BCM5482S_H_

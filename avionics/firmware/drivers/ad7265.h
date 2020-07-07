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

#ifndef AVIONICS_FIRMWARE_DRIVERS_AD7265_H_
#define AVIONICS_FIRMWARE_DRIVERS_AD7265_H_

#include <stdbool.h>
#include <stdint.h>

void Ad7265Init(void);
bool Ad7265ReadAsyncStart(void);
bool Ad7265ReadAsyncGet(int32_t *a1, int32_t *b1, int32_t *a2, int32_t *b2);
void Ad7265Read(int32_t *a1, int32_t *b1, int32_t *a2, int32_t *b2);

#endif  // AVIONICS_FIRMWARE_DRIVERS_AD7265_H_

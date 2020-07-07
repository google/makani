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

#ifndef AVIONICS_SERVO_FIRMWARE_R22_H_
#define AVIONICS_SERVO_FIRMWARE_R22_H_

#include <stdbool.h>

// Initialize hardware for R22 communications.
void R22Init(void);

// Hardware signals.
bool R22GetFaultMon(void);
bool R22GetClampMon(void);
void R22SetClampOverride(void);
void R22ClearClampOverride(void);
void R22EnableClamp(void);
void R22DisableClamp(void);

#endif  // AVIONICS_SERVO_FIRMWARE_R22_H_

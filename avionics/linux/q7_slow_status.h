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

// Shared bits of the Q7 Slow Status Sender, for use by
// nodes who don't want to run the whole thing.

#ifndef AVIONICS_LINUX_Q7_SLOW_STATUS_H_
#define AVIONICS_LINUX_Q7_SLOW_STATUS_H_

#if !defined(__linux__)
#error This is only functional on Linux.
#endif

#include <stdbool.h>

#include "avionics/linux/q7_slow_status_types.h"

#ifdef __cplusplus
extern "C" {
#endif

bool Q7SlowStatusInit(Q7SlowStatusContext *context);
void Q7SlowStatusInitMessage(const Q7SlowStatusContext *context,
                             Q7SlowStatusMessage *message);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // AVIONICS_LINUX_Q7_SLOW_STATUS_H_

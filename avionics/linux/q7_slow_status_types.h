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

#ifndef AVIONICS_LINUX_Q7_SLOW_STATUS_TYPES_H_
#define AVIONICS_LINUX_Q7_SLOW_STATUS_TYPES_H_

#if !defined(__linux__)
#error This is only functional on Linux.
#endif

#include <stdint.h>

#include "avionics/common/avionics_messages.h"
#include "avionics/network/aio_node.h"

typedef struct {
  AioNode node;          // Determined from IP address.
  GitHash git_hash;      // Hash of the Yocto distro build.
  int8_t num_cpus;       // From get_nprocs system call.
  BuildInfo build_info;  // From GetBuildInfo.
} Q7SlowStatusContext;

#endif  // AVIONICS_LINUX_Q7_SLOW_STATUS_TYPES_H_

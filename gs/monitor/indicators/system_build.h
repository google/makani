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

#ifndef GS_MONITOR_INDICATORS_SYSTEM_BUILD_H_
#define GS_MONITOR_INDICATORS_SYSTEM_BUILD_H_

#include <stdbool.h>
#include <stdint.h>

#include "avionics/common/build_info_types.h"
#include "avionics/network/aio_node.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  uint8_t checksum[GIT_CHECKSUM_LENGTH];
  uint8_t flags;
  int32_t ref_count;  // Number of nodes with matching this BuildInfo.
} BuildContainer;

typedef struct {
  // BuildInfo from last update.
  BuildInfo prev[kNumAioNodes];
  bool prev_valid[kNumAioNodes];

  // List of BuildContainers, descending sort by ref_count.
  int32_t count;  // Number of elements in list.
  BuildContainer list[kNumAioNodes];
} SystemBuild;

void SystemBuildInit(SystemBuild *sys);
bool SystemBuildUpdate(AioNode node, const BuildInfo *build, SystemBuild *sys);
void SystemBuildTimeout(AioNode node, SystemBuild *sys);
bool SystemBuildGetBuild(const SystemBuild *sys, const BuildContainer **build);
int32_t SystemBuildGetCount(const SystemBuild *sys);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // GS_MONITOR_INDICATORS_SYSTEM_BUILD_H_

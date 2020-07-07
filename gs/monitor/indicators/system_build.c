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

#include "gs/monitor/indicators/system_build.h"

#include <assert.h>
#include <stdint.h>
#include <string.h>

#include "avionics/common/build_info_types.h"
#include "avionics/network/aio_node.h"
#include "common/macros.h"

static bool CompareBuild(const BuildContainer *container,
                         const BuildInfo *info) {
  assert(sizeof(container->checksum) == sizeof(info->checksum));
  assert(sizeof(container->flags) == sizeof(info->flags));
  return (
      !memcmp(container->checksum, info->checksum, sizeof(info->checksum)) &&
      container->flags == info->flags);
}

static int32_t FindBuild(const SystemBuild *sys, const BuildInfo *elem) {
  for (int32_t i = 0; i < sys->count && i < ARRAYSIZE(sys->list); ++i) {
    if (CompareBuild(&sys->list[i], elem)) {
      return i;
    }
  }
  return -1;
}

static void SwapBuild(int32_t a, int32_t b, SystemBuild *sys) {
  BuildContainer tmp = sys->list[a];
  sys->list[a] = sys->list[b];
  sys->list[b] = tmp;
}

static void PromoteBuild(int32_t i, SystemBuild *sys) {
  // Descending sort.
  while (i > 0 && sys->list[i].ref_count > sys->list[i - 1].ref_count) {
    SwapBuild(i, i - 1, sys);
    --i;
  }
}

static void DemoteBuild(int32_t i, SystemBuild *sys) {
  // Descending sort.
  ++i;
  while (i < sys->count && i < ARRAYSIZE(sys->list) &&
         sys->list[i - 1].ref_count < sys->list[i].ref_count) {
    SwapBuild(i - 1, i, sys);
    ++i;
  }
}

static void InsertBuild(const BuildInfo *elem, SystemBuild *sys) {
  int32_t i = FindBuild(sys, elem);
  if (0 <= i && i < ARRAYSIZE(sys->list)) {
    ++sys->list[i].ref_count;
    PromoteBuild(i, sys);

  } else if (sys->count < ARRAYSIZE(sys->list)) {
    BuildContainer *container = &sys->list[sys->count];
    memcpy(container->checksum, elem->checksum, sizeof(elem->checksum));
    container->flags = elem->flags;
    container->ref_count = 1;
    PromoteBuild(sys->count, sys);
    ++sys->count;
  }
}

static void RemoveBuild(const BuildInfo *elem, SystemBuild *sys) {
  int32_t i = FindBuild(sys, elem);
  if (0 <= i && i < ARRAYSIZE(sys->list)) {
    --sys->list[i].ref_count;
    if (sys->list[i].ref_count <= 0 && sys->count > 0) {
      --sys->count;
      sys->list[i] = sys->list[sys->count];
    }
    if (sys->count > 0) {
      DemoteBuild(i, sys);
    }
  }
}

void SystemBuildInit(SystemBuild *sys) { memset(sys, 0, sizeof(*sys)); }

bool SystemBuildUpdate(AioNode node, const BuildInfo *build, SystemBuild *sys) {
  if (!sys->prev_valid[node]) {
    InsertBuild(build, sys);
  } else if (memcmp(&sys->prev[node], build, sizeof(*build)) != 0) {
    RemoveBuild(&sys->prev[node], sys);
    InsertBuild(build, sys);
  }
  sys->prev[node] = *build;
  sys->prev_valid[node] = true;

  // Return true when build matches system build.
  return CompareBuild(&sys->list[0], build);
}

void SystemBuildTimeout(AioNode node, SystemBuild *sys) {
  if (sys->prev_valid[node]) {
    RemoveBuild(&sys->prev[node], sys);
    sys->prev_valid[node] = false;
  }
}

bool SystemBuildGetBuild(const SystemBuild *sys, const BuildContainer **build) {
  if (sys->count > 0) {
    *build = &sys->list[0];
    return true;
  }
  return false;
}

int32_t SystemBuildGetCount(const SystemBuild *sys) { return sys->count; }

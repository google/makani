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

#include "avionics/common/build_info.h"

#include <assert.h>
#include <string.h>

#include "avionics/common/generated_build_info.h"
#include "avionics/common/strings.h"
#include "common/macros.h"

#ifndef GIT_DIRTY_FILES
#error "GIT_DIRTY_FILES must be defined."
#endif

#if (GIT_DIRTY_FILES == 0)
#define FILE_FLAG 0
#else
#define FILE_FLAG kBuildStatusModifiedFiles
#endif

#ifndef GIT_HASH
#error "GIT_HASH must be defined."
#endif

#define ASCII_CHECKSUM_LENGTH (2 * GIT_CHECKSUM_LENGTH)

COMPILE_ASSERT(sizeof(((BuildInfo *)0)->checksum) == GIT_CHECKSUM_LENGTH,
               Checksum_must_be_20_bytes);
COMPILE_ASSERT(ARRAYSIZE(GIT_HASH) == ASCII_CHECKSUM_LENGTH + 1,
               GIT_HASH_must_be_40_ascii_chars);

void GetBuildInfo(BuildInfo *build_info) {
#ifdef NDEBUG
  build_info->flags = FILE_FLAG;
#else
  build_info->flags = FILE_FLAG | kBuildStatusAssertsEnabled;
#endif

  static const char hash[ARRAYSIZE(GIT_HASH)] = GIT_HASH;

  int32_t length;
  int32_t delta = 0;
  for (length = 0; length < ASCII_CHECKSUM_LENGTH; length += delta) {
    delta = ReadHexUint8(&hash[length], 2, &build_info->checksum[length / 2]);
    assert(delta == 2);
    if (delta < 2) {
      break;
    }
  }
  assert(length == ASCII_CHECKSUM_LENGTH);

  strncpy(build_info->date, __DATE__,
          sizeof(build_info->date));
  strncpy(build_info->time, __TIME__,
          sizeof(build_info->time));
}

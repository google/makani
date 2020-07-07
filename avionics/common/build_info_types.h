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

#ifndef AVIONICS_COMMON_BUILD_INFO_TYPES_H_
#define AVIONICS_COMMON_BUILD_INFO_TYPES_H_

#include <stdint.h>

typedef enum {
  kBuildStatusModifiedFiles  = (1 << 0),
  kBuildStatusAssertsEnabled = (1 << 1),
} BuildStatusFlag;

#define GIT_CHECKSUM_LENGTH 20
#define DATE_LENGTH 12
#define TIME_LENGTH 9

typedef struct {
  uint8_t checksum[GIT_CHECKSUM_LENGTH];
  uint8_t flags;  // See BuildStatusFlag.
  char date[DATE_LENGTH];
  char time[TIME_LENGTH];
} BuildInfo;

#endif  // AVIONICS_COMMON_BUILD_INFO_TYPES_H_

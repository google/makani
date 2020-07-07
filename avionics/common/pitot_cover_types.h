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

#ifndef AVIONICS_COMMON_PITOT_COVER_TYPES_H_
#define AVIONICS_COMMON_PITOT_COVER_TYPES_H_

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  kPitotCoverStatusUnknown = 0,
  kPitotCoverStatusOpening = 1,
  kPitotCoverStatusOpened  = 2,
  kPitotCoverStatusClosing = 3,
  kPitotCoverStatusClosed  = 4
} PitotCoverStatus;

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // AVIONICS_COMMON_PITOT_COVER_TYPES_H_

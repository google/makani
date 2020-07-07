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

#ifndef AVIONICS_COMMON_CVT_ENTRIES_H_
#define AVIONICS_COMMON_CVT_ENTRIES_H_

#include <stdbool.h>
#include <stdint.h>

#include "avionics/network/aio_node.h"
#include "avionics/network/message_type.h"

// CVT entry state.
typedef struct {
  uint16_t sequence;
  int64_t timestamp;
  bool updated;
} CvtEntryState;

// CVT entry (store in .const memory).
typedef struct {
  const int32_t len;
  void * const data;
  CvtEntryState * const state;
} CvtEntry;

const CvtEntry *GetCvtEntry(AioNode source, MessageType type);

#endif  // AVIONICS_COMMON_CVT_ENTRIES_H_

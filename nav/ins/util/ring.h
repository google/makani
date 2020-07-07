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

#ifndef NAV_INS_UTIL_RING_H_
#define NAV_INS_UTIL_RING_H_

#include <stdbool.h>
#include <stdint.h>

#include "common/ring.h"
#include "nav/ins/messages/message_header.h"
#include "nav/ins/messages/seq_num.h"

static inline bool InsRingGetNewHeadIndex(const RingBuffer *ring,
                                          const InsMessageHeader buffer[],
                                          const InsMessageHeader *header,
                                          int32_t *index) {
  // Returns true when the index is set to the new_head index. When the
  // buffer is empty, RingGetHeadIndex() fails and RingGetNewHeadIndex
  // succeeds. When the buffer is not empty, InsSequenceGt de-duplicates
  // sequence numbers.
  return (!RingGetHeadIndex(ring, index)
          || InsSequenceGt(header->seq_num, buffer[*index].seq_num))
      && RingGetNewHeadIndex(ring, index);
}

#endif  // NAV_INS_UTIL_RING_H_

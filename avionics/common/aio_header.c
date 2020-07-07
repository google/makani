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

#include "avionics/common/aio_header.h"

#include <stdbool.h>
#include <stdint.h>

// This logic should match that in cvt.c.
bool AioHeaderIsDuplicate(int64_t timestamp, const AioHeader *header,
                          const AioDeduplicate *dedup) {
  // Sequence numbers expire after maximum latency.
  int64_t delta_time = timestamp - dedup->timestamp;
  if (delta_time < AIO_EXPIRATION_TIME_US) {
    // Handle out of order sequence numbers (including duplication).
    uint16_t delta_seq = (uint16_t)(header->sequence - dedup->sequence);
    return delta_seq == 0U || delta_seq > AIO_ACCEPTANCE_WINDOW;
  }
  return false;
}

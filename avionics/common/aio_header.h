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

#ifndef AVIONICS_COMMON_AIO_HEADER_H_
#define AVIONICS_COMMON_AIO_HEADER_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// The acceptance window defines the number of sequence numbers ahead of the
// current message to consider more recent. We consider all other messages
// as out-of-order. This window defines the maximum number of consecutive
// packets we could drop.
#define AIO_ACCEPTANCE_WINDOW 5000

// The expiration time defines the maximum amount of time to consider the
// current message within the CVT as current; thereafter it becomes stale.
// Sequence numbers within the AIO_ACCEPTANCE_WINDOW will continue to update
// the CVT. Sequence numbers outside of the AIO_ACCEPTANCE_WINDOW will be
// ignored while the current message is current. After the current message
// expires, the CVT will accept all sequence numbers.
//
// This number should be greater than the maximum link latency.
// This number should be less than the maximum acceptable outage from a reboot.
#define AIO_EXPIRATION_TIME_US 500000

typedef struct {
  // Set to AIO_VERSION.
  uint16_t version;
  // Source AioNode.
  int8_t source;
  // MessageType of payload.
  uint8_t type;
  // Sender's sequence number for this MessageType.
  uint16_t sequence;
  // Timestamp [usec].
  uint32_t timestamp;
} __attribute__((__packed__)) AioHeader;  // net.c requires packed types.

typedef struct {
  int64_t timestamp;  // [usec]
  uint16_t sequence;
} AioDeduplicate;

bool AioHeaderIsDuplicate(int64_t timestamp, const AioHeader *header,
                          const AioDeduplicate *dedup);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // AVIONICS_COMMON_AIO_HEADER_H_

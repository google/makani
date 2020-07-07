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

// Current Value Table (CVT).
//
// Stores the most recently received AIO message of a given source and type.

#ifndef AVIONICS_COMMON_CVT_H_
#define AVIONICS_COMMON_CVT_H_

#include <stddef.h>  // For size_t.
#include <stdint.h>

#include "avionics/common/avionics_messages.h"
#include "avionics/network/aio_node.h"
#include "avionics/network/message_type.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  kCvtEventNone                = 0,
  kCvtEventExpectedDuplication = (1 << 0),
  kCvtEventInvalidSource       = (1 << 1),
  kCvtEventInvalidMessageType  = (1 << 2),
  kCvtEventInvalidCvtEntry     = (1 << 3),
  kCvtEventInvalidCvtState     = (1 << 4),
  kCvtEventInvalidLength       = (1 << 5),
  kCvtEventSequenceOutOfOrder  = (1 << 6),
  kCvtEventSequenceExpired     = (1 << 7),
  kCvtEventInternalError       = (1 << 8),
  kCvtEventSequenceJumped      = (1 << 9)
} CvtEventCodes;

typedef size_t (*PackCvtFunction)(const void *in, size_t num, uint8_t *out);
typedef size_t (*UnpackCvtFunction)(const uint8_t *in, size_t num, void *out);

// Remove a message from CVT.
void CvtClearMessage(AioNode source, MessageType type);

// DEPRECATED. Use code-generated CvtPutX() functions.
// Inserts a message of a given source and type into the CVT if it is newer than
// the current message.
CvtEventCodes CvtPut(AioNode source, MessageType type, uint16_t sequence,
                     int64_t timestamp, const void *buf, int32_t len);

// Insert an unpacked message structure into the CVT using pack_func. If
// pack_func is NULL, then this function uses memcpy to copy the data into
// the CVT.
CvtEventCodes CvtPutPacked(AioNode source, MessageType type,
                           PackCvtFunction pack_func, const void *buf,
                           int32_t len, uint16_t sequence, int64_t timestamp);

// DEPRECATED. Use code-generated CvtGetX() functions.
// Returns a pointer to the most recently received message of a given source
// and type in the CVT, or NULL if no new message was received. The pointed-to
// data is guaranteed to not be modified until the next call to CvtPut. If
// sequence is not NULL, it will be written with the sequence number of the
// received message. If timestamp is not NULL, it will be written with the
// timestamp of when the message was received.
const void *CvtGet(AioNode source, MessageType type, uint16_t *sequence,
                   int64_t *timestamp);

// Behaves similarly to CvtGet, except does not check for whether the message
// has been updated, and does not change the message's update state. The caller
// is responsible for keeping track of sequence numbers to determine if the
// message has been updated.
const void *CvtPeek(AioNode source, MessageType type, uint16_t *sequence,
                    int64_t *timestamp);


// Unpack a packed message to an unpacked message structure. Returns true upon
// success (also indicating that a new message was received). If unpack_func
// is NULL, then this function uses memcpy to copy the CVT data to msg. If
// sequence is not NULL, it will be written with the sequence number of the
// received message. If timestamp is not NULL, it will be written with the
// timestamp of when the message was received.
bool CvtGetUnpacked(AioNode source, MessageType type,
                    UnpackCvtFunction unpack_func, void *msg,
                    uint16_t *sequence, int64_t *timestamp);

// Get the CVT statistics since last call.
void CvtGetStats(CvtStats *stats);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // AVIONICS_COMMON_CVT_H_

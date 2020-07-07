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

#include "avionics/common/cvt.h"

#include <limits.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "avionics/common/aio_header.h"
#include "avionics/common/cvt_entries.h"
#include "avionics/network/aio_node.h"
#include "avionics/network/message_type.h"

// TODO: Eliminate architecture-specific dependencies from common code.
#if !defined(__linux__)
#include "avionics/firmware/cpu/memcpy.h"
#endif

static CvtEventCodes g_event_codes = 0;
static int32_t g_unread_packets = 0;
static int32_t g_unique_packets = 0;
static int32_t g_invalid_packets = 0;

void CvtClearMessage(AioNode source, MessageType type) {
  const CvtEntry *cvt_entry = GetCvtEntry(source, type);
  if (cvt_entry != NULL && cvt_entry->state) {
    cvt_entry->state->timestamp = 0;
    cvt_entry->state->sequence = 0U;
    cvt_entry->state->updated = false;
  }
}

static CvtEventCodes CvtPutImpl(AioNode source, MessageType type,
                                PackCvtFunction pack_func, const void *buf,
                                int32_t len, uint16_t sequence,
                                int64_t timestamp) {
  if (source < 0 || kNumAioNodes <= source) {
    return kCvtEventInvalidSource;
  }

  if (kNumMessageTypes <= type) {
    return kCvtEventInvalidMessageType;
  }

  if (buf == NULL) {
    return kCvtEventInternalError;
  }

  if (len < 0) {
    return kCvtEventInternalError;
  }

  // Get CVT entry.
  const CvtEntry *cvt_entry = GetCvtEntry(source, type);
  if (cvt_entry == NULL) {
    return kCvtEventInvalidCvtEntry;
  }

  // Length mismatch.
  if (len != cvt_entry->len) {
    return kCvtEventInvalidLength;
  }

  // Get CVT state.
  CvtEntryState *state = cvt_entry->state;
  if (state == NULL) {
    return kCvtEventInvalidCvtState;
  }

  // Sequence numbers expire after maximum latency.
  // TODO: Reuse AioHeaderIsDuplicate()?
  CvtEventCodes event = kCvtEventNone;
  if ((timestamp - state->timestamp) < AIO_EXPIRATION_TIME_US) {
    // Expected duplication.
    if (sequence == state->sequence) {
      return kCvtEventExpectedDuplication;
    }
    // Out of order.
    if ((uint16_t)(sequence - state->sequence) > AIO_ACCEPTANCE_WINDOW) {
      return kCvtEventSequenceOutOfOrder;
    }
  } else {
    // Mark the first accepted message.
    event |= kCvtEventSequenceExpired;
  }

  // Flag jumps in sequence numbers.
  if ((uint16_t)(state->sequence + 1U) != sequence) {
    event |= kCvtEventSequenceJumped;
  }

  // Count packets.
  if (state->updated) {
    ++g_unread_packets;
  }
  ++g_unique_packets;

  // Update CVT.
  if (pack_func) {
    pack_func(buf, 1, cvt_entry->data);
  } else {
#if defined(__linux__)
    memcpy(cvt_entry->data, buf, (size_t)len);
#else
    FastCopy(len, buf, cvt_entry->data);  // Optimized copy for TMS570.
#endif
  }
  state->sequence = sequence;
  state->timestamp = timestamp;
  state->updated = true;

  return event;
}

CvtEventCodes CvtPut(AioNode source, MessageType type, uint16_t sequence,
                     int64_t timestamp, const void *buf, int32_t len) {
  return CvtPutPacked(source, type, NULL, buf, len, sequence, timestamp);
}

CvtEventCodes CvtPutPacked(AioNode source, MessageType type,
                           PackCvtFunction pack_func, const void *buf,
                           int32_t len, uint16_t sequence, int64_t timestamp) {
  CvtEventCodes event = CvtPutImpl(source, type, pack_func, buf, len, sequence,
                                   timestamp);
  if ((event & (kCvtEventInvalidSource
                | kCvtEventInvalidMessageType
                | kCvtEventInvalidCvtEntry
                | kCvtEventInvalidLength)) != 0) {
    ++g_invalid_packets;
  }
  g_event_codes |= event;

  return event;
}

// This is a helper to retrieves a message from the CVT table. Both CvtGet and
// CvtPeek use this function. If the peek flag is false, this function NULL if
// the message has not been updated, and updates the state of the message. If
// peek is true, it returns the message regardless of if it has been updated,
// and it does not update the state of the message. In the case where peek is
// true, the caller is responsible for tracking sequence numbers to see if the
// message has been updated.
static const void *CvtRetrieve(AioNode source, MessageType type,
                               uint16_t *sequence, int64_t *timestamp,
                               bool peek) {
  if (source < 0 || kNumAioNodes <= source) {
    return NULL;
  }

  if (kNumMessageTypes <= type) {
    return NULL;
  }

  // Get CVT entry.
  const CvtEntry *cvt_entry = GetCvtEntry(source, type);
  if (cvt_entry == NULL) {
    return NULL;
  }

  // Get CVT state.
  CvtEntryState *state = cvt_entry->state;
  if (state == NULL) {
    return NULL;
  }

  // Return NULL if peek is false and message is not updated.
  if (!peek && !state->updated) {
    return NULL;
  }

  // Get sequence if requested.
  if (sequence != NULL) {
    *sequence = state->sequence;
  }

  // Get timestamp if requested.
  if (timestamp != NULL) {
    *timestamp = state->timestamp;
  }

  // Update state if peek is false, and return data.
  if (!peek) {
    state->updated = false;
  }
  return cvt_entry->data;
}

const void *CvtGet(AioNode source, MessageType type,
                   uint16_t *sequence, int64_t *timestamp) {
  return CvtRetrieve(source, type, sequence, timestamp, false);
}

const void *CvtPeek(AioNode source, MessageType type,
                    uint16_t *sequence, int64_t *timestamp) {
  return CvtRetrieve(source, type, sequence, timestamp, true);
}

bool CvtGetUnpacked(AioNode source, MessageType type,
                    UnpackCvtFunction unpack_func, void *msg,
                    uint16_t *sequence, int64_t *timestamp) {
  const void *buf = CvtGet(source, type, sequence, timestamp);
  if (buf) {
    if (unpack_func) {
      unpack_func(buf, 1, msg);
    } else {
      const CvtEntry *cvt_entry = GetCvtEntry(source, type);
      memcpy(msg, buf, (size_t)cvt_entry->len);
    }
    return true;
  }
  return false;
}

void CvtGetStats(CvtStats *stats) {
  stats->unique_packets = (int16_t)g_unique_packets;
  stats->unread_packets = (int16_t)g_unread_packets;
  stats->invalid_packets = (int16_t)g_invalid_packets;
  stats->event_codes = (uint16_t)g_event_codes;

  g_unique_packets = 0;
  g_unread_packets = 0;
  g_invalid_packets = 0;
  g_event_codes = 0;
}

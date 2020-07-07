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

#include "avionics/common/tether_cvt_rtcm.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "avionics/common/avionics_messages.h"
#include "avionics/common/cvt_avionics_messages.h"
#include "avionics/network/aio_node.h"
#include "common/macros.h"

#define RTCM_AUX_PERIOD_US (5 * 1000 * 1000)
#define RTCM_MAX_AGE_US (500 * 1000)

typedef struct {
  int32_t length;
  const uint8_t *data;
} GpsRtcmLengthData;

static bool GetGpsRtcm1006(AioNode node, GpsRtcm1006Message *message,
                           int64_t *timestamp_usec) {
  return CvtGetGpsRtcm1006Message(node, message, NULL, timestamp_usec)
      && 0 < message->length && message->length <= ARRAYSIZE(message->data);
}

static bool GetGpsRtcm1033(AioNode node, GpsRtcm1033Message *message,
                           int64_t *timestamp_usec) {
  return CvtGetGpsRtcm1033Message(node, message, NULL, timestamp_usec)
      && 0 < message->length && message->length <= ARRAYSIZE(message->data);
}

static bool GetGpsRtcm1230(AioNode node, GpsRtcm1230Message *message,
                           int64_t *timestamp_usec) {
  return CvtGetGpsRtcm1230Message(node, message, NULL, timestamp_usec)
      && 0 < message->length && message->length <= ARRAYSIZE(message->data);
}

static bool GetGpsRtcm1072(AioNode node, GpsRtcm1072Message *message,
                           int64_t *timestamp_usec) {
  return CvtGetGpsRtcm1072Message(node, message, NULL, timestamp_usec)
      && 0 < message->length && message->length <= ARRAYSIZE(message->data);
}

static bool GetGpsRtcm1074(AioNode node, GpsRtcm1074Message *message,
                           int64_t *timestamp_usec) {
  return CvtGetGpsRtcm1074Message(node, message, NULL, timestamp_usec)
      && 0 < message->length && message->length <= ARRAYSIZE(message->data);
}

static bool GetGpsRtcm1082(AioNode node, GpsRtcm1082Message *message,
                           int64_t *timestamp_usec) {
  return CvtGetGpsRtcm1082Message(node, message, NULL, timestamp_usec)
      && 0 < message->length && message->length <= ARRAYSIZE(message->data);
}

static bool GetGpsRtcm1084(AioNode node, GpsRtcm1084Message *message,
                           int64_t *timestamp_usec) {
  return CvtGetGpsRtcm1084Message(node, message, NULL, timestamp_usec)
      && 0 < message->length && message->length <= ARRAYSIZE(message->data);
}

static bool TransmitNextRtcmMessage(int64_t now_usec, int32_t num_messages,
                                    const GpsRtcmLengthData messages[],
                                    const int64_t timestamps_usec[],
                                    bool updated[], int64_t *last_timestamp,
                                    int32_t *last_index,
                                    TetherUpCvtRtcmState *out) {
  for (int32_t i = 1; i <= num_messages; ++i) {
    int32_t j = (*last_index + i) % num_messages;

    // Don't transmit messages older than the previous message.
    if (updated[j] && timestamps_usec[j] >= *last_timestamp
        && now_usec - timestamps_usec[j] < RTCM_MAX_AGE_US
        && 0 < messages[j].length
        && messages[j].length <= ARRAYSIZE(out->data)) {
      *last_timestamp = timestamps_usec[j];
      *last_index = j;
      updated[j] = false;
      memcpy(out->data, messages[j].data, (size_t)messages[j].length);
      out->length = messages[j].length;
      out->index = 0;
      return true;
    }
  }
  return false;
}

static bool TransmitNextRtcmAuxMessage(int64_t now_usec,
                                       TetherUpCvtRtcmState *out) {
  const GpsRtcmLengthData message[kNumGpsRtcmAuxMessageTypes] = {
    [kGpsRtcmAuxMessageType1006] = {out->rtcm1006.length, out->rtcm1006.data},
    [kGpsRtcmAuxMessageType1033] = {out->rtcm1033.length, out->rtcm1033.data},
    [kGpsRtcmAuxMessageType1230] = {out->rtcm1230.length, out->rtcm1230.data},
  };
  return TransmitNextRtcmMessage(now_usec, ARRAYSIZE(message), message,
                                 out->aux_timestamp_usec, out->aux_updated,
                                 &out->aux_last_time, &out->aux_last_index,
                                 out);
}

static bool TransmitNextRtcmObsMessage(int64_t now_usec,
                                       TetherUpCvtRtcmState *out) {
  const GpsRtcmLengthData message[kNumGpsRtcmObsMessageTypes] = {
    [kGpsRtcmObsMessageType1072] = {out->rtcm1072.length, out->rtcm1072.data},
    [kGpsRtcmObsMessageType1074] = {out->rtcm1074.length, out->rtcm1074.data},
    [kGpsRtcmObsMessageType1082] = {out->rtcm1082.length, out->rtcm1082.data},
    [kGpsRtcmObsMessageType1084] = {out->rtcm1084.length, out->rtcm1084.data},
  };
  return TransmitNextRtcmMessage(now_usec, ARRAYSIZE(message), message,
                                 out->obs_timestamp_usec, out->obs_updated,
                                 &out->obs_last_time, &out->obs_last_index,
                                 out);
}

static void QueryRtcmMessages(int64_t now_usec, TetherUpCvtRtcmState *out) {
  const AioNode node = kAioNodeGpsBaseStation;
  static union {
    GpsRtcm1006Message rtcm1006;
    GpsRtcm1033Message rtcm1033;
    GpsRtcm1230Message rtcm1230;
    GpsRtcm1072Message rtcm1072;
    GpsRtcm1074Message rtcm1074;
    GpsRtcm1082Message rtcm1082;
    GpsRtcm1084Message rtcm1084;
  } u;
  int64_t timestamp_usec;

  // Query CVT for latest messages.

  // Auxiliary messages.
  if (GetGpsRtcm1006(node, &u.rtcm1006, &timestamp_usec)) {
    out->rtcm1006 = u.rtcm1006;
    out->aux_timestamp_usec[kGpsRtcmAuxMessageType1006] = timestamp_usec;
    out->aux_updated[kGpsRtcmAuxMessageType1006] = true;
  }
  if (GetGpsRtcm1033(node, &u.rtcm1033, &timestamp_usec)) {
    out->rtcm1033 = u.rtcm1033;
    out->aux_timestamp_usec[kGpsRtcmAuxMessageType1033] = timestamp_usec;
    out->aux_updated[kGpsRtcmAuxMessageType1033] = true;
  }
  if (GetGpsRtcm1230(node, &u.rtcm1230, &timestamp_usec)) {
    out->rtcm1230 = u.rtcm1230;
    out->aux_timestamp_usec[kGpsRtcmAuxMessageType1230] = timestamp_usec;
    out->aux_updated[kGpsRtcmAuxMessageType1230] = true;
  }

  // Observables.
  if (GetGpsRtcm1072(node, &u.rtcm1072, &timestamp_usec)) {
    out->rtcm1072 = u.rtcm1072;
    out->obs_timestamp_usec[kGpsRtcmObsMessageType1072] = timestamp_usec;
    out->obs_updated[kGpsRtcmObsMessageType1072] = true;
  }
  if (GetGpsRtcm1074(node, &u.rtcm1074, &timestamp_usec)) {
    out->rtcm1074 = u.rtcm1074;
    out->obs_timestamp_usec[kGpsRtcmObsMessageType1074] = timestamp_usec;
    out->obs_updated[kGpsRtcmObsMessageType1074] = true;
  }
  if (GetGpsRtcm1082(node, &u.rtcm1082, &timestamp_usec)) {
    out->rtcm1082 = u.rtcm1082;
    out->obs_timestamp_usec[kGpsRtcmObsMessageType1082] = timestamp_usec;
    out->obs_updated[kGpsRtcmObsMessageType1082] = true;
  }
  if (GetGpsRtcm1084(node, &u.rtcm1084, &timestamp_usec)) {
    out->rtcm1084 = u.rtcm1084;
    out->obs_timestamp_usec[kGpsRtcmObsMessageType1084] = timestamp_usec;
    out->obs_updated[kGpsRtcmObsMessageType1084] = true;
  }

  // Transmit auxiliary messages periodically.
  if (out->index >= out->length
      && now_usec - out->aux_last_time >= RTCM_AUX_PERIOD_US
      && TransmitNextRtcmAuxMessage(now_usec, out)) {
    out->aux_last_time = now_usec;
  }

  // Transmit observable messages as fast as possible.
  if (out->index >= out->length) {
    TransmitNextRtcmObsMessage(now_usec, out);
  }
}

void TetherUpCvtRtcmInit(TetherUpCvtRtcmState *state) {
  memset(state, 0, sizeof(*state));
}

void TetherUpCvtRtcmQuery(int64_t now_usec, TetherUpCvtRtcmState *state,
                          TetherUpMessage *out) {
  QueryRtcmMessages(now_usec, state);
  for (int32_t i = 0; i < ARRAYSIZE(out->rtcm); ++i) {
    if (state->index >= state->length) {
      QueryRtcmMessages(now_usec, state);
    }
    if (state->index < state->length) {
      out->rtcm[i] = state->data[state->index++];
    } else {
      out->rtcm[i] = 0x0;
    }
  }
}

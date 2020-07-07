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

#ifndef AVIONICS_COMMON_TETHER_CVT_RTCM_H_
#define AVIONICS_COMMON_TETHER_CVT_RTCM_H_

#include <stdbool.h>
#include <stdint.h>

#include "avionics/common/avionics_messages.h"

typedef enum {
  kGpsRtcmAuxMessageType1006,
  kGpsRtcmAuxMessageType1033,
  kGpsRtcmAuxMessageType1230,
  kNumGpsRtcmAuxMessageTypes,
} GpsRtcmAuxMessageType;

typedef enum {
  kGpsRtcmObsMessageType1072,
  kGpsRtcmObsMessageType1074,
  kGpsRtcmObsMessageType1082,
  kGpsRtcmObsMessageType1084,
  kNumGpsRtcmObsMessageTypes,
} GpsRtcmObsMessageType;

typedef struct {
  // Store pending RTCM messages.
  GpsRtcm1006Message rtcm1006;
  GpsRtcm1033Message rtcm1033;
  GpsRtcm1230Message rtcm1230;
  GpsRtcm1072Message rtcm1072;
  GpsRtcm1074Message rtcm1074;
  GpsRtcm1082Message rtcm1082;
  GpsRtcm1084Message rtcm1084;

  // Auxiliary message transmission state.
  int32_t aux_last_index;  // Last auxiliary message transmitted.
  int64_t aux_last_time;   // Time of last transmission.
  int64_t aux_timestamp_usec[kNumGpsRtcmAuxMessageTypes];
  bool aux_updated[kNumGpsRtcmAuxMessageTypes];

  // Observable message transmission state.
  int32_t obs_last_index;  // Last observable message transmitted.
  int64_t obs_last_time;   // Timestamp of last transmission.
  int64_t obs_timestamp_usec[kNumGpsRtcmObsMessageTypes];
  bool obs_updated[kNumGpsRtcmObsMessageTypes];

  // Current transmission state.
  uint8_t data[GPS_RTCM_MAX_DATA_SIZE];  // Single RTCM message data.
  int32_t length;  // Total length of message in data[].
  int32_t index;   // Position within data[].
} TetherUpCvtRtcmState;

void TetherUpCvtRtcmInit(TetherUpCvtRtcmState *state);
void TetherUpCvtRtcmQuery(int64_t now_usec, TetherUpCvtRtcmState *state,
                          TetherUpMessage *out);

#endif  // AVIONICS_COMMON_TETHER_CVT_RTCM_H_

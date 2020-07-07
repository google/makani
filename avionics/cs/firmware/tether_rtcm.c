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

#include "avionics/cs/firmware/tether_rtcm.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "avionics/common/aio_header.h"
#include "avionics/common/avionics_messages.h"
#include "avionics/common/rtcm3.h"
#include "avionics/common/serial_parse.h"
#include "avionics/common/tether_message_types.h"
#include "avionics/firmware/gps/gps_interface.h"
#include "common/macros.h"

static SerialReceiveBuffer g_stream;
static Rtcm3Receive g_rtcm;

static int32_t g_last_frame_index = 0;
static int64_t g_last_timestamp_usec = INT32_MIN;

static bool SerialParseRtcm(uint32_t sync_flags, int32_t length,
                            const uint8_t *data, void *context,
                            int32_t *parsed) {
  (void)sync_flags;
  return Rtcm3Parse(length, data, (Rtcm3Receive *)context, parsed);
}

static void HandleRtcmData(void) {
  // Parse.
  const SerialParser kSerialParsers[] = {
    {SerialParseRtcm, &g_rtcm}
  };
  int32_t length, protocol;
  const uint8_t *data = SerialParse(ARRAYSIZE(kSerialParsers), kSerialParsers,
                                    &g_stream, &protocol, &length);

  // Transmit parsed RTCM messages.
  if (length > 0 && data != NULL) {
    GpsSendRtcmMessage(g_rtcm.message_number, length, data);
  }
}

static bool IsNewRtcmData(int64_t timestamp_usec, int32_t frame_index) {
  // Frame ids expire after maximum latency.
  if ((timestamp_usec - g_last_timestamp_usec) < AIO_EXPIRATION_TIME_US) {
    // Expected duplication or out of order.
    int32_t delta = frame_index - g_last_frame_index;
    if (delta < 0) {
      delta += TETHER_FRAME_INDEX_ROLLOVER / TETHER_RADIO_DECIMATION;
    }
    return 0 < delta && delta <= TETHER_FRAME_INDEX_ACCEPTANCE_WINDOW;
  }
  return true;
}

void TetherRtcmInit(void) {
  SerialParseInit(&g_stream);
  memset(&g_rtcm, 0, sizeof(g_rtcm));

  g_last_frame_index = 0;
  g_last_timestamp_usec = INT32_MIN;
}

void TetherUpRtcmToNetwork(int64_t timestamp_usec, const TetherUpMessage *in) {
  int32_t frame_index = in->frame_index / TETHER_RADIO_DECIMATION;
  if (IsNewRtcmData(timestamp_usec, frame_index)) {
    g_last_frame_index = frame_index;
    g_last_timestamp_usec = timestamp_usec;
    SerialReadData(ARRAYSIZE(in->rtcm), in->rtcm, &g_stream);
    HandleRtcmData();
  }
}

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

#include "avionics/common/rtcm3.h"

#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "avionics/common/crc.h"
#include "avionics/common/endian.h"
#include "avionics/common/serial_parse.h"

#define RTCM3_PREAMBLE_LENGTH 3
#define RTCM3_CRC_LENGTH 3

bool Rtcm3Parse(int32_t length, const uint8_t *data, Rtcm3Receive *rtcm,
                int32_t *parsed) {
  assert(rtcm != NULL);
  assert(length > 0);
  assert(data != NULL);
  assert(parsed != NULL);

  if (length <= 1) {
    rtcm->crc24 = 0;
    rtcm->data_length = SERIAL_RECEIVE_SIZE;
  }
  if (length <= RTCM3_PREAMBLE_LENGTH + rtcm->data_length) {
    rtcm->crc24 = Crc24Qualcomm(rtcm->crc24, 1, &data[length - 1]);
  }

  bool sync = true;
  uint16_t u16;
  switch (length) {
    case 1:
      // Preamble (8-bits).
      sync = (data[0] == 0xD3);
      break;
    case 2:
      break;
    case 3:
      // Message length (6 bits reserved, 10-bit length).
      ReadUint16Be(&data[1], &u16);
      rtcm->data_length = u16 & 0x03FF;
      sync = (rtcm->data_length <= (SERIAL_RECEIVE_SIZE - RTCM3_PREAMBLE_LENGTH
                                    - RTCM3_CRC_LENGTH));
      break;
    case 4:
      break;
    case 5:
      // Message number (first 12 bits).
      ReadUint16Be(&data[3], &u16);
      rtcm->message_number = u16 >> 4;
      break;
    default:
      if (RTCM3_PREAMBLE_LENGTH + rtcm->data_length + RTCM3_CRC_LENGTH
          == length) {
        uint32_t crc24;
        ReadUint24Be(&data[RTCM3_PREAMBLE_LENGTH + rtcm->data_length], &crc24);
        sync = (rtcm->crc24 == crc24);
        *parsed = length;
      }
      break;
  }
  return sync;
}

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

#ifndef AVIONICS_COMMON_SERIAL_PARSE_H_
#define AVIONICS_COMMON_SERIAL_PARSE_H_

#include <stdbool.h>
#include <stdint.h>

#ifndef __linux__
#include "avionics/firmware/cpu/sci.h"
#endif  // !__linux__

#ifdef __cplusplus
extern "C" {
#endif

#define SERIAL_RECEIVE_SIZE 4096

typedef enum {
  kSerialSyncPacket = 1 << 0,
  kSerialSyncCrLf   = 1 << 1,
  kSerialSyncCr     = 1 << 2,
  kSerialSyncLf     = 1 << 3,
  kSerialSyncEol    = kSerialSyncCrLf | kSerialSyncCr | kSerialSyncLf
} SerialSync;

// This structure should not be accessed directly.
typedef struct {
  uint8_t data[SERIAL_RECEIVE_SIZE];
  bool synchronized;   // No dropped bytes since last message.
  int32_t offset;      // Starting position of next block of data.
  int32_t length;      // Number of bytes in data.
  int32_t parsed;      // Number of bytes parsed in data.
  uint32_t sync_bits;  // Bitmask of protocols able to parse data.
} SerialReceiveBuffer;

typedef struct {
  // Parse function. This function shall return false when it fails to
  // interpret the contents of data as a valid message or leading message
  // fragment. It shall return return true when valid or valid-but-incomplete.
  //
  // Args:
  //   sync_flags: bitmask of SerialSync enumeration.
  //   length: number of bytes available in data (length > 0).
  //   data: data available to parse (no alignment guarantee).
  //   context: arbitrary context pointer.
  //   parsed: output number of bytes the calling function should advance before
  //       the next execution of this function. For complete messages, this
  //       function should output the total message length. For incomplete
  //       messages, this function should output zero.
  //
  // Return:
  //   Return false when this function fails to interpret the contents of data
  //   as a valid message or leading message fragment. Return true for valid
  //   or valid-but-incomplete messages.
  //
  // The calling function should call this function once for each new data
  // byte, as long as the return value is true and parsed is zero.
  bool (* const func)(uint32_t sync_flags, int32_t length, const uint8_t *data,
                      void *context, int32_t *parsed);
  // Arbitrary context pointer to pass to parse function.
  void *context;
} SerialParser;

void SerialParseInit(SerialReceiveBuffer *rx);

// Returns pointer to message parsed by *protocol. Call this function
// repeatedly until it returns NULL. A NULL return value indicates this
// function requires more data to parse the next message.
const uint8_t *SerialParse(int32_t protocols, const SerialParser parsers[],
                           SerialReceiveBuffer *rx, int32_t *protocol,
                           int32_t *length);

// Returns the number of unallocated bytes in parsing buffer.
int32_t SerialReadGetAvailableBytes(const SerialReceiveBuffer *rx);

// Returns the number of bytes read into the parsing buffer.
int32_t SerialReadData(int32_t length, const uint8_t *data,
                       SerialReceiveBuffer *rx);

#ifndef __linux__
// Returns true when buffer requires parsing.
bool SerialPollSci(const SciDevice *dev, SerialReceiveBuffer *rx);
#endif  // !__linux__

#ifdef __linux__
// Returns true when buffer requires parsing.
bool SerialPollDevice(int32_t device_fd, SerialReceiveBuffer *rx);
#endif  // __linux__

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // AVIONICS_COMMON_SERIAL_PARSE_H_

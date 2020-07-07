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

#include "avionics/servo/firmware/r22_serial.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "avionics/common/endian.h"
#include "avionics/firmware/cpu/clock.h"
#include "avionics/firmware/cpu/sci.h"
#include "avionics/servo/firmware/r22_def.h"
#include "avionics/servo/firmware/r22_param.h"
#include "avionics/servo/firmware/r22_types.h"

// See Copley Control's Binary Serial Interface v1.2 document (ref AN112).
// http://www.copleycontrols.com/Motion/pdf/Binary-Serial.pdf
#define R22_SERIAL_CHECKSUM      0x5A  // XOR of all bytes equals this value.
#define R22_SERIAL_HEADER_LENGTH 4
#define R22_SERIAL_SYNC          0x00
#define R22_SERIAL_LENGTH        128

static struct {
  R22OpCode op_code;
  R22Parameter param;
  R22Error error;
  int32_t write_index;
  int32_t write_length;
  bool write_complete;
  int32_t read_length;
  uint8_t read_checksum;
  bool read_complete;
  int64_t read_timestamp;
  uint8_t data[R22_SERIAL_LENGTH];
} g_state;

// The R22 uses an XOR of all bytes (including checksum field) to represent the
// message checksum. The resulting checksum should equal R22_SERIAL_CHECKSUM.
static uint8_t ComputeChecksum(int32_t length, const uint8_t *data) {
  assert(length >= 0);
  assert(data != NULL);

  uint8_t chksum = 0U;
  for (int32_t i = 0; i < length; ++i) {
    chksum ^= data[i];
  }
  return chksum;
}

// This function handles all common write initialization operations. Initialize
// the data portion of g_state.data, then call this function with length
// representing the total transfer length including header.
static void WriteInit(R22OpCode op_code, R22Parameter param, int32_t length) {
  assert((op_code & 0xFF) == op_code);
  assert(R22_SERIAL_HEADER_LENGTH <= length && length <= R22_SERIAL_LENGTH);
  assert((length - R22_SERIAL_HEADER_LENGTH) % 2 == 0);

  g_state.op_code = op_code;
  g_state.param = param;
  g_state.error = kR22ErrorTimeout;
  g_state.write_index = 0;
  g_state.write_length = length;
  g_state.write_complete = false;
  g_state.read_length = 0;
  g_state.read_checksum = 0;
  g_state.read_complete = false;
  g_state.data[0] = R22_SERIAL_SYNC;
  g_state.data[1] = R22_SERIAL_CHECKSUM;  // Commute desired checksum.
  g_state.data[2] = (length - R22_SERIAL_HEADER_LENGTH) >> 1;
  g_state.data[3] = op_code;
  g_state.data[1] = ComputeChecksum(length, g_state.data);
}

// Write parameter variable identifier and parameter location. This function is
// common to get/set/copy write functions.
static int32_t WriteVariableIdentifier(R22Parameter param, R22Memory mem,
                                       uint8_t *out) {
  uint16_t serial_id = R22ParamGetInfo(param)->serial_id;
  assert((serial_id & 0x01FF) == serial_id);

  uint16_t var = (serial_id & 0x01FF) | ((mem == kR22MemoryFlash) << 12);
  return WriteUint16Be(var, out);
}

void R22SerialWriteNoOperation(void) {
  WriteInit(kR22OpCodeNoOperation, kR22ParamNone, R22_SERIAL_HEADER_LENGTH);
}

void R22SerialWriteRetrieveOperatingMode(void) {
  WriteInit(kR22OpCodeRetrieveOperatingMode, kR22ParamNone,
            R22_SERIAL_HEADER_LENGTH);
}

void R22SerialWriteSwapOperatingModes(void) {
  WriteInit(kR22OpCodeSwapOperatingModes, kR22ParamNone,
            R22_SERIAL_HEADER_LENGTH);
}

void R22SerialWriteGetVariableValue(R22Parameter param, R22Memory mem) {
  int32_t o = R22_SERIAL_HEADER_LENGTH;
  o += WriteVariableIdentifier(param, mem, &g_state.data[o]);
  WriteInit(kR22OpCodeGetVariableValue, param, o);
}

void R22SerialWriteSetVariableUint8(R22Parameter param, R22Memory mem,
                                    int32_t in_length, const uint8_t *in) {
  assert(in_length > 0);
  assert(in != NULL);

  int32_t o = R22_SERIAL_HEADER_LENGTH;
  o += WriteVariableIdentifier(param, mem, &g_state.data[o]);
  for (int32_t i = 0; i < in_length; ++i, ++o) {
    g_state.data[o] = in[i];
  }
  WriteInit(kR22OpCodeSetVariableValue, param, o);
}

void R22SerialWriteSetVariableUint16(R22Parameter param, R22Memory mem,
                                     int32_t in_length, const uint16_t *in) {
  assert(in_length > 0);
  assert(in != NULL);

  int32_t o = R22_SERIAL_HEADER_LENGTH;
  o += WriteVariableIdentifier(param, mem, &g_state.data[o]);
  for (int32_t i = 0; i < in_length; ++i) {
    o += WriteUint16Be(in[i], &g_state.data[o]);
  }
  WriteInit(kR22OpCodeSetVariableValue, param, o);
}

void R22SerialWriteSetVariableInt16(R22Parameter param, R22Memory mem,
                                    int32_t in_length, const int16_t *in) {
  R22SerialWriteSetVariableUint16(param, mem, in_length, (const uint16_t *)in);
}

void R22SerialWriteSetVariableUint32(R22Parameter param, R22Memory mem,
                                     int32_t in_length, const uint32_t *in) {
  assert(in_length > 0);
  assert(in != NULL);

  int32_t o = R22_SERIAL_HEADER_LENGTH;
  o += WriteVariableIdentifier(param, mem, &g_state.data[o]);
  for (int32_t i = 0; i < in_length; ++i) {
    o += WriteUint32Be(in[i], &g_state.data[o]);
  }
  WriteInit(kR22OpCodeSetVariableValue, param, o);
}

void R22SerialWriteSetVariableInt32(R22Parameter param, R22Memory mem,
                                    int32_t in_length, const int32_t *in) {
  R22SerialWriteSetVariableUint32(param, mem, in_length, (const uint32_t *)in);
}

void R22SerialWriteReset(void) {
  WriteInit(kR22OpCodeReset, kR22ParamNone, R22_SERIAL_HEADER_LENGTH);
}

void R22SerialWriteTrajectory(R22TrajectorySubcommand traj) {
  assert((traj & 0x0F) == traj);

  int32_t o = R22_SERIAL_HEADER_LENGTH;
  o += WriteUint16Be(traj & 0x0F, &g_state.data[o]);
  WriteInit(kR22OpCodeTrajectoryCommand, kR22ParamNone, o);
}

void R22SerialWriteErrorLog(R22ErrorLogSubcommand subcmd, uint16_t param) {
  int32_t o = R22_SERIAL_HEADER_LENGTH;
  o += WriteUint16Be(subcmd, &g_state.data[o]);
  if (subcmd == kR22ErrorLogGetData) {
    o += WriteUint16Be(param, &g_state.data[o]);
  }
  WriteInit(kR22OpCodeErrorLogCommand, kR22ParamNone, o);
}

bool R22SerialSend(void) {
  if (!g_state.write_complete) {
    while (g_state.write_index < g_state.write_length
           && SciWriteByte(&R22_SCI, g_state.data[g_state.write_index])) {
      ++g_state.write_index;
    }
    g_state.write_complete = ((g_state.write_index >= g_state.write_length)
                              && !SciIsWriteBusy(&R22_SCI));
  }
  return g_state.write_complete;
}

// Accumulate and parse incoming data. Call this function while !read_complete.
static void ReceiveByte(uint8_t ch) {
  assert(g_state.read_length < R22_SERIAL_LENGTH);
  assert(!g_state.read_complete);

  // Accumulate incoming data.
  g_state.data[g_state.read_length] = ch;
  ++g_state.read_length;
  g_state.read_checksum ^= ch;  // Checksum is XOR of all bytes.
  if (g_state.read_length == 1) {
    g_state.read_timestamp = ClockGetUs();
  }

  // Validate message (starting at data[0]) as much as possible. If the message
  // fails the criteria, shift the buffer until the next sync byte and
  // reevaluate.
  int32_t shift = 0;
  do {
    const uint16_t msg_length = R22_SERIAL_HEADER_LENGTH + 2 * g_state.data[2];
    if (g_state.read_length && g_state.data[0] != R22_SERIAL_SYNC) {
      // Invalid sync byte.
      shift = 1;
    } else if (g_state.read_length < R22_SERIAL_HEADER_LENGTH) {
      // Require more data.
      shift = 0;
    } else if (msg_length > R22_SERIAL_LENGTH) {
      // Invalid message length.
      shift = 1;
    } else if (g_state.read_length < msg_length) {
      // Require more data.
      shift = 0;
    } else if (g_state.read_checksum != R22_SERIAL_CHECKSUM) {
      // Invalid checksum.
      shift = 1;
    } else {
      // Valid message.
      // Zero remaining data since there may be an inconsistency between the
      // number of parameters expected and those returned. This practice also
      // ensures null termination of strings.
      for (int32_t i = g_state.read_length; i < R22_SERIAL_LENGTH; ++i) {
        g_state.data[i] = 0U;
      }
      g_state.read_complete = true;
      g_state.error = g_state.data[3];
      shift = 0;
    }

    // Handle parsing errors by shifting receive buffer to next sync byte.
    while (shift < g_state.read_length
           && g_state.data[shift] != R22_SERIAL_SYNC) {
      ++shift;
    }

    // Advance buffer to eliminate sync errors.
    if (shift) {
      g_state.read_length -= shift;
      memmove(&g_state.data[0], &g_state.data[shift], g_state.read_length);
      g_state.read_checksum = ComputeChecksum(g_state.read_length,
                                              g_state.data);
    }
  } while (shift && g_state.read_length);
}

bool R22SerialReceive(void) {
  uint8_t ch;
  while (!g_state.read_complete && SciReadByte(&R22_SCI, &ch)) {
    ReceiveByte(ch);
  }
  return g_state.read_complete;
}

int64_t R22SerialReadTimestamp(void) {
  return g_state.read_timestamp;
}

R22Error R22SerialReadError(void) {
  return g_state.error;
}

bool R22SerialReadReturnSuccess(void) {
  return g_state.read_complete && g_state.error == kR22ErrorNone;
}

bool R22SerialReadUint8(int32_t data_length, uint8_t *data) {
  assert(data_length > 0);
  assert(data_length <= (R22_SERIAL_LENGTH - R22_SERIAL_HEADER_LENGTH));
  assert(data != NULL);

  bool success = R22SerialReadReturnSuccess();
  if (success) {
    const uint8_t *reply = &g_state.data[R22_SERIAL_HEADER_LENGTH];
    for (int32_t i = 0; i < data_length; ++i) {
      reply += ReadUint8Be(reply, &data[i]);
    }
  }
  return success;
}

bool R22SerialReadInt8(int32_t data_length, int8_t *data) {
  return R22SerialReadUint8(data_length, (uint8_t *)data);
}

bool R22SerialReadUint16(int32_t data_length, uint16_t *data) {
  assert(data_length > 0);
  assert(data_length <= (R22_SERIAL_LENGTH - R22_SERIAL_HEADER_LENGTH)/2);
  assert(data != NULL);

  bool success = R22SerialReadReturnSuccess();
  if (success) {
    const uint8_t *reply = &g_state.data[R22_SERIAL_HEADER_LENGTH];
    for (int32_t i = 0; i < data_length; ++i) {
      reply += ReadUint16Be(reply, &data[i]);
    }
  }
  return success;
}

bool R22SerialReadInt16(int32_t data_length, int16_t *data) {
  return R22SerialReadUint16(data_length, (uint16_t *)data);
}

bool R22SerialReadUint32(int32_t data_length, uint32_t *data) {
  assert(data_length > 0);
  assert(data_length <= (R22_SERIAL_LENGTH - R22_SERIAL_HEADER_LENGTH)/4);
  assert(data != NULL);

  bool success = R22SerialReadReturnSuccess();
  if (success) {
    const uint8_t *reply = &g_state.data[R22_SERIAL_HEADER_LENGTH];
    for (int32_t i = 0; i < data_length; ++i) {
      reply += ReadUint32Be(reply, &data[i]);
    }
  }
  return success;
}

bool R22SerialReadInt32(int32_t data_length, int32_t *data) {
  return R22SerialReadUint32(data_length, (uint32_t *)data);
}

bool R22SerialCompare(int32_t data_length, const void *data) {
  assert(data_length > 0);
  assert(data_length <= (R22_SERIAL_LENGTH - R22_SERIAL_HEADER_LENGTH));
  assert(data != NULL);

  return R22SerialReadReturnSuccess()
      && !memcmp(&g_state.data[R22_SERIAL_HEADER_LENGTH], data, data_length);
}

bool R22SerialTransfer(void) {
  return R22SerialSend() && R22SerialReceive();
}

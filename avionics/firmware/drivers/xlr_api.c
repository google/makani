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

#include "avionics/firmware/drivers/xlr_api.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "avionics/common/endian.h"

// Transmit functions.

typedef struct {
  uint8_t chksum;
  uint8_t *data;   // Address of output buffer.
  int32_t count;   // Number of bytes written.
  int32_t length;  // Total length of output buffer.
  bool overflow;
} OutputBuffer;

static void OutputByte(uint8_t in, OutputBuffer *out) {
  if (out->count < out->length) {
    out->chksum = (uint8_t)(out->chksum - in);
    out->data[out->count] = in;
    ++out->count;
  } else {
    out->overflow = true;
  }
}

static void OutputData(int32_t length, const void *in, OutputBuffer *out) {
  const uint8_t *in8 = (const uint8_t *)in;
  for (int32_t i = 0; i < length; ++i) {
    OutputByte(in8[i], out);
  }
}

static int32_t GetOutputBytes(const OutputBuffer *out) {
  return out->overflow ? 0 : out->count;
}

static void WriteHeader(int32_t api_length, OutputBuffer *out) {
  uint8_t buf[4];

  // The checksum does not include the frame delimiter or length.
  OutputByte(XLR_FRAME_DELIM, out);
  OutputData(WriteUint16Be((uint16_t)api_length, buf), buf, out);
  out->chksum = XLR_CHKSUM_INIT;
}

static void WriteAtCommand(XlrApiFrameType frame_type, uint8_t frame_id,
                           XlrAtCommand command, int32_t param_length,
                           const void *param, OutputBuffer *out) {
  uint8_t buf[4];

  WriteHeader(4 + param_length, out);
  OutputByte(frame_type, out);
  OutputByte(frame_id, out);
  OutputData(WriteUint16Be((uint16_t)command, buf), buf, out);
  OutputData(param_length, param, out);
  OutputByte(out->chksum, out);
}

static void WriteTxRequest(uint8_t frame_id, uint64_t dest_addr, uint8_t hops,
                           uint8_t options, int32_t data_length,
                           const uint8_t *data, OutputBuffer *out) {
  uint8_t buf[8];

  WriteHeader(14 + data_length, out);
  OutputByte(kXlrApiFrameTypeTxRequest, out);
  OutputByte(frame_id, out);
  OutputData(WriteUint64Be(dest_addr, buf), buf, out);
  OutputData(WriteUint16Be(0xFFFE, buf), buf, out);  // Reserved.
  OutputByte(hops, out);  // Broadcast radius.
  OutputByte(options, out);
  OutputData(data_length, data, out);
  OutputByte(out->chksum, out);
}

// Parse receive functions.

static bool ParseCommandResponse(int32_t length, const uint8_t *data,
                                 XlrApiCommandResponse *out) {
  if (length >= 4) {
    int32_t o = 0;
    o += ReadUint8Be(&data[o], &out->frame_id);
    uint16_t command;
    o += ReadUint16Be(&data[o], &command);
    out->command = (XlrAtCommand)command;
    uint8_t status;
    o += ReadUint8Be(&data[o], &status);
    out->status = (XlrApiResponseStatus)status;
    out->length = length - o;
    out->data = &data[o];
    return true;
  }
  return false;
}

static bool ParseModemStatus(int32_t length, const uint8_t *data,
                             XlrApiModemStatus *out) {
  if (length == 1) {
    ReadUint8Be(&data[0], &out->status);
    return true;
  }
  return false;
}

static bool ParseTransmitStatus(int32_t length, const uint8_t *data,
                                XlrApiTransmitStatus *out) {
  if (length == 6) {
    int32_t o = 0;
    o += ReadUint8Be(&data[o], &out->frame_id);
    o += 2;  // Reserved.
    o += ReadUint8Be(&data[o], &out->retries);
    o += ReadUint8Be(&data[o], &out->delivery_status);
    o += ReadUint8Be(&data[o], &out->discovery_status);
    return true;
  }
  return false;
}

static bool ParseRxIndicator(int32_t length, const uint8_t *data,
                             XlrApiRxIndicator *out) {
  if (length >= 11) {
    int32_t o = 0;
    o += ReadUint64Be(&data[o], &out->source);
    o += 2;  // Reserved.
    o += ReadUint8Be(&data[o], &out->options);
    out->length = length - o;
    out->data = &data[o];
    return true;
  }
  return false;
}

static bool ParseData(int32_t length, const uint8_t *data, XlrApiData *out) {
  // Define payload as the data following the frame type field.
  int32_t payload_length = length - 1;
  const uint8_t *payload_data = &data[1];

  out->frame_type = (XlrApiFrameType)data[0];
  switch (out->frame_type) {
    case kXlrApiFrameTypeCommand:
    case kXlrApiFrameTypeCommandQueue:
    case kXlrApiFrameTypeTxRequest:
    case kXlrApiFrameTypeExplicitTxRequest:
    case kXlrApiFrameTypeRemoteCommandRequest:
      return false;  // Not supported; sent by host.
    case kXlrApiFrameTypeCommandResponse:
      return ParseCommandResponse(payload_length, payload_data,
                                  &out->u.command_response);
    case kXlrApiFrameTypeModemStatus:
      return ParseModemStatus(payload_length, payload_data,
                              &out->u.modem_status);
    case kXlrApiFrameTypeTransmitStatus:
      return ParseTransmitStatus(payload_length, payload_data,
                                 &out->u.transmit_status);
    case kXlrApiFrameTypeRxIndicator:
      return ParseRxIndicator(payload_length, payload_data,
                              &out->u.rx_indicator);
    case kXlrApiFrameTypeExplicitRxIndicator:
      return false;  // Not supported.
    case kXlrApiFrameTypeRemoteCommandResponse:
      return false;  // Not supported.
    default:
      return false;
  }
}

// Public (application) functions.

int32_t XlrApiWriteAtCommand(XlrApiFrameType frame_type, uint8_t frame_id,
                             XlrAtCommand command, int32_t param_length,
                             const void *param, int32_t out_length,
                             uint8_t *out) {
  assert(frame_type == kXlrApiFrameTypeCommand
         || frame_type == kXlrApiFrameTypeCommandQueue);
  assert((param_length > 0) ^ (param == NULL));
  assert(out != NULL);

  OutputBuffer buf = {
    .data = out,
    .length = out_length,
  };
  WriteAtCommand(frame_type, frame_id, command, param_length, param, &buf);
  return GetOutputBytes(&buf);
}

int32_t XlrApiWriteTxRequest(uint8_t frame_id, uint64_t dest_addr,
                             uint8_t hops, uint8_t options,
                             int32_t data_length, const uint8_t *data,
                             int32_t out_length, uint8_t *out) {
  assert(0 <= data_length && data_length < INT16_MAX);
  assert(data != NULL);
  assert(out != NULL);

  OutputBuffer buf = {
    .data = out,
    .length = out_length,
  };
  WriteTxRequest(frame_id, dest_addr, hops, options, data_length, data, &buf);
  return GetOutputBytes(&buf);
}

bool XlrApiParseData(int32_t length, const uint8_t *data, XlrApiData *out) {
  assert(length > 4);
  assert(data != NULL);
  assert(out != NULL);
  return ParseData(length - 4, &data[3], out);
}

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

#ifndef AVIONICS_FIRMWARE_DRIVERS_XLR_API_H_
#define AVIONICS_FIRMWARE_DRIVERS_XLR_API_H_

// This module implements the Digi modem XLR PRO API communication protocol.
// http://www.digi.com/resources/documentation/digidocs/PDFs/90002202.pdf

#include <stdbool.h>
#include <stdint.h>

#include "avionics/firmware/drivers/xlr_at.h"

#ifdef __cplusplus
extern "C" {
#endif

#define XLR_FRAME_DELIM 0x7E
#define XLR_CHKSUM_INIT 0xFF
#define XLR_ESCAPE      0x7D
#define XLR_ESCAPE_XOR  0x20
#define XLR_XON         0x11
#define XLR_XOFF        0x13

typedef enum {
  kXlrApiFrameTypeCommand               = 0x08,
  kXlrApiFrameTypeCommandQueue          = 0x09,
  kXlrApiFrameTypeTxRequest             = 0x10,
  kXlrApiFrameTypeExplicitTxRequest     = 0x11,
  kXlrApiFrameTypeRemoteCommandRequest  = 0x17,
  kXlrApiFrameTypeCommandResponse       = 0x88,
  kXlrApiFrameTypeModemStatus           = 0x8A,
  kXlrApiFrameTypeTransmitStatus        = 0x8B,
  kXlrApiFrameTypeRxIndicator           = 0x90,
  kXlrApiFrameTypeExplicitRxIndicator   = 0x91,
  kXlrApiFrameTypeRemoteCommandResponse = 0x97,
} XlrApiFrameType;

typedef enum {
  kXlrApiResponseStatusOk               = 0,
  kXlrApiResponseStatusError            = 1,
  kXlrApiResponseStatusInvalidCommand   = 2,
  kXlrApiResponseStatusInvalidParameter = 3,
} XlrApiResponseStatus;

typedef struct {
  uint8_t frame_id;
  XlrAtCommand command;
  XlrApiResponseStatus status;
  int32_t length;
  const uint8_t *data;
} XlrApiCommandResponse;

typedef struct {
  uint8_t status;
} XlrApiModemStatus;

typedef struct {
  uint8_t frame_id;
  uint8_t retries;
  uint8_t delivery_status;
  uint8_t discovery_status;
} XlrApiTransmitStatus;

typedef struct {
  uint64_t source;
  uint8_t options;
  int32_t length;
  const uint8_t *data;
} XlrApiRxIndicator;

typedef struct {
  XlrApiFrameType frame_type;
  union {
    XlrApiCommandResponse command_response;
    XlrApiModemStatus modem_status;
    XlrApiTransmitStatus transmit_status;
    XlrApiRxIndicator rx_indicator;
  } u;
} XlrApiData;

int32_t XlrApiWriteAtCommand(XlrApiFrameType frame_type, uint8_t frame_id,
                             XlrAtCommand command, int32_t param_length,
                             const void *param, int32_t out_length,
                             uint8_t *out);

int32_t XlrApiWriteTxRequest(uint8_t frame_id, uint64_t dest_addr,
                             uint8_t hops, uint8_t options,
                             int32_t data_length, const uint8_t *data,
                             int32_t out_length, uint8_t *out);

bool XlrApiParseData(int32_t length, const uint8_t *data, XlrApiData *out);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // AVIONICS_FIRMWARE_DRIVERS_XLR_API_H_

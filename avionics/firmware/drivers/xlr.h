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

#ifndef AVIONICS_FIRMWARE_DRIVERS_XLR_H_
#define AVIONICS_FIRMWARE_DRIVERS_XLR_H_

#include <stdbool.h>
#include <stdint.h>

#include "avionics/common/serial_parse.h"
#include "avionics/firmware/cpu/sci.h"
#include "avionics/firmware/drivers/xlr_api.h"
#include "avionics/firmware/drivers/xlr_serial.h"

typedef enum {
  kXlrStateInit,
  kXlrStateEnterAtCommandMode,
  kXlrStateSendAtInit,
  kXlrStateGetAtConfig,
  kXlrStateSetAtConfig,
  kXlrStateSaveAtConfig,
  kXlrStateLeaveAtCommandMode,
  kXlrStateReady,
  kXlrStateSetAesEncryptionKey,
  kXlrStateSetNetworkId,
  kXlrStateWatchdog,
} XlrState;

typedef struct {
  const SciDevice *device;
  const char **at_string;
  int32_t num_at_strings;
} XlrConfig;

typedef struct {
  int32_t index;
  int32_t length;
  uint8_t frame_id;  // Store frame_id of last command (frame_id != 0).
  uint8_t data[1024];
} XlrCommandBuffer;

typedef enum {
  kXlrResponseTypeNone,
  kXlrResponseTypeAtCommand,
  kXlrResponseTypeApiCommand,
} XlrResponseType;

typedef struct {
  int32_t length;
  uint8_t data[32];
} XlrAtCommandResponse;

typedef struct {
  XlrResponseType response_type;
  union {
    XlrAtCommandResponse at_command;
    XlrApiCommandResponse api_command;
  } u;
} XlrResponseData;

typedef struct {
  uint32_t last_api_cycles;
  uint32_t head;
  uint32_t tail;
  XlrResponseData data[8];
} XlrResponseBuffer;

typedef struct {
  uint32_t timeout;
  int32_t init_index;
  int32_t baud_index;
  bool first_entry;
  bool saved_config;
  bool save_config;
  bool api_mode;
  uint8_t frame_id;  // Cache frame_id of current state's command.
  uint32_t network_id;
  uint32_t aes_key[4];
  uint32_t flags;
  XlrState next;
} XlrStateMachine;

typedef struct {
  XlrStateMachine state;
  XlrCommandBuffer command;
  XlrResponseBuffer response;
  SerialReceiveBuffer receive;
  XlrReceive parse;
} XlrDevice;

void XlrInit(XlrDevice *x);
void XlrSetAesEncryptionKey(const uint32_t aes_key[4], XlrDevice *x);
void XlrSetNetworkId(uint32_t network_id, XlrDevice *x);

bool XlrPoll(const XlrConfig *conf, XlrDevice *x, XlrApiData *out);

bool XlrSendUnicast(const XlrConfig *conf, uint64_t dest_addr, uint8_t hops,
                    uint8_t options, int32_t data_length, const void *data,
                    uint8_t *frame_id, XlrDevice *x);

bool XlrSendBroadcast(const XlrConfig *conf, uint8_t hops, uint8_t options,
                      int32_t data_length, const void *data, uint8_t *frame_id,
                      XlrDevice *x);

bool XlrSendGetTemperature(const XlrConfig *conf, XlrDevice *x);
bool XlrSendGetReceivedSignalStrength(const XlrConfig *conf, XlrDevice *x);
bool XlrSendGetReceivedErrorCount(const XlrConfig *conf, XlrDevice *x);
bool XlrSendClearReceivedErrorCount(const XlrConfig *conf, XlrDevice *x);
bool XlrSendGetGoodPacketsReceived(const XlrConfig *conf, XlrDevice *x);
bool XlrSendClearGoodPacketsReceived(const XlrConfig *conf, XlrDevice *x);
bool XlrSendGetTransmissionFailureCount(const XlrConfig *conf, XlrDevice *x);
bool XlrSendClearTransmissionFailureCount(const XlrConfig *conf, XlrDevice *x);

#endif  // AVIONICS_FIRMWARE_DRIVERS_XLR_H_

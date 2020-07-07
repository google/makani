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

#ifndef AVIONICS_FIRMWARE_DRIVERS_CANOPEN_SDO_H_
#define AVIONICS_FIRMWARE_DRIVERS_CANOPEN_SDO_H_

#include <stdbool.h>
#include <stdint.h>

#include "avionics/firmware/cpu/dcan.h"

#define CANOPEN_SDO_DATA_BUFFER_LENGTH 128

typedef enum {
  kCanopenSdoStateInit,
  kCanopenSdoStateIdle,
  kCanopenSdoStateWriteTransmit,
  kCanopenSdoStateWriteReceive,
  kCanopenSdoStateWriteContinueTransmit,
  kCanopenSdoStateWriteContinueReceive,
  kCanopenSdoStateReadTransmit,
  kCanopenSdoStateReadReceive,
  kCanopenSdoStateReadContinueTransmit,
  kCanopenSdoStateReadContinueReceive,
  kCanopenSdoStateComplete,
  kCanopenSdoStateError
} CanopenSdoState;

// See Table 10: Abort Code Descriptions of
// http://www.a-m-c.com/download/sw/dw300_3-0-3/CAN_Manual300_3-0-3.pdf
typedef enum {
  kCanopenSdoAbortNone = 0,
  kCanopenSdoAbortToggleNotAlternated = 0x05030000,
  kCanopenSdoAbortSdoTimeout = 0x05040000,
  kCanopenSdoAbortCommandSpecifierNotValid = 0x05040001,
  kCanopenSdoAbortInvalidBlockSize = 0x05040002,
  kCanopenSdoAbortInvalidSequence = 0x05040003,
  kCanopenSdoAbortCrcSdoAbort = 0x05040004,
  kCanopenSdoAbortOutOfMemory = 0x05040005,
  kCanopenSdoAbortUnsupportedAccess = 0x06010000,
  kCanopenSdoAbortReadOnWriteOnly = 0x06010001,
  kCanopenSdoAbortWriteOnReadOnly = 0x06010002,
  kCanopenSdoAbortInvalidDictionaryObject = 0x06020000,
  kCanopenSdoAbortPdoCannotMapObject = 0x06040041,
  kCanopenSdoAbortPdoTooManyObjects = 0x06040042,
  kCanopenSdoAbortParameterIncompatiblity = 0x06040043,
  kCanopenSdoAbortInternalIncompatibility = 0x06040047,
  kCanopenSdoAbortHardwareSdoAbort = 0x06060000,
  kCanopenSdoAbortServiceParameterMismatch = 0x06070010,
  kCanopenSdoAbortServiceParameterTooHigh = 0x06070012,
  kCanopenSdoAbortServiceParameterTooLow = 0x06070013,
  kCanopenSdoAbortInvalidSubindex = 0x06090011,
  kCanopenSdoAbortWriteOutsideValueRange = 0x06090030,
  kCanopenSdoAbortValueTooHigh = 0x06090031,
  kCanopenSdoAbortValueTooLow = 0x06090032,
  kCanopenSdoAbortMaximumValueBelowMinimum = 0x06090036,
  kCanopenSdoAbortGeneral = 0x08000000,
  kCanopenSdoAbortDataCannotBeHandled = 0x08000020,
  kCanopenSdoAbortDataCannotBeHandledLocalControl = 0x08000021,
  kCanopenSdoAbortDataCannotBeHandledDeviceState = 0x08000022,
  kCanopenSdoAbortObjectDictionaryFailed = 0x08000023
} CanopenSdoAbort;

typedef struct {
  const DcanBus bus;
  const int32_t transmit_mailbox;
  const int32_t receive_mailbox;
  const int32_t node_id_remote;
} CanopenSdoConfig;

typedef struct {
  CanopenSdoState state;
  uint16_t index;
  uint8_t subindex;
  int32_t length;
  int32_t length_processed;
  int64_t read_timestamp;
  bool toggle;
  bool last_operation_error;
  CanopenSdoAbort abort_code;
  uint8_t data[CANOPEN_SDO_DATA_BUFFER_LENGTH];
} CanopenSdo;

bool CanopenSdoWriteLe(uint16_t index, uint8_t subindex, int32_t length,
                       const uint8_t *data, CanopenSdo *sdo);

bool CanopenSdoWriteBe(uint16_t index, uint8_t subindex, int32_t length,
                       const uint8_t *data, CanopenSdo *sdo);

bool CanopenSdoRead(uint16_t index, uint8_t subindex, int32_t length,
                    CanopenSdo *sdo);

void CanopenSdoInit(CanopenSdo *sdo);

bool CanopenSdoPoll(const CanopenSdoConfig *config, CanopenSdo *sdo);

bool CanopenSdoHadError(const CanopenSdo *sdo);

bool CanopenSdoIsIdle(const CanopenSdo *sdo);

CanopenSdoAbort CanopenSdoGetAbort(const CanopenSdo *sdo);

#endif  // AVIONICS_FIRMWARE_DRIVERS_CANOPEN_SDO_H_

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

#include "avionics/firmware/drivers/canopen_sdo.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "avionics/common/endian.h"
#include "avionics/firmware/cpu/clock.h"
#include "avionics/firmware/drivers/log.h"

#define LENGTH_UNKNOWN -1
#define SDO_MESSAGE_SIZE 8
#define SDO_HEADER_SIZE 4
#define SDO_PAYLOAD_SIZE (SDO_MESSAGE_SIZE - SDO_HEADER_SIZE)

// CANopen SDO summary at http://www.can-cia.org/index.php?id=152
// Referenced description from:
// http://www.a-m-c.com/download/sw/dw300_3-0-3/CAN_Manual300_3-0-3.pdf
// Referenced command specifier (CS) definition from:
// http://www.nikhef.nl/pub/departments/ct/po/doc/CANopen30.pdf

// CANopen command specifier.
typedef enum {
  kCanopenSdoCsDownloadInit = 1,
  kCanopenSdoCsDownloadInitReply = 3,
  kCanopenSdoCsDownload = 0,
  kCanopenSdoCsDownloadReply = 1,
  kCanopenSdoCsUploadInit = 2,
  kCanopenSdoCsUploadInitReply = 2,
  kCanopenSdoCsUpload = 3,
  kCanopenSdoCsUploadReply = 0,
  kCanopenSdoCsAbort = 4,
} CanopenSdoCs;

typedef struct {
  CanopenSdoCs cs;
  uint8_t size_empty;
  bool expedited;
  bool size_specified;
  bool toggle;
} CanopenSdoCommand;

typedef struct {
  CanopenSdoCommand cmd;
  uint16_t index;
  uint8_t subindex;
} CanopenSdoHeader;

// Pack the SDO command into the first byte of the CANopen message.
// SDO command format is:
// | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
// |     CS    | T |   N   |E/N| S |
// CS is the 3-bit command specifier.
// T is an optional toggle bit depending on mode.
// N represents the number of bytes which do not contain data (2 or 3 bits
// depending on mode).
// E indicates an expedited transfer is being used depending on mode.
// S indicates that the data segment of this message contains the number of
// bytes in the transfer (rather than using N).
static void CanopenSdoCommandPack(const CanopenSdoCommand *cmd, uint8_t *out) {
  assert(cmd != NULL && out != NULL);
  assert(cmd->cs < 5);

  *out = (uint8_t)(cmd->cs << 5);
  *out |= cmd->toggle ? 0x10 : 0;
  if (cmd->cs == kCanopenSdoCsDownloadInit
      || cmd->cs == kCanopenSdoCsUploadInit) {
    assert(cmd->size_empty <= 3);
    *out |= cmd->size_empty << 2;
    *out |= cmd->expedited ? 2 : 0;
  } else {
    assert(cmd->size_empty <= 7);
    assert(!cmd->expedited);
    *out |= cmd->size_empty << 1;
  }
  *out |= cmd->size_specified ? 1 : 0;
}

// Unpack the first byte of the CANopen message.
static void CanopenSdoCommandUnpack(const uint8_t *data,
                                    CanopenSdoCommand *cmd) {
  assert(cmd != NULL && data != NULL);

  cmd->cs = data[0] >> 5;
  cmd->toggle = (data[0] & 0x10) != 0;
  if (cmd->cs == kCanopenSdoCsDownloadInit
      || cmd->cs == kCanopenSdoCsUploadInit) {
    cmd->size_empty = (data[0] >> 2) & 3;
    cmd->expedited = (data[0] & 2) != 0;
  } else {
    cmd->size_empty = (data[0] >> 1) & 7;
    cmd->expedited = false;
  }
  cmd->size_specified = (data[0] & 1) != 0;
}

static void CanopenSdoPack(const CanopenSdoHeader *header, uint8_t *out) {
  assert(header != NULL && out != NULL);

  CanopenSdoCommandPack(&header->cmd, &out[0]);
  WriteUint16Le(header->index, &out[1]);
  out[3] = header->subindex;
}

static void CanopenSdoUnpack(const uint8_t *data, CanopenSdoHeader *header) {
  assert(header != NULL && data != NULL);

  CanopenSdoCommandUnpack(&data[0], &header->cmd);
  ReadUint16Le(&data[1], &header->index);
  header->subindex = data[3];
}

static void CanopenInit(const CanopenSdoConfig *config, CanopenSdo *sdo) {
  DcanSetTransmitMailbox(config->bus, config->transmit_mailbox, kDcanIdStandard,
                         DCAN_STANDARD_MASK, 0x600 + config->node_id_remote,
                         SDO_MESSAGE_SIZE);
  DcanSetReceiveMailbox(config->bus, config->receive_mailbox, kDcanIdStandard,
                        DCAN_STANDARD_MASK, 0x580 + config->node_id_remote,
                        SDO_MESSAGE_SIZE);

  sdo->state = kCanopenSdoStateIdle;
}

static void CanopenCheckAbort(const uint8_t *data, CanopenSdo *sdo) {
  assert(data != NULL);

  CanopenSdoHeader header;
  CanopenSdoUnpack(data, &header);
  if (header.cmd.cs == kCanopenSdoCsAbort) {
    uint32_t abort_code;
    ReadUint32Le(&data[SDO_HEADER_SIZE], &abort_code);
    LOG_PRINTF("Got CANopen abort: state=%d index=%X subindex=%X code=%08lX\n",
               sdo->state, header.index, header.subindex, abort_code);
    sdo->abort_code = (CanopenSdoAbort)abort_code;
  }
  sdo->state = kCanopenSdoStateError;
}

static void CanopenWriteTransmit(const CanopenSdoConfig *config,
                                 CanopenSdo *sdo) {
  uint8_t can_buf[SDO_MESSAGE_SIZE] = {0};

  if (sdo->length <= SDO_PAYLOAD_SIZE) {
    CanopenSdoHeader header = {
      .cmd = {.cs = kCanopenSdoCsDownloadInit,
              .size_empty = SDO_PAYLOAD_SIZE - sdo->length,
              .expedited = true, .size_specified = true, .toggle = false},
      .index = sdo->index, .subindex = sdo->subindex};
    memcpy(&can_buf[SDO_HEADER_SIZE], sdo->data, sdo->length);
    sdo->length_processed = sdo->length;
    CanopenSdoPack(&header, can_buf);
  } else {
    CanopenSdoHeader header = {
      .cmd = {.cs = kCanopenSdoCsDownloadInit, .size_empty = 0,
              .expedited = false, .size_specified = true, .toggle = false},
      sdo->index, sdo->subindex};
    WriteUint32Le(sdo->length, &can_buf[SDO_HEADER_SIZE]);
    sdo->length_processed = 0;
    CanopenSdoPack(&header, can_buf);
  }

  if (DcanTransmit(config->bus, config->transmit_mailbox, SDO_MESSAGE_SIZE,
                   can_buf)) {
    sdo->state = kCanopenSdoStateWriteReceive;
  } else {
    sdo->state = kCanopenSdoStateError;
  }
}

static void CanopenWriteReceive(const CanopenSdoConfig *config,
                                CanopenSdo *sdo) {
  assert(sdo->length_processed <= sdo->length);

  uint8_t data[SDO_MESSAGE_SIZE];
  CanopenSdoHeader header;
  if (DcanGetMailbox(config->bus, config->receive_mailbox, SDO_MESSAGE_SIZE,
                     data, NULL)) {
    CanopenSdoUnpack(data, &header);
    if (header.index == sdo->index && header.subindex == sdo->subindex &&
        header.cmd.cs == kCanopenSdoCsDownloadInitReply) {
      if (sdo->length_processed == sdo->length) {
        sdo->state = kCanopenSdoStateComplete;
      } else {
        sdo->state = kCanopenSdoStateWriteContinueTransmit;
        sdo->toggle = false;
      }
    } else {
      CanopenCheckAbort(data, sdo);
    }
  }
}

static void CanopenWriteContinueTransmit(const CanopenSdoConfig *config,
                                         CanopenSdo *sdo) {
  assert(sdo->length_processed < sdo->length);

  uint8_t can_buf[SDO_MESSAGE_SIZE] = {0};
  uint8_t write_size = 0;

  if ((sdo->length - sdo->length_processed) > 7) {
    write_size = 7;
  } else {
    write_size = sdo->length - sdo->length_processed;
  }
  memcpy(&can_buf[1], &sdo->data[sdo->length_processed], write_size);
  sdo->length_processed += write_size;
  CanopenSdoCommand cmd = {
    .cs = kCanopenSdoCsDownload, .size_empty = 7 - write_size,
    .expedited = false, .size_specified = write_size != 7,
    .toggle = sdo->toggle};
  CanopenSdoCommandPack(&cmd, can_buf);
  if (DcanTransmit(config->bus, config->transmit_mailbox, SDO_MESSAGE_SIZE,
                   can_buf)) {
    sdo->state = kCanopenSdoStateWriteContinueReceive;
  } else {
    sdo->state = kCanopenSdoStateError;
  }
}

static void CanopenWriteContinueReceive(const CanopenSdoConfig *config,
                                        CanopenSdo *sdo) {
  assert(sdo->length_processed <= sdo->length);

  uint8_t data[SDO_MESSAGE_SIZE];
  CanopenSdoCommand cmd;
  if (DcanGetMailbox(config->bus, config->receive_mailbox, SDO_MESSAGE_SIZE,
                     data, NULL)) {
    CanopenSdoCommandUnpack(data, &cmd);
    if (cmd.cs == kCanopenSdoCsDownloadReply && cmd.toggle == sdo->toggle) {
      if (sdo->length_processed == sdo->length) {
        sdo->state = kCanopenSdoStateComplete;
      } else {
        sdo->state = kCanopenSdoStateWriteContinueTransmit;
        sdo->toggle = !sdo->toggle;
      }
    } else {
      CanopenCheckAbort(data, sdo);
    }
  }
}

static void CanopenReadTransmit(const CanopenSdoConfig *config,
                                CanopenSdo *sdo) {
  uint8_t can_buf[SDO_MESSAGE_SIZE];
  CanopenSdoHeader header = {
    .cmd = {.cs = kCanopenSdoCsUploadInit, .size_empty = 0,
            .expedited = false, .size_specified = false, .toggle = false},
    .index = sdo->index, .subindex = sdo->subindex};
  memset(&can_buf[SDO_HEADER_SIZE], 0, SDO_PAYLOAD_SIZE);
  CanopenSdoPack(&header, can_buf);

  if (DcanTransmit(config->bus, config->transmit_mailbox, SDO_MESSAGE_SIZE,
                   can_buf)) {
    sdo->state = kCanopenSdoStateReadReceive;
  } else {
    sdo->state = kCanopenSdoStateError;
  }
}

static void CanopenReadReceive(const CanopenSdoConfig *config,
                               CanopenSdo *sdo) {
  uint8_t data[SDO_MESSAGE_SIZE];
  CanopenSdoHeader header;
  if (DcanGetMailbox(config->bus, config->receive_mailbox, SDO_MESSAGE_SIZE,
                     data, NULL)) {
    CanopenSdoUnpack(data, &header);
    if (header.index == sdo->index && header.subindex == sdo->subindex &&
        header.cmd.cs == kCanopenSdoCsUploadInitReply) {
      sdo->read_timestamp = ClockGetUs();
      if (header.cmd.expedited) {
        if (header.cmd.size_specified) {
          memcpy(sdo->data, &data[SDO_HEADER_SIZE],
                 SDO_PAYLOAD_SIZE - header.cmd.size_empty);
        } else {
          memcpy(sdo->data, &data[SDO_HEADER_SIZE], SDO_PAYLOAD_SIZE);
        }
        sdo->state = kCanopenSdoStateComplete;
      } else {
        if (header.cmd.size_specified) {
          ReadInt32Le(&data[SDO_HEADER_SIZE], &sdo->length);
        } else {
          sdo->length = LENGTH_UNKNOWN;
        }
        sdo->state = kCanopenSdoStateReadContinueTransmit;
        sdo->length_processed = 0;
        sdo->toggle = false;
      }
    } else {
      CanopenCheckAbort(data, sdo);
    }
  }
}

static void CanopenReadContinueTransmit(const CanopenSdoConfig *config,
                                        CanopenSdo *sdo) {
  uint8_t can_buf[SDO_MESSAGE_SIZE] = {0};
  CanopenSdoCommand cmd = {
    .cs = kCanopenSdoCsUpload, .size_empty = 0, .expedited = false,
    .size_specified = false, .toggle = sdo->toggle};
  CanopenSdoCommandPack(&cmd, can_buf);
  sdo->state = kCanopenSdoStateWriteContinueReceive;

  if (DcanTransmit(config->bus, config->transmit_mailbox, SDO_MESSAGE_SIZE,
                   can_buf)) {
    sdo->state = kCanopenSdoStateReadReceive;
  } else {
    sdo->state = kCanopenSdoStateError;
  }
}

static void CanopenReadContinueReceive(const CanopenSdoConfig *config,
                                       CanopenSdo *sdo) {
  assert((sdo->length_processed < sdo->length)
         || (sdo->length == LENGTH_UNKNOWN));

  uint8_t data[SDO_MESSAGE_SIZE];
  CanopenSdoCommand cmd;
  if (DcanGetMailbox(config->bus, config->receive_mailbox, SDO_MESSAGE_SIZE,
                     data, NULL)) {
    CanopenSdoCommandUnpack(data, &cmd);
    if (cmd.toggle == sdo->toggle && cmd.cs == kCanopenSdoCsUploadReply) {
      uint8_t read_size;
      if (!cmd.size_specified) {
        // Transfer is not finished yet.
        read_size = 7;
        sdo->toggle = !sdo->toggle;
        sdo->state = kCanopenSdoStateReadContinueTransmit;
      } else {
        // This is the last block.
        assert(cmd.size_empty < 7);
        read_size = 7 - cmd.size_empty;
        sdo->state = kCanopenSdoStateComplete;
      }
      assert(sdo->length_processed + read_size <= (int32_t)sizeof(sdo->data));
      memcpy(&sdo->data[sdo->length_processed], &data[1], read_size);
      sdo->length_processed += read_size;
    } else {
      CanopenCheckAbort(data, sdo);
    }
  }
}

static void CanopenComplete(CanopenSdo *sdo) {
  sdo->state = kCanopenSdoStateIdle;
}

static bool CanopenSdoWriteSetup(uint16_t index, uint8_t subindex,
                                 int32_t length, const uint8_t *data,
                                 CanopenSdo *sdo) {
  assert(sdo != NULL && data != NULL);
  assert(0 <= length);
  assert(length < (int32_t)sizeof(sdo->data));
  assert(index != 0);

  if (sdo->state != kCanopenSdoStateIdle) {
    return false;
  }
  sdo->last_operation_error = false;
  sdo->abort_code = kCanopenSdoAbortNone;
  sdo->state = kCanopenSdoStateWriteTransmit;
  sdo->index = index;
  sdo->subindex = subindex;
  sdo->length = length;
  return true;
}

void CanopenSdoInit(CanopenSdo *sdo) {
  assert(sdo != NULL);

  memset(sdo, 0, sizeof(CanopenSdo));
  sdo->state = kCanopenSdoStateInit;
}

bool CanopenSdoPoll(const CanopenSdoConfig *config, CanopenSdo *sdo) {
  assert(config != NULL && sdo != NULL);

  switch (sdo->state) {
    case kCanopenSdoStateInit:
      CanopenInit(config, sdo);
      break;
    case kCanopenSdoStateWriteTransmit:
      CanopenWriteTransmit(config, sdo);
      break;
    case kCanopenSdoStateWriteReceive:
      CanopenWriteReceive(config, sdo);
      break;
    case kCanopenSdoStateWriteContinueTransmit:
      CanopenWriteContinueTransmit(config, sdo);
      break;
    case kCanopenSdoStateWriteContinueReceive:
      CanopenWriteContinueReceive(config, sdo);
      break;
    case kCanopenSdoStateReadTransmit:
      CanopenReadTransmit(config, sdo);
      break;
    case kCanopenSdoStateReadReceive:
      CanopenReadReceive(config, sdo);
      break;
    case kCanopenSdoStateReadContinueTransmit:
      CanopenReadContinueTransmit(config, sdo);
      break;
    case kCanopenSdoStateReadContinueReceive:
      CanopenReadContinueReceive(config, sdo);
      break;
    case kCanopenSdoStateComplete:
      CanopenComplete(sdo);
      break;
    case kCanopenSdoStateIdle:
      break;
    case kCanopenSdoStateError:
      // Fall-through intentional.
    default:
      sdo->last_operation_error = true;
      sdo->state = kCanopenSdoStateInit;
      break;
  }

  return sdo->state == kCanopenSdoStateIdle;
}

bool CanopenSdoWriteLe(uint16_t index, uint8_t subindex, int32_t length,
                       const uint8_t *data, CanopenSdo *sdo) {
  if (!CanopenSdoWriteSetup(index, subindex, length, data, sdo)) {
    return false;
  }
  memcpy(sdo->data, data, length);
  return true;
}

bool CanopenSdoWriteBe(uint16_t index, uint8_t subindex, int32_t length,
                       const uint8_t *data, CanopenSdo *sdo) {
  if (!CanopenSdoWriteSetup(index, subindex, length, data, sdo)) {
    return false;
  }
  for (int i = 0; i < length; i++) {
    sdo->data[length - i - 1] = data[i];
  }
  return true;
}

bool CanopenSdoRead(uint16_t index, uint8_t subindex, int32_t length,
                    CanopenSdo *sdo) {
  assert(sdo != NULL);
  assert(0 <= length);
  assert(length < (int32_t)sizeof(sdo->data));
  assert(index != 0);

  if (sdo->state != kCanopenSdoStateIdle || !index) {
    return false;
  }

  sdo->last_operation_error = false;
  sdo->state = kCanopenSdoStateReadTransmit;
  sdo->index = index;
  sdo->subindex = subindex;
  sdo->length = length;
  return true;
}

bool CanopenSdoIsIdle(const CanopenSdo *sdo) {
  assert(sdo != NULL);

  return sdo->state == kCanopenSdoStateIdle;
}

bool CanopenSdoHadError(const CanopenSdo *sdo) {
  assert(sdo != NULL);

  return sdo->last_operation_error;
}

CanopenSdoAbort CanopenSdoGetAbort(const CanopenSdo *sdo) {
  return sdo->abort_code;
}

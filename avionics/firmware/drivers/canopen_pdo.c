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

#include "avionics/firmware/drivers/canopen_pdo.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "avionics/common/endian.h"
#include "avionics/firmware/cpu/dcan.h"
#include "avionics/firmware/drivers/canopen_sdo.h"
#include "common/macros.h"

#define PDO_MAX_SIZE 8

// CANopen PDO description at http://www.can-cia.org/index.php?id=153
// Referenced description from:
// http://www.a-m-c.com/download/sw/dw300_3-0-3/CAN_Manual300_3-0-3.pdf
// Referenced addresses and implementation details from:
// http://www.copleycontrols.com/Motion/pdf/CANopenProgrammersManual.pdf

static uint16_t GetConfigIndex(const CanopenPdoConfig *config, bool mapping) {
  return (config->type == kCanopenPdoTypeTransmit ? 0x1800 : 0x1400)
      + (mapping ? 0x200 : 0) + config->pdo_index;
}

static bool SdoWriteUint32(uint16_t index, uint8_t sub, uint32_t value,
                           CanopenSdo *sdo) {
  uint8_t buf[4];
  WriteUint32Le(value, buf);
  return CanopenSdoWriteLe(index, sub, sizeof(buf), buf, sdo);
}

// PDO COB_ID is stored at subindex 1 of a PDO's configuration index.
static bool CanopenPdoSetId(const CanopenPdoConfig *config,
                            CanopenSdo *sdo) {
  uint32_t value = config->cob_id & DCAN_STANDARD_MASK;
  return SdoWriteUint32(GetConfigIndex(config, false), 1, value, sdo);
}

// PDO type is stored at subindex 2 of a PDO's configuration index.
static bool CanopenPdoSetType(const CanopenPdoConfig *config,
                              CanopenSdo *sdo) {
  uint8_t value = config->sync_period + (uint8_t)config->transmission;
  if (config->transmission == kCanopenPdoTransmissionSynchronousCyclic) {
    assert(config->sync_period > 0 && config->sync_period < 0xF0);
    value = config->sync_period;
  }
  return CanopenSdoWriteLe(GetConfigIndex(config, false), 2, 1, &value, sdo);
}

static bool CanopenPdoMapEntry(const CanopenPdoConfig *config,
                               uint8_t mapping_index, CanopenSdo *sdo) {
  assert(mapping_index < config->num_mappings);

  const CanopenPdoMapping *mapping = &config->mappings[mapping_index];
  // For simplicity we currently only support mapping full bytes.
  assert(mapping->bit_length > 0 && (mapping->bit_length % 8) == 0);
  uint32_t value = (mapping->can_index << 16) | (mapping->can_sub << 8)
      | mapping->bit_length;
  return SdoWriteUint32(GetConfigIndex(config, true), mapping_index + 1, value,
                        sdo);
}

// The number of objects mapped to a PDO is subindex 0 of the PDO mapping index.
static bool CanopenPdoMapLength(const CanopenPdoConfig *config, uint8_t length,
                                CanopenSdo *sdo) {
  uint8_t buf[1] = {length};
  return CanopenSdoWriteLe(GetConfigIndex(config, true), 0, 1, buf, sdo);
}

// PDO type is defined with respect to the remote node.  Therefore a transmit
// PDO is a message from the remote and assigned to a receive mailbox.  A
// receive PDO represents a message expected to be received by the remote and is
// assigned to a transmit mailbox.
static void CanopenInit(const CanopenPdoConfig *config) {
  if (config->type == kCanopenPdoTypeReceive) {
    DcanSetTransmitMailbox(config->bus, config->mailbox, kDcanIdStandard,
                           DCAN_STANDARD_MASK, config->cob_id, PDO_MAX_SIZE);
  } else {
    DcanSetReceiveMailbox(config->bus, config->mailbox, kDcanIdStandard,
                          DCAN_STANDARD_MASK, config->cob_id, PDO_MAX_SIZE);
  }
}

bool CanopenPdoPoll(const CanopenPdoConfig *config, CanopenPdo *pdo,
                    CanopenSdo *sdo) {
  assert(config != NULL && pdo != NULL && sdo != NULL);

  switch (pdo->state) {
    case kCanopenPdoStateInit:
      CanopenInit(config);
      pdo->state = kCanopenPdoStateZeroMapping;
      pdo->index = 0;
      break;
    case kCanopenPdoStateZeroMapping:
      if (CanopenPdoMapLength(config, 0, sdo)) {
        pdo->state = kCanopenPdoStateSetType;
      }
      break;
    case kCanopenPdoStateSetType:
      if (CanopenPdoSetType(config, sdo)) {
        pdo->state = kCanopenPdoStateSetId;
      }
      break;
    case kCanopenPdoStateSetId:
      if (CanopenPdoSetId(config, sdo)) {
        pdo->state = kCanopenPdoStateMapping;
      }
      break;
    case kCanopenPdoStateMapping:
      if (CanopenPdoMapEntry(config, pdo->index, sdo)) {
        if (++pdo->index >= config->num_mappings) {
          pdo->state = kCanopenPdoStateSetLength;
        }
      }
      break;
    case kCanopenPdoStateSetLength:
      if (CanopenPdoMapLength(config, config->num_mappings, sdo)) {
        pdo->state = kCanopenPdoStateIdleWait;
      }
      break;
    case kCanopenPdoStateIdleWait:
      if (CanopenSdoIsIdle(sdo)) {
        pdo->state = kCanopenPdoStateConfigured;
      }
      break;
    case kCanopenPdoStateConfigured:
      break;
    default:
      pdo->state = kCanopenPdoStateInit;
      assert(false);
  }
  return pdo->state == kCanopenPdoStateConfigured;
}

void CanopenPdoInit(CanopenPdo *pdo) {
  assert(pdo != NULL);

  memset(pdo, 0, sizeof(*pdo));
  pdo->state = kCanopenPdoStateInit;
}

bool CanopenPdoIsConfigured(const CanopenPdo *pdo) {
  return pdo->state == kCanopenPdoStateConfigured;
}

bool CanopenPdoTransmit(const CanopenPdoConfig *config,
                        const uint32_t *values) {
  assert(config != NULL && values != NULL);
  assert(config->type == kCanopenPdoTypeReceive);
  assert(config->mappings != NULL);

  uint8_t can_buf[PDO_MAX_SIZE];
  int32_t buf_pos = 0;
  // For simplicity we currently only support mapping full bytes.
  for (uint8_t i = 0; i < config->num_mappings; i++) {
    const CanopenPdoMapping *mapping = &config->mappings[i];
    assert(buf_pos + (mapping->bit_length / 8) <= PDO_MAX_SIZE);
    switch (mapping->bit_length) {
      case 8:
        buf_pos += WriteUint8Le(values[i], &can_buf[buf_pos]);
        break;
      case 16:
        buf_pos += WriteUint16Le(values[i], &can_buf[buf_pos]);
        break;
      case 32:
        buf_pos += WriteUint32Le(values[i], &can_buf[buf_pos]);
        break;
      default:
        assert(false);
        return false;
    }
  }
  return DcanTransmit(config->bus, config->mailbox, buf_pos, can_buf);
}

bool CanopenPdoReceive(const CanopenPdoConfig *config,
                       uint32_t *values) {
  assert(config != NULL && values != NULL);
  assert(config->type == kCanopenPdoTypeTransmit);
  assert(config->mappings != NULL);

  uint8_t can_buf[PDO_MAX_SIZE];
  if (DcanGetMailbox(config->bus, config->mailbox, PDO_MAX_SIZE, can_buf,
                     NULL)) {
    int buf_pos = 0;
    uint8_t v8;
    uint16_t v16;
    // For simplicity we currently only support mapping full bytes.
    for (int i = 0; i < config->num_mappings; i++) {
      const CanopenPdoMapping *mapping = &config->mappings[i];
      assert(buf_pos + (mapping->bit_length / 8) <= PDO_MAX_SIZE);
      switch (mapping->bit_length) {
        case 8:
          buf_pos += ReadUint8Le(&can_buf[buf_pos], &v8);
          values[i] = v8;
          break;
        case 16:
          buf_pos += ReadUint16Le(&can_buf[buf_pos], &v16);
          values[i] = v16;
          break;
        case 32:
          buf_pos += ReadUint32Le(&can_buf[buf_pos], &values[i]);
          break;
        default:
          assert(false);
          return false;
      }
    }
    return true;
  }
  return false;
}

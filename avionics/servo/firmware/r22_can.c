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

#include "avionics/servo/firmware/r22_can.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "avionics/common/endian.h"
#include "avionics/firmware/drivers/canopen_nmt.h"
#include "avionics/firmware/drivers/canopen_pdo.h"
#include "avionics/firmware/drivers/canopen_sdo.h"
#include "avionics/servo/firmware/r22_param.h"
#include "common/macros.h"

#define R22_BUS kDcanBus2
// The R22 CANopen node ID must also be changed in r22_param.py.
#define R22_NODE_ID 4

// Note that transmit PDOs are actually connect to receive mailboxes,
// and vice versa.
typedef enum {
  // Transmit mailboxes.
  kServoMailboxNmtTx     = 0,
  kServoMailboxNmtSyncTx = 1,
  kServoMailboxRxPdo1    = 2,
  kServoMailboxRxPdo2    = 3,
  kServoMailboxRxPdo3    = 4,
  kServoMailboxSdoTx     = 5,
  // Receive mailboxes.
  kServoMailboxNmtRx     = 32,
  kServoMailboxTxPdo1    = 33,
  kServoMailboxTxPdo2    = 34,
  kServoMailboxTxPdo3    = 35,
  kServoMailboxSdoRx     = 36,
} ServoMailbox;

static const DcanBus kR22Bus = R22_BUS;

static const CanopenNmtConfig kServoNmtConfig = {
  .bus = R22_BUS,
  .transmit_mailbox = kServoMailboxNmtTx,
  .transmit_sync_mailbox = kServoMailboxNmtSyncTx,
  .receive_mailbox = kServoMailboxNmtRx
};

static const CanopenSdoConfig kServoSdoConfig = {
  .bus = R22_BUS,
  .transmit_mailbox = kServoMailboxSdoTx,
  .receive_mailbox = kServoMailboxSdoRx,
  .node_id_remote = R22_NODE_ID
};

static const CanopenPdoMapping kServoTxPdo1Mappings[] = {
  {kR22CanIndexEventStatus, 0, 32},
  {kR22CanIndexStatusWord, 0, 16},
  {kR22CanIndexModeOfOperationDisplay, 0, 8},
};

static const CanopenPdoConfig kServoTxPdo1Config = {
  .bus = R22_BUS,
  .mailbox = kServoMailboxTxPdo1,
  .cob_id = 0x184,
  .type = kCanopenPdoTypeTransmit,
  .pdo_index = 0,
  .transmission = kCanopenPdoTransmissionSynchronousCyclic,
  .sync_period = 1,
  .mappings = kServoTxPdo1Mappings,
  .num_mappings = ARRAYSIZE(kServoTxPdo1Mappings)
};

static const CanopenPdoMapping kServoTxPdo2Mappings[] = {
  {kR22CanIndexActualPosition, 0, 32},
  {kR22CanIndexActualVelocity, 0, 32},
};

static const CanopenPdoConfig kServoTxPdo2Config = {
  .bus = R22_BUS,
  .mailbox = kServoMailboxTxPdo2,
  .cob_id = 0x284,
  .type = kCanopenPdoTypeTransmit,
  .pdo_index = 1,
  .transmission = kCanopenPdoTransmissionSynchronousCyclic,
  .sync_period = 1,
  .mappings = kServoTxPdo2Mappings,
  .num_mappings = ARRAYSIZE(kServoTxPdo2Mappings)
};

static const CanopenPdoMapping kServoTxPdo3Mappings[] = {
  {kR22CanIndexActualCurrentQ, 0, 16},
  {kR22CanIndexActualTorque, 0, 16},
  {kR22CanIndexDriveTemperature, 0, 16},
};

static const CanopenPdoConfig kServoTxPdo3Config = {
  .bus = R22_BUS,
  .mailbox = kServoMailboxTxPdo3,
  .cob_id = 0x384,
  .type = kCanopenPdoTypeTransmit,
  .pdo_index = 2,
  .transmission = kCanopenPdoTransmissionSynchronousCyclic,
  .sync_period = 1,
  .mappings = kServoTxPdo3Mappings,
  .num_mappings = ARRAYSIZE(kServoTxPdo3Mappings)
};

static const CanopenPdoMapping kServoRxPdo1Mappings[] = {
  {kR22CanIndexControlWord, 0, 16},
  {kR22CanIndexModeOfOperation, 0, 8},
};

// This PDO is async to provide the reset of the update bit in the control word.
// It is applied immediately and the updated position (with raised update bit)
// is applied on the next sync message.
static const CanopenPdoConfig kServoRxPdo1Config = {
  .bus = R22_BUS,
  .mailbox = kServoMailboxRxPdo1,
  .cob_id = 0x104,
  .type = kCanopenPdoTypeReceive,
  .pdo_index = 0,
  .transmission = kCanopenPdoTransmissionAsynchronous,
  .sync_period = 1,
  .mappings = kServoRxPdo1Mappings,
  .num_mappings = ARRAYSIZE(kServoRxPdo1Mappings)
};

static const CanopenPdoMapping kServoRxPdo2Mappings[] = {
  {kR22CanIndexControlWord, 0, 16},
  {kR22CanIndexTargetPosition, 0, 32},
  {kR22CanIndexUserPeakCurrentLimit, 0, 16}
};

static const CanopenPdoConfig kServoRxPdo2Config = {
  .bus = R22_BUS,
  .mailbox = kServoMailboxRxPdo2,
  .cob_id = 0x204,
  .type = kCanopenPdoTypeReceive,
  .pdo_index = 1,
  .transmission = kCanopenPdoTransmissionSynchronousCyclic,
  .sync_period = 1,
  .mappings = kServoRxPdo2Mappings,
  .num_mappings = ARRAYSIZE(kServoRxPdo2Mappings)
};

static const CanopenPdoMapping kServoRxPdo3Mappings[] = {
  {kR22CanIndexTargetVelocity, 0, 32},
  {kR22CanIndexTargetTorque, 0, 16},
};

static const CanopenPdoConfig kServoRxPdo3Config = {
  .bus = R22_BUS,
  .mailbox = kServoMailboxRxPdo3,
  .cob_id = 0x304,
  .type = kCanopenPdoTypeReceive,
  .pdo_index = 2,
  .transmission = kCanopenPdoTransmissionSynchronousCyclic,
  .sync_period = 1,
  .mappings = kServoRxPdo3Mappings,
  .num_mappings = ARRAYSIZE(kServoRxPdo3Mappings)
};

static const CanopenPdoConfig *kServoPdoConfig[] = {
  &kServoTxPdo1Config, &kServoTxPdo2Config, &kServoTxPdo3Config,
  &kServoRxPdo1Config, &kServoRxPdo2Config, &kServoRxPdo3Config,
};

static CanopenPdo g_r22_pdo[ARRAYSIZE(kServoPdoConfig)];

static CanopenSdo g_r22_sdo;

void R22CanInit() {
  DcanInit(kR22Bus, kDcanBitRate1000kbps);
  CanopenNmtInit(&kServoNmtConfig);
  CanopenSdoInit(&g_r22_sdo);
  for (int32_t i = 0; i < ARRAYSIZE(g_r22_pdo); i++) {
    CanopenPdoInit(&g_r22_pdo[i]);
  }
}

void R22CanPdoInitPoll() {
  for (int32_t i = 0; i < ARRAYSIZE(g_r22_pdo); i++) {
    CanopenPdoPoll(kServoPdoConfig[i], &g_r22_pdo[i], &g_r22_sdo);
  }
}

bool R22CanPdoConfigured() {
  bool configured = true;
  for (int32_t i = 0; i < ARRAYSIZE(g_r22_pdo); i++) {
    configured = configured && CanopenPdoIsConfigured(&g_r22_pdo[i]);
  }
  return configured;
}

void R22CanPoll() {
  CanopenSdoPoll(&kServoSdoConfig, &g_r22_sdo);
}

bool R22CanUpdateRxParams1(ServoReceiveParams *params) {
  uint32_t values[3];
  if (CanopenPdoReceive(&kServoTxPdo1Config, values)) {
    params->event_status = values[0];
    params->status_word = values[1];
    params->mode_of_operation = values[2];
    return true;
  }
  return false;
}

bool R22CanUpdateRxParams2(ServoReceiveParams *params) {
  uint32_t values[2];
  if (CanopenPdoReceive(&kServoTxPdo2Config, values)) {
    params->actual_position = values[0];
    params->actual_velocity = values[1];
    return true;
  }
  return false;
}

bool R22CanUpdateRxParams3(ServoReceiveParams *params) {
  uint32_t values[3];
  if (CanopenPdoReceive(&kServoTxPdo3Config, values)) {
    params->actual_current = values[0];
    params->actual_torque = values[1];
    params->temperature = values[2];
    return true;
  }
  return false;
}

bool R22CanUpdateTxParams1(const ServoTransmitParams *params) {
  uint32_t values[2];
  values[0] = params->control_word;
  values[1] = params->mode_of_operation;
  return CanopenPdoTransmit(&kServoRxPdo1Config, values);
}

bool R22CanUpdateTxParams2(const ServoTransmitParams *params) {
  uint32_t values[3];
  values[0] = params->control_word;
  values[1] = params->target_position;
  values[2] = params->current_limit;
  return CanopenPdoTransmit(&kServoRxPdo2Config, values);
}

bool R22CanUpdateTxParams3(const ServoTransmitParams *params) {
  uint32_t values[2];
  values[0] = params->target_velocity;
  values[1] = params->target_torque;
  return CanopenPdoTransmit(&kServoRxPdo3Config, values);
}

bool R22CanGetValue(R22Parameter param) {
  // This only reads from RAM, or flash if reg is flash only.
  const R22ParamInfo *info = R22ParamGetInfo(param);
  if (info == NULL) {
    assert(false);
    return false;
  }
  return CanopenSdoRead(info->can_index, info->can_sub, info->addr_length,
                        &g_r22_sdo);
}

bool R22CanIsIdle() {
  return CanopenSdoIsIdle(&g_r22_sdo);
}

bool R22CanHadError() {
  return CanopenSdoHadError(&g_r22_sdo);
}

bool R22CanReadUint8(int32_t data_length, uint8_t *data) {
  assert(data_length > 0);
  assert(data_length <= ARRAYSIZE(g_r22_sdo.data));
  assert(data != NULL);

  bool success = !R22CanHadError();
  if (success) {
    const uint8_t *reply = g_r22_sdo.data;
    for (int32_t i = 0; i < data_length; ++i) {
      reply += ReadUint8Le(reply, &data[i]);
    }
  }
  return success;
}

bool R22CanReadInt8(int32_t data_length, int8_t *data) {
  return R22CanReadUint8(data_length, (uint8_t *)data);
}

bool R22CanReadUint16(int32_t data_length, uint16_t *data) {
  assert(data_length > 0);
  assert(data_length <= ARRAYSIZE(g_r22_sdo.data));
  assert(data != NULL);

  bool success = !R22CanHadError();
  if (success) {
    const uint8_t *reply = g_r22_sdo.data;
    for (int32_t i = 0; i < data_length; ++i) {
      reply += ReadUint16Le(reply, &data[i]);
    }
  }
  return success;
}

bool R22CanReadInt16(int32_t data_length, int16_t *data) {
  return R22CanReadUint16(data_length, (uint16_t *)data);
}

bool R22CanReadUint32(int32_t data_length, uint32_t *data) {
  assert(data_length > 0);
  assert(data_length <= ARRAYSIZE(g_r22_sdo.data));
  assert(data != NULL);

  bool success = !R22CanHadError();
  if (success) {
    const uint8_t *reply = g_r22_sdo.data;
    for (int32_t i = 0; i < data_length; ++i) {
      reply += ReadUint32Le(reply, &data[i]);
    }
  }
  return success;
}

bool R22CanReadInt32(int32_t data_length, int32_t *data) {
  return R22CanReadUint32(data_length, (uint32_t *)data);
}

bool R22CanCompare(int32_t data_length, const void *data) {
  assert(data_length > 0);
  assert(data_length <= ARRAYSIZE(g_r22_sdo.data));
  assert(data != NULL);

  uint8_t sdo_data_reverse[ARRAYSIZE(g_r22_sdo.data)];
  for (int32_t i = 0; i < data_length; ++i) {
    sdo_data_reverse[i] = g_r22_sdo.data[data_length - i - 1];
  }

  return !R22CanHadError() && !memcmp(sdo_data_reverse, data, data_length);
}

void R22CanSetUint8Be(R22Parameter param, int32_t in_length,
                      const uint8_t *in) {
  assert(in_length > 0);
  assert(in != NULL);

  const R22ParamInfo *info = R22ParamGetInfo(param);
  if (info == NULL) {
    assert(false);
    return;
  }
  CanopenSdoWriteBe(info->can_index, info->can_sub, in_length, in, &g_r22_sdo);
}

void R22CanSetUint8Le(R22Parameter param, int32_t in_length,
                      const uint8_t *in) {
  assert(in_length > 0);
  assert(in != NULL);

  const R22ParamInfo *info = R22ParamGetInfo(param);
  if (info == NULL) {
    assert(false);
    return;
  }
  CanopenSdoWriteLe(info->can_index, info->can_sub, in_length, in, &g_r22_sdo);
}

void R22CanSetUint8(R22Parameter param, uint8_t value) {
  uint8_t data[1];
  const R22ParamInfo *info = R22ParamGetInfo(param);
  if (info == NULL) {
    assert(false);
    return;
  }
  WriteUint8Le(value, data);
  CanopenSdoWriteLe(info->can_index, info->can_sub, sizeof(data), data,
                    &g_r22_sdo);
}

void R22CanSetInt8(R22Parameter param, int8_t value) {
  R22CanSetUint8(param, (uint8_t)value);
}

void R22CanSetUint16(R22Parameter param, uint16_t value) {
  uint8_t data[2];
  const R22ParamInfo *info = R22ParamGetInfo(param);
  if (info == NULL) {
    assert(false);
    return;
  }
  WriteUint16Le(value, data);
  CanopenSdoWriteLe(info->can_index, info->can_sub, sizeof(data), data,
                    &g_r22_sdo);
}

void R22CanSetInt16(R22Parameter param, int16_t value) {
  R22CanSetUint16(param, (uint16_t)value);
}

void R22CanSetUint32(R22Parameter param, uint32_t value) {
  uint8_t data[4];
  const R22ParamInfo *info = R22ParamGetInfo(param);
  if (info == NULL) {
    assert(false);
    return;
  }
  WriteUint32Le(value, data);
  CanopenSdoWriteLe(info->can_index, info->can_sub, sizeof(data), data,
                    &g_r22_sdo);
}

void R22CanSetInt32(R22Parameter param, int32_t value) {
  R22CanSetUint32(param, (uint32_t)value);
}

void R22CanCommandReset(void) {
  CanopenNmtSend(&kServoNmtConfig, R22_NODE_ID, kCanopenNmtMessageReset);
}

void R22CanCommandPreOperational(void) {
  CanopenNmtSend(&kServoNmtConfig, R22_NODE_ID,
                 kCanopenNmtMessagePreOperational);
}

void R22CanCommandOperational(void) {
  CanopenNmtSend(&kServoNmtConfig, R22_NODE_ID, kCanopenNmtMessageStart);
}

void R22CanClearErrorLog(void) {
  R22CanSetUint8(kR22ParamErrorHistory, 0);
}

void R22CanClearErrors(void) {
  R22CanSetUint32(kR22ParamLatchedFaultStatus, 0x1FFFF);
}

void R22CanSendSync(void) {
  CanopenNmtSendSync(&kServoNmtConfig);
}

int64_t R22CanReadTimestamp(void) {
  return g_r22_sdo.read_timestamp;
}

CanopenSdoAbort R22CanGetAbort(void) {
  return g_r22_sdo.abort_code;
}

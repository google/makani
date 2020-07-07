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

#ifndef AVIONICS_FIRMWARE_DRIVERS_CANOPEN_PDO_H_
#define AVIONICS_FIRMWARE_DRIVERS_CANOPEN_PDO_H_

#include <stdbool.h>
#include <stdint.h>

#include "avionics/firmware/cpu/dcan.h"
#include "avionics/firmware/drivers/canopen_sdo.h"

// This field represents the PDO type from the perspective of the remote node,
// in order to be consistent with the CANopen documentation.  This behavior is
// slightly unintuitive: one should actually receive from a transmit PDO, and
// transmit to a recieve PDO.
typedef enum {
  kCanopenPdoTypeTransmit,
  kCanopenPdoTypeReceive
} CanopenPdoType;

typedef enum {
  kCanopenPdoTransmissionSynchronousCyclic = 0x01,
  kCanopenPdoTransmissionSynchronousRemote = 0xFC,
  kCanopenPdoTransmissionAsynchronousRemote = 0xFD,
  kCanopenPdoTransmissionAsynchronous = 0xFE
} CanopenPdoTransmission;

typedef enum {
  kCanopenPdoStateInit,
  kCanopenPdoStateDisable,
  kCanopenPdoStateZeroMapping,
  kCanopenPdoStateMapping,
  kCanopenPdoStateSetLength,
  kCanopenPdoStateSetType,
  kCanopenPdoStateSetId,
  kCanopenPdoStateIdleWait,
  kCanopenPdoStateConfigured
} CanopenPdoState;

typedef struct {
  const uint16_t can_index;
  const uint8_t can_sub;
  const uint8_t bit_length;
} CanopenPdoMapping;

typedef struct {
  const DcanBus bus;
  const int32_t mailbox;
  const int32_t cob_id;
  const CanopenPdoType type;
  const uint16_t pdo_index;
  const CanopenPdoTransmission transmission;
  const uint8_t sync_period;  // This is unsupported in the R22.
  const CanopenPdoMapping *mappings;
  const uint8_t num_mappings;
} CanopenPdoConfig;

typedef struct {
  CanopenPdoState state;
  uint8_t index;
} CanopenPdo;

void CanopenPdoInit(CanopenPdo *config);

bool CanopenPdoIsConfigured(const CanopenPdo *pdo);

bool CanopenPdoTransmit(const CanopenPdoConfig *config, const uint32_t *values);

bool CanopenPdoReceive(const CanopenPdoConfig *config, uint32_t *values);

bool CanopenPdoPoll(const CanopenPdoConfig *config, CanopenPdo *pdo,
                    CanopenSdo *sdo);

#endif  // AVIONICS_FIRMWARE_DRIVERS_CANOPEN_PDO_H_

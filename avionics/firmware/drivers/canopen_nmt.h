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

#ifndef AVIONICS_FIRMWARE_DRIVERS_CANOPEN_NMT_H_
#define AVIONICS_FIRMWARE_DRIVERS_CANOPEN_NMT_H_

#include <stdbool.h>
#include <stdint.h>

#include "avionics/firmware/cpu/dcan.h"

typedef struct {
  const DcanBus bus;
  const int32_t transmit_mailbox;
  const int32_t transmit_sync_mailbox;
  const int32_t receive_mailbox;
} CanopenNmtConfig;

typedef enum {
  kCanopenNmtMessageStart = 0x01,
  kCanopenNmtMessageStop = 0x02,
  kCanopenNmtMessagePreOperational = 0x80,
  kCanopenNmtMessageReset = 0x81,
  kCanopenNmtMessageResetComm = 0x82
} CanopenNmtMessage;

typedef enum {
  kCanopenNmtStatePreOperational = 0x7F,
  kCanopenNmtStateOperational = 0x05,
  kCanopenNmtStateStopped = 0x04
} CanopenNmtState;

void CanopenNmtInit(const CanopenNmtConfig *config);

bool CanopenNmtSend(const CanopenNmtConfig *config, uint8_t node_id,
                    CanopenNmtMessage message);

bool CanopenNmtGetState(const CanopenNmtConfig *config, uint8_t *node_id,
                        CanopenNmtState *state);

bool CanopenNmtSendSync(const CanopenNmtConfig *config);

#endif  // AVIONICS_FIRMWARE_DRIVERS_CANOPEN_NMT_H_

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

#ifndef AVIONICS_COMMON_TETHER_OP_H_
#define AVIONICS_COMMON_TETHER_OP_H_

#include <stdint.h>

#include "avionics/common/avionics_messages.h"
#include "avionics/network/aio_node.h"
#include "common/ring.h"

#define TETHER_OP_QUEUE_SIZE 32

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  kTetherOpTypeForceSigned = -1,
  kTetherOpTypeNone,
  kTetherOpTypeBattCommand,
  kTetherOpTypeFlightCommand,
  kTetherOpTypeFpvSetState,
  kTetherOpTypeMotorAckParam,
  kTetherOpTypeMotorGetParam,
  kTetherOpTypeMotorSetParam,
  kTetherOpTypeMotorSetState,
  kTetherOpTypeMvlvCommand,
  kTetherOpTypePitotSetState,
  kTetherOpTypeServoAckParam,
  kTetherOpTypeServoGetParam,
  kTetherOpTypeServoSetParam,
  kTetherOpTypeServoSetState,
  kTetherOpTypeTetherReleaseSetState,
  kNumTetherOpTypes
} TetherOpType;

typedef union {
  BattCommandMessage batt_command;
  FlightCommandMessage flight_command;
  FpvSetStateMessage fpv_set_state;
  MotorAckParamMessage motor_ack_param;
  MotorGetParamMessage motor_get_param;
  MotorSetParamMessage motor_set_param;
  MotorSetStateMessage motor_set_state;
  MvlvCommandMessage mvlv_command;
  PitotSetStateMessage pitot_set_state;
  ServoAckParamMessage servo_ack_param;
  ServoGetParamMessage servo_get_param;
  ServoSetParamMessage servo_set_param;
  ServoSetStateMessage servo_set_state;
  TetherReleaseSetStateMessage tether_release_set_state;
} TetherOpData;

typedef struct {
  RingBuffer ring;
  AioNode source[TETHER_OP_QUEUE_SIZE];
  TetherOpType op_type[TETHER_OP_QUEUE_SIZE];
  uint16_t sequence[TETHER_OP_QUEUE_SIZE];
  TetherOpData data[TETHER_OP_QUEUE_SIZE];
} TetherOpQueue;

void TetherOpInit(TetherOpQueue *q);
MessageType TetherOpTypeToMessageType(TetherOpType type);
bool TetherOpIsPending(const TetherOpQueue *q);

TetherOpData *TetherOpGetTail(TetherOpQueue *q, AioNode *source,
                              TetherOpType *op_type, uint16_t *sequence);
void TetherOpPopTail(TetherOpQueue *q);

TetherOpData *TetherOpGetNewHead(AioNode source, TetherOpType op_type,
                                 uint16_t sequence, TetherOpQueue *q);
void TetherOpPushNewHead(TetherOpQueue *q);

void TetherOpTransmitQueue(TetherOpQueue *q);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // AVIONICS_COMMON_TETHER_OP_H_

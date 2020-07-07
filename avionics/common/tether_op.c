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

#include "avionics/common/tether_op.h"

#include <assert.h>
#include <stdint.h>
#include <string.h>

#include "avionics/common/avionics_messages.h"
#include "avionics/network/aio_node.h"
#include "common/macros.h"
#include "common/ring.h"

void TetherOpInit(TetherOpQueue *q) {
  memset(q, 0, sizeof(*q));
  RingInit(ARRAYSIZE(q->data), &q->ring);
}

MessageType TetherOpTypeToMessageType(TetherOpType type) {
  switch (type) {
    case kTetherOpTypeBattCommand:
      return kMessageTypeBattCommand;
    case kTetherOpTypeFlightCommand:
      return kMessageTypeFlightCommand;
    case kTetherOpTypeFpvSetState:
      return kMessageTypeFpvSetState;
    case kTetherOpTypeMotorAckParam:
      return kMessageTypeMotorAckParam;
    case kTetherOpTypeMotorGetParam:
      return kMessageTypeMotorGetParam;
    case kTetherOpTypeMotorSetParam:
      return kMessageTypeMotorSetParam;
    case kTetherOpTypeMotorSetState:
      return kMessageTypeMotorSetState;
    case kTetherOpTypeMvlvCommand:
      return kMessageTypeMvlvCommand;
    case kTetherOpTypePitotSetState:
      return kMessageTypePitotSetState;
    case kTetherOpTypeServoAckParam:
      return kMessageTypeServoAckParam;
    case kTetherOpTypeServoGetParam:
      return kMessageTypeServoGetParam;
    case kTetherOpTypeServoSetParam:
      return kMessageTypeServoSetParam;
    case kTetherOpTypeServoSetState:
      return kMessageTypeServoSetState;
    case kTetherOpTypeTetherReleaseSetState:
      return kMessageTypeTetherReleaseSetState;
    case kTetherOpTypeNone:
    case kTetherOpTypeForceSigned:
    case kNumTetherOpTypes:
    default:
      assert(false);
      return kMessageTypeStdio;  // Null type.
  }
}

bool TetherOpIsPending(const TetherOpQueue *q) {
  return !RingIsEmpty(&q->ring);
}

TetherOpData *TetherOpGetTail(TetherOpQueue *q, AioNode *source,
                              TetherOpType *op_type, uint16_t *sequence) {
  int32_t index;
  if (RingGetTailIndex(&q->ring, &index)) {
    *source = q->source[index];
    *op_type = q->op_type[index];
    *sequence = q->sequence[index];
    return &q->data[index];
  }
  return NULL;
}

void TetherOpPopTail(TetherOpQueue *q) {
  RingPopTail(&q->ring);
}

TetherOpData *TetherOpGetNewHead(AioNode source, TetherOpType op_type,
                                 uint16_t sequence, TetherOpQueue *q) {
  int32_t index;
  if (op_type != kTetherOpTypeNone && RingGetNewHeadIndex(&q->ring, &index)) {
    q->source[index] = source;
    q->op_type[index] = op_type;
    q->sequence[index] = sequence;
    return &q->data[index];
  }
  return NULL;
}

void TetherOpPushNewHead(TetherOpQueue *q) {
  RingPushNewHead(&q->ring);
}

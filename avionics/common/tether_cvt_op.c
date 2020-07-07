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

#include "avionics/common/tether_cvt_op.h"

#include <stddef.h>
#include <stdint.h>

#include "avionics/common/avionics_messages.h"
#include "avionics/common/cvt_avionics_messages.h"
#include "avionics/common/tether_op.h"
#include "avionics/network/aio_labels.h"
#include "avionics/network/aio_node.h"
#include "common/macros.h"
#include "common/ring.h"

static void PushNewHead(int32_t index, AioNode source, TetherOpType op_type,
                        TetherOpQueue *q) {
  q->source[index] = source;
  q->op_type[index] = op_type;
  RingPushNewHead(&q->ring);
}

static void QueryBattCommandMessage(AioNode source, TetherOpQueue *q) {
  int32_t index;
  if (RingGetNewHeadIndex(&q->ring, &index)
      && CvtGetBattCommandMessage(source, &q->data[index].batt_command,
                                  &q->sequence[index], NULL)) {
    PushNewHead(index, source, kTetherOpTypeBattCommand, q);
  }
}

static void QueryFlightCommandMessage(AioNode source, TetherOpQueue *q) {
  int32_t index;
  if (RingGetNewHeadIndex(&q->ring, &index)
      && CvtGetFlightCommandMessage(source,
                                    &q->data[index].flight_command,
                                    &q->sequence[index], NULL)) {
    PushNewHead(index, source, kTetherOpTypeFlightCommand, q);
  }
}

static void QueryFpvSetStateMessage(AioNode source, TetherOpQueue *q) {
  int32_t index;
  if (RingGetNewHeadIndex(&q->ring, &index)
      && CvtGetFpvSetStateMessage(source, &q->data[index].fpv_set_state,
                                  &q->sequence[index], NULL)) {
    PushNewHead(index, source, kTetherOpTypeFpvSetState, q);
  }
}

static void QueryMotorAckParamMessage(AioNode source, TetherOpQueue *q) {
  int32_t index;
  if (RingGetNewHeadIndex(&q->ring, &index)
      && CvtGetMotorAckParamMessage(source, &q->data[index].motor_ack_param,
                                    &q->sequence[index], NULL)) {
    PushNewHead(index, source, kTetherOpTypeMotorAckParam, q);
  }
}

static void QueryMotorGetParamMessage(AioNode source, TetherOpQueue *q) {
  int32_t index;
  if (RingGetNewHeadIndex(&q->ring, &index)
      && CvtGetMotorGetParamMessage(source, &q->data[index].motor_get_param,
                                    &q->sequence[index], NULL)) {
    PushNewHead(index, source, kTetherOpTypeMotorGetParam, q);
  }
}

static void QueryMotorSetParamMessage(AioNode source, TetherOpQueue *q) {
  int32_t index;
  if (RingGetNewHeadIndex(&q->ring, &index)
      && CvtGetMotorSetParamMessage(source, &q->data[index].motor_set_param,
                                    &q->sequence[index], NULL)) {
    PushNewHead(index, source, kTetherOpTypeMotorSetParam, q);
  }
}

static void QueryMotorSetStateMessage(AioNode source, TetherOpQueue *q) {
  int32_t index;
  if (RingGetNewHeadIndex(&q->ring, &index)
      && CvtGetMotorSetStateMessage(source, &q->data[index].motor_set_state,
                                    &q->sequence[index], NULL)) {
    PushNewHead(index, source, kTetherOpTypeMotorSetState, q);
  }
}

static void QueryMvlvCommandMessage(AioNode source, TetherOpQueue *q) {
  int32_t index;
  if (RingGetNewHeadIndex(&q->ring, &index)
      && CvtGetMvlvCommandMessage(source, &q->data[index].mvlv_command,
                                  &q->sequence[index], NULL)) {
    PushNewHead(index, source, kTetherOpTypeMvlvCommand, q);
  }
}

static void QueryPitotSetStateMessage(AioNode source, TetherOpQueue *q) {
  int32_t index;
  if (RingGetNewHeadIndex(&q->ring, &index)
      && CvtGetPitotSetStateMessage(source, &q->data[index].pitot_set_state,
                                    &q->sequence[index], NULL)) {
    PushNewHead(index, source, kTetherOpTypePitotSetState, q);
  }
}

static void QueryServoAckParamMessage(AioNode source, TetherOpQueue *q) {
  int32_t index;
  if (RingGetNewHeadIndex(&q->ring, &index)
      && CvtGetServoAckParamMessage(source, &q->data[index].servo_ack_param,
                                    &q->sequence[index], NULL)) {
    PushNewHead(index, source, kTetherOpTypeServoAckParam, q);
  }
}

static void QueryServoGetParamMessage(AioNode source, TetherOpQueue *q) {
  int32_t index;
  if (RingGetNewHeadIndex(&q->ring, &index)
      && CvtGetServoGetParamMessage(source, &q->data[index].servo_get_param,
                                    &q->sequence[index], NULL)) {
    PushNewHead(index, source, kTetherOpTypeServoGetParam, q);
  }
}

static void QueryServoSetParamMessage(AioNode source, TetherOpQueue *q) {
  int32_t index;
  if (RingGetNewHeadIndex(&q->ring, &index)
      && CvtGetServoSetParamMessage(source, &q->data[index].servo_set_param,
                                    &q->sequence[index], NULL)) {
    PushNewHead(index, source, kTetherOpTypeServoSetParam, q);
  }
}

static void QueryServoSetStateMessage(AioNode source, TetherOpQueue *q) {
  int32_t index;
  if (RingGetNewHeadIndex(&q->ring, &index)
      && CvtGetServoSetStateMessage(source, &q->data[index].servo_set_state,
                                    &q->sequence[index], NULL)) {
    PushNewHead(index, source, kTetherOpTypeServoSetState, q);
  }
}

static void QueryTetherReleaseSetStateMessage(AioNode source,
                                              TetherOpQueue *q) {
  int32_t index;
  if (RingGetNewHeadIndex(&q->ring, &index)
      && CvtGetTetherReleaseSetStateMessage(
          source, &q->data[index].tether_release_set_state,
          &q->sequence[index], NULL)) {
    PushNewHead(index, source, kTetherOpTypeTetherReleaseSetState, q);
  }
}

void TetherDownCvtOpQueueQuery(TetherOpQueue *q) {
  for (int32_t i = 0; i < kNumMotors; ++i) {
    AioNode source = MotorLabelToMotorAioNode((MotorLabel)i);
    QueryMotorAckParamMessage(source, q);
  }
  for (int32_t i = 0; i < kNumServos; ++i) {
    AioNode source = ServoLabelToServoAioNode((ServoLabel)i);
    QueryServoAckParamMessage(source, q);
  }
}

void TetherUpCvtOpQueueQuery(TetherOpQueue *q) {
  const AioNode source = kAioNodeOperator;

  QueryBattCommandMessage(source, q);
  QueryFlightCommandMessage(source, q);
  QueryFpvSetStateMessage(source, q);
  QueryMotorGetParamMessage(source, q);
  QueryMotorSetParamMessage(source, q);
  QueryMotorSetStateMessage(source, q);
  QueryMvlvCommandMessage(source, q);
  QueryPitotSetStateMessage(source, q);
  QueryServoGetParamMessage(source, q);
  QueryServoSetParamMessage(source, q);
  QueryServoSetStateMessage(source, q);
  QueryTetherReleaseSetStateMessage(source, q);
}

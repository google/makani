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

#include "nav/ins/inertial/inertial.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "common/ring.h"
#include "nav/ins/inertial/params.h"
#include "nav/ins/messages/labels.h"
#include "nav/ins/messages/messages.h"
#include "nav/ins/messages/seq_num.h"
#include "nav/ins/util/ring.h"

static bool GetNewHead(const InsMessageHeader *header,
                       InsInertialBuffer *buf, InsInertialMessage **data_b) {
  int32_t index;
  if (header->label == kInsImuLabelPrimary
      && InsRingGetNewHeadIndex(&buf->ring, buf->header, header, &index)) {
    buf->header[index] = *header;
    *data_b = &buf->data[index];
    return true;
  }
  return false;
}

static void RemoveTail(int64_t min_time, InsInertialBuffer *buf) {
  int32_t index;
  while (RingGetTailIndex(&buf->ring, &index)
         && buf->header[index].timestamp <= min_time) {
    RingPopTail(&buf->ring);
  }
}

static bool IterateToSeqNum(const InsInertialBuffer *buf, uint16_t seq_num,
                            InsInertialIterateFunc func, void *arg,
                            uint32_t *iter) {
  uint32_t start_iter = *iter;
  int32_t index;
  while (RingGetIndex(&buf->ring, *iter, &index)
         && InsSequenceLe(buf->header[index].seq_num, seq_num)) {
    func(&buf->header[index], &buf->data[index], arg);
    ++(*iter);
  }
  return start_iter != *iter;  // True when updated.
}

static bool IterateToHead(const InsInertialBuffer *buf,
                          InsInertialIterateFunc func, void *arg,
                          uint32_t *iter) {
  uint32_t start_iter = *iter;
  int32_t index;
  while (RingGetIndex(&buf->ring, *iter, &index)) {
    func(&buf->header[index], &buf->data[index], arg);
    ++(*iter);
  }
  return start_iter != *iter;  // True when updated.
}

// Public functions.

void InsInertialInit(InsInertial *in) {
  memset(in, 0, sizeof(*in));
  RingInit(INS_INERTIAL_RING_ELEMENTS, &in->buffer.ring);
}

uint32_t InsInertialGetTailIter(const InsInertial *in) {
  return RingGetTail(&in->buffer.ring);
}

bool InsInertialGetNewHead(const InsMessageHeader *header,
                           InsInertial *inertial, InsInertialMessage **data_b) {
  return GetNewHead(header, &inertial->buffer, data_b);
}

void InsInertialPushNewHead(InsInertial *inertial) {
  RingPushNewHead(&inertial->buffer.ring);
}

bool InsInertialInsertMessage(const InsMessageHeader *header,
                              const InsInertialMessage *data_s,
                              const InsInertialParams *params,
                              InsInertial *inertial) {
  InsInertialMessage *data_b;
  if (InsInertialGetNewHead(header, inertial, &data_b)) {
    InsInertialRotateSensorToBody(params, data_s, data_b);
    InsInertialPushNewHead(inertial);
    return true;
  }
  return false;
}

void InsInertialRemoveTail(int64_t min_time, InsInertial *in) {
  RemoveTail(min_time, &in->buffer);
}

void InsInertialRotateSensorToBody(const InsInertialParams *param,
                                   const InsInertialMessage *sensor,
                                   InsInertialMessage *body) {
  // TODO: Rotate to body frame.
  (void)param;
  *body = *sensor;
}

bool InsInertialIterateToSeqNum(const InsInertial *in, uint16_t seq_num,
                                InsInertialIterateFunc func, void *arg,
                                uint32_t *iter) {
  return IterateToSeqNum(&in->buffer, seq_num, func, arg, iter);
}

bool InsInertialIterateToHead(const InsInertial *in,
                              InsInertialIterateFunc func, void *arg,
                              uint32_t *iter) {
  return IterateToHead(&in->buffer, func, arg, iter);
}

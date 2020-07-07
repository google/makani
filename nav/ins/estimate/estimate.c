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

#include "nav/ins/estimate/estimate.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "nav/ins/inertial/inertial.h"
#include "nav/ins/messages/messages.h"

static void PropagateBufferForward(const InsMessageHeader *header,
                                   const InsInertialMessage *data,
                                   void *arg) {
  InsEstimateBuffer *buf = (InsEstimateBuffer *)arg;
  const InsEstimate *xhat_1 = InsEstimateGetCurrent(buf);
  InsEstimate *xhat = InsEstimateGetNext(buf);

  assert(buf->inertial_iter == xhat_1->inertial_iter);
  InsEstimatePropagateForward(header, data, xhat_1, xhat);
}

// Public functions.

void InsEstimateInit(InsEstimate *xhat) {
  memset(xhat, 0, sizeof(*xhat));

  // TODO: Initialize inertial estimate.
}

void InsEstimateBufferInit(uint32_t inertial_iter, InsEstimateBuffer *buf) {
  memset(buf, 0, sizeof(*buf));

  InsEstimateInit(&buf->xhat[0]);
  InsEstimateInit(&buf->xhat[1]);
  buf->inertial_iter = inertial_iter;
}

void InsEstimateSetCurrent(const InsEstimate *xhat, InsEstimateBuffer *buf) {
  buf->inertial_iter = xhat->inertial_iter;
  *InsEstimateGetCurrent(buf) = *xhat;
}

void InsEstimatePropagateForward(const InsMessageHeader *header,
                                 const InsInertialMessage *data,
                                 const InsEstimate *xhat_1, InsEstimate *xhat) {
  // Propagate sequence information.
  xhat->inertial_iter = xhat_1->inertial_iter + 1U;
  xhat->inertial_seq_num = header->seq_num;
  xhat->filter_seq_num = xhat_1->filter_seq_num;

  // TODO: Propagate inertial estimate.
  (void)data;
}

bool InsEstimatePropagateForwardToSeqNum(const InsInertial *inertial,
                                         uint16_t seq_num,
                                         InsEstimateBuffer *buf) {
  return InsInertialIterateToSeqNum(inertial, seq_num, PropagateBufferForward,
                                    buf, &buf->inertial_iter);
}

bool InsEstimatePropagateForwardToHead(const InsInertial *inertial,
                                       InsEstimateBuffer *buf) {
  return InsInertialIterateToHead(inertial, PropagateBufferForward, buf,
                                  &buf->inertial_iter);
}

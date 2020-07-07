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

#include "nav/ins/output/output.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "common/ring.h"
#include "nav/ins/estimate/estimate.h"
#include "nav/ins/inertial/inertial.h"
#include "nav/ins/messages/messages.h"

// The design of this module permits an algorithm thread and a real-time data
// acquisition thread. The algorithm thread lags behind the real-time thread
// and updates the filter estimate via InsOutputUpdateFilterEstimate. This
// function stores the updated estimate in a circular buffer for the real-time
// thread. The real-time thread then calls InsOutputPropagateInertialEstimate
// for each inertial update. If a filter update exists, it copies the new
// estimate to the current estimate, and then propagates that estimate forward
// to the current inertial update.

static bool UpdateInertialEstimate(InsOutputBuffer *filter,
                                   InsEstimateBuffer *inertial) {
  // Called from the real-time data acquisition thread.
  int32_t used = RingGetUsed(&filter->ring);
  if (used > 0) {
    RingAdvanceTail(used - 1, &filter->ring);

    int32_t index;
    RingGetTailIndex(&filter->ring, &index);
    InsEstimateSetCurrent(&filter->xhat[index], inertial);
    RingPopTail(&filter->ring);
    return true;
  }
  return false;
}

// Public functions.

void InsOutputInit(const InsInertial *inertial, InsOutput *output) {
  memset(output, 0, sizeof(*output));

  RingInit(INS_OUTPUT_RING_ELEMENTS, &output->filter.ring);
  InsEstimateBufferInit(InsInertialGetTailIter(inertial), &output->inertial);
}

void InsOutputUpdateFilterEstimate(const InsEstimate *xhat, InsOutput *output) {
  // Called from the algorithm thread.
  int32_t index;
  if (RingGetNewHeadIndex(&output->filter.ring, &index)) {
    output->filter.xhat[index] = *xhat;
    RingPushNewHead(&output->filter.ring);
  }
}

const InsEstimate *InsOutputGetInertialEstimate(const InsOutput *output) {
  // Called from the real-time data acquisition thread.
  return InsEstimateGetCurrentConst(&output->inertial);
}

bool InsOutputPropagateInertialEstimate(const InsInertial *inertial,
                                        InsOutput *output) {
  // Called from the real-time data acquisition thread.
  bool updated = UpdateInertialEstimate(&output->filter, &output->inertial);
  return InsEstimatePropagateForwardToHead(inertial, &output->inertial)
      || updated;
}

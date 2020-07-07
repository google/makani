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

#ifndef NAV_INS_ESTIMATE_TYPES_H_
#define NAV_INS_ESTIMATE_TYPES_H_

#include <stdint.h>

typedef struct {
  // This iterator corresponds to the iterator of the ring buffer in
  // InsInertialBuffer. Use it to propagate the estimate forward.
  uint32_t inertial_iter;

  // This seq_num corresponds to the sequence number of the inertial data for
  // the last inertial update. Sequence numbers may not be contiguous.
  uint16_t inertial_seq_num;

  // This seq_num corresponds to the sequence number of the inertial data for
  // the last filter update. Sequence numbers may not be contiguous.
  uint16_t filter_seq_num;

  // TODO: Add inertial estimate.
} InsEstimate;

typedef struct {
  // Use a double buffer to store the previous and current estimate. This
  // strategy avoids extra memory copies during the propagation and
  // correction steps.
  InsEstimate xhat[2];

  // This iterator corresponds to the iterator of the ring buffer in
  // InsInertialBuffer of the last inertial update. The least significant
  // bit specifies the index of the xhat[] double buffer.
  uint32_t inertial_iter;
} InsEstimateBuffer;

// Obtain xhat[k].
static inline InsEstimate *InsEstimateGetCurrent(InsEstimateBuffer *buf) {
  return &buf->xhat[buf->inertial_iter & 1U];
}

// Obtain xhat[k] (constant version).
static inline const InsEstimate *InsEstimateGetCurrentConst(
    const InsEstimateBuffer *buf) {
  return &buf->xhat[buf->inertial_iter & 1U];
}

// Obtain xhat[k+1]. Use for propagating forward. Estimate is not valid.
static inline InsEstimate *InsEstimateGetNext(InsEstimateBuffer *buf) {
  return &buf->xhat[(buf->inertial_iter + 1U) & 1U];
}

// Obtain xhat[k-1].
static inline const InsEstimate *InsEstimateGetPreviousConst(
    const InsEstimateBuffer *buf) {
  return &buf->xhat[(buf->inertial_iter - 1U) & 1U];
}

#endif  // NAV_INS_ESTIMATE_TYPES_H_

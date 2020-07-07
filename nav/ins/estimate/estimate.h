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

#ifndef NAV_INS_ESTIMATE_ESTIMATE_H_
#define NAV_INS_ESTIMATE_ESTIMATE_H_

#include <stdbool.h>
#include <stdint.h>

#include "nav/ins/inertial/types.h"
#include "nav/ins/estimate/types.h"
#include "nav/ins/messages/messages.h"

#ifdef __cplusplus
extern "C" {
#endif

void InsEstimateInit(InsEstimate *xhat);
void InsEstimateBufferInit(uint32_t iter, InsEstimateBuffer *buf);

void InsEstimateSetCurrent(const InsEstimate *xhat, InsEstimateBuffer *buf);

void InsEstimatePropagateForward(const InsMessageHeader *header,
                                 const InsInertialMessage *data,
                                 const InsEstimate *xhat_1, InsEstimate *xhat);

bool InsEstimatePropagateForwardToSeqNum(const InsInertial *inertial,
                                         uint16_t seq_num,
                                         InsEstimateBuffer *buf);

bool InsEstimatePropagateForwardToHead(const InsInertial *inertial,
                                       InsEstimateBuffer *buf);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // NAV_INS_ESTIMATE_ESTIMATE_H_

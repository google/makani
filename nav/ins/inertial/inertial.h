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

#ifndef NAV_INS_INERTIAL_INERTIAL_H_
#define NAV_INS_INERTIAL_INERTIAL_H_

#include <stdbool.h>
#include <stdint.h>

#include "nav/ins/inertial/params.h"
#include "nav/ins/inertial/types.h"
#include "nav/ins/messages/messages.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void (* const InsInertialIterateFunc)(const InsMessageHeader *header,
                                              const InsInertialMessage *data,
                                              void *arg);

void InsInertialInit(InsInertial *in);

uint32_t InsInertialGetTailIter(const InsInertial *in);

bool InsInertialGetNewHead(const InsMessageHeader *header,
                           InsInertial *inertial, InsInertialMessage **data_b);

void InsInertialPushNewHead(InsInertial *inertial);

bool InsInertialInsertMessage(const InsMessageHeader *header,
                              const InsInertialMessage *data_s,
                              const InsInertialParams *params,
                              InsInertial *inertial);

void InsInertialRemoveTail(int64_t min_time, InsInertial *in);

void InsInertialRotateSensorToBody(const InsInertialParams *param,
                                   const InsInertialMessage *sensor,
                                   InsInertialMessage *body);

bool InsInertialIterateToSeqNum(const InsInertial *in, uint16_t seq_num,
                                InsInertialIterateFunc func, void *arg,
                                uint32_t *iter);

bool InsInertialIterateToHead(const InsInertial *in,
                              InsInertialIterateFunc func, void *arg,
                              uint32_t *iter);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // NAV_INS_INERTIAL_INERTIAL_H_

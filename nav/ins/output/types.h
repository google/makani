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

#ifndef NAV_INS_OUTPUT_TYPES_H_
#define NAV_INS_OUTPUT_TYPES_H_

#include "common/ring.h"
#include "nav/ins/estimate/types.h"

#define INS_OUTPUT_RING_ELEMENTS 32

typedef struct {
  RingBuffer ring;
  InsEstimate xhat[INS_OUTPUT_RING_ELEMENTS];
} InsOutputBuffer;

typedef struct {
  InsOutputBuffer filter;  // Owned by algorithm thread.
  InsEstimateBuffer inertial;  // Owned by real-time data acquisition thread.
} InsOutput;

#endif  // NAV_INS_OUTPUT_TYPES_H_

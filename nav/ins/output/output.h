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

#ifndef NAV_INS_OUTPUT_OUTPUT_H_
#define NAV_INS_OUTPUT_OUTPUT_H_

#include <stdbool.h>

#include "nav/ins/inertial/types.h"
#include "nav/ins/messages/messages.h"
#include "nav/ins/output/types.h"

#ifdef __cplusplus
extern "C" {
#endif

void InsOutputInit(const InsInertial *inertial, InsOutput *output);

void InsOutputUpdateFilterEstimate(const InsEstimate *xhat, InsOutput *output);

const InsEstimate *InsOutputGetInertialEstimate(const InsOutput *output);

bool InsOutputPropagateInertialEstimate(const InsInertial *inertial,
                                        InsOutput *output);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // NAV_INS_OUTPUT_OUTPUT_H_

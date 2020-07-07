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

#include "nav/ins/algorithm/algorithm.h"

#include <assert.h>
#include <stdbool.h>
#include <string.h>

#include "nav/ins/algorithm/params.h"
#include "nav/ins/algorithm/types.h"
#include "nav/ins/estimate/types.h"
#include "nav/ins/inertial/inertial.h"
#include "nav/ins/messages/messages.h"
#include "nav/ins/output/output.h"

void InsAlgorithmInit(InsAlgorithmState *state) {
  memset(state, 0, sizeof(*state));

  InsInertialInit(&state->inertial);
  InsOutputInit(&state->inertial, &state->output);
}

// Called from the real-time data acquisition thread.
bool InsAlgorithmInsertMessage(const InsMessage *m,
                               const InsAlgorithmParams *params,
                               InsAlgorithmState *state) {
  switch (m->header.type) {
    case kInsMessageTypeInertial:
      return InsInertialInsertMessage(&m->header, &m->u.inertial,
                                      &params->inertial, &state->inertial);
    case kInsMessageTypeForceSigned:  // Fall-through intentional.
    case kNumInsMessageTypes:  // Fall-through intentional.
    default:
      assert(!(bool)"Unknown message type.");
      return false;
  }
}

// Called from the real-time data acquisition thread.
bool InsAlgorithmPropagateEstimate(InsAlgorithmState *state) {
  return InsOutputPropagateInertialEstimate(&state->inertial, &state->output);
}

// Called from the real-time data acquisition thread.
const InsEstimate *InsAlgorithmGetEstimate(const InsAlgorithmState *state) {
  return InsOutputGetInertialEstimate(&state->output);
}

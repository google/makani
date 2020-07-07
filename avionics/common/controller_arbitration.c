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

#include "avionics/common/controller_arbitration.h"

#include <assert.h>
#include <stdint.h>
#include <string.h>

#include "avionics/common/avionics_messages.h"
#include "avionics/common/cvt_avionics_messages.h"
#include "avionics/network/aio_labels.h"

// This value is represented by a function rather than a constant
// to allow for easier unit testing.
static int32_t ControllerArbitrationGetTimeoutUsec(void) {
  return 50000;
}

void ControllerArbitrationInit(int64_t now, ControllerArbitrationState *state) {
  memset(state, 0, sizeof(*state));
  for (int32_t i = 0; i < kNumControllers; ++i) {
    // Set all controller messages to be stale.
    state->timestamps[i] = now - ControllerArbitrationGetTimeoutUsec() - 1;
  }
}

const ControllerCommandMessage *ControllerArbitrationGetCommand(
    int64_t now, const MessageValidator func,
    const ControllerArbitrationState *state,
    ControllerLabel *source) {

  assert(source != NULL);
#ifdef USE_REAL_CONTROLLER_ARBITRATION
  // TODO: Turn this back on when we're ready for it.
  int32_t limit = kNumControllers;
#else
  int32_t limit = kControllerB;
#endif
  for (int32_t i = 0; i < limit; ++i) {
    if (now - state->timestamps[i] < ControllerArbitrationGetTimeoutUsec() &&
        func(&state->commands[i])) {
      *source = (ControllerLabel)i;
      return &state->commands[i];
    }
  }
  return NULL;
}

void ControllerArbitrationUpdateFromCvt(ControllerArbitrationState *state) {
  for (int32_t i = 0; i < kNumControllers; ++i) {
    AioNode source = ControllerLabelToControllerAioNode((ControllerLabel)i);
    CvtGetControllerCommandMessage(source, &state->commands[i],
                                   &state->sequence_numbers[i],
                                   &state->timestamps[i]);
  }
}

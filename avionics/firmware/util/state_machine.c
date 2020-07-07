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

#include "avionics/firmware/util/state_machine.h"

#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "avionics/firmware/cpu/clock.h"

static void UpdateState(int32_t new_state, FsmState *state) {
  assert(state != NULL);
  assert(state->state >= 0);

  state->state = new_state;
  state->first_entry = true;
  state->entry_cycles = Clock32GetCycles();
}

void StateMachineInit(const StateMachine *fsm, FsmState *state) {
  memset(state, 0, sizeof(*state));
  UpdateState(fsm->init_state, state);
}

void StateMachinePoll(const StateMachine *fsm, FsmState *state) {
  assert(fsm != NULL);
  assert(state != NULL);
  assert(fsm->states != NULL);
  assert(state->state >= 0 && state->state < fsm->num_states);

  if (state->state < 0 || state->state >= fsm->num_states) {
    UpdateState(fsm->init_state, state);
  }

  const StateTransition *state_trans = &fsm->states[state->state];
  assert(state_trans->function != NULL);
  state->next = state_trans->state_next;
  state->error = state_trans->state_error >= 0 ?
      state_trans->state_error : fsm->error_state_default;
  int32_t timeout_state = state_trans->state_timeout >= 0 ?
      state_trans->state_timeout : fsm->timeout_state_default;
  bool advance = state_trans->function(state);
  if (!advance) {
    int32_t timeout_cycles = state_trans->timeout_cycles >= 0 ?
        state_trans->timeout_cycles : fsm->timeout_cycles_default;
    if (timeout_cycles > 0 && CLOCK32_GE(Clock32GetCycles(), state->entry_cycles
                                         + state_trans->timeout_cycles)) {
      UpdateState(timeout_state, state);
    } else {
      state->first_entry = false;
    }
  } else {
    UpdateState(state->next, state);
  }
}

int32_t GetState(const FsmState *state) {
  return state->state;
}

void SetState(int32_t new_state, FsmState *state) {
  UpdateState(new_state, state);
}

void SetStateUserData(void *user_data, FsmState *state) {
  state->user_data = user_data;
}

void *GetStateUserData(const FsmState *state) {
  return state->user_data;
}

bool StateMachineStateIdle(FsmState *state) {
  (void)state;
  return false;
}

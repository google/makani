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

#ifndef AVIONICS_FIRMWARE_UTIL_STATE_MACHINE_H_
#define AVIONICS_FIRMWARE_UTIL_STATE_MACHINE_H_

#include <stdbool.h>
#include <stdint.h>

#include "avionics/firmware/cpu/clock.h"

#define STATE_INVALID -1
#define STATE_TRANS(state, fn, next, error, timeout, timeout_us)        \
  [state] = {fn, next, error, timeout, CLOCK32_USEC_TO_CYCLES(timeout_us)}
#define STATE_TRANS_DEFAULT(state, fn, next)                    \
  [state] = {fn, next, STATE_INVALID, STATE_INVALID, -1}
#define STATE_TRANS_DELAY(state, next, delay_us)                        \
  [state] = {StateMachineStateIdle, STATE_INVALID, STATE_INVALID, next, \
             CLOCK32_USEC_TO_CYCLES(delay_us)}
#define STATE_TRANS_IDLE(state)                                         \
  [state] = {StateMachineStateIdle, STATE_INVALID, STATE_INVALID,       \
             STATE_INVALID, -1}

typedef struct {
  int32_t state;
  bool first_entry;
  uint32_t entry_cycles;
  int32_t next;
  int32_t error;
  void *user_data;
} FsmState;

typedef bool (* const StateFunction)(FsmState *state);

// Define the default transitions for a given state.  The next state is the
// default action intended after the function of this state is completed.  The
// error state is the transition in case of a problem during the current state.
// The timeout state is used if the current state is active for longer than
// the timeout (in microseconds).
//
// For state_error or state_timeout, any negative value will cause the default
// from the StateMachine definition to be used instead.  -1 is the conventional
// value to invoke the default state or to represent an unused transition.
// A negative timeout_cycles value will invoke the default timeout.
typedef struct {
  StateFunction function;
  int32_t state_next;
  int32_t state_error;
  int32_t state_timeout;
  int32_t timeout_cycles;
} StateTransition;

typedef struct {
  const int32_t num_states;
  const int32_t init_state;
  const int32_t error_state_default;
  const int32_t timeout_state_default;
  const int32_t timeout_cycles_default;
  const StateTransition * const states;
} StateMachine;

void StateMachineInit(const StateMachine *fsm, FsmState *state);
void StateMachinePoll(const StateMachine *fsm, FsmState *state);
int32_t GetState(const FsmState *state);
void SetState(int32_t new_state, FsmState *state);
void SetStateUserData(void *user_data, FsmState *state);
void *GetStateUserData(const FsmState *state);
bool StateMachineStateIdle(FsmState *state);

#endif  // AVIONICS_FIRMWARE_UTIL_STATE_MACHINE_H_

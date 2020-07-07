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

#include "avionics/common/debounce.h"

void DebounceStateInit(bool value, DebounceState *state) {
  state->output_value = value;
  state->last_value = value;
  state->count = 0;
}

bool Debounce(bool input_value,
              int32_t high_threshold,
              int32_t low_threshold,
              DebounceState *state) {
  if (input_value != state->last_value) {
    state->last_value = input_value;
    state->count = 0;
  } else if ((input_value && state->count < high_threshold)
             || (!input_value && state->count < low_threshold)) {
    ++state->count;
  }

  if ((input_value && state->count >= high_threshold)
      || (!input_value && state->count >= low_threshold)) {
    state->output_value = input_value;
  }

  return state->output_value;
}

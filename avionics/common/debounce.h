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

#ifndef AVIONICS_COMMON_DEBOUNCE_H_
#define AVIONICS_COMMON_DEBOUNCE_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  bool output_value;
  bool last_value;
  int32_t count;
} DebounceState;

// Initialize state with value.
void DebounceStateInit(bool value, DebounceState *state);

// Debounce input value.  Input must be high for at least high_threshold
// cycles before transitioning to high.  Same for low/low_threshold.
bool Debounce(bool input_value,
              int32_t high_threshold,
              int32_t low_threshold,
              DebounceState *state);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // AVIONICS_COMMON_DEBOUNCE_H_

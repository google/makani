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

#include "control/estimator/estimator_joystick.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "common/c_math/filter.h"
#include "common/c_math/util.h"
#include "control/estimator/estimator_types.h"
#include "control/fault_detection/fault_detection_types.h"
#include "control/system_params.h"
#include "control/system_types.h"
#include "system/labels.h"

void EstimatorJoystickInit(EstimatorJoystickState *state) {
  assert(state != NULL);
  memset(state, 0, sizeof(*state));
  state->last_valid_data.throttle = 0.0;
  state->last_valid_data.roll = 0.0;
  state->last_valid_data.pitch = 0.0;
  state->last_valid_data.yaw = 0.0;
  state->last_valid_data.switch_position = kJoystickSwitchPositionUp;
  state->last_valid_data.release = false;
  state->throttle_f_z1 = 0.0;
  state->pitch_f_z1 = 0.0;
  state->debounce_joystick_release_count = 0;
  state->switch_position_z1 = kJoystickSwitchPositionUp;
  state->switch_up_count = 0;
  state->switch_mid_count = 0;
  state->switch_down_count = 0;
}

// Returns true only if release has been true for num_debounce iterations.
static bool DebounceJoystickRelease(bool release, int32_t num_debounce,
                                    EstimatorJoystickState *state) {
  if (!release) {
    state->debounce_joystick_release_count = 0;
  } else if (state->debounce_joystick_release_count < INT32_MAX) {
    state->debounce_joystick_release_count++;
  }
  return state->debounce_joystick_release_count > num_debounce;
}

// Debounce the switch position.
static JoystickSwitchPositionLabel DebounceJoystickSwitchPosition(
    JoystickSwitchPositionLabel switch_position, int32_t num_debounce,
    EstimatorJoystickState *state) {
  // Debounce switch.  This block of code and the one that follows
  // have the effect of incrementing the counter for the current
  // switch state and decrementing the counters for the other states,
  // followed by a saturation.
  if (switch_position == kJoystickSwitchPositionMiddle) {
    state->switch_mid_count += 2;
  } else if (switch_position == kJoystickSwitchPositionDown) {
    state->switch_down_count += 2;
  } else {
    state->switch_up_count += 2;
  }
  state->switch_mid_count =
      SaturateInt32(state->switch_mid_count - 1, 0, num_debounce);
  state->switch_down_count =
      SaturateInt32(state->switch_down_count - 1, 0, num_debounce);
  state->switch_up_count =
      SaturateInt32(state->switch_up_count - 1, 0, num_debounce);

  JoystickSwitchPositionLabel switch_position_debounced;
  if (state->switch_up_count >= num_debounce) {
    switch_position_debounced = kJoystickSwitchPositionUp;
  } else if (state->switch_mid_count >= num_debounce) {
    switch_position_debounced = kJoystickSwitchPositionMiddle;
  } else if (state->switch_down_count >= num_debounce) {
    switch_position_debounced = kJoystickSwitchPositionDown;
  } else {
    switch_position_debounced = state->switch_position_z1;
  }
  state->switch_position_z1 = switch_position_debounced;

  return switch_position_debounced;
}

void EstimatorJoystickStep(const JoystickData *joystick_data,
                           const FaultMask *joystick_fault,
                           const EstimatorJoystickParams *params,
                           EstimatorJoystickState *state,
                           JoystickEstimate *joystick) {
  assert(joystick_data != NULL && joystick_fault != NULL && params != NULL &&
         state != NULL && joystick != NULL);
  joystick->valid = !HasAnyFault(joystick_fault);
  if (joystick->valid) {
    state->last_valid_data = *joystick_data;
    state->last_valid_data.throttle =
        Saturate(state->last_valid_data.throttle, 0.0, 1.0);
    state->last_valid_data.roll =
        Saturate(state->last_valid_data.roll, -1.0, 1.0);
    state->last_valid_data.pitch =
        Saturate(state->last_valid_data.pitch, -1.0, 1.0);
    state->last_valid_data.yaw =
        Saturate(state->last_valid_data.yaw, -1.0, 1.0);
  }
  joystick->data = state->last_valid_data;

  joystick->data.release = DebounceJoystickRelease(
      joystick->data.release, params->joystick_num_debounce, state);

  joystick->data.switch_position = DebounceJoystickSwitchPosition(
      joystick->data.switch_position, params->joystick_num_debounce, state);

  joystick->throttle_f = Lpf(joystick->data.throttle, params->fc_throttle,
                             *g_sys.ts, &state->throttle_f_z1);
  joystick->pitch_f = Lpf(joystick->data.pitch, params->fc_pitch, *g_sys.ts,
                          &state->pitch_f_z1);
}

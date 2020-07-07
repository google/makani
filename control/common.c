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

#include "control/common.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

#include "control/control_types.h"
#include "system/labels.h"

bool IsControlSystemRunning(InitializationState init_state) {
  return (init_state == kInitializationStateFirstLoop ||
          init_state == kInitializationStateRunning);
}

bool IsValidFlightMode(FlightMode flight_mode) {
  return 0 <= flight_mode && flight_mode < kNumFlightModes;
}

// Return true if flight_mode represents an autonomous hover mode.
bool AnyAutoHoverFlightMode(FlightMode flight_mode) {
  assert(IsValidFlightMode(flight_mode));
  return (flight_mode == kFlightModeHoverAscend) ||
         (flight_mode == kFlightModeHoverPayOut) ||
         (flight_mode == kFlightModeHoverPrepTransformGsUp) ||
         (flight_mode == kFlightModeHoverTransformGsUp) ||
         (flight_mode == kFlightModeHoverFullLength) ||
         (flight_mode == kFlightModeHoverAccel) ||
         (flight_mode == kFlightModeHoverTransOut) ||
         (flight_mode == kFlightModeHoverPrepTransformGsDown) ||
         (flight_mode == kFlightModeHoverTransformGsDown) ||
         (flight_mode == kFlightModeHoverReelIn) ||
         (flight_mode == kFlightModeHoverDescend);
}

bool AnyHoverFlightMode(FlightMode flight_mode) {
  assert(IsValidFlightMode(flight_mode));
  return AnyAutoHoverFlightMode(flight_mode) ||
         flight_mode == kFlightModePilotHover;
}

// Return true if flight_mode represents a crosswind mode.
bool AnyCrosswindFlightMode(FlightMode flight_mode) {
  assert(IsValidFlightMode(flight_mode));
  return (flight_mode == kFlightModeCrosswindNormal) ||
         (flight_mode == kFlightModeCrosswindPrepTransOut);
}

// TODO: This is a hack that selects the next flight mode
// (i.e. gated mode).  Eventually, we should formalize this either in
// control_planner.c or in the individual controllers. Note, this
// function is not used by the planner but by auxiliarly tools such as
// the monitors.
FlightMode GetNextFlightMode(FlightMode flight_mode) {
  switch (flight_mode) {
    case kFlightModePilotHover:
      return kFlightModePerched;
    case kFlightModePerched:
      return kFlightModeHoverAscend;
    case kFlightModeHoverAscend:
      return kFlightModeHoverPayOut;
    case kFlightModeHoverPayOut:
      return kFlightModeHoverPrepTransformGsUp;
    case kFlightModeHoverPrepTransformGsUp:
      return kFlightModeHoverTransformGsUp;
    case kFlightModeHoverTransformGsUp:
      return kFlightModeHoverFullLength;
    case kFlightModeHoverFullLength:
      return kFlightModeHoverAccel;
    case kFlightModeHoverAccel:
      return kFlightModeTransIn;
    case kFlightModeTransIn:
      return kFlightModeCrosswindNormal;
    case kFlightModeCrosswindNormal:
      return kFlightModeCrosswindPrepTransOut;
    case kFlightModeCrosswindPrepTransOut:
      return kFlightModeHoverTransOut;
    case kFlightModeHoverTransOut:
      return kFlightModeHoverPrepTransformGsDown;
    case kFlightModeHoverPrepTransformGsDown:
      return kFlightModeHoverTransformGsDown;
    case kFlightModeHoverTransformGsDown:
      return kFlightModeHoverReelIn;
    case kFlightModeHoverReelIn:
      return kFlightModeHoverDescend;
    case kFlightModeHoverDescend:
      return kFlightModeOffTether;
    case kFlightModeOffTether:
      return kFlightModePerched;
    default:
    case kFlightModeForceSigned:
    case kNumFlightModes:
      assert(false);
      return kFlightModeForceSigned;
  }
}

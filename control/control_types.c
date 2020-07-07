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

#include "control/control_types.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

#include "common/macros.h"
#include "control/crosswind/crosswind_types.h"
#include "control/hover/hover_types.h"
#include "control/trans_in/trans_in_types.h"

int32_t GetNumFlightModeGates(FlightMode flight_mode) {
  switch (flight_mode) {
    case kFlightModePilotHover:
      return 0;
    case kFlightModePerched:
      return kNumHoverPerchedGates;
    case kFlightModeHoverAscend:
      return kNumHoverAscendGates;
    case kFlightModeHoverPayOut:
      return kNumHoverPayOutGates;
    case kFlightModeHoverFullLength:
      return kNumHoverFullLengthGates;
    case kFlightModeHoverTransformGsUp:
      return kNumHoverTransformGsUpGates;
    case kFlightModeHoverAccel:
      return kNumHoverAccelGates;
    case kFlightModeTransIn:
      return kNumTransInGates;
    case kFlightModeCrosswindNormal:
      return kNumCrosswindNormalGates;
    case kFlightModeCrosswindPrepTransOut:
      return kNumCrosswindPrepTransOutGates;
    case kFlightModeHoverTransformGsDown:
      return kNumHoverTransformGsDownGates;
    case kFlightModeHoverTransOut:
      return kNumCrosswindHoverTransOutGates;
    case kFlightModeHoverReelIn:
      return kNumHoverReelInGates;
    case kFlightModeHoverDescend:
      return kNumHoverDescendGates;
    case kFlightModeHoverPrepTransformGsUp:
      return kNumHoverPrepTransformGsUpGates;
    case kFlightModeHoverPrepTransformGsDown:
      return kNumHoverPrepTransformGsDownGates;
    case kFlightModeOffTether:
      return 0;
    default:
    case kFlightModeForceSigned:
    case kNumFlightModes:
      assert(false);
      return 0;
  }
}

const char *InitializationStateToString(InitializationState init_state) {
  switch (init_state) {
    case kInitializationStateFirstEntry:
      return "FirstEntry";
    case kInitializationStateWaitForValidData:
      return "WaitForValidData";
    case kInitializationStateFirstLoop:
      return "FirstLoop";
    case kInitializationStateRunning:
      return "Running";
    default:
      assert(false);
      return "<Unknown>";
  }
}

const char *FlightModeToString(FlightMode flight_mode) {
  switch (flight_mode) {
    case kFlightModePilotHover:
      return "PilotHover";
    case kFlightModePerched:
      return "Perched";
    case kFlightModeHoverAscend:
      return "HoverAscend";
    case kFlightModeHoverPayOut:
      return "HoverPayOut";
    case kFlightModeHoverFullLength:
      return "HoverFullLength";
    case kFlightModeHoverTransformGsUp:
      return "HoverTransformGsUp";
    case kFlightModeHoverAccel:
      return "HoverAccel";
    case kFlightModeTransIn:
      return "TransIn";
    case kFlightModeCrosswindNormal:
      return "CrosswindNormal";
    case kFlightModeCrosswindPrepTransOut:
      return "CrosswindPrepTransOut";
    case kFlightModeHoverTransformGsDown:
      return "HoverTransformGsDown";
    case kFlightModeHoverTransOut:
      return "HoverTransOut";
    case kFlightModeHoverReelIn:
      return "HoverReelIn";
    case kFlightModeHoverDescend:
      return "HoverDescend";
    case kFlightModeOffTether:
      return "OffTether";
    case kFlightModeHoverPrepTransformGsUp:
      return "PrepHoverTransformUp";
    case kFlightModeHoverPrepTransformGsDown:
      return "PrepHoverTransformDown";
    default:
    case kFlightModeForceSigned:
    case kNumFlightModes:
      assert(false);
      return "<Unknown>";
  }
}

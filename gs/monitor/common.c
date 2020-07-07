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

#include "gs/monitor/common.h"

#include <stdbool.h>
#include <stdint.h>

#include "control/crosswind/crosswind_types.h"
#include "control/hover/hover_types.h"
#include "control/trans_in/trans_in_types.h"

const char *GateToString(FlightMode gated_mode, int32_t gate) {
  switch (gated_mode) {
    case kFlightModePerched:
      return HoverPerchedGateToString(gate);
    case kFlightModeHoverAscend:
      return HoverAscendGateToString(gate);
    case kFlightModeHoverPayOut:
      return HoverPayOutGateToString(gate);
    case kFlightModeHoverPrepTransformGsUp:
      return HoverPrepTransformGsUpGateToString(gate);
    case kFlightModeHoverTransformGsUp:
      return HoverTransformGsUpGateToString(gate);
    case kFlightModeHoverFullLength:
      return HoverFullLengthGateToString(gate);
    case kFlightModeHoverAccel:
      return HoverAccelGateToString(gate);
    case kFlightModeTransIn:
      return TransInGateToString(gate);
    case kFlightModeCrosswindNormal:
      return CrosswindNormalGateToString(gate);
    case kFlightModeCrosswindPrepTransOut:
      return CrosswindPrepTransOutGateToString(gate);
    case kFlightModeHoverTransOut:
      return CrosswindHoverTransOutGateToString(gate);
    case kFlightModeHoverTransformGsDown:
      return HoverTransformGsDownGateToString(gate);
    case kFlightModeHoverPrepTransformGsDown:
      return HoverPrepTransformGsDownGateToString(gate);
    case kFlightModeHoverReelIn:
      return HoverReelInGateToString(gate);
    case kFlightModeHoverDescend:
      return HoverDescendGateToString(gate);

    default:
    case kFlightModeForceSigned:
    case kFlightModePilotHover:
    case kFlightModeOffTether:
    case kNumFlightModes:
      return "<Unknown>";
  }
}

const char *GetHitlDispStr(SimulatorHitlLevel sim_level,
                           bool use_software_joystick) {
  switch (sim_level) {
    case kSimulatorHitlLevelNone:
      return "";
    case kSimulatorHitlLevelAsync:
      return use_software_joystick ? "HITL (SW Joystick)" : "HITL";
    case kSimulatorHitlLevelSync:
      return "Sim";
    default:
      return "Unknown";
  }
}

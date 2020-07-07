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

#include "control/crosswind/crosswind_types.h"

#include <assert.h>
#include <stdbool.h>

const char *CrosswindNormalGateToString(CrosswindNormalGate gate) {
  switch (gate) {
    case kCrosswindNormalGateSpeed:
      return "Speed";
    case kCrosswindNormalGateTension:
      return "Tension";
    case kCrosswindNormalGateAltitude:
      return "Altitude";
    case kCrosswindNormalGateAirspeed:
      return "Airspeed";
    case kCrosswindNormalGateFlightMode:
      return "Flight Mode";
    default:
    case kCrosswindNormalGateForceSigned:
    case kNumCrosswindNormalGates:
      assert(false);
      return "<Unknown>";
  }
}

const char *CrosswindPrepTransOutGateToString(CrosswindPrepTransOutGate gate) {
  switch (gate) {
    default:
    case kCrosswindPrepTransOutGateForceSigned:
    case kNumCrosswindPrepTransOutGates:
      assert(false);
      return "<Unknown>";
  }
}

const char *CrosswindHoverTransOutGateToString(
    CrosswindHoverTransOutGate gate) {
  switch (gate) {
    case kCrosswindHoverTransOutGateAirspeed:
      return "Airspeed";
    case kCrosswindHoverTransOutGateAlpha:
      return "Angle of Attack";
    case kCrosswindHoverTransOutGatePathType:
      return "PathType";
    case kCrosswindHoverTransOutGateStillAccelerating:
      return "StillAccelerating";
    default:
    case kCrosswindHoverTransOutGateForceSigned:
    case kNumCrosswindHoverTransOutGates:
      assert(false);
      return "<Unknown>";
  }
}

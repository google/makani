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

#include "control/hover/hover_types.h"

#include <assert.h>
#include <stdbool.h>

const char *HoverPerchedGateToString(HoverPerchedGate gate) {
  switch (gate) {
    case kHoverPerchedGateDisabled:
      return "Disabled";
    default:
    case kHoverPerchedGateForceSigned:
    case kNumHoverPerchedGates:
      assert(false);
      return "<Unknown>";
  }
}

const char *HoverAscendGateToString(HoverAscendGate gate) {
  switch (gate) {
    case kHoverAscendGateProximityValid:
      return "ProximityValid";
    case kHoverAscendGatePerchWindMisalignment:
      return "PerchWindMisalignment";
    case kHoverAscendGateTension:
      return "Tension";
    default:
    case kHoverAscendGateForceSigned:
    case kNumHoverAscendGates:
      assert(false);
      return "<Unknown>";
  }
}

const char *HoverPayOutGateToString(HoverPayOutGate gate) {
  switch (gate) {
    case kHoverPayOutGateAscentComplete:
      return "AscentComplete";
    case kHoverPayOutGateGainRampDone:
      return "GainRampDone";
    case kHoverPayOutGateZPosition:
      return "ZPosition";
    case kHoverPayOutGateYawError:
      return "YawError";
    case kHoverPayOutGateYawRate:
      return "YawRate";
    default:
    case kHoverPayOutGateForceSigned:
    case kNumHoverPayOutGates:
      assert(false);
      return "<Unknown>";
  }
}

const char *HoverFullLengthGateToString(HoverFullLengthGate gate) {
  switch (gate) {
    case kHoverFullLengthGateGroundStationMode:
      return "GroundStationMode";
    case kHoverFullLengthGateForceDetwistTurn:
      return "ForceDetwistTurn";
    default:
    case kHoverFullLengthGateForceSigned:
    case kNumHoverFullLengthGates:
      assert(false);
      return "<Unknown>";
  }
}

const char *HoverPrepTransformGsUpGateToString(
    HoverPrepTransformGsUpGate gate) {
  switch (gate) {
    case kHoverPrepTransformGsUpGateFlightPlan:
      return "FlightPlan";
    case kHoverPrepTransformGsUpGateGroundStationMode:
      return "GroundStationMode";
    case kHoverPrepTransformGsUpGateWinchPosition:
      return "WinchPosition";
    default:
    case kHoverPrepTransformGsUpGateForceSigned:
    case kNumHoverPrepTransformGsUpGates:
      assert(false);
      return "<Unknown>";
  }
}

const char *HoverTransformGsUpGateToString(HoverTransformGsUpGate gate) {
  switch (gate) {
    case kHoverTransformGsUpGateTetherElevation:
      return "TetherElevation";
    case kHoverTransformGsUpGateAzimuthError:
      return "AzimuthError";
    case kHoverTransformGsUpGateZError:
      return "ZError";
    default:
    case kHoverTransformGsUpGateForceSigned:
    case kNumHoverTransformGsUpGates:
      assert(false);
      return "<Unknown>";
  }
}

const char *HoverAccelGateToString(HoverAccelGate gate) {
  switch (gate) {
    case kHoverAccelGateFlightPlan:
      return "FlightPlan";
    case kHoverAccelGateRollError:
      return "RollError";
    case kHoverAccelGateYawError:
      return "YawError";
    case kHoverAccelGateYawRate:
      return "YawRate";
    case kHoverAccelGateAngularRate:
      return "AngularRate";
    case kHoverAccelGateAzimuthError:
      return "AzimuthError";
    case kHoverAccelGateZError:
      return "ZError";
    case kHoverAccelGateSpeed:
      return "Speed";
    case kHoverAccelGateYVelocity:
      return "YVelocity";
    case kHoverAccelGateTension:
      return "Tension";
    case kHoverAccelGateGroundStationMode:
      return "GroundStationMode";
    case kHoverAccelGateForceDetwistTurn:
      return "ForceDetwistTurn";
    default:
    case kHoverAccelGateForceSigned:
    case kNumHoverAccelGates:
      assert(false);
      return "<Unknown>";
  }
}

const char *HoverPrepTransformGsDownGateToString(
    HoverPrepTransformGsDownGate gate) {
  switch (gate) {
    case kHoverPrepTransformGsDownGateTimeInTransOut:
      return "TimeInTransOut";
    case kHoverPrepTransformGsDownGateGroundStationMode:
      return "GroundStationMode";
    case kHoverPrepTransformGsDownGateForceDetwistTurn:
      return "ForceDetwistTurn";
    default:
    case kHoverPrepTransformGsDownGateForceSigned:
    case kNumHoverPrepTransformGsDownGates:
      assert(false);
      return "<Unknown>";
  }
}

const char *HoverTransformGsDownGateToString(HoverTransformGsDownGate gate) {
  switch (gate) {
    case kHoverTransformGsDownGateTetherElevation:
      return "TetherElevation";
    case kHoverTransformGsDownGateAzimuthError:
      return "AzimuthError";
    case kHoverTransformGsDownGateZError:
      return "ZError";
    case kHoverTransformGsDownGateForceDetwistTurn:
      return "ForceDetwistTurn";
    default:
    case kHoverTransformGsDownGateForceSigned:
    case kNumHoverTransformGsDownGates:
      assert(false);
      return "<Unknown>";
  }
}

const char *HoverReelInGateToString(HoverReelInGate gate) {
  switch (gate) {
    case kHoverReelInGateFlightPlan:
      return "FlightPlan";
    case kHoverReelInGateGroundStationMode:
      return "GroundStationMode";
    default:
    case kHoverReelInGateForceSigned:
    case kNumHoverReelInGates:
      assert(false);
      return "<Unknown>";
  }
}

const char *HoverDescendGateToString(HoverDescendGate gate) {
  switch (gate) {
    case kHoverDescendGateAbovePerch:
      return "AbovePerch";
    case kHoverDescendGateProximity:
      return "Proximity";
    case kHoverDescendGateSpeed:
      return "Speed";
    default:
    case kHoverDescendGateForceSigned:
    case kNumHoverDescendGates:
      assert(false);
      return "<Unknown>";
  }
}

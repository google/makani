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

#include "system/labels_util.h"

#include <assert.h>
#include <stdbool.h>

#include "avionics/common/gps_receiver.h"
#include "avionics/network/aio_labels.h"
#include "avionics/network/aio_node.h"
#include "system/labels.h"

FlapLabel ServoToFlap(ServoLabel servo_label) {
  switch (servo_label) {
    case kServoA1: return kFlapA1;
    case kServoA2: return kFlapA2;
    case kServoA4: return kFlapA4;
    case kServoA5: return kFlapA5;
    case kServoA7: return kFlapA7;
    case kServoA8: return kFlapA8;
    case kServoE1: return kFlapEle;
    case kServoE2: return kFlapEle;
    case kServoR1: return kFlapRud;
    case kServoR2: return kFlapRud;
    default:
    case kServoLabelForceSigned:
    case kNumServos:
      assert(false);
      return kFlapA1;
  }
}

const char *LoadcellSensorLabelToString(LoadcellSensorLabel loadcell_label) {
  switch (loadcell_label) {
    case kLoadcellSensorPort0:
      return "Port0";
    case kLoadcellSensorPort1:
      return "Port1";
    case kLoadcellSensorStarboard0:
      return "Starboard0";
    case kLoadcellSensorStarboard1:
      return "Starboard1";
    default:
    case kLoadcellSensorLabelForceSigned:
    case kNumLoadcellSensors:
      assert(false);
      return "<Unknown>";
  }
}

// TODO: Move this to config/m600/sensor_layout.py
AioNode WingGpsReceiverLabelToAioNode(WingGpsReceiverLabel wing_gps_receiver) {
  switch (wing_gps_receiver) {
    case kWingGpsReceiverCrosswind:
      return kAioNodeFcA;
    case kWingGpsReceiverHover:
      return kAioNodeFcB;
    case kWingGpsReceiverPort:
      return kAioNodeLightPort;
    case kWingGpsReceiverStar:
      return kAioNodeLightStbd;
    case kWingGpsReceiverLabelForceSigned:
    case kNumWingGpsReceivers:
    default:
      assert(false);
      return kAioNodeUnknown;
  }
}

GpsReceiverType WingGpsReceiverLabelToGpsReceiverType(
    WingGpsReceiverLabel wing_gps_receiver) {
  switch (wing_gps_receiver) {
    case kWingGpsReceiverCrosswind:
      return kGpsReceiverTypeNovAtel;
    case kWingGpsReceiverHover:
      return kGpsReceiverTypeNovAtel;
    case kWingGpsReceiverPort:
      return kGpsReceiverTypeNovAtel;
    case kWingGpsReceiverStar:
      return kGpsReceiverTypeNovAtel;
    case kWingGpsReceiverLabelForceSigned:
    case kNumWingGpsReceivers:
    default:
      assert(false);
      return kGpsReceiverTypeNone;
  }
}

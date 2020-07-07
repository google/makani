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

#ifndef SYSTEM_LABELS_H_
#define SYSTEM_LABELS_H_

#include <stdbool.h>

// Flaps A1 through A8 correspond to the main wing flaps,
// i.e. ailerons and center flaps, going from port to starboard.
// Flaps A3 and A6 do not exist because they are fixed on the M600.
typedef enum {
  kFlapLabelForceSigned = -1,
  kFlapA1,
  kFlapA2,
  kFlapA4,
  kFlapA5,
  kFlapA7,
  kFlapA8,
  kFlapEle,
  kFlapRud,
  kNumFlaps
} FlapLabel;

typedef enum {
  kWingImuLabelForceSigned = -1,
  kWingImuA,
  kWingImuB,
  kWingImuC,
  kNumWingImus
} WingImuLabel;

typedef enum {
  kGsImuLabelForceSigned = -1,
  kGsImuA,
  kNumGsImus
} GsImuLabel;

typedef enum {
  kPitotSensorLabelForceSigned = -1,
  kPitotSensorHighSpeed,
  kPitotSensorLowSpeed,
  kNumPitotSensors
} PitotSensorLabel;

typedef enum {
  kWingGpsReceiverLabelForceSigned = -1,
  kWingGpsReceiverCrosswind,
  kWingGpsReceiverHover,
  kWingGpsReceiverPort,
  kWingGpsReceiverStar,
  kNumWingGpsReceivers
} WingGpsReceiverLabel;

typedef enum {
  kJoystickChannelLabelForceSigned = -1,
  kJoystickChannelPitch,
  kJoystickChannelRoll,
  kJoystickChannelYaw,
  kJoystickChannelThrottle,
  kJoystickChannelSwitches,
  kNumJoystickChannels
} JoystickChannelLabel;

typedef enum {
  kJoystickSwitchPositionLabelForceSigned = -1,
  kJoystickSwitchPositionUp,
  kJoystickSwitchPositionMiddle,
  kJoystickSwitchPositionDown,
  kNumJoystickSwitchPositions
} JoystickSwitchPositionLabel;

typedef enum {
  kLoadcellSensorLabelForceSigned = -1,
  kLoadcellSensorPort0,
  kLoadcellSensorPort1,
  kLoadcellSensorStarboard0,
  kLoadcellSensorStarboard1,
  kNumLoadcellSensors
} LoadcellSensorLabel;

typedef enum {
  kProximitySensorLabelForceSigned = -1,
  kProximitySensorEarlyA,
  kProximitySensorEarlyB,
  kProximitySensorFinalA,
  kProximitySensorFinalB,
  kNumProximitySensors
} ProximitySensorLabel;

#endif  // SYSTEM_LABELS_H_

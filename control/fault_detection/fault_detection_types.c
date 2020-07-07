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

#include "control/fault_detection/fault_detection_types.h"

#include <assert.h>
#include <stdbool.h>
#include <stddef.h>

bool HasFault(FaultType fault_type, const FaultMask *fault) {
  assert(fault != NULL && 0 <= fault_type && fault_type <= kNumFaultTypes);
  return fault->code & (1 << fault_type);
}

bool HasAnyFault(const FaultMask *fault) {
  assert(fault != NULL);
  return fault->code;
}

int32_t FaultMaskToInt32(const FaultMask *fault) {
  assert(fault != NULL);
  return fault->code;
}

void FaultMaskFromInt32(int32_t code, FaultMask *fault) {
  assert(fault != NULL && 0 <= code && code < (1 << kNumFaultTypes));
  fault->code = code;
}

const char *FaultTypeToString(FaultType fault_type) {
  switch (fault_type) {
    case kFaultTypeDisabled:
      return "Disabled";
    case kFaultTypeImplausible:
      return "Implausible";
    case kFaultTypeNoUpdate:
      return "NoUpdate";
    case kFaultTypeOutOfRange:
      return "OutOfRange";
    case kFaultTypeThrownError:
      return "ThrownError";
    default:
    case kFaultTypeForceSigned:
    case kNumFaultTypes:
      assert(false);
      return "<Unknown>";
  }
}

const char *SubsystemLabelToString(SubsystemLabel label) {
  switch (label) {
    case kSubsysControllerA:
      return "ControllerA";
    case kSubsysControllerB:
      return "ControllerB";
    case kSubsysControllerC:
      return "ControllerC";
    case kSubsysGroundEstimatorPosition:
      return "GroundEstimatorPosition";
    case kSubsysGroundEstimatorAttitude:
      return "GroundEstimatorAttitude";
    case kSubsysGroundStation:
      return "GroundStation";
    case kSubsysDetwist:
      return "Detwist";
    case kSubsysDrum:
      return "Drum";
    case kSubsysGsAcc:
      return "GsAcc";
    case kSubsysGsGyro:
      return "GsGyro";
    case kSubsysGsMag:
      return "GsMag";
    case kSubsysGsCompass:
      return "GsCompass";
    case kSubsysGsCompassAngles:
      return "GsCompassAngles";
    case kSubsysGsCompassAngularRates:
      return "GsCompassAngularRates";
    case kSubsysGsGpsPos:
      return "GsGpsPos";
    case kSubsysGsGpsVel:
      return "GsGpsVel";
    case kSubsysGsgAAzi:
      return "GsgAAzi";
    case kSubsysGsgAEle:
      return "GsgAEle";
    case kSubsysGsgBAzi:
      return "GsgBAzi";
    case kSubsysGsgBEle:
      return "GsgBEle";
    case kSubsysHvBus:
      return "HvBus";
    case kSubsysImuAAcc:
      return "ImuAAcc";
    case kSubsysImuAGyro:
      return "ImuAGyro";
    case kSubsysImuAMag:
      return "ImuAMag";
    case kSubsysImuBAcc:
      return "ImuBAcc";
    case kSubsysImuBGyro:
      return "ImuBGyro";
    case kSubsysImuBMag:
      return "ImuBMag";
    case kSubsysImuCAcc:
      return "ImuCAcc";
    case kSubsysImuCGyro:
      return "ImuCGyro";
    case kSubsysImuCMag:
      return "ImuCMag";
    case kSubsysJoystick:
      return "Joystick";
    case kSubsysLevelwindEleA:
      return "LevelwindEleA";
    case kSubsysLevelwindEleB:
      return "LevelwindEleB";
    case kSubsysLoadcellSensorPort0:
      return "LoadcellSensorPort0";
    case kSubsysLoadcellSensorPort1:
      return "LoadcellSensorPort1";
    case kSubsysLoadcellSensorStarboard0:
      return "LoadcellSensorStarboard0";
    case kSubsysLoadcellSensorStarboard1:
      return "LoadcellSensorStarboard1";
    case kSubsysMotorSbo:
      return "MotorSbo";
    case kSubsysMotorSbi:
      return "MotorSbi";
    case kSubsysMotorPbi:
      return "MotorPbi";
    case kSubsysMotorPbo:
      return "MotorPbo";
    case kSubsysMotorPto:
      return "MotorPto";
    case kSubsysMotorPti:
      return "MotorPti";
    case kSubsysMotorSti:
      return "MotorSti";
    case kSubsysMotorSto:
      return "MotorSto";
    case kSubsysPerchAziA:
      return "PerchAziA";
    case kSubsysPerchAziB:
      return "PerchAziB";
    case kSubsysPitotSensorHighSpeedStatic:
      return "PitotSensorHighSpeedStatic";
    case kSubsysPitotSensorHighSpeedAlpha:
      return "PitotSensorHighSpeedAlpha";
    case kSubsysPitotSensorHighSpeedBeta:
      return "PitotSensorHighSpeedBeta";
    case kSubsysPitotSensorHighSpeedDynamic:
      return "PitotSensorHighSpeedDynamic";
    case kSubsysPitotSensorLowSpeedStatic:
      return "PitotSensorLowSpeedStatic";
    case kSubsysPitotSensorLowSpeedAlpha:
      return "PitotSensorLowSpeedAlpha";
    case kSubsysPitotSensorLowSpeedBeta:
      return "PitotSensorLowSpeedBeta";
    case kSubsysPitotSensorLowSpeedDynamic:
      return "PitotSensorLowSpeedDynamic";
    case kSubsysProximitySensor:
      return "ProximitySensor";
    case kSubsysServoA1:
      return "ServoA1";
    case kSubsysServoA2:
      return "ServoA2";
    case kSubsysServoA4:
      return "ServoA4";
    case kSubsysServoA5:
      return "ServoA5";
    case kSubsysServoA7:
      return "ServoA7";
    case kSubsysServoA8:
      return "ServoA8";
    case kSubsysServoE1:
      return "ServoE1";
    case kSubsysServoE2:
      return "ServoE2";
    case kSubsysServoR1:
      return "ServoR1";
    case kSubsysServoR2:
      return "ServoR2";
    case kSubsysServoTetherDetwist:
      return "ServoTetherDetwist";
    case kSubsysTetherRelease:
      return "TetherRelease";
    case kSubsysWeather:
      return "Weather";
    case kSubsysWinch:
      return "Winch";
    case kSubsysWindSensor:
      return "WindSensor";
    case kSubsysWingGpsCrosswindPos:
      return "WingGpsCrosswindPos";
    case kSubsysWingGpsCrosswindVel:
      return "WingGpsCrosswindVel";
    case kSubsysWingGpsHoverPos:
      return "WingGpsHoverPos";
    case kSubsysWingGpsHoverVel:
      return "WingGpsHoverVel";
    case kSubsysWingGpsPortPos:
      return "WingGpsPortPos";
    case kSubsysWingGpsPortVel:
      return "WingGpsPortVel";
    case kSubsysWingGpsStarPos:
      return "WingGpsStarPos";
    case kSubsysWingGpsStarVel:
      return "WingGpsStarVel";
    default:
    case kNumSubsystems:
      assert(false);
      return "<Unknown>";
  }
}

const FaultMask *GetImuFaults(const FaultMask all_faults[],
                              WingImuLabel label) {
  switch (label) {
    default:
    case kNumWingImus:
    case kWingImuLabelForceSigned:
      assert(false);
    // Fall-through intentional.
    case kWingImuA:
      return &all_faults[SUBSYS_IMU_A];
    case kWingImuB:
      return &all_faults[SUBSYS_IMU_B];
    case kWingImuC:
      return &all_faults[SUBSYS_IMU_C];
  }
}

FaultMask *GetWingGpsSubsysFaults(FaultMask all_faults[],
                                  WingGpsReceiverLabel label) {
  switch (label) {
    default:
    case kNumWingGpsReceivers:
    case kWingGpsReceiverLabelForceSigned:
      assert(false);
    // Fall-through intentional.
    case kWingGpsReceiverCrosswind:
      return &all_faults[SUBSYS_WING_GPS_CROSSWIND];
    case kWingGpsReceiverHover:
      return &all_faults[SUBSYS_WING_GPS_HOVER];
    case kWingGpsReceiverPort:
      return &all_faults[SUBSYS_WING_GPS_PORT];
    case kWingGpsReceiverStar:
      return &all_faults[SUBSYS_WING_GPS_STAR];
  }
}

const FaultMask *GetWingGpsPosFault(const FaultMask all_faults[],
                                    WingGpsReceiverLabel label) {
  switch (label) {
    default:
    case kNumWingGpsReceivers:
    case kWingGpsReceiverLabelForceSigned:
      assert(false);
    // Fall-through intentional.
    case kWingGpsReceiverCrosswind:
      return &all_faults[kSubsysWingGpsCrosswindPos];
    case kWingGpsReceiverHover:
      return &all_faults[kSubsysWingGpsHoverPos];
    case kWingGpsReceiverPort:
      return &all_faults[kSubsysWingGpsPortPos];
    case kWingGpsReceiverStar:
      return &all_faults[kSubsysWingGpsStarPos];
  }
}

const FaultMask *GetWingGpsVelFault(const FaultMask all_faults[],
                                    WingGpsReceiverLabel label) {
  switch (label) {
    default:
    case kNumWingGpsReceivers:
    case kWingGpsReceiverLabelForceSigned:
      assert(false);
    // Fall-through intentional.
    case kWingGpsReceiverCrosswind:
      return &all_faults[kSubsysWingGpsCrosswindVel];
    case kWingGpsReceiverHover:
      return &all_faults[kSubsysWingGpsHoverVel];
    case kWingGpsReceiverPort:
      return &all_faults[kSubsysWingGpsPortVel];
    case kWingGpsReceiverStar:
      return &all_faults[kSubsysWingGpsStarVel];
  }
}

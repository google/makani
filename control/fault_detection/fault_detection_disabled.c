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

#include "control/fault_detection/fault_detection_disabled.h"

#include <stdint.h>

#include "common/macros.h"
#include "control/fault_detection/fault_detection_types.h"
#include "control/fault_detection/fault_detection_util.h"
#include "control/system_params.h"
#include "control/system_types.h"

void SetDisabledFaults(const FaultDetectionDisabledParams *params,
                       FaultMask faults[]) {
  for (int32_t i = 0; i < kNumControllers; ++i) {
    SetFault(kFaultTypeDisabled, params->controllers[i],
             &faults[SUBSYS_CONTROLLERS + i]);
  }

  SetFault(kFaultTypeDisabled, params->gs02, &faults[kSubsysGroundStation]);

  for (int32_t i = 0; i < kNumFaultDetectionImuSignals; ++i) {
    SetFault(kFaultTypeDisabled, params->imus[kWingImuA],
             &faults[SUBSYS_IMU_A + i]);
    SetFault(kFaultTypeDisabled, params->imus[kWingImuB],
             &faults[SUBSYS_IMU_B + i]);
    SetFault(kFaultTypeDisabled, params->imus[kWingImuC],
             &faults[SUBSYS_IMU_C + i]);
  }

  for (int32_t i = 0; i < kNumLoadcellSensors; ++i) {
    SetFault(kFaultTypeDisabled, params->loadcells,
             &faults[SUBSYS_LOADCELLS + i]);
  }

  const SubsystemLabel gsg_subsystems[] = {SUBSYS_GSG_A, SUBSYS_GSG_B};
  assert(ARRAYSIZE(gsg_subsystems) == kNumDrums);
  for (int32_t i = 0; i < kNumDrums; ++i) {
    SetFault(kFaultTypeDisabled, params->gsg_azimuth[i],
             &faults[gsg_subsystems[i] + kFaultDetectionGsgSignalAzi]);
    SetFault(kFaultTypeDisabled, params->gsg_elevation[i],
             &faults[gsg_subsystems[i] + kFaultDetectionGsgSignalEle]);
  }

  assert(kNumPlatforms == 2);
  SetFault(kFaultTypeDisabled, params->levelwind_ele[kPlatformSensorsA],
           &faults[kSubsysLevelwindEleA]);
  SetFault(kFaultTypeDisabled, params->levelwind_ele[kPlatformSensorsB],
           &faults[kSubsysLevelwindEleB]);
  SetFault(kFaultTypeDisabled, params->perch_azi[kPlatformSensorsA],
           &faults[kSubsysPerchAziA]);
  SetFault(kFaultTypeDisabled, params->perch_azi[kPlatformSensorsB],
           &faults[kSubsysPerchAziB]);
  SetFault(kFaultTypeDisabled, params->drum, &faults[kSubsysDrum]);
  SetFault(kFaultTypeDisabled, params->detwist, &faults[kSubsysDetwist]);

  SetFault(kFaultTypeDisabled, params->gs_compass, &faults[kSubsysGsCompass]);

  for (int32_t i = 0; i < kNumFaultDetectionPitotSignals; ++i) {
    SetFault(kFaultTypeDisabled, params->pitot,
             &faults[SUBSYS_PITOT_SENSOR_HIGH_SPEED + i]);
    SetFault(kFaultTypeDisabled, params->pitot,
             &faults[SUBSYS_PITOT_SENSOR_LOW_SPEED + i]);
  }

  SetFault(kFaultTypeDisabled, params->proximity_sensor,
           &faults[kSubsysProximitySensor]);

  SetFault(kFaultTypeDisabled, params->winch, &faults[kSubsysWinch]);

  for (WingGpsReceiverLabel label = kWingGpsReceiverCrosswind;
       label < kNumWingGpsReceivers; ++label) {
    FaultMask *fault_mask;
    switch (label) {
      default:
      case kNumWingGpsReceivers:
      case kWingGpsReceiverLabelForceSigned:
        assert(false);
        fault_mask = 0;
        break;
      case kWingGpsReceiverCrosswind:
        fault_mask = &faults[SUBSYS_WING_GPS_CROSSWIND];
        break;
      case kWingGpsReceiverHover:
        fault_mask = &faults[SUBSYS_WING_GPS_HOVER];
        break;
      case kWingGpsReceiverPort:
        fault_mask = &faults[SUBSYS_WING_GPS_PORT];
        break;
      case kWingGpsReceiverStar:
        fault_mask = &faults[SUBSYS_WING_GPS_STAR];
        break;
    }
    if (fault_mask) {
      // Disable GPS pos fault.
      SetFault(kFaultTypeDisabled, params->wing_gps[(int32_t)label],
               fault_mask);
      // Disable GPS vel fault.
      SetFault(kFaultTypeDisabled, params->wing_gps[(int32_t)label],
               fault_mask + 1);
    }
  }
}

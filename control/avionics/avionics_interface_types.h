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

#ifndef CONTROL_AVIONICS_AVIONICS_INTERFACE_TYPES_H_
#define CONTROL_AVIONICS_AVIONICS_INTERFACE_TYPES_H_

#include <stdbool.h>
#include <stdint.h>

#include "avionics/common/avionics_messages.h"
#include "avionics/common/tether_message_types.h"
#include "avionics/network/aio_labels.h"
#include "common/c_math/vec3.h"
#include "control/fault_detection/fault_detection_types.h"
#include "control/system_types.h"
#include "system/labels.h"

typedef struct {
  ControllerSyncMessage controller_sync[kNumControllers];
  FlightComputerImuMessage flight_comp_imus[kNumFlightComputers];
  FlightComputerSensorMessage flight_comp_sensors[kNumFlightComputers];
  JoystickStatusMessage joystick;
  LoadcellMessage loadcell_messages[kNumLoadcellNodes];
  MotorStatusMessage motor_statuses[kNumMotors];
  ServoStatusMessage servo_statuses[kNumServos];
  TetherUpMessage tether_up_messages[kNumTetherUpSources];
  NovAtelSolutionMessage wing_gps_novatel[kNumWingGpsReceivers];
  SeptentrioSolutionMessage wing_gps_septentrio[kNumWingGpsReceivers];
  GroundEstimateMessage ground_estimate;
} ControlInputMessages;

// TODO: Replace the hard-coded size of ground_comp_imus and
// ground_comp_sensors by the call of an enum.
typedef struct {
  FlightComputerImuMessage ground_comp_imus[1];
  FlightComputerSensorMessage ground_comp_sensors[1];
  NovAtelCompassMessage ground_compass[1];
  NovAtelSolutionMessage ground_gps[1];
} GroundEstimatorInputMessages;

typedef struct {
  bool controller_sync[kNumControllers];
  bool flight_comp_imus[kNumFlightComputers];
  bool flight_comp_sensors[kNumFlightComputers];
  bool joystick;
  bool loadcell_messages[kNumLoadcellNodes];
  bool motor_statuses[kNumMotors];
  bool servo_statuses[kNumServos];
  bool tether_up_messages[kNumTetherUpSources];
  bool wing_gps_novatel[kNumWingGpsReceivers];
  bool wing_gps_septentrio[kNumWingGpsReceivers];
  bool ground_estimate;
} ControlInputMessagesUpdated;

typedef struct {
  bool ground_comp_imus[1];
  bool ground_comp_sensors[1];
  bool ground_compass[1];
  bool ground_gps[1];
} GroundEstimatorInputMessagesUpdated;

typedef struct {
  uint16_t controller_sync[kNumControllers];
  uint16_t flight_comp_imus[kNumFlightComputers];
  uint16_t flight_comp_sensors[kNumFlightComputers];
  uint16_t loadcell_messages[kNumLoadcellNodes];
  uint16_t motor_statuses[kNumMotors];
  uint16_t servo_statuses[kNumServos];
  uint16_t wing_gps[kNumWingGpsReceivers];
  uint16_t ground_estimate;
} AvionicsSequenceNumbers;

typedef struct {
  int32_t num_no_updates;
  uint8_t sequence_z1;
  bool initialized;
} AvionicsFaultsControllerSyncState;

typedef struct {
  int32_t position_num_no_updates;
  int32_t attitude_num_no_updates;
} AvionicsFaultsGroundEstimatorState;

typedef struct { int32_t num_no_updates; } AvionicsFaultsGroundStationState;

typedef struct { int32_t num_no_updates; } AvionicsFaultsGsCompassState;

typedef struct { int32_t num_no_updates; } AvionicsFaultsGsGpsState;

typedef struct {
  int32_t num_no_updates[kNumFaultDetectionGsgSignals];
} AvionicsFaultsGsgState;

typedef struct {
  int32_t num_no_updates[kNumFaultDetectionImuSignals][3];
  Vec3 acc_z1;
  Vec3 gyro_z1;
  Vec3 mag_z1;
  bool initialized[kNumFaultDetectionImuSignals];
} AvionicsFaultsImuState;

typedef struct { int32_t num_no_updates; } AvionicsFaultsJoystickState;

typedef struct { int32_t num_no_updates; } AvionicsFaultsLevelwindEleState;

typedef struct {
  int32_t num_no_updates[kNumLoadcellNodes];
} AvionicsFaultsLoadcellsState;

typedef struct { int32_t num_no_updates; } AvionicsFaultsMotorState;

typedef struct { int32_t num_no_updates; } AvionicsFaultsPitotState;

typedef struct { int32_t num_no_updates; } AvionicsFaultsPerchAziState;

typedef struct { int32_t num_no_updates; } AvionicsFaultsProximitySensorState;

typedef struct { int32_t num_no_updates; } AvionicsFaultsWeatherState;

typedef struct { int32_t num_no_updates; } AvionicsFaultsWinchSensorState;

typedef struct { int32_t num_no_updates; } AvionicsFaultsWindSensorState;

typedef struct {
  int32_t num_no_updates[kNumFaultDetectionGpsSignals][3];
  int32_t time_of_week_z1;
  Vec3 pos_z1;
  Vec3 vel_z1;
  bool initialized;
} AvionicsFaultsGpsState;

typedef struct {
  AvionicsFaultsControllerSyncState controller_sync[kNumControllers];
  AvionicsFaultsGroundEstimatorState ground_estimate;
  AvionicsFaultsGroundStationState ground_station;
  AvionicsFaultsGsCompassState gs_compass;
  AvionicsFaultsGsGpsState gs_gps;
  AvionicsFaultsGsgState gsg[kNumDrums];
  AvionicsFaultsImuState imus[kNumWingImus];
  AvionicsFaultsJoystickState joystick;
  AvionicsFaultsLevelwindEleState levelwind_ele[kNumPlatforms];
  AvionicsFaultsLoadcellsState loadcells;
  AvionicsFaultsMotorState motors[kNumMotors];
  AvionicsFaultsPerchAziState perch_azi[kNumPlatforms];
  AvionicsFaultsPitotState pitots[kNumPitotSensors];
  AvionicsFaultsProximitySensorState proximity_sensor;
  AvionicsFaultsWeatherState weather;
  AvionicsFaultsWinchSensorState winch_sensor;
  AvionicsFaultsWindSensorState wind_sensor;
  AvionicsFaultsGpsState wing_gps[kNumWingGpsReceivers];
} AvionicsFaultsState;

typedef struct {
  AvionicsFaultsState faults_state;
  int32_t last_used_gs_gps_position_no_update_count;
  uint16_t last_used_gs_gps_position_seq;
  int32_t last_used_gs_gps_status_no_update_count;
  uint16_t last_used_gs_gps_status_seq;
  AvionicsSequenceNumbers sequence_numbers;
  TetherUpMergeState tether_up_merge_state;
} AvionicsInterfaceState;

typedef struct {
  uint16_t ground_comp_imu;
  uint16_t ground_comp_sensor;
  uint16_t ground_compass;
  uint16_t ground_gps;
} GroundvionicsSequenceNumbers;

typedef struct {
  AvionicsFaultsGsCompassState gs_compass;
  AvionicsFaultsGpsState gs_gps;
  AvionicsFaultsImuState imu;
} GroundvionicsFaultsState;

typedef struct {
  GroundvionicsFaultsState faults_state;
  int32_t last_used_gs_gps_position_no_update_count;
  uint16_t last_used_gs_gps_position_seq;
  int32_t last_used_gs_gps_status_no_update_count;
  uint16_t last_used_gs_gps_status_seq;
  GroundvionicsSequenceNumbers sequence_numbers;
} GroundvionicsInterfaceState;

#ifdef __cplusplus
extern "C" {
#endif

// Accessors for individual strain readings.
float GetStrain(const LoadcellMessage messages[], const StrainLocation *loc);
float *GetMutableStrain(LoadcellMessage messages[], const StrainLocation *loc);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // CONTROL_AVIONICS_AVIONICS_INTERFACE_TYPES_H_

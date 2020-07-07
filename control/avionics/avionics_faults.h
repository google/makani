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

#ifndef CONTROL_AVIONICS_AVIONICS_FAULTS_H_
#define CONTROL_AVIONICS_AVIONICS_FAULTS_H_

#include <stdbool.h>

#include "avionics/common/avionics_messages.h"
#include "control/avionics/avionics_interface_types.h"
#include "control/control_types.h"
#include "control/fault_detection/fault_detection_types.h"

#ifdef __cplusplus
extern "C" {
#endif

// TODO: Reimplement CheckMotorFaults and CheckServoFaults for the
// M600.  The old versions still exist as of commit 6e2dc901.

// Initializes state for the avionics interface's fault detection.
void AvionicsFaultsInit(AvionicsFaultsState *state);

void GroundvionicsFaultsInit(GroundvionicsFaultsState *state);

void AvionicsFaultsCheckControllerSync(
    const ControlSyncData *sync, bool cvt_updated,
    const FaultDetectionControllerParams *params,
    AvionicsFaultsControllerSyncState *state, FaultMask *fault);

void AvionicsFaultsCheckGsGps(const TetherGsGpsPosition *position,
                              const TetherGpsStatus *status, bool updated,
                              const FaultDetectionGsGpsParams *params,
                              AvionicsFaultsGsGpsState *state,
                              FaultMask *fault);

void AvionicsFaultsCheckGsg(const TetherDrum *status, bool status_updated,
                            const GsgData *gsg,
                            const FaultDetectionGsgParams *params,
                            AvionicsFaultsGsgState *state, FaultMask faults[]);

void AvionicsFaultsCheckWingGpsNovAtel(
    const NovAtelSolutionMessage *gps_message, bool initializing,
    const FaultDetectionGpsParams *params, AvionicsFaultsGpsState *state,
    FaultMask faults[], GpsData *gps);

void AvionicsFaultsCheckWingGpsSeptentrio(
    const SeptentrioSolutionMessage *gps_message, bool initializing,
    const FaultDetectionGpsParams *params, AvionicsFaultsGpsState *state,
    FaultMask faults[], GpsData *gps);

void AvionicsFaultsCheckNovAtelCompass(
    const NovAtelCompassMessage *novatel_compass, bool cvt_updated,
    const FaultDetectionGsCompassParams *params, FaultMask faults[],
    GpsCompassData *compass);

void AvionicsFaultsCheckGroundEstimate(
    const GroundEstimateMessage *ground_estimate, bool updated,
    const FaultDetectionGroundEstimatorParams *params,
    AvionicsFaultsGroundEstimatorState *state, FaultMask *fault);

void AvionicsFaultsCheckGroundStation(
    const TetherGroundStation *ground_station, bool updated,
    const FaultDetectionGroundStationParams *params,
    AvionicsFaultsGroundStationState *state, FaultMask *fault);

void AvionicsFaultsCheckDetwist(const TetherGroundStation *ground_station,
                                FaultMask *fault);

void AvionicsFaultsCheckDrum(const TetherGroundStation *ground_station,
                             FaultMask *fault);

void AvionicsFaultsCheckGsCompass(const TetherGsGpsCompass *compass,
                                  bool updated,
                                  const FaultDetectionGsCompassParams *params,
                                  AvionicsFaultsGsCompassState *state,
                                  FaultMask *fault);

void AvionicsFaultsCheckImuAccGyro(const FlightComputerImuMessage *imu_sensor,
                                   const ImuData *imu, bool cvt_updated,
                                   const FaultDetectionImuParams *params,
                                   AvionicsFaultsImuState *state,
                                   FaultMask faults[]);

void AvionicsFaultsCheckImuMag(const FlightComputerSensorMessage *mag_sensor,
                               const ImuData *imu, bool cvt_updated,
                               const FaultDetectionImuParams *params,
                               AvionicsFaultsImuState *state,
                               FaultMask faults[]);

void AvionicsFaultsCheckJoystick(const TetherJoystick *joystick, bool updated,
                                 const FaultDetectionJoystickParams *params,
                                 AvionicsFaultsJoystickState *state,
                                 FaultMask *fault);

void AvionicsFaultsCheckLevelwindEle(
    const TetherPlatform *platform_sensors, bool platform_sensors_updated,
    const FaultDetectionLevelwindEleParams *params,
    AvionicsFaultsLevelwindEleState *state, FaultMask *fault);

void AvionicsFaultsCheckLoadcells(const bool loadcell_messages_updated[],
                                  const LoadcellParams loadcell_params[],
                                  const FaultDetectionLoadcellParams *params,
                                  AvionicsFaultsLoadcellsState *state,
                                  FaultMask sensor_faults[],
                                  bool nodes_faulted[],
                                  FaultMask *tether_release_fault);

void AvionicsFaultsCheckMotor(const MotorStatusMessage *motor_status,
                              bool updated,
                              const FaultDetectionMotorParams *params,
                              AvionicsFaultsMotorState *state,
                              FaultMask *fault);

void AvionicsFaultsCheckPerchAzi(const TetherPlatform *platform_sensors,
                                 bool platform_sensors_updated,
                                 const FaultDetectionPerchAziParams *params,
                                 AvionicsFaultsPerchAziState *state,
                                 FaultMask *fault);

void AvionicsFaultsCheckPitot(const PitotSensor *pitot_sensor, bool updated,
                              const FaultDetectionPitotParams *params,
                              const StatusFlags *flags,
                              AvionicsFaultsPitotState *state,
                              FaultMask faults[]);

void AvionicsFaultsCheckWeather(const TetherWeather *weather,
                                bool weather_updated,
                                const FaultDetectionWeatherParams *params,
                                const GsWeather *min, const GsWeather *max,
                                AvionicsFaultsWeatherState *state,
                                FaultMask *fault);

void AvionicsFaultsCheckWinchSensor(const TetherPlc *plc_status,
                                    bool plc_status_updated,
                                    const FaultDetectionWinchParams *params,
                                    AvionicsFaultsWinchSensorState *state,
                                    FaultMask *fault);

void AvionicsFaultsCheckWindSensor(const TetherWind *wind_sensor,
                                   bool wind_sensor_updated,
                                   const FaultDetectionWindSensorParams *params,
                                   AvionicsFaultsWindSensorState *state,
                                   FaultMask *fault);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // CONTROL_AVIONICS_AVIONICS_FAULTS_H_

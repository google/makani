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

#ifndef CONTROL_FAULT_DETECTION_FAULT_DETECTION_TYPES_H_
#define CONTROL_FAULT_DETECTION_FAULT_DETECTION_TYPES_H_

#include <stdbool.h>
#include <stdint.h>

#include "avionics/network/aio_labels.h"
#include "control/sensor_types.h"
#include "system/labels.h"

// Enumerate subsystems to be tested for faults.
typedef enum {
  kSubsysControllerA,
  kSubsysControllerB,
  kSubsysControllerC,
  kSubsysGroundEstimatorPosition,
  kSubsysGroundEstimatorAttitude,
  kSubsysGroundStation,
  kSubsysDetwist,
  kSubsysDrum,
  kSubsysGsAcc,
  kSubsysGsGyro,
  kSubsysGsMag,
  kSubsysGsCompass,
  kSubsysGsCompassAngles,
  kSubsysGsCompassAngularRates,
  kSubsysGsGpsPos,
  kSubsysGsGpsVel,
  kSubsysGsgAAzi,
  kSubsysGsgAEle,
  kSubsysGsgBAzi,
  kSubsysGsgBEle,
  kSubsysHvBus,
  kSubsysImuAAcc,
  kSubsysImuAGyro,
  kSubsysImuAMag,
  kSubsysImuBAcc,
  kSubsysImuBGyro,
  kSubsysImuBMag,
  kSubsysImuCAcc,
  kSubsysImuCGyro,
  kSubsysImuCMag,
  kSubsysJoystick,
  kSubsysLevelwindEleA,
  kSubsysLevelwindEleB,
  kSubsysLoadcellSensorPort0,
  kSubsysLoadcellSensorPort1,
  kSubsysLoadcellSensorStarboard0,
  kSubsysLoadcellSensorStarboard1,
  kSubsysMotorSbo,
  kSubsysMotorSbi,
  kSubsysMotorPbi,
  kSubsysMotorPbo,
  kSubsysMotorPto,
  kSubsysMotorPti,
  kSubsysMotorSti,
  kSubsysMotorSto,
  kSubsysPerchAziA,
  kSubsysPerchAziB,
  kSubsysPitotSensorHighSpeedStatic,
  kSubsysPitotSensorHighSpeedAlpha,
  kSubsysPitotSensorHighSpeedBeta,
  kSubsysPitotSensorHighSpeedDynamic,
  kSubsysPitotSensorLowSpeedStatic,
  kSubsysPitotSensorLowSpeedAlpha,
  kSubsysPitotSensorLowSpeedBeta,
  kSubsysPitotSensorLowSpeedDynamic,
  kSubsysProximitySensor,
  kSubsysServoA1,
  kSubsysServoA2,
  kSubsysServoA4,
  kSubsysServoA5,
  kSubsysServoA7,
  kSubsysServoA8,
  kSubsysServoE1,
  kSubsysServoE2,
  kSubsysServoR1,
  kSubsysServoR2,
  kSubsysServoTetherDetwist,
  kSubsysTetherRelease,
  kSubsysWinch,
  kSubsysWindSensor,
  kSubsysWingGpsCrosswindPos,
  kSubsysWingGpsCrosswindVel,
  kSubsysWingGpsHoverPos,
  kSubsysWingGpsHoverVel,
  kSubsysWingGpsPortPos,
  kSubsysWingGpsPortVel,
  kSubsysWingGpsStarPos,
  kSubsysWingGpsStarVel,
  kSubsysWeather,
  kNumSubsystems
} SubsystemLabel;
#define SUBSYS_CONTROLLERS kSubsysControllerA
#define SUBSYS_DRUM kSubsys
#define SUBSYS_GSG_A kSubsysGsgAAzi
#define SUBSYS_GSG_B kSubsysGsgBAzi
#define SUBSYS_IMU_A kSubsysImuAAcc
#define SUBSYS_IMU_B kSubsysImuBAcc
#define SUBSYS_IMU_C kSubsysImuCAcc
#define SUBSYS_LOADCELLS kSubsysLoadcellSensorPort0
#define SUBSYS_MOTORS kSubsysMotorSbo
#define SUBSYS_PITOT_SENSOR_HIGH_SPEED kSubsysPitotSensorHighSpeedStatic
#define SUBSYS_PITOT_SENSOR_LOW_SPEED kSubsysPitotSensorLowSpeedStatic
#define SUBSYS_WING_GPS_CROSSWIND kSubsysWingGpsCrosswindPos
#define SUBSYS_WING_GPS_HOVER kSubsysWingGpsHoverPos
#define SUBSYS_WING_GPS_PORT kSubsysWingGpsPortPos
#define SUBSYS_WING_GPS_STAR kSubsysWingGpsStarPos

typedef struct {
  // Bit mask indicating what fault types are active for a subsystem.
  int32_t code;
} FaultMask;

typedef enum {
  kFaultTypeForceSigned = -1,
  kFaultTypeDisabled,
  kFaultTypeImplausible,
  kFaultTypeNoUpdate,
  kFaultTypeOutOfRange,
  kFaultTypeThrownError,
  kNumFaultTypes
} FaultType;

typedef enum {
  kFaultDetectonPitotSignalTypeForceSigned = -1,
  kFaultDetectionPitotSignalStatic,
  kFaultDetectionPitotSignalAlpha,
  kFaultDetectionPitotSignalBeta,
  kFaultDetectionPitotSignalDynamic,
  kNumFaultDetectionPitotSignals
} FaultDetectionPitotSignalType;

typedef enum {
  kFaultDetectionGpsSignalTypeForceSigned = -1,
  kFaultDetectionGpsSignalPos,
  kFaultDetectionGpsSignalVel,
  kNumFaultDetectionGpsSignals
} FaultDetectionGpsSignalType;

typedef enum {
  kFaultDetectionGpsCompassSignalTypeForceSigned = -1,
  kFaultDetectionGpsCompassSignalAngles,
  kFaultDetectionGpsCompassSignalAngularRates,
  kNumFaultDetectionGpsCompassSignals
} FaultDetectionGpsCompassSignalType;

typedef enum {
  kFaultDetectionGsgSignalTypeForceSigned = -1,
  kFaultDetectionGsgSignalAzi,
  kFaultDetectionGsgSignalEle,
  kNumFaultDetectionGsgSignals
} FaultDetectionGsgSignalType;

typedef enum {
  kFaultDetectionImuSignalTypeForceSigned = -1,
  kFaultDetectionImuSignalAcc,
  kFaultDetectionImuSignalGyro,
  kFaultDetectionImuSignalMag,
  kNumFaultDetectionImuSignals
} FaultDetectionImuSignalType;

typedef enum {
  kFaultDetectionGroundStationEstimatorSignalTypeForceSigned = -1,
  kFaultDetectionGroundStationEstimatorSignalPosition,
  kFaultDetectionGroundStationEstimatorSignalAttitude,
  kNumFaultDetectionGroundStationEstimatorSignals
} FaultDetectionGroundStationEstimatorSignalType;

typedef struct {
  int32_t no_update_counts_limit;
} FaultDetectionControllerParams;

typedef struct {
  int32_t no_update_counts_limit;
} FaultDetectionGroundEstimatorParams;

typedef struct {
  int32_t no_update_counts_limit;
} FaultDetectionGroundStationParams;

typedef struct {
  int32_t no_update_counts_limit;
  double max_latency;
} FaultDetectionGsCompassParams;

typedef struct {
  int32_t no_update_counts_limit;
  double pos_sigma_max;
} FaultDetectionGsGpsParams;

typedef struct {
  int32_t no_update_counts_limit[kNumFaultDetectionGsgSignals];
  double signal_min[kNumFaultDetectionGsgSignals];
  double signal_max[kNumFaultDetectionGsgSignals];
} FaultDetectionGsgParams;

typedef struct {
  double max_latency;
  int32_t no_update_counts_limits[kNumFaultDetectionImuSignals];
  double mag_min_plausible;  // [gauss]
  double mag_max_plausible;  // [gauss]
  double mag_max_latency;
} FaultDetectionImuParams;

typedef struct { int32_t no_update_counts_limit; } FaultDetectionJoystickParams;

typedef struct {
  int32_t no_update_counts_limit;
} FaultDetectionLevelwindEleParams;

typedef struct { int32_t no_update_counts_limit; } FaultDetectionLoadcellParams;

typedef struct { int32_t no_update_counts_limit; } FaultDetectionMotorParams;

typedef struct { int32_t no_update_counts_limit; } FaultDetectionPerchAziParams;

typedef struct {
  int32_t no_update_counts_limit;
  double max_latency;  // [s]
} FaultDetectionPitotParams;

typedef struct {
  int32_t no_update_counts_limit;
} FaultDetectionProximitySensorParams;

typedef struct { int32_t no_update_counts_limit; } FaultDetectionWinchParams;

typedef struct { int32_t no_update_counts_limit; } FaultDetectionWeatherParams;

typedef struct {
  int32_t no_update_counts_limit;
} FaultDetectionWindSensorParams;

typedef struct {
  int32_t no_update_counts_limit[kNumFaultDetectionGpsSignals];
  double max_latency;
} FaultDetectionGpsParams;

// A true value in this structure indicates that a sensor is disabled.
typedef struct {
  bool controllers[kNumControllers];
  bool gs_compass;
  bool gsg_azimuth[kNumDrums];
  bool gsg_elevation[kNumDrums];
  bool gs02;
  bool detwist;
  bool drum;
  bool imus[kNumWingImus];
  bool levelwind_ele[kNumPlatforms];
  bool loadcells;
  bool perch_azi[kNumPlatforms];
  bool pitot;
  bool proximity_sensor;
  bool winch;
  bool wing_gps[kNumWingGpsReceivers];
} FaultDetectionDisabledParams;

typedef struct {
  FaultDetectionControllerParams control;
  FaultDetectionDisabledParams disabled;
  FaultDetectionGroundEstimatorParams ground_estimator;
  FaultDetectionGroundStationParams ground_station;
  FaultDetectionGsCompassParams gs_compass;
  FaultDetectionGsGpsParams gs_gps;
  FaultDetectionGsgParams gsg;
  FaultDetectionImuParams imu;
  FaultDetectionJoystickParams joystick;
  FaultDetectionLevelwindEleParams levelwind_ele;
  FaultDetectionLoadcellParams loadcell;
  FaultDetectionMotorParams motor;
  FaultDetectionPerchAziParams perch_azi;
  FaultDetectionPitotParams pitot;
  FaultDetectionProximitySensorParams proximity_sensor;
  FaultDetectionWeatherParams weather;
  FaultDetectionWinchParams winch;
  FaultDetectionWindSensorParams wind;
  FaultDetectionGpsParams wing_gps;
} FaultDetectionParams;

#ifdef __cplusplus
extern "C" {
#endif

// Return true if a fault code indicates the given fault.
bool HasFault(FaultType fault_type, const FaultMask *fault);

// Return true if any fault is indicated by a given fault code.
bool HasAnyFault(const FaultMask *fault);

int32_t FaultMaskToInt32(const FaultMask *fault);
void FaultMaskFromInt32(int32_t code, FaultMask *fault);

const char *FaultTypeToString(FaultType fault_type);

const char *SubsystemLabelToString(SubsystemLabel label);

const FaultMask *GetImuFaults(const FaultMask all_faults[], WingImuLabel label);

FaultMask *GetWingGpsSubsysFaults(FaultMask all_faults[],
                                  WingGpsReceiverLabel label);
const FaultMask *GetWingGpsPosFault(const FaultMask all_faults[],
                                    WingGpsReceiverLabel label);
const FaultMask *GetWingGpsVelFault(const FaultMask all_faults[],
                                    WingGpsReceiverLabel label);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // CONTROL_FAULT_DETECTION_FAULT_DETECTION_TYPES_H_

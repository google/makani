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

#ifndef AVIONICS_COMMON_PLC_MESSAGES_H_
#define AVIONICS_COMMON_PLC_MESSAGES_H_

#include <stdbool.h>
#include <stdint.h>

#include "avionics/common/faults.h"

// This file specifies message types communicated between the tophat/GS02 PLC
// and the PLC AIO node. It does not specify AIO message types. Updating this
// file means updating the PLC.

typedef enum {
  kGroundStationModeManual = 0,
  kGroundStationModeHighTension = 1,
  kGroundStationModeReel = 2,
  kGroundStationModeTransform = 3,
  kNumGroundStationModes = 4
} GroundStationMode;

typedef enum {
  kDetwistCommandNone = 0,
  kDetwistCommandMove = 0x005A,
  kDetwistCommandReference = 0x00C3,
  kDetwistCommandPopError = 0x0100,
  kDetwistCommandClearError = 0x0200,
  kDetwistCommandClearWarning = 0x0400
} DetwistCommand;

typedef enum {
  kGs02CommandPopError      = (1 << 0),
  kGs02CommandClearErrors   = (1 << 1),
  kGs02CommandClearWarnings = (1 << 2)
} Gs02Command;

typedef enum {
  kPlcMessageTypeStatus = 1,
  kPlcMessageTypeCommand = 2,
  kPlcMessageTypeGs02Input = 3,
  kPlcMessageTypeGs02Status = 4
} PlcMessageType;

typedef enum {
  kPlcErrorFlagDetwistServoABad = (1 << 0),
  kPlcErrorFlagDetwistServoBBad = (1 << 1),
  kPlcErrorFlagDetwistCmdOutage = (1 << 2),
  kPlcErrorFlagDetwistCmdJump = (1 << 3),
  kPlcErrorFlagDetwistServoOvertemp = (1 << 4)
} PlcErrorFlag;

typedef enum {
  kPlcWarningFlagDetwistCmdSequence = (1 << 0),
  kPlcWarningFlagDetwistCmdOutage = (1 << 1),
  kPlcWarningFlagDetwistCmdJump = (1 << 2)
} PlcWarningFlag;

typedef enum {
  kPlcInfoFlagEstopped = (1 << 0),
  kPlcInfoFlagPowerReady = (1 << 1),
  kPlcInfoFlagPowerOn = (1 << 2),
  kPlcInfoFlagDetwistEnabled = (1 << 4),
  kPlcInfoFlagDetwistReferenced = (1 << 5),
  kPlcInfoFlagDetwistReady = (1 << 6)
} PlcInfoFlag;

typedef enum {
  kGsMotorStatusFlagExecute = (1 << 0)
} GsMotorStatusFlag;

typedef enum {
  kGsMotorWarningFlagTorqueLimitNotReady = (1 << 0),
  kGsMotorWarningFlagTorqueLimitActive   = (1 << 1),
  kGsMotorWarningFlagNotPowered          = (1 << 2),
  kGsMotorWarningFlagNotReferenced       = (1 << 3)
} GsMotorWarningFlag;

typedef enum {
  kGsMotorErrorFlagMotor   = (1 << 0),
  kGsMotorErrorFlagEncoder = (1 << 1)
} GsMotorErrorFlag;

typedef enum {
  kGsAxisStatusFlagExecute       = (1 << 0),
  kGsAxisStatusFlagHpuEnabled    = (1 << 1)
} GsAxisStatusFlag;

typedef enum {
  kGsAxisWarningFlagTorqueLimitNotReady = (1 << 0),
  kGsAxisWarningFlagTorqueLimitActive   = (1 << 1),
  kGsAxisWarningFlagAOnlyMode           = (1 << 2),
  kGsAxisWarningFlagBOnlyMode           = (1 << 3),
  kGsAxisWarningFlagEncoder             = (1 << 4),
  kGsAxisWarningFlagEncoderValue        = (1 << 5),
  kGsAxisWarningFlagEncoderKnownBad     = (1 << 6),
  kGsAxisWarningFlagEncoderHardware     = (1 << 7)
} GsAxisWarningFlag;

typedef enum {
  kGsAxisErrorFlagMotor         = (1 << 0),
  kGsAxisErrorFlagEncoder       = (1 << 1),
  kGsAxisErrorFlagHpu           = (1 << 2),
  kGsAxisErrorFlagNotPowered    = (1 << 3),
  kGsAxisErrorFlagNotReferenced = (1 << 4)
} GsAxisErrorFlag;

typedef enum {
  kGsStatusFlagCatwalkMode   = (1 << 0),
  kGsStatusFlagAzimuthJogPos = (1 << 1),
  kGsStatusFlagAzimuthJogNeg = (1 << 2),
  kGsStatusFlagDetwistJogPos = (1 << 3),
  kGsStatusFlagDetwistJogNeg = (1 << 4),
  kGsStatusFlagWinchJogPos   = (1 << 5),
  kGsStatusFlagWinchJogNeg   = (1 << 6)
} GsStatusFlag;

typedef enum {
  kGsWarningFlagTorqueLimitNotReady    = (1 << 0),
  kGsWarningFlagTorqueLimitNotActive   = (1 << 1),
  kGsWarningFlagAxisSingleMotor        = (1 << 2),
  kGsWarningFlagIgnoringComms          = (1 << 3),
  kGsWarningFlagPsuABad                = (1 << 4),
  kGsWarningFlagPsuBBad                = (1 << 5),
  kGsWarningFlagEncoder                = (1 << 6),
  kGsWarningFlagDetwistCommandJump     = (1 << 7),
  kGsWarningFlagProximityCheckDisabled = (1 << 8)
} GsWarningFlag;

typedef enum {
  kGsErrorFlagAzimiuth          = (1 << 0),
  kGsErrorFlagDetwist           = (1 << 1),
  kGsErrorFlagLevelwind         = (1 << 2),
  kGsErrorFlagWinch             = (1 << 3),
  kGsErrorFlagHpuAzimuth        = (1 << 4),
  kGsErrorFlagHpuWinch          = (1 << 5),
  kGsErrorFlagEncoder           = (1 << 6),
  kGsErrorFlagPsu               = (1 << 7),
  kGsErrorFlagAxesNotPowered    = (1 << 8),
  kGsErrorFlagAxesNotReferenced = (1 << 9),
  kGsErrorFlagEstopped          = (1 << 10),
  kGsErrorFlagNo480Vac          = (1 << 11),
  kGsErrorFlagsTetherEngagement = (1 << 12)
} GsErrorFlag;

typedef enum {
  kGsAxisStateOff = 0,
  kGsAxisStateDual,
  kGsAxisStateAOnly,
  kGsAxisStateBOnly,
  kGsAxisStateConfigDrives,
  kGsAxisStateChangingToOff,
  kGsAxisStateChangingToDual,
  kGsAxisStateChangingToA = 20,
  kGsAxisStateChangingToB = 30,
  kGsAxisStateNotReady = 255
} GsAxisState;

typedef enum {
  kGsSystemModeManual = 0,
  kGsSystemModeHighTension,
  kGsSystemModeReel,
  kGsSystemModeTransform,
  kGsSystemModeParked,
  kGsSystemModeOff,
  kGsSystemModeError = 254,
  kGsSystemModeChangingModes = 255
} GsSystemMode;

typedef enum {
  PlcOpenStateInvalid = 0,
  PlcOpenStateErrorStop,
  PlcOpenStateDisabled,
  PlcOpenStateStandstill,
  PlcOpenStateStopping,
  PlcOpenStateHoming,
  PlcOpenStateDiscreteMotion,
  PlcOpenStateSyncMotion,
  PlcOpenStateContinuousMotion
} PlcOpenState;

typedef struct {
  uint16_t version;
  uint8_t message_type;
  uint16_t sequence;
} PlcHeader;

typedef struct {
  PlcHeader header;
  uint16_t detwist_cmd;     // See DetwistCommand.
  double detwist_position;  // [rad] Multi-turn.
} PlcCommandMessage;

typedef struct {
  PlcHeader header;
  double detwist_position;  // [rad] Multi-turn.
  float detwist_velocity;   // [rad/s]
  float detwist_torque;     // [N-m]
  float detwist_motor_temp;
  uint16_t last_error;
  uint32_t error_flags;     // See PlcErrorFlag.
  uint32_t warning_flags;   // See PlcWarningFlag.
  uint32_t info_flags;      // See PlcInfoFlag.
} PlcStatusMessage;

typedef struct {
  float v_rms_a;
  float v_rms_b;
  float v_rms_c;
  float i_rms_a;
  float i_rms_b;
  float i_rms_c;
  float frequency;  // [Hz]
} GroundStationInputPower;

typedef struct {
  StatusFlags flags;  // See GsMotor[Status|Warning|Error]Flag.
  uint8_t plc_open_state;  // See PlcOpenState.
  double command;     // [rad], [rad/s], or [m] depending on axis.
  double position;    // [rad] or [m]
  float velocity;     // [rad/s] or [m/s]
  float torque;       // [Nm]
  float temperature;  // [C]
  float bus_voltage;
} GroundStationMotorStatus;

typedef struct {
  StatusFlags flags;                  // See GsAxis[Status|Warning|Error]Flag.
  uint8_t plc_open_state;             // See PlcOpenState.
  uint8_t requested_motor_mode;       // See GsAxisState.
  uint8_t requested_mode;             // See GsSystemMode.
  uint8_t current_mode;               // See GsSystemMode.
  uint8_t axis_requested_motor_mode;  // See GsAxisState.
  uint8_t axis_requested_status;      // See GsAxisState.
  double position;
  double velocity;
  // Velocity the axis will travel when jogging. Can be changed using the GS02
  // PLC laptop.
  double jog_velocity;   // [mm/s] for levelwind, [rad/s] for others.
  GroundStationMotorStatus motor[2];
} GroundStationAxisStatus;

typedef struct {
  bool sensor_raw[2];  // Light-on and dark-on outputs of sensor.
  bool engaged;
} TetherEngagement;

typedef struct {
  bool sensor_raw[2];  // Proximity sensor A/B.
  bool proximity;      // Proximity estimate.
} BridleProximity;

typedef struct {
  // TODO(): Define coolant temp/flow indices.
  float temperature[6];  // [C]
  uint8_t flow;          // Bit field of binary sensors.
} GroundStationCoolant;

typedef struct {
  StatusFlags flags;             // See Gs[Status|Warning|Error]Flag.
  GroundStationAxisStatus detwist;
  GroundStationAxisStatus winch;
  GroundStationAxisStatus azimuth;
  GroundStationAxisStatus levelwind;
  BridleProximity bridle_proximity;
  GroundStationCoolant coolant;
  uint8_t mode;                  // See GroundStationMode.
  uint8_t transform_stage;       // Stages are defined by PLC MAT controller.
  TetherEngagement tether_engagement;
  GroundStationInputPower input_power;
} GroundStationStatus;

typedef struct {
  PlcHeader header;
  GroundStationStatus status;
} PlcGs02StatusMessage;

typedef struct {
  PlcHeader header;
  uint8_t enable_azimuth;       // TODO(): Define enable signal.
  uint8_t enable_winch;         // TODO(): Define enable signal.
  uint8_t enable_levelwind;     // TODO(): Define enable signal.
  uint8_t enable_detwist;       // TODO(): Define enable signal.
  uint8_t mode_request;         // See GroundStationMode.
  float wind_direction;         // [rad] Unused.
  float wind_speed;             // [m/s] Unused.
  float weather_temp;           // [C]
  float weather_dewpoint;       // [C]
  float perch_azi_angle[2];     // [rad]

  // Component 0: Target azimuth angle [rad]
  // Component 1: Unused
  // Component 2: Aziuth dead zone half-width [rad]
  //
  // TODO(): Replace this with two named fields, and update the MAT
  // controller accordingly.
  float azi_cmd[3];

  float winch_velocity_cmd;     // [m/s]
  double detwist_position_cmd;  // [rad] Multi-turn.
  uint8_t command;              // See Gs02Command.

  uint8_t unpause_transform;    // Boolean to unpause the GS02 transform.
} PlcGs02InputMessage;

// Naming of structs and fields below are inherited from the "MAT" controller
// code running on the PLC, and are preserved here to make it clear what
// values they reprent in that code. See the MAT documentation for additional
// explanation.
// TODO(): Move these defs to a new header, plc_mat_messages.h?
typedef struct {
  double MBrake;  // [Nm] Moment on brake.
  double P;       // [Pa] Applied pressure.
  double Z;       // [mm] Brake caliper distance.
  bool bReady;    // HPU is ready.
} HPUSensorBusInternal;

typedef struct {
  double xShuttle;    // [m] Levelwind shuttle position.
  double aPivot;      // Unused.
  double aCassette;   // Unused.
  double aDeparture;  // Unused.
} LevelWindSensorBusInternal;

typedef struct {
  double TorqueA;  // [Nm]
  double TorqueB;  // [Nm]
  bool bReady;
  bool bEnabled;
  uint8_t NMode;
} MotorSensorBusInternal;

typedef struct {
  double Position;  // [rad]
  double Velocity;  // [rad/s]
  double aGSG1;     // Unused.
  double aGSG2;     // Unused.
  MotorSensorBusInternal Motor;
} DetwistSensorBusInternal;

typedef struct {
  double Position;  // [rad]
  double Velocity;  // [rad/s]
  MotorSensorBusInternal Motors;
  HPUSensorBusInternal HPU;
} AxesSensorBusInternal;

typedef struct {
  AxesSensorBusInternal Azimuth;
  AxesSensorBusInternal Winch;
  LevelWindSensorBusInternal LevelWind;
  DetwistSensorBusInternal Detwist;
} GroundStationBusInternal;

typedef struct {
  double vReelCommand;       // [m/s]
  double aAzimuthTarget;
  double aTetherElevation;
  double aDeadZone;
  double FTether;            // [N] Tether force.
  uint8_t NOpModeDemand;     // See GroundStationMode.
  bool bBridleProximitySensor;
  bool bPauseWinch;
  double aDetwistDemand;     // [rad] Absolute.
  bool bContinueTransform;
} WingBusInternal;

typedef struct {
  double perch_azi_A;
  double perch_azi_B;
  double perch_azi_vel_A;
  double perch_azi_vel_B;
} PerchSensorBusInternal;

typedef struct {
  PerchSensorBusInternal perch_azi;
} GroundStationBusInternal_AIO;

typedef struct {
  GroundStationBusInternal GroundStation;
  WingBusInternal Wing;
  GroundStationBusInternal_AIO GroundStation_AIO;
} PlcGs02ControlInput;

typedef struct {
  double Velocity;  // [rad/s]
} MotorVelocityControlBusExternal;

typedef struct {
  double IPPV;  // [mA] Proportional valve current.
  bool BSVR;
  bool BSVE;
  bool BSV;
} HPUControlBusExternal;

typedef struct {
  double Position;  // [rad]
} MotorPositionControlBus;

typedef struct {
  int8_t NStateMachine;
  bool BEnableMotorControl;
  MotorPositionControlBus Motor;
} DetwistControlBus;

typedef struct {
  int8_t NStateMachine;
  bool BEnableMotorControl;
  MotorVelocityControlBusExternal Motor;
  HPUControlBusExternal HPU;
} AxesControlBusExternal;

typedef struct {
  bool bAziOnTarget;
  bool bAziOK;
  bool bWinchOnTarget;
  bool bWinchOK;
  bool bElevationOK;
  bool bDetwistOnTarget;
  int8_t NStateMachine;
  int8_t NTransformStage;
} SupervisoryBus;

typedef struct {
  double nDemand;
  double SignOfFriction;
  bool Engage;
  double IPPV_Control;
  double NHPUMode;
  double MTetherKFAzi;
} HpuSupervisoryBus;

typedef struct {
  AxesControlBusExternal Azimuth;
  AxesControlBusExternal Winch;
  DetwistControlBus Detwist;
  uint8_t NOpMode;
  double bTransitioningMode;
  bool bReelPaused;
  SupervisoryBus Supervisory;
  HpuSupervisoryBus HpuSupervisory;
} PlcGs02ControlOutput;

typedef struct {
  PlcHeader header;
  PlcGs02ControlInput in;
  PlcGs02ControlOutput out;
} PlcGs02ControlMessage;

#endif  // AVIONICS_COMMON_PLC_MESSAGES_H_

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

#ifndef CONTROL_CONTROL_TYPES_H_
#define CONTROL_CONTROL_TYPES_H_

#include <stdbool.h>
#include <stdint.h>

#include "common/c_math/vec3.h"
#include "control/actuator_types.h"
#include "control/avionics/avionics_interface_types.h"
#include "control/crosswind/crosswind_types.h"
#include "control/estimator/estimator_types.h"
#include "control/experiments/experiment_types.h"
#include "control/fault_detection/fault_detection_types.h"
#include "control/hover/hover_types.h"
#include "control/manual/manual_types.h"
#include "control/sensor_types.h"
#include "control/simple_aero_types.h"
#include "control/system_types.h"
#include "control/trans_in/trans_in_types.h"
#include "system/labels.h"

typedef enum {
  kControllerForceSigned = -1,
  kControllerNone,
  kControllerCrosswind,
  kControllerHover,
  kControllerManual,
  kControllerTransIn
} ControllerType;

typedef enum {
  kInitializationStateFirstEntry,
  kInitializationStateWaitForValidData,
  kInitializationStateFirstLoop,
  kInitializationStateRunning
} InitializationState;

typedef enum {
  kControlOptHardCodeWind = 1 << 0,
  kControlOptHardCodeInitialPayout = 1 << 1,
  kControlOptHoverThrottleEStop = 1 << 2,
} ControlOption;

// These values should never be changed (except for kNumFlightModes).
// New flight modes should be appended to the end.
typedef enum {
  kFlightModeForceSigned = -1,
  kFlightModePilotHover = 0,
  kFlightModePerched = 1,
  kFlightModeHoverAscend = 2,
  kFlightModeHoverPayOut = 3,
  kFlightModeHoverFullLength = 4,
  kFlightModeHoverAccel = 5,
  kFlightModeTransIn = 6,
  kFlightModeCrosswindNormal = 7,
  kFlightModeCrosswindPrepTransOut = 8,
  kFlightModeHoverTransOut = 9,
  kFlightModeHoverReelIn = 10,
  kFlightModeHoverDescend = 11,
  kFlightModeOffTether = 12,
  kFlightModeHoverTransformGsUp = 13,
  kFlightModeHoverTransformGsDown = 14,
  kFlightModeHoverPrepTransformGsUp = 15,
  kFlightModeHoverPrepTransformGsDown = 16,
  kNumFlightModes
} FlightMode;

typedef struct {
  uint8_t sequence;
  int32_t flight_mode;
} ControlSyncData;

typedef struct {
  FlightMode flight_mode;
  FlightMode last_flight_mode;
  bool flight_mode_first_entry;
  double flight_mode_time;
  bool fully_autonomous_mode;
} FlightStatus;

typedef struct {
  double temperature;  // [C]
  double pressure;     // [Pa]
  double humidity;     // [fraction]
} GsWeather;

typedef struct {
  ControlSyncData sync[kNumControllers];
  GsgData gsg[kNumDrums];
  GsSensorData gs_sensors;
  GpsData wing_gps[kNumWingGpsReceivers];
  JoystickData joystick;
  double loadcells[kNumLoadcellSensors];
  bool tether_released;
  ImuData imus[kNumWingImus];
  PitotData pitots[kNumPitotSensors];
  double flaps[kNumFlaps];    // [rad]
  double rotors[kNumMotors];  // [rad/s]
  int32_t stacking_state;     // See StackingState.
  Vec3 wind_ws;
  GsGpsData gs_gps;
  PerchData perch;
  GsWeather weather;
  bool force_hover_accel;
  bool force_high_tension;
  bool force_reel;
  bool gs_unpause_transform;
  bool force_detwist_turn_once;
  GroundEstimateMessage ground_estimate;
  ExperimentType experiment_type;
  uint8_t experiment_case_id;
} ControlInput;

typedef struct {
  // Note that "target" is not a direct command for the GS azimuth, but rather
  // the angle the GS should point its "pointing thing" at. The identity of the
  // pointing thing is dependendent on the GS mode. During Reel mode, it's the
  // perch panels; during HighTension mode, it's the GSG; during Transform mode,
  // it's one or the other, depending on the transform stage.
  double target;     // [rad]
  double dead_zone;  // [rad]
} GsAzimuthCommand;

typedef struct {
  ControlSyncData sync;
  double flaps[kNumFlaps];                     // [rad]
  double motor_speed_upper_limit[kNumMotors];  // [rad/s]
  double motor_speed_lower_limit[kNumMotors];  // [rad/s]
  double motor_torque[kNumMotors];             // [Nm]
  double winch_vel_cmd;                        // [m/s]
  double detwist_cmd;                          // [rad]
  bool stop_motors;  // Set by inner loop; overrides run_motors.
  bool run_motors;   // Set by outer loop.
  bool tether_release;
  GroundStationMode gs_mode_request;
  bool gs_unpause_transform;
  GsAzimuthCommand gs_azi_cmd;

  // When this is set by one of the controllers, ControlOutputStep will hold the
  // value of gs_azi_cmd.
  bool hold_gs_azi_cmd;
} ControlOutput;

// Common parameters.

typedef struct {
  double ascend_payout_throttle;
  double return_to_crosswind_throttle;
  double prep_trans_out_throttle;
  double descend_reel_in_throttle;
  double e_stop_throttle;
  double e_stop_throttle_latch_time;
} JoystickControlParams;

typedef struct {
  ControlInput max;
  ControlInput min;
} SensorLimitsParams;

typedef struct {
  ImuData imu;
  GpsData gs_gps;
  GpsCompassData gps_compass;
} GroundEstimatorInput;

typedef struct {
  GroundEstimatorInput max;
  GroundEstimatorInput min;
} GroundSensorLimitsParams;

typedef struct {
  double wind_speed_min;
  double wind_speed_max;
  double wind_azi_min;
  double wind_azi_max;
} WindWindow;

typedef struct {
  double takeoff_delay;
  WindWindow takeoff;
  WindWindow landing;
} PlannerParams;

typedef struct {
  FlightStatus flight_status;
  FlightMode autonomous_flight_mode;
  double flight_mode_zero_throttle_timer;
  uint8_t controller_sync_sequence;
  bool force_hover_accel_latched;
  FlightMode desired_flight_mode;
  bool throttle_gate_satisfied;
  bool enable_autonomous_operation;
  double autonomous_takeoff_countdown_timer;
  bool inside_launch_window;
  bool inside_landing_window;
  bool landing_fault_detected;
} PlannerState;

typedef struct {
  double flaps_min[kNumFlaps];
  double flaps_max[kNumFlaps];
  double rate_limit;
} ControlOutputParams;

typedef struct {
  bool run_motors_latched;
  double flaps_z1[kNumFlaps];
  GsAzimuthCommand gs_azi_cmd_z1;
  bool last_gs_azi_cmd_valid;
} ControlOutputState;

typedef struct {
  double sim_init_timeout;
  double sim_update_timeout;
} HitlControlParams;

typedef struct {
  FlightPlan flight_plan;
  ControlOption control_opt;
  SimpleAeroModelParams simple_aero_model;
  RotorControlParams rotor_control;
  SensorLimitsParams sensor_limits;
  GroundSensorLimitsParams ground_sensor_limits;
  EstimatorParams estimator;
  HoverParams hover;
  TransInParams trans_in;
  CrosswindParams crosswind;
  ManualParams manual;
  PlannerParams planner;
  ControlOutputParams control_output;
  JoystickControlParams joystick_control;
  FaultDetectionParams fault_detection;
  HitlControlParams hitl;
} ControlParams;

typedef struct {
  int32_t buffer_counter;
  int64_t max_usecs[2];
} LoopTimeState;

typedef struct {
  InitializationState init_state;
  FlightStatus flight_status;
  ControlInputMessages input_messages;
  double time;
  FaultMask faults[kNumSubsystems];
  AvionicsInterfaceState avionics_interface;
  PlannerState planner;
  EstimatorState estimator;
  HoverState hover;
  TransInState trans_in;
  CrosswindState crosswind;
  ManualState manual;
  ControlOutputState control_output;
  LoopTimeState loop_time;
} ControlState;

typedef struct {
  InitializationState init_state;
  GroundEstimatorInputMessages input_messages;
  GroundEstimatorInput input;
  double time;
  FaultMask faults[kNumSubsystems];
  GroundvionicsInterfaceState avionics_interface;
  EstimatorNavGroundState estimator;
  GroundEstimateMessage ground_estimate;
} GroundEstimatorState;

#ifdef __cplusplus
extern "C" {
#endif

int32_t GetNumFlightModeGates(FlightMode flight_mode);
const char *InitializationStateToString(InitializationState init_state);
const char *FlightModeToString(FlightMode flight_mode);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // CONTROL_CONTROL_TYPES_H_

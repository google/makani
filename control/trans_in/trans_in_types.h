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

#ifndef CONTROL_TRANS_IN_TRANS_IN_TYPES_H_
#define CONTROL_TRANS_IN_TRANS_IN_TYPES_H_

#include "control/actuator_types.h"
#include "control/system_types.h"
#include "system/labels.h"

typedef enum {
  kTransInGateForceSigned = -1,
  kTransInGateFlightPlan,
  kTransInGateMinDynamicPressure,
  kTransInGateStillAccelerating,
  kNumTransInGates
} TransInGate;

typedef struct {
  double min_dynamic_pressure;
  double min_time_in_accel;
  double max_time_keep_accelerating;
  double acc_stopped_accelerating_threshold;
  double min_pitch_angle;
} TransInModeParams;

typedef struct {
  double min_aero_climb_angle_cmd;
  double max_aero_climb_angle_cmd;
  double min_airspeed;
  double thrust_pitch;
  double radial_tracking_freq_hz;
  double radial_tracking_damping_ratio;
  double tension_control_radial_error_threshold;
  double tension_control_elevation_angle_threshold;
  double min_tension_cmd;
  double CL_0, dCL_dalpha, dCL_dflap;
  double min_angle_of_attack_cmd;
  double max_angle_of_attack_cmd;
  double min_delta_flap_cmd;
  double max_delta_flap_cmd;
  double max_pitch_rate_b_cmd;
  double thrust_cmd;
} TransInLongitudinalParams;

typedef struct {
  double CL_max;
  double max_aero_climb_angle;
  double lateral_tracking_ref_length;
  double max_lateral_tracking_freq_hz;
  double lateral_tracking_damping_ratio;
  double max_pos_ti_y_err;
  double max_delta_roll_ti_cmd;
  double max_yaw_rate_ti_cmd;
  double angle_of_sideslip_cmd;
  double roll_ti_cmd, yaw_ti_cmd;
} TransInLateralParams;

typedef enum {
  kTransInLateralInputMotorYaw,
  kTransInLateralInputAileron,
  kTransInLateralInputRudder,
  kNumTransInLateralInputs
} TransInLateralAttitudeInputs;

typedef enum {
  kTransInLateralStateRoll,
  kTransInLateralStateYaw,
  kTransInLateralStateRollRate,
  kTransInLateralStateYawRate,
  kTransInLateralStateIntRoll,
  kTransInLateralStateAngleOfSideslip,
  kNumTransInLateralStates
} TransInLateralStates;

typedef enum {
  kTransInLongitudinalInputMotorPitch,
  kTransInLongitudinalInputElevator,
  kNumTransInLongitudinalInputs
} TransInLongitudinalInputs;

typedef enum {
  kTransInLongitudinalStatePitch,
  kTransInLongitudinalStatePitchRate,
  kTransInLongitudinalStateIntAngleOfAttack,
  kNumTransInLongitudinalStates
} TransInLongitudinalStates;

typedef struct {
  double min_initial_pitch_moment;
  double max_initial_yaw_moment;
  double pitch_forward_max_pitch_rate;
  double pitch_forward_max_pitch_accel;
  double pitch_forward_max_pitch_error;
  double pitch_forward_max_duration;
  double delta_elevator_alpha_zero;
  double ddelta_elevator_dalpha;
  double int_release_airspeed_threshold;
  double int_release_alpha_threshold;
  double max_int_angle_of_attack_rate;
  double max_int_angle_of_attack;
  double max_int_roll;
  double low_tension, high_tension;
  double lat_gains_pitch_forward[kNumTransInLateralInputs]
                                [kNumTransInLateralStates];
  double lat_gains_low_tension[kNumTransInLateralInputs]
                              [kNumTransInLateralStates];
  double lat_gains_high_tension[kNumTransInLateralInputs]
                               [kNumTransInLateralStates];
  double long_gains_pitch_forward[kNumTransInLongitudinalInputs]
                                 [kNumTransInLongitudinalStates];
  double long_gains_low_tension[kNumTransInLongitudinalInputs]
                               [kNumTransInLongitudinalStates];
  double long_gains_high_tension[kNumTransInLongitudinalInputs]
                                [kNumTransInLongitudinalStates];
  double midboard_flap_ratio;
} TransInAttitudeParams;

typedef struct {
  // Pitch moment command [N-m] at the hand-off from hover.
  double initial_pitch_moment;
  // Yaw moment command [N-m] at the hand-off from hover.
  double initial_yaw_moment;
  // Initial command pitch angle [rad].
  double initial_pitch_ti;
  // Duration [s] of the pitch forward maneuver.
  double pitch_forward_duration;
  // Desired maximum pitch rate [rad/s] for the pitch forward maneuver.
  double pitch_forward_pitch_rate;
  // Whether the integrator hold has been released.
  bool release_integrator;
  // Integrated angle-of-attack error [rad-s].
  double int_angle_of_attack;
  // Integrated roll error [rad-s].
  double int_roll;
} TransInAttitudeState;

typedef struct {
  ThrustMoment thrust_moment_weights;
  double flap_offsets[kNumFlaps];
  double lower_flap_limits[kNumFlaps];
  double upper_flap_limits[kNumFlaps];
} TransInOutputParams;

typedef struct {
  double prop_inflow_airspeed_bias;
  double prop_inflow_low_airspeed;
  double prop_inflow_high_airspeed;
  double turn_start_pos_ti_x;
  double turn_radius;
  double turn_course_angle;
  TransInModeParams mode;
  TransInLongitudinalParams longitudinal;
  TransInLateralParams lateral;
  TransInAttitudeParams attitude;
  TransInOutputParams output;
} TransInParams;

typedef struct {
  // Azimuth [rad] of the origin of the trans_in coordinate frame.
  double ti_origin_azimuth;
  TransInAttitudeState attitude;
} TransInState;

#ifdef __cplusplus
extern "C" {
#endif

const char *TransInGateToString(TransInGate gate);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // CONTROL_TRANS_IN_TRANS_IN_TYPES_H_

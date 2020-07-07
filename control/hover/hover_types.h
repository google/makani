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

#ifndef CONTROL_HOVER_HOVER_TYPES_H_
#define CONTROL_HOVER_HOVER_TYPES_H_

#include <stdint.h>

#include "avionics/common/plc_messages.h"
#include "common/c_math/filter.h"
#include "common/c_math/vec3.h"
#include "control/actuator_types.h"
#include "control/experiments/hover_experiment_types.h"

typedef enum {
  kHoverPerchedGateForceSigned = -1,
  kHoverPerchedGateDisabled,
  kNumHoverPerchedGates
} HoverPerchedGate;

typedef enum {
  kHoverAscendGateForceSigned = -1,
  kHoverAscendGateProximityValid,
  kHoverAscendGatePerchWindMisalignment,
  kHoverAscendGateTension,
  kNumHoverAscendGates
} HoverAscendGate;

typedef enum {
  kHoverPayOutGateForceSigned = -1,
  kHoverPayOutGateAscentComplete,
  kHoverPayOutGateGainRampDone,
  kHoverPayOutGateZPosition,
  kHoverPayOutGateYawError,
  kHoverPayOutGateYawRate,
  kNumHoverPayOutGates
} HoverPayOutGate;

typedef enum {
  kHoverPrepTransformGsUpGateForceSigned = -1,
  kHoverPrepTransformGsUpGateFlightPlan,
  kHoverPrepTransformGsUpGateWinchPosition,
  kHoverPrepTransformGsUpGateGroundStationMode,
  kNumHoverPrepTransformGsUpGates
} HoverPrepTransformGsUpGate;

typedef enum {
  kHoverTransformGsUpGateForceSigned = -1,
  kHoverTransformGsUpGateTetherElevation,
  kHoverTransformGsUpGateAzimuthError,
  kHoverTransformGsUpGateZError,
  kNumHoverTransformGsUpGates
} HoverTransformGsUpGate;

typedef enum {
  kHoverFullLengthGateForceSigned = -1,
  kHoverFullLengthGateGroundStationMode,
  kHoverFullLengthGateForceDetwistTurn,
  kNumHoverFullLengthGates
} HoverFullLengthGate;

typedef enum {
  kHoverAccelGateForceSigned = -1,
  kHoverAccelGateFlightPlan,
  kHoverAccelGateRollError,
  kHoverAccelGateYawError,
  kHoverAccelGateYawRate,
  kHoverAccelGateAngularRate,
  kHoverAccelGateAzimuthError,
  kHoverAccelGateZError,
  kHoverAccelGateSpeed,
  kHoverAccelGateYVelocity,
  kHoverAccelGateTension,
  kHoverAccelGateGroundStationMode,
  kHoverAccelGateForceDetwistTurn,
  kNumHoverAccelGates
} HoverAccelGate;

typedef enum {
  kHoverPrepTransformGsDownGateForceSigned = -1,
  kHoverPrepTransformGsDownGateTimeInTransOut,
  kHoverPrepTransformGsDownGateGroundStationMode,
  kHoverPrepTransformGsDownGateForceDetwistTurn,
  kNumHoverPrepTransformGsDownGates
} HoverPrepTransformGsDownGate;

typedef enum {
  kHoverTransformGsDownGateForceSigned = -1,
  kHoverTransformGsDownGateTetherElevation,
  kHoverTransformGsDownGateAzimuthError,
  kHoverTransformGsDownGateZError,
  kHoverTransformGsDownGateForceDetwistTurn,
  kNumHoverTransformGsDownGates
} HoverTransformGsDownGate;

typedef enum {
  kHoverReelInGateForceSigned = -1,
  kHoverReelInGateFlightPlan,
  kHoverReelInGateGroundStationMode,
  kNumHoverReelInGates
} HoverReelInGate;

typedef enum {
  kHoverDescendGateForceSigned = -1,
  kHoverDescendGateAbovePerch,
  kHoverDescendGateProximity,
  kHoverDescendGateSpeed,
  kNumHoverDescendGates
} HoverDescendGate;

typedef enum {
  kGainStateForceSigned = -1,
  kGainStateEStopped,
  kGainStateZeroLatched,
  kGainStateRampUp,
  kGainStateFull,
  kGainStateRampDown,
  kNumGainStates
} GainState;

typedef struct { double good_elevation_since; } HoverTetherElevationState;

typedef struct {
  double max_pilot_thrust_to_weight_ratio;
  PidParams low_altitude_pid;
  PidParams high_altitude_pid;
  double low_altitude;
  double high_altitude;
  double max_thrust;
  double max_thrust_rate;
  double boost_fc;
  double boost_output_min;
  double boost_output_max;
  bool boost_enabled;
  double transout_thrust_fade_start;
  double transout_thrust_fade_end;
} HoverAltitudeParams;

typedef struct {
  double thrust_cmd_z1;
  double int_thrust;
  double int_boost;
  bool boost_enable_z;
} HoverAltitudeState;

typedef struct {
  double perch_contact_extra_pitch_moment_min;
  double perch_contact_extra_pitch_moment_max;
  double perch_contact_extra_pitch_moment_fade_angle_min;
  double perch_contact_extra_pitch_moment_fade_angle_max;
  double perch_contact_total_pitch_moment_min;
  double perch_contact_total_pitch_moment_max;
  double bridle_roll_damping_factor;
  double blown_flaps_roll_rate_gain;
  PidParams roll_pid;
  PidParams low_thrust_pitch_pid;
  PidParams pitch_pid;
  PidParams yaw_pid;
  Vec3 min_moment;
  Vec3 max_moment;
  Vec3 min_accel_moment;
  Vec3 max_accel_moment;
  double nominal_elevator_pitch_moment;
  PidParams int_pitch_pid;
  PidParams int_yaw_pid;
} HoverAnglesParams;

typedef struct {
  double int_elevator_moment;
  double int_rudder_moment;
  Vec3 int_moment;
  Vec3 angles_error;
  double extra_pitch_moment_scale_z1;
} HoverAnglesState;

typedef struct {
  bool use_signal_injection;
  Vec3 position_amplitude;
  Vec3 position_start_time;
  Vec3 position_stop_time;
  Vec3 angles_amplitude;
  Vec3 angles_start_time;
  Vec3 angles_stop_time;
  double blown_flaps_amplitude;
  double blown_flaps_period;
  double blown_flaps_start_time;
  double blown_flaps_stop_time;
  double drag_flaps_start_time;
  double drag_flaps_stop_time;
  double drag_flaps_period;
  double drag_flaps_low_drag_pos;
  double drag_flaps_high_drag_pos;
  double elevator_amplitude;
  double elevator_start_time;
  double elevator_stop_time;
  double rudder_amplitude;
  double rudder_start_time;
  double rudder_stop_time;
} HoverInjectParams;

typedef struct {
  double aligned_perch_azi_to_wind_angle_for_ascend;
  double max_perch_wind_misalignment_for_ascend;
  double max_platform_misalignment_for_descend;
  double max_z_for_pay_out;
  double max_yaw_angle_error_for_pay_out;
  double max_yaw_rate_for_pay_out;
  double min_winch_pos_for_transform_gs_up;
  double max_roll_angle_error_for_accel;
  double max_yaw_angle_error_for_accel;
  double max_yaw_rate_for_accel;
  double max_angular_rate_for_accel;
  double max_azimuth_error_for_accel;
  double max_z_error_for_accel;
  double max_speed_for_accel;
  double max_body_y_vel_for_accel;
  double min_tension_for_ascend;
  double min_tension_for_accel;
  double min_time_in_trans_out;
  double max_azimuth_error_for_transform;
  double max_z_error_for_transform;
  double max_tether_elevation_error_for_gs_transform_staging;
  double max_tether_elevation_error_for_gs_transform_kickoff;
  double min_gs_transform_staging_time;
} HoverModeParams;

typedef struct {
  // Motor control.
  ThrustMoment weights;
  double gain_ramp_time;

  // Flap control.
  Vec3 propwash_b;
  double zero_propwash_wind_speed;
  double full_propwash_wind_speed;
  double center_propwash_wind_speed;
  double delta_ele_trans_out;
  double delta_elevator_per_pitch_moment;
  double min_delta_elevator_fb;
  double max_delta_elevator_fb;
  double elevator_cutoff_freq;
  double no_aileron_rudder_speed;
  double full_aileron_rudder_speed;
  double cl_da;
  double cn_dr;
  double delta_blown_aileron_per_roll_moment;
  double zero_blown_flaps_forward_speed;
  double full_blown_flaps_forward_speed;
  double flap_offsets[kNumFlaps];
  double lower_flap_limits[kNumFlaps];
  double upper_flap_limits[kNumFlaps];
  double gs02_deadzone_while_perched;
  double gs02_deadzone_during_flight;
  double spoiled_aileron_angle;
} HoverOutputParams;

typedef struct {
  double gain_ramp_latch_timer;
  double gain_ramp_scale;
  GainState gain_ramp_state;
  bool align_with_propwash;
  double delta_ele_ff_z1;
  ThrustMoment thrust_moment_out;
  GroundStationMode gs_mode_request_z1;
  bool use_high_tension_gs_azi_cmd;
  bool forcing_detwist_turn;
  double detwist_cmd_frozen;
  double force_detwist_t0;
  Vec3 cw_loop_center_v_f_z1;
} HoverOutputState;

#define HOVER_PATH_ALTITUDE_TABLE_LENGTH 3
typedef struct {
  Vec3 max_acceleration_g;
  Vec3 perched_wing_pos_p;
  double max_normal_radial_speed;
  double max_normal_tangential_speed;
  double max_ascend_perch_z_speed;
  double max_ascend_near_perch_z_speed;
  double max_ascend_normal_z_speed;
  double max_descend_perch_z_speed;
  double max_descend_near_perch_z_speed;
  double max_descend_normal_z_speed;
  double max_accel_z_speed;
  double gps_error_tolerance;
  double ascend_offset_g_z;
  double descend_offset_g_z;
  double velocity_cutoff_freq;
  double velocity_damping_ratio;
  double vessel_heel_ff;
  double target_reel_tether_elevation;
  double target_transform_tether_elevation;
  double target_above_perch_tether_elevation;
  double launch_perch_elevation_max;
  double launch_perch_elevation_min;
  double reel_short_tether_length;
  double reel_long_tether_length;
  double reel_azimuth_offset;
  double reel_elevation_min;
  double reel_elevation_max;
  double accel_start_elevation;
  double transout_vg_cmd_crossfade_duration;
  double transout_vel_cmd_multiplier;
  double transout_min_altitude;
  double max_altitude_error_for_translation;
  // The following are used by flight modes PayOut/ReelIn.
  PidParams reel_tether_elevation_pid;
  double max_payout_for_perching_prep;
  double tether_elevation_error_fc;
  double tether_elevation_error_zeta;
  // The following are used by flight mode PrepGsTransformUp/Down.
  PidParams transform_tether_elevation_pid;
  // The following are used by flight mode GsTransformUp/Down.
  double transform_azimuth_offset;
} HoverPathParams;

typedef struct {
  Vec3 raw_wing_pos_g_cmd;
  Vec3 wing_pos_g_cmd_z1;
  Vec3 wing_vel_g_cmd_z1;
  Vec3 wing_vel_g_cmd_zs[2];
  Vec3 fixed_pos_g;
  Vec3 perched_pos_g;
  double transform_azi;
  double transout_Xg_z_min;
  double transout_azi;
  // The following value is computed from the crosswind playbook, and stored
  // here to be used by HoverIsReadyForMode to determine when HoverAccel is
  // possible.
  Vec3 accel_start_pos_g;
  // The following are used by flight modes PayOut/ReelIn.
  double int_kite_elevation;
  double tether_elevation_error_zs[2];
} HoverPathState;

typedef struct {
  Vec3 eulers_ff;
  double short_tether;
  double long_tether;
  double low_altitude;
  double high_altitude;
  PidParams short_tether_radial_pid;
  PidParams long_tether_radial_pid;
  PidParams short_tether_tangential_pid;
  PidParams low_altitude_long_tether_tangential_pid;
  PidParams high_altitude_long_tether_tangential_pid;
  double max_pos_angle_fb;
  double max_vel_angle_fb;
  Vec3 k_pilot;
  Vec3 min_angles;
  Vec3 max_angles;
  Vec3 transout_angles_cmd_crossfade_start_times;
  Vec3 transout_angles_cmd_crossfade_end_times;
  double transout_low_wind_pitch_cmd;
  double transout_high_wind_pitch_cmd;
  double transout_pitch_low_wind_speed;
  double transout_pitch_high_wind_speed;
  Vec3 transout_pqr_cmd_crossfade_duration;
  double transformdown_pitch_cmd_crossfade_time;
} HoverPositionParams;

typedef struct { Vec3 int_angles; } HoverPositionState;

#define HOVER_TENSION_PITCH_LIMIT_TABLE_LENGTH 4
typedef struct {
  double tension_min_set_point;
  PidParams tension_hard_pid;
  PidParams tension_soft_pid;
  double hard_spring_payout;
  double soft_spring_payout;
  double additional_pitch_cmd_rate_limit;
  double payout_table[HOVER_TENSION_PITCH_LIMIT_TABLE_LENGTH];
  double min_pitch_table[HOVER_TENSION_PITCH_LIMIT_TABLE_LENGTH];
  double max_pitch_table[HOVER_TENSION_PITCH_LIMIT_TABLE_LENGTH];
  double hover_drag_coeff;
  double max_pilot_extra_tension;
  double horizontal_tension_cmd_rate_limit;
  double horizontal_tension_joystick_roll_threshold;
  int32_t horizontal_tension_num_cycles_for_increment;
  double horizontal_tension_pilot_increment;
  double horizontal_tension_pilot_offset_fc;
  double horizontal_tension_pilot_offset_zeta;
  double horizontal_tension_max_pilot_offset;
} HoverTensionParams;

typedef struct {
  double additional_pitch_cmd_z1;
  double int_pitch;
  double horizontal_tension_cmd_z1;
  double horizontal_tension_pilot_offset_target;
  double horizontal_tension_pilot_offset;
  double horizontal_tension_pilot_offset_filter_state[2];
  double joystick_roll_z1;
  int32_t cycles_above_roll_threshold;
  bool horizontal_tension_increment_enabled;
} HoverTensionState;

#define HOVER_WINCH_PAY_OUT_TABLE_LENGTH 6
#define HOVER_WINCH_REEL_IN_TABLE_LENGTH 5
typedef struct {
  double winch_position_pay_out_table[HOVER_WINCH_PAY_OUT_TABLE_LENGTH];
  double winch_speed_pay_out_table[HOVER_WINCH_PAY_OUT_TABLE_LENGTH];
  double winch_position_reel_in_table[HOVER_WINCH_REEL_IN_TABLE_LENGTH];
  double winch_speed_reel_in_table[HOVER_WINCH_REEL_IN_TABLE_LENGTH];
  double contact_payout;
  double contact_winch_speed;
  double max_winch_speed;
  double max_winch_accel;
  double max_tension;
} HoverWinchParams;

typedef struct { double winch_vel_cmd_z1; } HoverWinchState;

typedef struct {
  HoverAltitudeParams altitude;
  HoverAnglesParams angles;
  HoverInjectParams inject;
  HoverModeParams mode;
  HoverOutputParams output;
  HoverPathParams path;
  HoverPositionParams position;
  HoverTensionParams tension;
  HoverWinchParams winch;
  HoverExperiments experiments;
} HoverParams;

typedef struct {
  HoverAltitudeState altitude;
  HoverAnglesState angles;
  HoverExperimentState experiment_state;
  HoverOutputState output;
  HoverPathState path;
  HoverPositionState position;
  HoverTensionState tension;
  HoverTetherElevationState tether_elevation;
  HoverWinchState winch;
  ThrustMoment thrust_moment_z1;
} HoverState;

#ifdef __cplusplus
extern "C" {
#endif

const char *HoverAscendGateToString(HoverAscendGate gate);
const char *HoverPayOutGateToString(HoverPayOutGate gate);
const char *HoverFullLengthGateToString(HoverFullLengthGate gate);
const char *HoverPrepTransformGsUpGateToString(HoverPrepTransformGsUpGate gate);
const char *HoverTransformGsUpGateToString(HoverTransformGsUpGate gate);
const char *HoverAccelGateToString(HoverAccelGate gate);
const char *HoverPrepTransformGsDownGateToString(
    HoverPrepTransformGsDownGate gate);
const char *HoverTransformGsDownGateToString(HoverTransformGsDownGate gate);
const char *HoverReelInGateToString(HoverReelInGate gate);
const char *HoverDescendGateToString(HoverDescendGate gate);
const char *HoverPerchedGateToString(HoverPerchedGate gate);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // CONTROL_HOVER_HOVER_TYPES_H_

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

#ifndef CONTROL_CROSSWIND_CROSSWIND_TYPES_H_
#define CONTROL_CROSSWIND_CROSSWIND_TYPES_H_

#include <stdbool.h>
#include <stdint.h>

#include "common/c_math/filter.h"
#include "common/c_math/vec3.h"
#include "control/actuator_types.h"
#include "control/crosswind/crosswind_playbook_types.h"
#include "control/experiments/crosswind_experiment_types.h"
#include "control/system_types.h"
#include "system/labels.h"

typedef enum {
  kCrosswindNormalGateForceSigned = -1,
  kCrosswindNormalGateSpeed,
  kCrosswindNormalGateTension,
  kCrosswindNormalGateAltitude,
  kCrosswindNormalGateAirspeed,
  kCrosswindNormalGateFlightMode,
  kNumCrosswindNormalGates
} CrosswindNormalGate;

typedef enum {
  kCrosswindPrepTransOutGateForceSigned = -1,
  kNumCrosswindPrepTransOutGates
} CrosswindPrepTransOutGate;

// TODO: This gate belongs with the hover controller, but
// since it relies on loop angle and whether the path has arrived we
// currently put it in the crosswind controller.
typedef enum {
  kCrosswindHoverTransOutGateForceSigned = -1,
  kCrosswindHoverTransOutGateAirspeed,
  kCrosswindHoverTransOutGateAlpha,
  kCrosswindHoverTransOutGatePathType,
  kCrosswindHoverTransOutGateStillAccelerating,
  kNumCrosswindHoverTransOutGates
} CrosswindHoverTransOutGate;

typedef enum {
  kCrosswindInnerMinAirspeed,
  kCrosswindInnerNominalAirspeed,
  kCrosswindInnerMaxAirspeed,
  kCrosswindInnerNumAirspeeds,
} CrosswindInnerAirspeeds;

typedef enum {
  kCrosswindLongitudinalInputElevator,
  kCrosswindLongitudinalInputMotorPitch,
  kNumCrosswindLongitudinalInputs
} CrosswindLongitudinalInputs;

typedef enum {
  kCrosswindLongitudinalStatePositionGroundZ,
  kCrosswindLongitudinalStateVelocityGroundZ,
  kCrosswindLongitudinalStateAngleOfAttack,
  kCrosswindLongitudinalStatePitchRate,
  kCrosswindLongitudinalStateIntegratedAngleOfAttack,
  kNumCrosswindLongitudinalStates
} CrosswindLongitudinalStates;

typedef enum {
  kCrosswindLateralInputAileron,
  kCrosswindLateralInputRudder,
  kCrosswindLateralInputMotorYaw,
  kNumCrosswindLateralInputs
} CrosswindLateralInputs;

typedef enum {
  kCrosswindLateralStateTetherRoll,
  kCrosswindLateralStateSideslip,
  kCrosswindLateralStateRollRate,
  kCrosswindLateralStateYawRate,
  kCrosswindLateralStateIntegratedTetherRoll,
  kCrosswindLateralStateIntegratedSideslip,
  kNumCrosswindLateralStates
} CrosswindLateralStates;

typedef enum { kLoopDirectionCw = -1, kLoopDirectionCcw = 1 } LoopDirection;

typedef enum {
  kCrosswindPathNormal,
  kCrosswindPathPrepareTransitionOut
} CrosswindPathType;

typedef struct {
  double transition_smooth_time;
  double min_airspeed;
  double max_airspeed;
  double ele_min;
  double ele_max;
  double ele_rate_lim;
  double azi_rate_lim;
  double loop_angle_path_switch_max;
  double loop_angle_path_switch_min;
  double transout_path_switch_crossfade_distance;
  double transout_airspeed_target;
  double corner_angle_low_wind_speed;
  double corner_angle_high_wind_speed;
  double low_wind_transout_airspeed_corner_angle;
  double high_wind_transout_airspeed_corner_angle;
  double transout_final_airspeed_target;
  double transout_final_airspeed_crossfade_angle;
  double transout_elevation_cmd;
  double transout_path_radius_target_threshold;
  double min_transout_altitude;
  double transout_airspeed_crossfade_time;
  double max_crosswind_y_position_for_slew;
} CrosswindPowerParams;

typedef struct {
  double raw_path_azimuth;
  double raw_path_elevation;
  Vec3 path_center_g_z1;
  CrosswindPathType path_type;
  double azi_setpoint;
  double dloop_angle;
  double airspeed_cmd_z1;
} CrosswindPowerState;

#define CROSSWIND_PATH_CURVATURE_TABLE_LENGTH 9
typedef struct {
  LoopDirection loop_dir;
  double k_tab[CROSSWIND_PATH_CURVATURE_TABLE_LENGTH];
  double commanded_curvature_time;
  double current_curvature_time;
  double time_horizon_speed_limit;
  double min_turning_radius;
  double preptransout_radius_cmd;
  double cw_z_for_vertical_paths;
  double fc_k_geom_curr;
  double fc_speed;
  double fc_k_aero_cmd;
  double fc_k_geom_cmd;
  PidParams crosstrack_pid;
  double crosswind_max_radius_error;
  double max_k_aero_error;
  double path_radius_rate;
} CrosswindPathParams;

typedef struct {
  double k_geom_curr_f_z1;
  double speed_f_z1;
  double k_geom_cmd_f_z1;
  double k_aero_cmd_f_z1;
  double current_curvature_time;
  double int_crosstrack;
  double path_radius_target;
} CrosswindPathState;

typedef struct {
  double alpha_min;
  double alpha_min_airspeed;
  double dalpha_dairspeed;
  double alpha_cmd_rate_min;
  double alpha_cmd_rate_max;
  double beta_cmd_rate_min;
  double beta_cmd_rate_max;
  double tether_roll_max_excursion_low;
  double tether_roll_max_excursion_high;
  double tether_roll_nom;
  double tether_roll_tension_low;
  double tether_roll_tension_high;
  double tether_roll_ff_amplitude;
  double tether_roll_ff_phase;
  double alpha_cmd_min;
  double alpha_cmd_max;
  double alpha_cmd_max_flare;
  double dCL_cmd_max;
  double beta_cmd_min;
  double beta_cmd_max;
  double fc_tension;
  double kp_tension_hf;
  double transout_tether_tension_cmd;
  double transout_tether_roll_cmd;
  double transout_tether_roll_cmd_rate_limit;
  double preptransout_alpha_cmd;
  double preptransout_alpha_rate;
  double transout_flare_alpha_cmd;
  double transout_flare_alpha_rate;
  double transout_flare_beta_cmd;
  double transout_flare_beta_rate;
  double transout_flare_airspeed;
} CrosswindCurvatureParams;

typedef struct {
  double tension_hpf_z1;
  double tension_z1;
  double alpha_cmd_z1;
  double beta_cmd_z1;
  double transout_flare_time;
} CrosswindCurvatureState;

typedef struct {
  double elevator_flap_ratio;
  double delevator_dalpha;
  double kp_flap;
  Mat3 B_flaps_to_pqr;
  double airspeed_table[kCrosswindInnerNumAirspeeds];
  double longitudinal_states_max[kNumCrosswindLongitudinalStates];
  double longitudinal_inputs_max[kNumCrosswindLongitudinalInputs];
  double longitudinal_gains_min_airspeed[kNumCrosswindLongitudinalInputs]
                                        [kNumCrosswindLongitudinalStates];
  double longitudinal_gains_nominal_airspeed[kNumCrosswindLongitudinalInputs]
                                            [kNumCrosswindLongitudinalStates];
  double longitudinal_gains_max_airspeed[kNumCrosswindLongitudinalInputs]
                                        [kNumCrosswindLongitudinalStates];
  double int_alpha_min;
  double int_alpha_max;
  double lateral_states_max[kNumCrosswindLateralStates];
  double lateral_inputs_max[kNumCrosswindLateralInputs];
  double lateral_gains_min_airspeed[kNumCrosswindLateralInputs]
                                   [kNumCrosswindLateralStates];
  double lateral_gains_nominal_airspeed[kNumCrosswindLateralInputs]
                                       [kNumCrosswindLateralStates];
  double lateral_gains_max_airspeed[kNumCrosswindLateralInputs]
                                   [kNumCrosswindLateralStates];
  double int_tether_roll_min;
  double int_tether_roll_max;
  double int_beta_min;
  double int_beta_max;
  PidParams airspeed_pid;
  double max_airspeed_error;
  double max_airspeed_control_power_gen;
  double max_airspeed_control_power_motor;
  double max_airspeed_control_thrust_rate;
  double initial_thrust;
  double airspeed_error_spoiler_on;
  double airspeed_error_spoiler_off;
  double delta_spoiler_on_rate;
  double delta_spoiler_off_rate;
  double delta_spoiler;
  double beta_harmonic_gain;
  double beta_harmonic_integrator_max;
  bool enable_acceleration_ff;
} CrosswindInnerParams;

typedef struct {
  double int_tether_roll_error;
  double int_alpha_error;
  double int_beta_error;
  double int_thrust;
  bool spoiler_on;
  double delta_spoiler_z1;
  double thrust_cmd_z1;
  double loop_angle_z1;
  double beta_harmonic_state[2];
  double accumulated_loop_angle;
} CrosswindInnerState;

#define CROSSWIND_RUDDER_LIMIT_BETAS 5
#define CROSSWIND_RUDDER_LIMIT_AIRSPEEDS 6
typedef struct {
  double rudder_limit_betas[CROSSWIND_RUDDER_LIMIT_BETAS];
  double rudder_limit_airspeeds[CROSSWIND_RUDDER_LIMIT_AIRSPEEDS];
  double rudder_limits_lower[CROSSWIND_RUDDER_LIMIT_AIRSPEEDS]
                            [CROSSWIND_RUDDER_LIMIT_BETAS];
  double rudder_limits_upper[CROSSWIND_RUDDER_LIMIT_AIRSPEEDS]
                            [CROSSWIND_RUDDER_LIMIT_BETAS];
  ThrustMoment thrust_moment_weights;
  double flap_offsets[kNumFlaps];
  double lower_flap_limits[kNumFlaps];
  double upper_flap_limits[kNumFlaps];
  double lower_flap_limits_flare[kNumFlaps];
  double release_wait_period;
  double release_aileron_cmd;
  bool adaptive_detwist_cmd;
} CrosswindOutputParams;

typedef struct {
  double detwist_loop_angle;
  int32_t detwist_rev_count;
  double prerelease_timer;
  bool prerelease_flag;
  Vec3 path_center_v_f_z1;
} CrosswindOutputState;

typedef struct {
  bool reset_int;
  bool loadcell_fault;
  bool alpha_beta_fault;
  bool spoiler_enabled;
} CrosswindFlags;

#define CROSSWIND_TRANS_OUT_SPEED_TABLE_LENGTH 6
typedef struct {
  double min_wing_speed;
  double min_tension;
  double max_wing_pos_g_z;
  double loop_angle_table[CROSSWIND_TRANS_OUT_SPEED_TABLE_LENGTH];
  double max_trans_out_speed_table[CROSSWIND_TRANS_OUT_SPEED_TABLE_LENGTH];
  double min_time_in_accel;
  double max_time_in_accel;
  double acc_slow_down_threshold;
  double min_airspeed_return_to_crosswind;
  double transout_max_time_in_flare;
  double transout_airspeed;
  double transout_alpha;
} CrosswindModeParams;

typedef struct {
  LoopDirection loop_dir;
  CrosswindPowerParams power;
  CrosswindPathParams path;
  CrosswindCurvatureParams curvature;
  CrosswindInnerParams inner;
  CrosswindOutputParams output;
  CrosswindModeParams mode;
  Playbook playbook;
  PlaybookFallbackParams playbook_fallback;
  bool enable_new_pitch_rate_cmd;
  CrosswindExperiments experiments;
} CrosswindParams;

typedef struct {
  CrosswindPowerState power;
  CrosswindPathState path;
  CrosswindCurvatureState curvature;
  CrosswindInnerState inner;
  CrosswindOutputState output;
  double tether_roll_cmd_zs[2];
  double playbook_fallback_crossfade;
} CrosswindState;

#ifdef __cplusplus
extern "C" {
#endif

const char *CrosswindNormalGateToString(CrosswindNormalGate gate);
const char *CrosswindPrepTransOutGateToString(CrosswindPrepTransOutGate gate);
const char *CrosswindHoverTransOutGateToString(CrosswindHoverTransOutGate gate);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // CONTROL_CROSSWIND_CROSSWIND_TYPES_H_

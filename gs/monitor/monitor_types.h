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

#ifndef GS_MONITOR_MONITOR_TYPES_H_
#define GS_MONITOR_MONITOR_TYPES_H_

#include <stdint.h>
#include "common/c_math/vec2.h"
#include "common/c_math/vec3.h"

// Flight stoplights turn yellow if values are high or low
// and red if values are very high or very low.
typedef struct { double very_low, low, high, very_high; } StoplightLimits;

typedef struct {
  int32_t telemetry_timeout;
  int32_t control_telemetries_timeout;
  int32_t control_slow_telemetries_timeout;
  int32_t controller_q7_slow_statuses_timeout;
  int32_t core_switch_statuses_timeout;
  int32_t core_switch_slow_statuses_timeout;
  int32_t drum_sensors_timeout;
  int32_t flight_computer_sensors_timeout;
  int32_t ground_station_weather_timeout;
  int32_t gs_gps_compass_timeout;
  int32_t gs_gps_observations_timeout;
  int32_t gs_gps_solution_timeout;
  int32_t joystick_timeout;
  int32_t joystick_monitor_timeout;
  int32_t loadcell_statuses_timeout;
  int32_t motor_statuses_timeout;
  int32_t platform_sensors_timeout;
  int32_t recorder_q7_slow_statuses_timeout;
  int32_t recorder_statuses_timeout;
  int32_t self_test_timeout;
  int32_t servo_statuses_timeout;
  int32_t slow_statuses_timeout;
  int32_t tether_down_timeout;
  int32_t winch_plc_timeout;
  int32_t wing_gps_novatel_observations_timeout;
  int32_t wing_gps_novatel_solutions_timeout;
  int32_t wing_gps_septentrio_observations_timeout;
  int32_t wing_gps_septentrio_solutions_timeout;
} MonAioParams;

typedef struct {
  int32_t no_udp, slow_udp, no_rf_link, slow_rf_link, no_gs_comms;
  int32_t high_omap_timeout, high_joystick_hold;
  StoplightLimits rssi;
  StoplightLimits invalid_rate, max_loop_time, mean_loop_time;
  MonAioParams aio;
} MonCommsParams;

typedef struct {
  StoplightLimits v_48, v_avionics, v_servo, v_release;
} MonAvionicsParams;

// TODO: Change monitor_filter names to match v_<source>.
typedef struct {
  double R_source;
  int32_t slow_bbox;
  StoplightLimits v_bus, v_12v_pri, v_12v_aux, v_380, state_of_charge;
} MonPowerParams;

typedef struct { StoplightLimits speed; } MonRotorsParams;

typedef struct {
  int32_t hot_pitot, very_hot_pitot;
  StoplightLimits rotors_temp, drives_temp, servos_temp;
} MonTempParams;

typedef struct {
  StoplightLimits wind, wind_gust, rel_direction;
} MonEnviroParams;

typedef struct {
  double max_gs_heading_error;
  double max_gs_pos_ecef_error;
  double max_attitude_diff;
  double max_mag_diff;
  StoplightLimits gyro_bias;
  StoplightLimits gyro_bias_crosswind_drift;
  double far_accel_start;
  double far_payout_at_perch;
  double winch_distance_disagreement_threshold;
  double winch_position_max_perched_disagreement;
  double close_range, mid_range, full_range;
  double max_gps_pos_sigma, max_gps_vel_sigma;
  double min_current_gps_receiver_time;
  double perch_azimuth_max_disagreement;
  double detwist_max_disagreement;
  double winch_max_disagreement;
  double lvlwind_max_disagreement;
  double gsg_max_disagreement;
  double lvlwind_shuttle_max_disagreement;
  StoplightLimits detwist_kite_loop_diff;
  StoplightLimits gs_azi_error_ht;
  StoplightLimits gs_azi_error_reel;
  StoplightLimits hover_angles[3];
  StoplightLimits hover_tether_angles[3];
  StoplightLimits crosswind_tether_angles[2];
  StoplightLimits tether_pitch;
  StoplightLimits alpha, beta, airspeed, airspeed_error;
  StoplightLimits alpha_error;
  StoplightLimits beta_error;
  StoplightLimits crosswind_altitude_agl;
  StoplightLimits gsg_bias_azi;
  StoplightLimits gsg_bias_ele;
  StoplightLimits glas_pos_diff;
} MonEstimatorParams;

#define MAX_CONSTRAINT_VERTICES 10

typedef struct {
  int32_t num_vertices_xy_cross;
  double xs_low[MAX_CONSTRAINT_VERTICES];
  double ys_low[MAX_CONSTRAINT_VERTICES];
  double xs_mid[MAX_CONSTRAINT_VERTICES];
  double ys_mid[MAX_CONSTRAINT_VERTICES];
  double xs_high[MAX_CONSTRAINT_VERTICES];
  double ys_high[MAX_CONSTRAINT_VERTICES];
  int32_t num_vertices_xz_cross;
  double xs_vertical[MAX_CONSTRAINT_VERTICES];
  double zs_vertical[MAX_CONSTRAINT_VERTICES];
  double azi_g2indicator;
  double plot_xlim[2];
  double plot_ylim[2];
} ConstraintWindowParams;

typedef struct {
  StoplightLimits alt_sens_diff, alt_kalman_cov, perch_offset;
  ConstraintWindowParams constraint_window;
} MonContParams;

// Preflight autochecks turn yellow if values are higher or lower
// than nominal.
typedef struct { double low, high; } AutoChecksLimits;

typedef struct {
  double high_diff;
  AutoChecksLimits magnitude, std;
} MonChecksAcc;

typedef struct { AutoChecksLimits magnitude, std; } MonChecksAccLowg;

typedef struct { AutoChecksLimits mean, std; } MonChecksGyro;

typedef struct { AutoChecksLimits magnitude, std, kiteloft[3]; } MonChecksMag;

typedef struct {
  AutoChecksLimits outboard_mean, center_mean, std;
} MonChecksLoadcell;

typedef struct {
  double high_std;
  AutoChecksLimits static_mean, diff_mean;
} MonChecksPitot;

typedef struct {
  double north_of_sherman, east_of_sherman;
  double south_of_cloverdale, west_of_cloverdale;
  AutoChecksLimits alt_offset, sigmas;
} MonChecksGPS;

typedef struct {
  MonChecksAcc acc;
  MonChecksAccLowg acc_lowg;
  MonChecksGyro gyro;
  MonChecksMag mag;
  MonChecksLoadcell loadcell;
  MonChecksPitot pitot;
  MonChecksGPS gps;
} MonAutochecksParams;

typedef struct MonFiltLoopTimeParams {
  double fc;
  double max_hold_time;
} MonFiltLoopTimeParams;

typedef struct MonFiltServoParams {
  double fc_temp, fc_load;
} MonFiltServoParams;

typedef struct MonFilterParams {
  double fc_1min, fc_5min;
  double fc_invalid_rate, fc_gps_sigmas, fc_imu, fc_lc, fc_pitot;
  MonFiltLoopTimeParams loop_time;
  MonFiltServoParams servo;
} MonFilterParams;

typedef struct MonLandingZoneParams {
  Vec2 primary_vertices[4];
  Vec2 secondary_vertices[4];
  Vec2 pr_pad_vertices[14];
  double glideslope;
  double threshold;
  double overrun;
} MonLandingZoneParams;

typedef struct TetherSphereParams {
  double radius_nom;
  StoplightLimits deviation;
  int history_len;
  double latch_duration_sec;
} TetherSphereParams;

typedef struct MonTetherParams {
  double proof_load;
  StoplightLimits tension;
  StoplightLimits tension_crosswind;
  StoplightLimits tension_hover;
  TetherSphereParams tether_sphere;
} MonTetherParams;

typedef struct MonThermalLimitsParams {
  StoplightLimits aiomon_default;
  StoplightLimits bridle_box;
  StoplightLimits capacitor_box;
  StoplightLimits detwist;
  StoplightLimits eop_boxes;
  StoplightLimits fcu_aio_atom_q7;
  StoplightLimits fcu_aio_boardcom;
  StoplightLimits fcu_flight_recorder;
  StoplightLimits fcu_imu_chip;
  StoplightLimits gs_slip_ring;
  StoplightLimits lipo_batteries;
  StoplightLimits motor_controller_board;
  StoplightLimits motor_controller_capacitor;
  StoplightLimits motor_controller_heat_plate;
  StoplightLimits motor_controller_module;
  StoplightLimits motor_stator_core;
  StoplightLimits motor_stator_winding;
  StoplightLimits motor_rotor;
  StoplightLimits rpx_soft_bridle;
  StoplightLimits satcontainer_ambient;
  StoplightLimits servo_controller;
  StoplightLimits tether_electronics;
  StoplightLimits wingside_radio;
} MonThermalLimitsParams;

#define NUM_JOYSTICK_HOLD_SAT 1000
#define NUM_10S 2500
typedef struct MonitorParams {
  MonCommsParams comms;
  MonAvionicsParams avionics;
  MonPowerParams power;
  MonRotorsParams rotors;
  MonTempParams temperature;
  MonEnviroParams environment;
  MonEstimatorParams est;
  MonContParams control;
  MonAutochecksParams autochecks;
  MonFilterParams filter;
  MonLandingZoneParams landing_zone;
  MonTetherParams tether;
  MonThermalLimitsParams thermal;
  bool v1_monitor_use_sim_sensors;
} MonitorParams;

#endif  // GS_MONITOR_MONITOR_TYPES_H_

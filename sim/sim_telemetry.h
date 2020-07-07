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

#ifndef SIM_SIM_TELEMETRY_H_
#define SIM_SIM_TELEMETRY_H_

#include <stdint.h>

#include "common/c_math/quaternion.h"
#include "common/c_math/vec3.h"
#include "control/actuator_types.h"
#include "control/sensor_types.h"
#include "control/system_types.h"
#include "sim/sim_types.h"
#include "system/labels.h"

#ifdef __cplusplus
extern "C" {
#endif

// State objects.

typedef struct {
  int32_t start_ind, end_ind;
  double L_start, L_end;
  Vec3 Xg_start, Vg_start, Xg_end, Vg_end;
  Vec3 Xg_nodes[MAX_TETHER_NODES + 2];
  Vec3 Vg_nodes[MAX_TETHER_NODES + 2];
  Vec3 Fg_nodes[MAX_TETHER_NODES + 2];
  Vec3 Fg_aero_nodes[MAX_TETHER_NODES + 2];
  Vec3 start_force_g, end_force_g;
  double aero_power;
  int32_t released;
  double Xv_start_elevation, Xv_start_azimuth;
} TetherTelemetry;

typedef struct {
  double length;
  double tension;
} ConstraintTelemetry;

typedef struct {
  Vec3 Xg;
  Vec3 Vg;
  Vec3 Ab;
  Vec3 Vb;
  Vec3 dVb_center_of_mass;
  Vec3 omega;
  Vec3 domega;
  Vec3 eulers;
  Quat q;
  Mat3 dcm_g2b;
  double flaps[kNumFlaps];
  Vec3 wind_g;
  Vec3 wind_omega_g;
  ForceMoment fm_aero;
  ForceMoment fm_gravity;
  ForceMoment fm_tether;
  ForceMoment fm_rotors;
  ForceMoment fm_disturb;
  ForceMoment fm_total;
  ForceMoment fm_blown_wing;

  double CL;
  double CD;

  double reynolds_number;
  ThrustMoment rotor_thrust_moment;
  ApparentWindSph apparent_wind_b;
  TetherForceSph tether_force_b;
  double constraint_tension;
  Vec3 proboscis_pos_g;
} WingTelemetry;

typedef struct {
  double omega;
  double thrust;
  double aero_power;
  double aero_torque;
  double rotor_accel;
  Vec3 gyro_moment;
  Vec3 local_apparent_wind_b;
  double v_freestream;
  double motor_torque;
} RotorTelemetry;

typedef struct {
  double theta_p;
  double omega_p;
  double theta_wd;
  double omega_wd;
  int32_t levelwind_engaged;
  double tether_free_length;
  Vec3 anchor_pos_g;
  Vec3 gsg_pos_g;
  Vec3 levelwind_pos_g;
} PerchTelemetry;

typedef struct {
  double theta_cmd;
  double omega_cmd;
} WinchTelemetry;

typedef struct {
  double v_wing;
  double i_teth;
  double P_elec;
  double int_motor_vel_errs[kNumMotors];
} PowerSysTelemetry;

typedef struct {
  double motor_torques[kNumMotors];
  double motor_torque_cmds[kNumMotors];
  double motor_torque_upper_limits[kNumMotors];
  double motor_torque_lower_limits[kNumMotors];
  int32_t motor_constraints[kNumMotors];
  double block_voltages[4];
  double speed_correction[4];
  double tether_current;
  double filtered_tether_current;
  double ground_voltage;
  double block_powers[4];
  double voltage_correction[kNumMotors];
  double voltage_correction_state_x[kNumMotors];
} StackedPowerSysTelemetry;

// Sensors.

typedef struct {
  Vec3 actual_acc;
  Vec3 actual_gyro;
  Vec3 actual_mag;
  Vec3 acc_bias_parent;
  Vec3 gyro_bias_parent;
  Vec3 acc;
  Vec3 gyro;
  Vec3 mag;
  double actual_P_stat;
  double P_stat;
} ImuTelemetry;

typedef struct { double tensions[kNumLoadcellSensors]; } LoadcellTelemetry;

typedef struct { double P_dyn, P_stat, P_alpha, P_beta; } PitotTelemetry;

typedef struct { double theta_l, theta_r; } GlasTelemetry;

// TODO: Remove tophat related variables in GsgTelemetry when tophat
// model is deprecated.
typedef struct {
  double ele, azi, twist;
  double perch_azi, levelwind_ele;
  double gsg_yoke, gsg_termination;
} GsgTelemetry;

typedef struct {
  Vec3 wind_g;
  Vec3 measured_wind_ws;
} WindSensorTelemetry;

typedef struct {
  double throttle, roll, pitch, yaw;
  int32_t tri_switch;
  int32_t momentary_switch;
} JoystickTelemetry;

typedef struct {
  Vec3 pos, pos_sigma, vel, vel_sigma;
  int32_t pos_type, vel_type;
  int32_t time_of_week_ms;
} GpsTelemetry;

typedef struct {
  int32_t num_rotors;
  double rotor_speeds[kNumMotors];
} RotorSensorTelemetry;

typedef struct {
  int32_t num_servos;
  double shaft_angles[kNumServos];
  double shaft_angular_vels[kNumServos];
  double external_shaft_torques[kNumServos];
  double motor_powers[kNumServos];
} ServoSensorTelemetry;

typedef struct { uint16_t command_sequence[kNumControllers]; } CommsTelemetry;

typedef struct {
  double azimuth;
  double azimuth_vel;
  double detwist_angle;
  double detwist_vel;
  Mat3 dcm_v2p;
  double drum_angle;
  double drum_omega;
  double mclaren_azi_vel_cmd;
  double mclaren_drum_vel_cmd;
  double azi_target;
  int32_t mode;
  int32_t transform_stage;
  int32_t n_state_machine;
  int32_t n_hpu_mode;
  double azi_velocity_dir;
  double total_torque;
  double tether_torque;
  double brake_torque;
  double brake_torque_cmd;
  double a_error;
  double wing_azi;
  double gs_azi;
  bool levelwind_engaged;
} Gs02Telemetry;

typedef struct {
  Vec3 Xg;
  Vec3 Vg;
  Vec3 Xg_center_of_mass;
  Vec3 Vg_center_of_mass;
  Quat q;
  Vec3 omega;
  Mat3 dcm_g2v;
  ForceMoment fm_hydro;
  ForceMoment fm_tether;
  ForceMoment fm_gravity;
  ForceMoment fm_mooring;
  ForceMoment fm_total;
  double water_line_pos_z_v;
  double yaw_angle_from_eq;
  Vec3 vessel_origin_accel_g;
} BuoyTelemetry;

typedef struct {
  double wave_transl_coord[SEA_N_GRID_SEGMENTS];
  double wave_elev_g[SEA_N_GRID_SEGMENTS];
} SeaTelemetry;

// Everything.

typedef struct {
  double time;
  int64_t aio_idle_usec;
  int64_t integration_usec;
  WingTelemetry wing;
  RotorTelemetry rotors[kNumMotors];
  TetherTelemetry tether;
  PerchTelemetry perch;
  WinchTelemetry winch;
  PowerSysTelemetry power_sys;
  StackedPowerSysTelemetry stacked_power_sys;
  ImuTelemetry imus[kNumWingImus];
  LoadcellTelemetry loadcell;
  PitotTelemetry pitots[kNumPitotSensors];
  GlasTelemetry glas;
  GsgTelemetry gsg;
  WindSensorTelemetry wind_sensor;
  JoystickTelemetry joystick;
  GpsTelemetry gps[kNumWingGpsReceivers];
  RotorSensorTelemetry rotor_sensor;
  ServoSensorTelemetry servo_sensor;
  ConstraintTelemetry constraint;
  CommsTelemetry comms;
  Gs02Telemetry gs02;
  BuoyTelemetry buoy;
  SeaTelemetry sea;
} SimTelemetry;

extern SimTelemetry sim_telem;

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // SIM_SIM_TELEMETRY_H_

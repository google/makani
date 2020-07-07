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

#ifndef CONTROL_CONTROL_TELEMETRY_H_
#define CONTROL_CONTROL_TELEMETRY_H_

#include <stdint.h>

#include "avionics/common/avionics_messages.h"
#include "avionics/linux/q7_slow_status_types.h"
#include "common/c_math/quaternion.h"
#include "common/c_math/vec2.h"
#include "common/c_math/vec3.h"
#include "control/actuator_types.h"
#include "control/avionics/avionics_interface_types.h"
#include "control/control_types.h"
#include "control/estimator/estimator_types.h"
#include "control/fault_detection/fault_detection_types.h"
#include "control/sensor_types.h"
#include "control/system_types.h"
#include "system/labels.h"

// Number of control cycles at which to send various messages.
#define CONTROL_SLOW_TELEMETRY_DECIMATION 100
#define CONTROL_TELEMETRY_DECIMATION 10
#define Q7_SLOW_STATUS_DECIMATION 100

typedef struct {
  bool autonomous_flight_enabled;
  double takeoff_countdown_timer;
  bool inside_launch_window;
  bool inside_landing_window;
  bool landing_fault_detected;
  int32_t desired_flight_mode;
} PlannerTelemetry;

typedef struct {
  int32_t initializing;
  EstimatorPositionBaroEstimate baro;
  EstimatorPositionBaroState pos_baro_state;
  EstimatorPositionGlasEstimate glas;
  EstimatorPositionCorrections position_corrections;
  EstimatorAttitudeCorrections attitude_corrections[kNumWingImus];
  GsgData gsg_bias;
  int32_t current_gps_receiver;
  EstimatorPositionGpsEstimate gps[kNumWingGpsReceivers];
  Vec3 cov_vel_g;
  Vec3 cov_pos_g;
  ApparentWindSph apparent_wind_est;
  ApparentWindSph apparent_wind_tether;
  ApparentWindSph apparent_wind_pitot;
  ApparentWindSph apparent_wind_cf;
  Quat q_g2b[kNumWingImus];
  Vec3 gyro_biases[kNumWingImus];
  Vec3 cov_attitude_err[kNumWingImus];
  Vec3 cov_gyro_bias[kNumWingImus];
  Vec3 acc_b_estimates[kNumWingImus];
  double rho_instantaneous;
  // Fixed values used after the initialization period.
  GroundStationEstimate ground_station;
} EstimatorTelemetry;

typedef struct {
  double horizontal_tension;
  double horizontal_tension_cmd;
  double horizontal_tension_pilot_offset;
  double tension_cmd;
  double pitch_ff;
  double pitch_fb;
  double pitch_cmd;
  double pitch_min;
  double pitch_max;
  double int_pitch;
  double elevation_ff;
  double elevation_fb;
  double elevation_cmd;
  double elevation_cmd_reel;
  double elevation_min;
  double elevation_max;
  double int_kite_elevation;
  Vec3 raw_wing_pos_g_cmd;
  Vec3 perched_pos_g;
  Vec3 wing_pos_g_cmd;
  Vec3 wing_vel_g_cmd;
  double thrust_ff;
  double thrust_fb;
  double int_thrust;
  double int_boost;
  Vec3 wing_pos_b_error;
  Vec3 wing_vel_b_error;
  Vec3 angles_ff;
  Vec3 angles_fb;
  Vec3 angles_cmd;
  Vec3 pqr_cmd;
  Vec3 int_angles;
  Vec3 angles;
  Vec3 moment_ff;
  Vec3 moment_fb;
  Vec3 moment_cmd;
  Vec3 int_moment;
  double elevator_pitch_moment;
  double rudder_yaw_moment;
  double delta_ele_ff;
  double delta_ele_fb;
  double gain_ramp_scale;
  double tether_elevation_cmd;
} HoverTelemetry;

typedef struct {
  double ti_origin_azimuth;
  Vec3 wing_pos_ti, wing_vel_ti, eulers_ti2b;
  double wing_vel_ti_y_cmd, wing_pos_ti_y_cmd;
  double aero_climb_angle, aero_climb_angle_cmd;
  double angle_of_attack_cmd, pitch_rate_b_cmd;
  double tension_cmd;
  double radial_vel_ti_cmd, radial_vel_ti;
  Vec3 eulers_ti2cmd, axis_b2cmd, pqr_cmd;
  double int_angle_of_attack;
  double int_roll;
} TransInTelemetry;

typedef struct {
  double elevation;
  int32_t path_type;
  Vec3 path_center_g;
  double path_radius_target;
  double loop_angle;
  double azi_offset;
  double azi_target;
  Vec3 eulers_cw;
  Vec2 target_pos_cw;
  Vec2 current_pos_cw;
  double k_geom_cmd;
  double k_aero_cmd;
  double k_geom_curr;
  double k_aero_curr;
  Vec3 pqr_cmd_new;
  Vec3 pqr_cmd_old;
  Vec3 pqr_cmd;
  double alpha_cmd;
  double beta_cmd;
  double tether_roll_cmd;
  double airspeed_cmd;
  double thrust_ff;
  double thrust_fb;
  double int_elevator;
  double int_rudder;
  double int_aileron;
  double int_thrust;
  double int_crosstrack;
  double int_tether_roll_error;
  double int_beta_error;
  double beta_harmonic_state[2];
  Vec3 aero_force_b;
  double loop_count;
} CrosswindTelemetry;

typedef struct {
  int32_t leader;
  int32_t proposed_flight_mode;
} SyncTelemetry;

typedef struct {
  double pitch_error;
  double roll_error;
  bool auto_glide_active;
  bool release_latched;
} ManualTelemetry;

typedef struct {
  int32_t controller_label;
  int32_t init_state;
  double time;
  double flight_mode_time;
  int32_t flight_mode;
  int32_t autonomous_flight_mode;
  int32_t flight_mode_gates[kNumFlightModes];
  SyncTelemetry sync;
  FaultMask faults[kNumSubsystems];
  int64_t start_usec;
  int64_t finish_usec;
  int64_t loop_usec;
  int64_t max_loop_usec;
  ControlInput control_input;
  StateEstimate state_est;
  PlannerTelemetry planner;
  EstimatorTelemetry estimator;
  HoverTelemetry hover;
  TransInTelemetry trans_in;
  CrosswindTelemetry crosswind;
  ManualTelemetry manual;
  Deltas deltas;
  Deltas deltas_avail;
  ThrustMoment thrust_moment;
  ThrustMoment thrust_moment_avail;
  double v_app_locals[kNumMotors];
  ControlOutput control_output;
  ControllerCommandMessage command_message;
  ControllerSyncMessage sync_message;
  AvionicsSequenceNumbers sequence_numbers;
  double detwist_loop_count;
  double gs_azi_target_raw;
} ControlTelemetry;

typedef struct {
  int32_t controller_label;  // See ControllerLabel.
  int32_t flight_plan;       // See FlightPlan.
  int32_t gs_model;          // See GroundStationModel.
  HitlConfiguration hitl_config;
  int32_t test_site;    // See TestSite.
  int32_t wing_serial;  // See WingSerial.
  BuildInfo build_info;
  int32_t control_opt;  // See ControlOption.
} ControlSlowTelemetry;

// ControlTelemetry is decimated and sent down to the ground at a
// reduced rate, while ControlDebugMessage is logged at full rate on
// the flight recorder.  Long term it may be desirable to
// differentiate these messages; however, currently they just use the
// same struct.
typedef ControlTelemetry ControlDebugMessage;

#ifdef __cplusplus
extern "C" {
#endif

ControlDebugMessage *GetControlDebugMessage(void);
SmallControlTelemetryMessage *GetSmallControlTelemetry(void);
ControlSlowTelemetry *GetControlSlowTelemetry(void);
ControlTelemetry *GetControlTelemetry(void);
EstimatorTelemetry *GetEstimatorTelemetry(void);
HoverTelemetry *GetHoverTelemetry(void);
ManualTelemetry *GetManualTelemetry(void);
TransInTelemetry *GetTransInTelemetry(void);
CrosswindTelemetry *GetCrosswindTelemetry(void);
SyncTelemetry *GetSyncTelemetry(void);
Q7SlowStatusMessage *GetQ7SlowStatusMessage(void);

void ControlTelemetryToSmallControlTelemetryMessage(
    const ControlTelemetry *in, SmallControlTelemetryMessage *out);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // CONTROL_CONTROL_TELEMETRY_H_

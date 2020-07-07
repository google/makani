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

#ifndef SIM_SIM_TYPES_H_
#define SIM_SIM_TYPES_H_

#include <linux/limits.h>
#include <stdbool.h>
#include <stdint.h>

#include "avionics/common/gps_receiver.h"
#include "common/c_math/coord_trans.h"
#include "common/c_math/force_moment.h"
#include "common/c_math/mat2.h"
#include "common/c_math/quaternion.h"
#include "common/c_math/util.h"
#include "common/c_math/vec3.h"
#include "control/sensor_types.h"
#include "control/system_types.h"
#include "sim/physics/aero_types.h"
#include "system/labels.h"

// This trivial struct works around a bug in the bindings-dsl package:
// https://github.com/jwiegley/bindings-dsl/issues/12.
// This is needed so that dynobutt can read sim parameters and compare models,
// so that we can generate optimal power curves based on the simulator model.
// Increasing limit from default to handle long local abs paths to
// downloaded TurbSim databases.
typedef struct { char name[PATH_MAX]; } DatabaseName;

typedef enum {
  kPerchContactorLabelForceSigned = -1,
  kPerchContactorPortTalon,
  kPerchContactorStarboardTalon,
  kPerchContactorPeg,
  kNumPerchContactors
} PerchContactorLabel;

typedef enum {
  kGroundContactorLabelForceSigned = -1,
  kGroundContactorPortWheel,
  kGroundContactorStarboardWheel,
  kGroundContactorPortTusk,
  kGroundContactorStarboardTusk,
  kGroundContactorRearSkid,
  kNumGroundContactors
} GroundContactorLabel;

typedef enum {
  kSimOptConstraintSystem = 1 << 0,
  kSimOptFaults = 1 << 1,
  kSimOptGroundContact = 1 << 2,
  kSimOptImperfectSensors = 1 << 3,
  kSimOptPerch = 1 << 4,
  kSimOptPerchContact = 1 << 5,
  // Value 1 << 6 currently unused. Available for future use.
  kSimOptStackedPowerSystem = 1 << 7,
  kSimOptTiedDown = 1 << 8,
  // Value 1 << 9 currently unused. Available for future use.
  kSimOptExitOnCrash = 1 << 10,
} SimOption;

typedef enum {
  kIecCaseNormalWindProfile,
  kIecCaseNormalTurbulenceModel,
  kIecCaseExtremeWindSpeed1Year,
  kIecCaseExtremeWindSpeed50Year,
  kIecCaseExtremeOperatingGust,
  kIecCaseExtremeTurbulenceModel,
  kIecCaseExtremeDirectionChange,
  kIecCaseExtremeCoherentGustWithDirectionChange,
  kIecCaseExtremeWindShearVertical,
  kIecCaseExtremeWindShearHorizontal
} IecDesignLoadCase;

typedef enum {
  kSimJoystickTypeProgrammed,
  kSimJoystickTypeSoftware,
  kSimJoystickTypeHardware
} SimJoystickType;

typedef enum {
  kSimJoystickUpdateNone,
  kSimJoystickUpdateThrottle,
  kSimJoystickUpdateRoll,
  kSimJoystickUpdatePitch,
  kSimJoystickUpdateYaw,
  kSimJoystickUpdateSwitchUp,
  kSimJoystickUpdateSwitchMiddle,
  kSimJoystickUpdateSwitchDown,
  kSimJoystickUpdateReleasePulled,
  kSimJoystickUpdateReleaseNotPulled,
} SimJoystickUpdateType;

typedef enum {
  kSimJoystickThrottleManual,
  kSimJoystickThrottleOff,
  kSimJoystickThrottleEnterCrosswind,
  kSimJoystickThrottleCrosswindNormal,
  kSimJoystickThrottleRemainInHover,
  kSimJoystickThrottleReturnToPerch,
  kNumSimJoystickThrottles,
} SimJoystickThrottle;

typedef enum {
  kSimFaultNoFault,
  kSimFaultActuatorZero,
  kSimFaultMeasurementBiasDriftMean,
  kSimFaultMeasurementBiasDriftRate,
  kSimFaultMeasurementBiasOffset,
  kSimFaultMeasurementFixValue,
  kSimFaultMeasurementHoldCurrent,
  kSimFaultMeasurementNoiseRescale,
  kSimFaultMeasurementRescale,
  kSimFaultDisturbanceBodyForceSine,
  kSimFaultDisturbanceBodyForceStep,
  kSimFaultDisturbanceBodyTorqueSine,
  kSimFaultDisturbanceBodyTorqueStep,
  kSimFaultGpsDropout,
  kSimFaultGpsSolutionStateChange,
  kSimFaultServoFixValue,
  kSimFaultServoHoldCurrent,
} SimFaultType;

typedef enum {
  kSimOdeSolverGslRk2,
  kSimOdeSolverGslRkck,
  kSimOdeSolverGslRkf45,
  kSimOdeSolverGslMsadams,
  kSimOdeSolverOdeintRkck
} SimOdeSolverType;

typedef enum {
  kWindModelForceSigned = -1,
  kWindModelDatabase,
  kWindModelDatabaseWithDrydenTurbulence,
  kWindModelDrydenTurbulence,
  kWindModelIec,
  kWindModelNoTurbulence,
  kNumWindModels
} WindModel;

typedef enum {
  kSimMotorLimitNone,
  kSimMotorLimitGroundPower,
  kSimMotorLimitPhaseCurrent,
  kSimMotorLimitPower
} SimMotorLimit;

typedef struct {
  SimOdeSolverType type;
  double initial_time_step;
  double abs_tolerance;
  double rel_tolerance;
} SimOdeSolverParams;

typedef struct {
  double v_in, v_out, v_r1, v_r2;
  double hub_height_agl;
  double rotor_diameter;
  double event_t_start;
  IecDesignLoadCase load_case;
} IecSimParams;

typedef struct {
  double t_update;
  double offset;
} WindSpeedOffset;

#define MAX_WIND_SPEED_UPDATES 10
typedef struct {
  int32_t num_updates;
  WindSpeedOffset offsets[MAX_WIND_SPEED_UPDATES];
} WindSpeedUpdate;

// TODO: Make an EnvironmentParams?
typedef struct {
  double air_density;
  double dynamic_viscosity;
  DatabaseName wind_database;
  double wind_database_initial_time;
  double wind_database_y_offset;
  WindModel wind_model;
  double wind_speed;
  WindSpeedUpdate wind_speed_update;
  double wind_speed_update_rate_limit;
  double wind_direction;
  double wind_elevation;
  double wind_shear_exponent;
  double wind_shear_ref_height_agl;
  double wind_veer_start_height_agl;
  double wind_veer_end_height_agl;
  double wind_veer;
} PhysSimParams;

typedef struct {
  double mass_scale;
  double moment_of_inertia_scale[3];
  Vec3 center_of_mass_offset;
} MassPropUncertainties;

typedef struct {
  Vec3 Xg_0, Vb_0, omega_0;
  Quat q_0;
  MassPropUncertainties mass_prop_uncertainties;
} WingSimParams;

typedef struct {
  double noise_level;
  double bias;
  double scale;
  double bound_low;
  double bound_high;
  double quantization;
} SensorModelParams;

typedef struct {
  double c, c_deltad, c_alphad, c_alphad_deltad;
} HingeMomentCoeffs;

#define MAX_SMALL_DEFLECTION_DATABASES 3
#define MAX_LARGE_DEFLECTION_DATABASES 10
typedef struct {
  bool merge_databases;
  bool force_use_both_databases;
  bool use_nonlinear_flaps;
  double empirical_pitching_moment_correction;
  bool use_spoilers;
  DatabaseName small_deflection_databases[MAX_SMALL_DEFLECTION_DATABASES];
  DatabaseName large_deflection_databases[MAX_LARGE_DEFLECTION_DATABASES];
  DatabaseName spoiler_offset_database;
  AeroCoeffs force_coeff_w_scale_factors;
  AeroCoeffs moment_coeff_b_scale_factors;
  AeroCoeffOffsets coeff_offsets;
  double flap_offsets[kNumFlaps];
  double low_alpha_stall_angle;
  double high_alpha_stall_angle;
  double low_beta_stall_angle;
  double high_beta_stall_angle;
  double linear_to_stalled_blending_angle;
  HingeMomentCoeffs hinge_moment_coeffs[kNumFlaps];
  double min_avl_flap_angles[kNumFlaps];
  double max_avl_flap_angles[kNumFlaps];
  double positive_rudder_deflection_scaling_threshold;
  double positive_rudder_deflection_scaling;
  double negative_rudder_deflection_scaling_threshold;
  double negative_rudder_deflection_scaling;
} AeroSimParams;

typedef struct {
  Vec3 pos;
  double spring_const, damping_coeff, friction_coeff;
} ContactorParams;

typedef struct {
  ContactorParams ground_contactors[kNumGroundContactors];
  ContactorParams perch_contactors[kNumPerchContactors];
} ContactSimParams;

typedef struct {
  DatabaseName database_names[kNumMotors];
  DatabaseName database_3d_names[kNumMotors];
  bool apply_3d_rotor_tables;
  double fc_hitl_rotor_acc;
  bool apply_blown_wing_effect;
  double thrust_vectoring_angle;
  Vec3 thrust_vectoring_pos_b;
  double full_blown_wing_freestream_vel;
  double zero_blown_wing_freestream_vel;
  double min_freestream_vel_for_thrust_coeff;
} RotorSimParams;

typedef struct {
  double modulation_limit;
  double phase_current_cmd_limit;
  double iq_cmd_lower_limit;
  double iq_cmd_upper_limit;
  double Lq;
  double Ld;
  double Rs;
  double flux_linkage;
  int32_t num_pole_pairs;
  double hysteresis_loss_coefficient;
  double omega_loss_coefficient_cubic;
  double omega_loss_coefficient_sq;
  double omega_loss_coefficient_lin;
  double rds_on;
  double specific_switching_loss;
  double fixed_loss_sq_coeff;
  double fixed_loss_lin_coeff;
  double switching_frequency;
} MotorParams;

typedef struct {
  double current_filter_cutoff_freq;
  double ground_voltage_pole;
  double min_ground_voltage_compensation;
  double max_ground_voltage_compensation;
  double kp_rotor_vel_err;
  double ki_rotor_vel_err;
  double rotor_vel_err_pole;
  double rotor_vel_err_torque_pole;
  double kp_voltage_err;
  double voltage_control_pole;
  double voltage_control_zero;
  double fc_stacking_speed_correction;
  double kp_stacking_speed_correction;
  double cap_drain_conductance;
  MotorParams motor;
  double omega_cmd_rate_limit;
  double speed_cmd_pole;
  double min_tether_current;
  double kp_excess_tether_current;
  double voltage_average_upper_sat;
} PowerSysSimParams;

typedef struct { Vec3 pos_ecef; } GroundFrameSimParams;

typedef struct {
  int32_t num_nodes;
  double stiff_len_lim;
  double longitudinal_damping_ratio_active;
  double longitudinal_damping_ratio_staged;
  double bending_damping_ratio_active;

  ContactorParams ground_contactor_template;
} TetherSimParams;

typedef struct {
  double frequencies[3];
  double damping_ratios[3];
  Vec3 acc_scale, gyro_scale;
} ImuMountSimParams;

typedef struct {
  Vec3 pos;
  double conductor_spacing;
} HighVoltageHarnessSimParams;

#define NUM_MAGNETOMETER_HARMONICS 3
typedef struct {
  double glitch_period;
  double glitch_duration;
  Vec3 glitch_magnitudes;
  double harmonics_amplitudes[NUM_MAGNETOMETER_HARMONICS];
  double harmonics_frequencies[NUM_MAGNETOMETER_HARMONICS];
} MagnetometerNoiseSimParams;

typedef struct {
  double random_walk_scale;
  double markov_process_scale;
  double markov_process_cutoff_freq;
} BiasParams;

typedef struct {
  double ts;
  double delay;
  Quat q_m2actual;
  MagnetometerNoiseSimParams mag_noise;
  BiasParams acc_bias, gyro_bias;
  SensorModelParams acc_sensor[3];
  SensorModelParams gyro_sensor[3];
  SensorModelParams mag_sensor[3];
  SensorModelParams stat_sensor;
} ImuSimParams;

typedef struct {
  double ref_model_cutoff_freq, ref_model_rate_lim;
  double ref_model_min_position_limit;
  double ref_model_max_position_limit;
  double kp, kd;
  double bus_voltage, current_lim;
} ServoDriveSimParams;

#define NUM_SERVO_FRICTION_TABLE 5
typedef struct {
  double motor_torque_constant, motor_inductance, motor_resistance;
  double moment_of_inertia;
  int32_t num_elec_poles;
  double gear_ratio;
  double friction_angular_vel_table[NUM_SERVO_FRICTION_TABLE];
  double friction_torque_table[NUM_SERVO_FRICTION_TABLE];
  ServoDriveSimParams servo_drive;
} ServoSimParams;

typedef struct {
  double ts;
  SensorModelParams sensors[kNumLoadcellSensors];
} LoadcellSimParams;

typedef struct {
  bool include_rotor_inflow;
  double total_rotor_area;
  double induced_vel_at_pitot_fraction;
  double local_pressure_coeff_offset;
  Vec3 rotor_axis;
  double ts;
  double pitch_offset;
  double yaw_offset;
  SensorModelParams stat_sensor;
  SensorModelParams alpha_sensor;
  SensorModelParams beta_sensor;
  SensorModelParams dyn_sensor;
} PitotSimParams;

typedef struct { double sample_time; } WindSensorSimParams;

typedef struct {
  double ts;
  SensorModelParams gsg_azi_sensor[kNumDrums];
  SensorModelParams gsg_ele_sensor[kNumDrums];
  SensorModelParams gsg_twist_sensor[kNumDrums];
} GsgSimParams;

typedef struct {
  double ts, ts_rtcm_update_noise;
  double pos_delay, vel_delay;
  double antenna_dir_dropout_rate;
  double acc_dropout_rate;
  double dropin_rate_coeffs[2];
  double sigma_ratio;
  double pos_rtcm_update_noise_scale, vel_rtcm_update_noise_scale;
  Vec3 sigma_per_dropout_rate;
  double pos_sigma_scales[kNumGpsSolutionTypes];
  double vel_sigma_scales[kNumGpsSolutionTypes];
} GpsSimParams;

typedef struct {
  double ts;
  Vec3 pos_sigma;
  Vec3 vel_sigma;
  double heading_sigma;
  double pitch_sigma;
  double length_sigma;
} GsGpsSimParams;

typedef struct {
  Vec2 center_panel;
  double radius;
  double z_extents_p[2];
} SinglePanelSimParams;

typedef struct {
  SinglePanelSimParams port;
  SinglePanelSimParams starboard;

  Vec3 origin_pos_p;
  Mat3 dcm_p2panel;

  double y_extents_panel[2];
} PanelSimParams;

typedef struct {
  double levelwind_radius;
  Vec2 levelwind_hub_p;
  double levelwind_engage_min_tension;
  Mat2 A;
  Mat2 B;
  double theta_p_0;
  bool initialize_in_crosswind_config;
  PanelSimParams panel;
  double ts;
  SensorModelParams levelwind_ele_sensor[kNumPlatforms];
  SensorModelParams perch_azi_sensor[kNumPlatforms];
} PerchSimParams;

typedef struct {
  double azi_cmd_filter_omega;
  double azi_cmd_filter_zeta;
  double azi_vel_cmd_ff_gain;
  double azi_error_max;
  double azi_vel_cmd_kp;
  double azi_vel_cmd_rate_limit;
  double azi_offset_from_wing;
  double drum_angle_upper_limit;
  double max_drum_accel;
  double little_dead_zone;
} Gs02SimMcLarenReelParams;

#define NUM_GS02_SIM_TRANSFORM_STAGES 5

typedef struct {
  double azi_dead_zone_half_width;
  double azi_targets_ht2reel[NUM_GS02_SIM_TRANSFORM_STAGES];
  double azi_tols_ht2reel[NUM_GS02_SIM_TRANSFORM_STAGES];
  double azi_targets_reel2ht[NUM_GS02_SIM_TRANSFORM_STAGES];
  double azi_offset_from_wing;
  double azi_tols_reel2ht[NUM_GS02_SIM_TRANSFORM_STAGES];
  double azi_nominal_vel;
  double azi_max_accel;
  double azi_decel_ratio;

  double winch_dead_zone_half_width;
  double winch_targets_ht2reel[NUM_GS02_SIM_TRANSFORM_STAGES];
  double winch_tols_ht2reel[NUM_GS02_SIM_TRANSFORM_STAGES];
  double winch_targets_reel2ht[NUM_GS02_SIM_TRANSFORM_STAGES];
  double winch_tols_reel2ht[NUM_GS02_SIM_TRANSFORM_STAGES];
  double winch_nominal_vel;
  double winch_max_accel;
  double winch_decel_ratio;

  double detwist_targets_ht2reel[NUM_GS02_SIM_TRANSFORM_STAGES];
  double detwist_targets_reel2ht[NUM_GS02_SIM_TRANSFORM_STAGES];
  double detwist_max_vel;
} Gs02SimMcLarenTransformParams;

typedef struct {
  double m_max_azi_ht;
  double a_control_threshold_azi_ht;
  double m_control_threshold_azi_ht;
  double n_demand_azi_ht;
  double a_control_tolerance_azi_ht;
  double n_control_tolerance_azi_ht;
  double t_threshold_wait_azi_ht;
  double omega_nom;
  double test_threshold;
  double tau_spin;
  double tau_stop;
  double Iz_gndstation;
  double k_spin;
  double k_stop;
  double detwist_max_vel;
} Gs02SimMcLarenHighTensionParams;

typedef struct {
  double ts;
  double detwist_setpoint;
  Gs02SimMcLarenReelParams reel;
  Gs02SimMcLarenTransformParams transform;
  Gs02SimMcLarenHighTensionParams high_tension;
} Gs02SimMcLarenControllerParams;

typedef struct {
  Gs02SimMcLarenControllerParams mclaren;
  double azi_accel_kp;
  double detwist_angle_kp;
  double winch_drive_natural_freq;
  double winch_drive_damping_ratio;
  double dx_dtheta_main_wrap;
  double dx_dtheta_wide_wrap;
  double initial_drum_angle;
  double initial_platform_azi;
  PanelSimParams panel;
  double platform_radius;
  double wrap_start_posx_drum;
  double wrap_transition_posx_drum;
  bool init_tether_tension;
  double prox_sensor_tether_free_length;
  double min_levelwind_angle_for_tether_engagement;
} Gs02SimParams;

typedef struct {
  double t_update;
  SimJoystickUpdateType type;
  SimJoystickThrottle enum_value;
  double value;
} SimJoystickUpdate;

#define MAX_JOYSTICK_UPDATES 10
typedef struct {
  SimJoystickType joystick_type;
  int32_t num_updates;
  SimJoystickUpdate updates[MAX_JOYSTICK_UPDATES];
  double throttle_settings[kNumSimJoystickThrottles];
} SimJoystickParams;

#define MAX_COMPONENT_NAME_LENGTH 40
#define MAX_FAULT_EVENT_PARAMETERS 4
typedef struct {
  double t_start;
  double t_end;
  char component[MAX_COMPONENT_NAME_LENGTH + 1];
  SimFaultType type;
  int32_t num_parameters;
  double parameters[MAX_FAULT_EVENT_PARAMETERS];
} SimFaultEvent;

#define MAX_FAULT_EVENTS 10
typedef struct {
  int32_t num_fault_events;
  SimFaultEvent fault_events[MAX_FAULT_EVENTS];
} SimFaultParams;

typedef struct {
  double k_theta;
  double k_omega;
  double max_torque;
} WinchSimParams;

typedef struct {
  Vec3 anchor_pos_g;
  double initial_length;
  double maximum_length;
  double maximum_rate;
  double slack_command;
  double tension_command;
  double fc_pilot_response;
  double spring_constant;
} ConstraintSimParams;

typedef struct {
  double max_speed;
  double max_torque;
  double torque_scales[kNumMotors];
} DynoSimParams;

typedef struct {
  double torsional_damping_x_scale;
  double torsional_damping_y_scale;
  double torsional_damping_z_scale;
  double buoyancy_damping_coeff_scale;
  double Ca_scale;
  double Dh_scale;
  double ki_scale;
} BuoyHydrodynamicsUncertainties;

typedef struct {
  double torsional_damping_x;
  double torsional_damping_y;
  double torsional_damping_z;
  double buoyancy_damping_coeff;
  double Cd;
  double Ca;
  double Dh;
  double ki;
  BuoyHydrodynamicsUncertainties uncertainties;
} BuoyHydrodynamicsSimParams;

typedef struct {
  double yaw_equilibrium_heading_delta;
  double torsional_stiffness_z_scale;
  double mooring_attach_pos_x_delta;
  double mooring_attach_pos_y_delta;
  double mooring_attach_pos_z_delta;
  double kt0_scale;
  double kt1_scale;
  double ct_scale;
} BuoyMooringModelUncertainties;

typedef struct {
  double yaw_equilibrium_heading;
  Vec3 mooring_attach_v;
  double torsional_stiffness_z;
  double kt0;
  double kt1;
  double ct;
  BuoyMooringModelUncertainties uncertainties;
} BuoyMooringLineSimParams;

typedef struct {
  double msl_pos_z_g;
  BuoyHydrodynamicsSimParams hydrodynamics;
  BuoyMooringLineSimParams mooring_lines;
  MassPropUncertainties mass_prop_uncertainties;
  Quat q_0;
  Vec3 omega_0;
  Vec3 Xg_0;
  Vec3 Vg_0;
} BuoySimParams;

#define SEA_N_GRID_SEGMENTS 100
typedef struct {
  bool use_waves;
  double significant_height;
  double peak_period;
  double gamma;
  double water_depth;
  int number_of_waves;
  double initial_frequency;
  double cutoff_frequency;
  double initial_frequency_sampling_delta;
  double wave_heading_ned;
  double water_density;
  double grid_half_length;
} SeaSimParams;

// SimTelemetry grows very quickly with this number.
#define MAX_TETHER_NODES 30

typedef struct {
  SimOption sim_opt;
  SimOdeSolverParams ode_solver;
  double sim_time;
  double sensor_blackout_duration;
  double telemetry_sample_period;
  uint32_t random_seed_offset;
  PhysSimParams phys_sim;
  IecSimParams iec_sim;
  WingSimParams wing_sim;
  AeroSimParams aero_sim;
  ContactSimParams contact_sim;
  GroundFrameSimParams ground_frame_sim;
  ImuMountSimParams wing_imu_mount_sim;
  ImuMountSimParams gs_imu_mount_sim;
  HighVoltageHarnessSimParams high_voltage_harness_sim;
  TetherSimParams tether_sim;
  PerchSimParams perch_sim;
  RotorSimParams rotor_sim;
  ConstraintSimParams constraint_sim;
  DynoSimParams dyno_sim;
  SimFaultParams faults_sim;

  Gs02SimParams gs02_sim;

  // Actuators.
  PowerSysSimParams power_sys_sim;
  ServoSimParams servos_sim[kNumServos];
  WinchSimParams winch_sim;

  // Sensors.
  ImuSimParams wing_imus_sim[kNumWingImus];
  ImuSimParams gs_imus_sim[kNumGsImus];
  LoadcellSimParams loadcell_sim;
  PitotSimParams pitots_sim[kNumPitotSensors];
  WindSensorSimParams wind_sensor_sim;
  GpsSimParams gps_sim[kNumGpsReceiverTypes];
  GsGpsSimParams gs_gps_sim;
  GsgSimParams gsg_sim;
  SimJoystickParams joystick_sim;

  // Offshore buoy parameters.
  BuoySimParams buoy_sim;

  // Sea parameters.
  SeaSimParams sea_sim;
} SimParams;

#endif  // SIM_SIM_TYPES_H_

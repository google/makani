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

#ifndef SIM_MODELS_ACTUATORS_GROUND_STATION_V2_H_
#define SIM_MODELS_ACTUATORS_GROUND_STATION_V2_H_

#include <string>

#include "avionics/common/avionics_messages.h"
#include "common/c_math/vec3.h"
#include "common/macros.h"
#include "sim/models/actuators/actuator.h"
#include "sim/models/ground_station_util.h"
#include "sim/physics/contactor.h"
#include "sim/physics/reference_frame.h"
#include "sim/sim_types.h"
#include "sim/state.h"

// Base class for controllers of individual modes for GS02.
class Gs02ModeController : public Model {
 public:
  virtual ~Gs02ModeController() {}

  // Sets inputs that are common to all mode controllers.
  void SetCommonInputs(double t, double azi_pos, double azi_vel,
                       double drum_pos, double drum_vel, double wing_azi);

  // Enable or disable the controller. When disabled, the controller should
  // update states for rate limits, etc., so they behave sanely when the
  // controller is re-enabled.
  void Enable(double t) { enabled_.DiscreteUpdate(t, true); }
  void Disable(double t) { enabled_.DiscreteUpdate(t, false); }

  // Output azimuth and drum velocity commands.
  double azi_vel_cmd() const { return azi_vel_cmd_.val(); }
  double drum_vel_cmd() const { return drum_vel_cmd_.val(); }

  double azi_pos_val() const { return azi_pos_.val(); }
  double wing_azi_val() const { return wing_azi_.val(); }

 protected:
  Gs02ModeController(const std::string &name__, double ts);

  // Inputs from the ground station.
  DiscreteState<double> azi_pos_;
  DiscreteState<double> azi_vel_;
  DiscreteState<double> drum_pos_;
  DiscreteState<double> drum_vel_;
  DiscreteState<double> wing_azi_;

  // Whether this controller is enabled.
  DiscreteState<bool> enabled_;

  // Output commands.
  DiscreteState<double> azi_vel_cmd_;
  DiscreteState<double> drum_vel_cmd_;
};

// Provides azimuth, drum and detwist velocity commands during reel mode.
class ReelController : public Gs02ModeController {
 public:
  explicit ReelController(const Gs02SimMcLarenControllerParams &params,
                          double drum_radius);
  virtual ~ReelController() {}

  void set_azi_cmd_dead_zone(double t, double val) {
    azi_cmd_dead_zone_.DiscreteUpdate(t, val);
  }

  void set_drum_linear_vel_cmd_from_wing(double t, double val) {
    drum_linear_vel_cmd_from_wing_.DiscreteUpdate(t, val);
  }

  void set_levelwind_engaged(double t, bool val) {
    levelwind_engaged_.DiscreteUpdate(t, val);
  }

 private:
  void DiscreteStepHelper(double t) override;

  const Gs02SimMcLarenControllerParams &params_;
  const double drum_radius_;

  // Inputs from the ground station.
  DiscreteState<double> drum_linear_vel_cmd_from_wing_;

  // States for each component of the azimuth velocity controller.
  DiscreteState<std::array<double, 2>> azi_cmd_z_;
  DiscreteState<double> azi_cmd_delay_state_;
  DiscreteState<double> azi_cmd_dead_zone_;
  DiscreteState<bool> azi_in_dead_zone_z1_;
  DiscreteState<double> azi_delay_state_;
  DiscreteState<double> azi_rate_limit_state_;

  // Drum velocity controller state.
  DiscreteState<double> drum_vel_rate_limit_state_;

  DiscreteState<bool> levelwind_engaged_;

  DISALLOW_COPY_AND_ASSIGN(ReelController);
};

// Provides azimuth, drum and detwist velocity commands during transform mode.
class TransformController : public Gs02ModeController {
 public:
  explicit TransformController(const Gs02SimMcLarenControllerParams &params);
  virtual ~TransformController() {}

  double detwist_pos_cmd() const { return detwist_pos_cmd_.val(); }

  double detwist_max_vel() const { return params_.detwist_max_vel; }

  bool transform_complete() const { return transform_complete_.val(); }

  int32_t stage() const { return transform_stage_.val(); }

  void StartTransform(double t, bool ht2reel);

  void Publish() const override;

  void set_unpause_transform(double t, bool val) {
    unpause_transform_.DiscreteUpdate(t, val);
  }
  bool unpause_transform() { return unpause_transform_.val(); }

 private:
  void DiscreteStepHelper(double t) override;

  const Gs02SimMcLarenTransformParams &params_;

  // Rate limit states.
  DiscreteState<double> azi_rate_limit_state_;
  DiscreteState<double> winch_rate_limit_state_;

  DiscreteState<double> detwist_pos_cmd_;
  DiscreteState<bool> unpause_transform_;

  // Other stuff
  DiscreteState<int32_t> transform_stage_;
  DiscreteState<bool> ht2reel_;
  DiscreteState<bool> transform_complete_;

  DISALLOW_COPY_AND_ASSIGN(TransformController);
};

//  Provides azimuth, drum and detwist velocity commands during the ground
//  station high-tension mode.  The ground station high-tension mode is active
//  when the kite is in the crosswind mode.
class HighTensionController : public Gs02ModeController {
 public:
  explicit HighTensionController(const Gs02SimMcLarenControllerParams &params);
  virtual ~HighTensionController() {}

  double detwist_pos_cmd() const { return detwist_pos_cmd_.val(); }

  double detwist_max_vel() const { return params_.detwist_max_vel; }

  void set_detwist_pos_cmd_from_wing(double t, double val) {
    detwist_pos_cmd_from_wing_.DiscreteUpdate(t, val);
  }

  double brake_torque() const { return brake_torque_.val(); }
  double brake_torque_cmd() const { return brake_torque_cmd_.val(); }
  double tether_torque() const { return tether_torque_.val(); }
  double total_torque() const { return total_torque_.val(); }
  double azi_velocity_dir() const { return azi_velocity_dir_.val(); }
  double a_error() const { return a_error_.val(); }
  int32_t n_hpu_mode() const { return n_hpu_mode_.val(); }
  int32_t n_hpu_mode_demand() const { return n_hpu_mode_demand_.val(); }
  int32_t n_state_machine() const { return n_state_machine_.val(); }

  void set_tether_torque(double t, Vec3 gsg_pos_p, Vec3 tether_force_p) {
    Vec3 tether_torques_;
    Vec3Cross(&gsg_pos_p, &tether_force_p, &tether_torques_);
    tether_torque_.DiscreteUpdate(t, tether_torques_.z);
  }

  Gs02SimMcLarenHighTensionParams mclarenparams() { return params_; }

 private:
  void DiscreteStepHelper(double t) override;
  void RunControlSwitchingLogic(double t);
  void RunBrakeModel(double t);
  void RunHpuModel(double t);
  void RunControlLaw(double t);

  const Gs02SimMcLarenHighTensionParams &params_;

  // High tension controller state machine variables.
  enum SwitchingLogicState : int32_t {
    kBaseCase = 0,
    kWaitForPositiveTetherOverload = 1,
    kWaitForHpuPositive = 2,
    kAzimuthPositiveRotation = 3,
    kActiveBraking = 5,
    kWaitForNegativeTetherOverload = 6,
    kWaitForHpuNegative = 7,
    kAzimuthNegativeRotation = 8
  };

  enum HpuModeState : int32_t {
    kBrakesOn = 1,
    kBrakesRegulated = 2,
    kBrakesOff = 3
  };

  bool active_braking_first_entry_;  // kActiveBraking first pass flag [].
  double active_braking_t0_;         // kActiveBraking first pass timer [sec].
  double angular_rate_tol_;          // Tolerance on angular rate [rad/sec].
  double hpu_cmd_change_t0_;         // Time when HPU command changes [sec].
  const double hpu_delay_;  // Time delay on HPU command actuation [sec].
  const double ht_ts_;      // High tension controller sampling time [sec].

  // High tension controller modeling variables.
  DiscreteState<double> brake_torque_;   // Brake torque [Nm].
  DiscreteState<double> tether_torque_;  // Tether torque on GS02 azimuth [Nm].
  DiscreteState<double> brake_torque_cmd_;    // Commanded brake torque [Nm].
  DiscreteState<int32_t> n_hpu_mode_demand_;  // Commanded HPU mode [].
  DiscreteState<double> azi_velocity_dir_;    // Commanded azimuth direction [].
  DiscreteState<int32_t> n_state_machine_;    // Controller state [].
  DiscreteState<double> total_torque_;  // Total torque on GS02 azimuth [Nm].
  DiscreteState<int32_t> n_hpu_mode_;   // HPU mode [].
  DiscreteState<int32_t> n_hpu_mode_last_;  // Previous HPU mode [].
  DiscreteState<double> a_error_;           // Azimuth position error [rad].

  // Detwist commands.
  DiscreteState<double> detwist_pos_cmd_;
  DiscreteState<double> detwist_pos_cmd_from_wing_;

  DISALLOW_COPY_AND_ASSIGN(HighTensionController);
};

class GroundStationV2Base : public Actuator {
 public:
  GroundStationV2Base(const ReferenceFrame &ned_frame,
                      const ReferenceFrame &parent_frame,
                      const Gs02Params &params,
                      const Gs02SimParams &sim_params__);
  virtual ~GroundStationV2Base() {}

  // Initializes the ground station with a particular perch azimuth and drum
  // angle.
  virtual void Init(double azimuth, double drum_angle) = 0;

  void SetFromAvionicsPackets(const AvionicsPackets &avionics_packets) override;

  // Returns the current free length of the tether, i.e. the length that is not
  // on the drum.
  double CalcTetherFreeLength() const;

  virtual void SetTetherForce(const Vec3 *tether_start_force) {
    tether_force_g_.set_val(*tether_start_force);
    ned_frame_.RotateTo(platform_frame_.val(), tether_force_g_.val(),
                        &tether_force_p_);
  }

  virtual Vec3 gsg_pos_p() {
    wd_frame_.val().TransformTo(platform_frame_.val(),
                                ReferenceFrame::kPosition,
                                gs02_params().gsg_pos_drum, &gsg_pos_p_);
    return gsg_pos_p_;
  }

  Vec3 tether_force_g() { return tether_force_g_.val(); }

  Vec3 tether_force_p() { return tether_force_p_; }

  // Calculates the position and velocity in NED of the point at which the
  // tether is anchored to the drum.
  void CalcTetherAnchorPosVelNed(Vec3 *pos_ned, Vec3 *vel_ned) const;

  // Returns the minimum drum angle, at which the tether is fully payed out.
  double MinDrumAngle() const;

  const ContactSurface &panel_surface() { return panel_surface_; }

  void Publish() const override;

  // Calculates tether elevation and GSG angles from the tether departure
  // direction.
  void TetherDirToAngles(const Vec3 &tether_dir_v, double *tether_ele_v,
                         double *tether_azi_v, double *gsg_yoke,
                         double *gsg_termination) const;

  const ReferenceFrame &platform_frame() const { return platform_frame_.val(); }
  const Mat3 &dcm_v2p() const { return dcm_v2p_.val(); }
  const ReferenceFrame &gsg0_frame() const { return gsg0_frame_.val(); }

  virtual double platform_azi() const = 0;
  virtual double platform_azi_vel() const = 0;
  virtual double dplatform_azi_vel() const = 0;
  virtual double drum_angle() const = 0;
  virtual double detwist_angle() const = 0;

  GroundStationMode mode() const { return mode_.val(); }
  GroundStationMode mode_cmd() const { return mode_cmd_.val(); }
  uint8_t transform_stage() const {
    return static_cast<uint8_t>(transform_stage_.val());
  }
  bool unpause_transform() { return unpause_transform_.val(); }
  bool levelwind_engaged() const { return levelwind_engaged_.val(); }
  void SetLevelwindEngaged(double tether_elevation_p);
  virtual bool prox_sensor_active() const = 0;

 protected:
  virtual void UpdateDerivedStates();
  const Gs02SimParams &sim_params() const { return sim_params_; }
  const Gs02Params &gs02_params() const { return params_; }
  double azi_cmd_target() const { return azi_cmd_target_.val(); }
  double azi_cmd_dead_zone() const { return azi_cmd_dead_zone_.val(); }
  double drum_linear_vel_cmd_from_wing() const {
    return drum_linear_vel_cmd_from_wing_.val();
  }
  double detwist_pos_cmd_from_wing() const {
    return detwist_pos_cmd_from_wing_.val();
  }
  void set_mode(double t, GroundStationMode val) {
    mode_.DiscreteUpdate(t, val);
  }
  void set_transform_stage(double t, int32_t val) {
    transform_stage_.DiscreteUpdate(t, val);
  }

 private:
  virtual double drum_omega() const = 0;

  // Calculates the tether free length for a particular drum angle.
  double CalcTetherFreeLengthInternal(double drum_angle) const;

  const Gs02Params &params_;
  const Gs02SimParams &sim_params_;

  // Helper for determining the length of tether wrapped around the racetrack.
  const RacetrackTetherIntegrator racetrack_integrator_;

  // NED and parent (vessel) frames.
  const ReferenceFrame &ned_frame_;
  const ReferenceFrame &parent_frame_;

  // Input values.
  State<double> azi_cmd_target_;
  State<double> azi_cmd_dead_zone_;
  State<double> drum_linear_vel_cmd_from_wing_, detwist_pos_cmd_from_wing_;
  State<GroundStationMode> mode_cmd_;
  State<Vec3> tether_force_g_;
  State<bool> unpause_transform_;
  State<bool> levelwind_engaged_;

  // Variable to hold value of tether force transformed to the platform frame.
  Vec3 tether_force_p_;

  // Variable to hold value of gsg position transformed to the platform frame.
  Vec3 gsg_pos_p_;

  // Discrete states.
  DiscreteState<GroundStationMode> mode_;
  DiscreteState<int32_t> transform_stage_;

  // Derived values.
  State<ReferenceFrame> platform_frame_;
  State<Mat3> dcm_v2p_;

  // The wd0 frame is an internal concept of this class. It is equal to the
  // drum frame at zero drum angle, and it remains fixed with respect to the
  // platform frame as the drum rotates.
  State<ReferenceFrame> wd0_frame_;
  // Drum frame.
  State<ReferenceFrame> wd_frame_;

  // GSG/Detwist frame.
  State<ReferenceFrame> gsg0_frame_;

  // This frame has the same origin as platform_frame_ but is pitched by
  // perch_sim_params_.panel.pitch_angle radians.
  State<ReferenceFrame> panel_frame_;

  // Contact surface for the perch panels.
  ContactSurface panel_surface_;

  DISALLOW_COPY_AND_ASSIGN(GroundStationV2Base);
};

class GroundStationV2 : public GroundStationV2Base {
  friend class GroundStationV2Test;

 public:
  GroundStationV2(const ReferenceFrame &ned_frame,
                  const ReferenceFrame &parent_frame, const Gs02Params &params,
                  const Gs02SimParams &sim_params__);
  virtual ~GroundStationV2() {}

  double platform_azi() const override { return platform_azi_.val(); }
  double platform_azi_vel() const override { return platform_azi_vel_.val(); }
  double dplatform_azi_vel() const override {
    return platform_azi_vel_.deriv();
  }
  double drum_angle() const override { return drum_angle_.val(); }
  double detwist_angle() const override { return detwist_angle_.val(); }
  bool prox_sensor_active() const override { return prox_sensor_active_.val(); }

  void Init(double azimuth, double drum_angle) override;

  void Publish() const override;

 private:
  double drum_omega() const override { return drum_omega_.val(); }

  void CalcDerivHelper(double /*t*/) override;
  void DiscreteStepHelper(double t) override;

  void AddInternalConnections(ConnectionStore *connections) override;
  void UpdateModeControllerCommands();

  // Mode controllers that provide azimuth and drum velocity commands.
  ReelController reel_controller_;
  TransformController transform_controller_;
  HighTensionController high_tension_controller_;

  // Continuous states.
  ContinuousState<double> platform_azi_;
  ContinuousState<double> platform_azi_vel_;
  ContinuousState<double> drum_angle_;
  ContinuousState<double> detwist_angle_;
  ContinuousState<double> drum_omega_;
  ContinuousState<double> ddrum_omega_dt_;

  // Derived values.
  State<double> azi_vel_cmd_;
  State<double> drum_vel_cmd_;
  State<double> detwist_vel_;

  // Discrete states.
  DiscreteState<GroundStationMode> mode_z1_;
  DiscreteState<bool> ht2reel_;
  DiscreteState<bool> prox_sensor_active_;

  DISALLOW_COPY_AND_ASSIGN(GroundStationV2);
};

class HitlGroundStationV2 : public GroundStationV2Base {
 public:
  HitlGroundStationV2(const ReferenceFrame &ned_frame,
                      const ReferenceFrame &parent_frame,
                      const Gs02Params &params,
                      const Gs02SimParams &sim_params__);
  virtual ~HitlGroundStationV2() {}

  void SetFromAvionicsPackets(const AvionicsPackets &avionics_packets) override;

  double platform_azi() const override { return platform_azi_.val(); }
  double platform_azi_vel() const override { return dplatform_azi_.val(); }
  double dplatform_azi_vel() const override { return dplatform_azi_vel_.val(); }
  double drum_angle() const override { return drum_angle_.val(); }
  double detwist_angle() const override { return detwist_angle_.val(); }
  bool prox_sensor_active() const override { return prox_sensor_active_.val(); }

  void Init(double azimuth, double drum_angle) override;

 private:
  void CalcDerivHelper(double /*t*/) override;
  void DiscreteStepHelper(double t) override;

  double drum_omega() const override { return ddrum_angle_.val(); }

  void AddInternalConnections(ConnectionStore *connections) override;

  ContinuousState<double> platform_azi_;
  ContinuousState<double> platform_azi_vel_;
  DiscreteState<double> dplatform_azi_;
  DiscreteState<double> dplatform_azi_vel_;
  DiscreteState<double> int_err_platform_azi_;
  ContinuousState<double> drum_angle_;
  ContinuousState<double> drum_angle_vel_;
  DiscreteState<double> ddrum_angle_;
  DiscreteState<double> ddrum_angle_vel_;
  ContinuousState<double> detwist_angle_vel_;
  DiscreteState<double> ddetwist_angle_;
  DiscreteState<double> ddetwist_angle_vel_;
  DiscreteState<double> int_err_drum_angle_;

  ContinuousState<double> detwist_angle_;
  DiscreteState<bool> prox_sensor_active_;

  State<GroundStationMode> actual_mode_;
  State<uint8_t> actual_transform_stage_;
  State<double> actual_platform_azi_;
  State<double> actual_drum_angle_;
  State<double> actual_detwist_angle_;
  State<bool> actual_prox_sensor_active_;

  DISALLOW_COPY_AND_ASSIGN(HitlGroundStationV2);
};

#endif  // SIM_MODELS_ACTUATORS_GROUND_STATION_V2_H_

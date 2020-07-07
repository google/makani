// Copyright 2020 Makani Technologies LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "sim/models/full_system.h"

#include <gflags/gflags.h>
#include <stdint.h>
#include <string.h>

#include "avionics/common/gps_receiver.h"
#include "avionics/motor/firmware/flags.h"
#include "common/c_math/mat3.h"
#include "common/c_math/vec3.h"
#include "sim/models/actuators/rotor.h"
#include "sim/models/actuators/servo.h"
#include "sim/models/actuators/tether_release.h"
#include "sim/models/actuators/winch.h"
#include "sim/models/environment.h"
#include "sim/models/perch.h"
#include "sim/models/rigid_bodies/buoy.h"
#include "sim/models/rigid_bodies/platform.h"
#include "sim/models/rigid_bodies/wing.h"
#include "sim/models/sea.h"
#include "sim/models/sensors/gps.h"
#include "sim/models/sensors/ground_station_v2_sensors.h"
#include "sim/models/sensors/gs_gps.h"
#include "sim/models/sensors/gsg.h"
#include "sim/models/sensors/imu.h"
#include "sim/models/sensors/joystick.h"
#include "sim/models/sensors/loadcell.h"
#include "sim/models/sensors/pitot.h"
#include "sim/models/sensors/rotor_sensor.h"
#include "sim/models/sensors/sensor.h"
#include "sim/models/sensors/servo_sensor.h"
#include "sim/models/sensors/winch_sensor.h"
#include "sim/models/sensors/wind_sensor.h"
#include "sim/models/simple_power_sys.h"
#include "sim/models/spring_constraint.h"
#include "sim/models/stacked_power_sys.h"
#include "sim/models/tether.h"
#include "sim/physics/contactor.h"
#include "sim/physics/ground_frame.h"
#include "sim/sim_messages.h"
#include "sim/sim_params.h"
#include "sim/sim_telemetry.h"
#include "sim/sim_types.h"
#include "system/labels.h"
#include "system/labels_util.h"

FullSystem::FullSystem(const SystemParams &system_params,
                       const SimParams &sim_params, FaultSchedule *faults,
                       const AvionicsPackets &external_avionics_packets,
                       SimSensorMessage *sensor_message)
    : BaseSystemModel("FullSystem", system_params, sim_params, faults),
      environment_(sim_params_.iec_sim, sim_params_.phys_sim,
                   system_params_.phys, system_params_.wind_sensor,
                   system_params_.ground_frame),
      ground_frame_(environment_.ned_frame(), system_params_.ground_frame,
                    sim_params_.ground_frame_sim),
      wing_imu_mount_(sim_params_.wing_imu_mount_sim),
      gs_imu_mount_(sim_params_.gs_imu_mount_sim),
      wing_(environment_, ground_frame_, system_params_.wing,
            sim_params_.aero_sim, sim_params_.wing_sim, faults_),
      tether_(environment_, system_params_.tether, sim_params_.tether_sim),
      rotors_(),
      servos_(),
      tether_release_(nullptr),
      winch_(nullptr),
      perch_(nullptr),
      platform_(nullptr),
      gs02_(nullptr),
      buoy_(nullptr),
      sea_(nullptr),
      power_sys_(nullptr),
      constraint_(nullptr),
      high_voltage_harness_(nullptr),
      external_avionics_packets_(external_avionics_packets),
      avionics_packets_(new_input_value(), "avionics_packets"),
      sensor_message_(sensor_message),
      dyno_command_packet_(new_derived_value(), "dyno_command") {
  Init();
  SetupDone();
}

void FullSystem::Init() {
  const HitlConfiguration &hitl_config = system_params_.hitl.config;

  sub_models_.push_back(&environment_);
  sub_models_.push_back(&wing_);

  connections_.Add(0, [this](double /*t*/) {
    avionics_packets_.set_val(external_avionics_packets_);
  });

  connections_.Add(2, [this](double /*t*/) {
    Vec3 wind_g;
    Vec3 wind_omega_g;
    this->environment_.CalcWind(this->wing_.Xg(), &wind_g, &wind_omega_g);
    this->wing_.set_wind_g(wind_g);
    this->wing_.set_wind_omega_g(wind_omega_g);
  });

  // Build tether release.
  if (hitl_config.tether_release_level == kActuatorHitlLevelReal) {
    tether_release_.reset(new HitlTetherRelease());
  } else {
    tether_release_.reset(new TetherRelease());
  }
  sub_models_.push_back(tether_release_.get());
  connections_.Add(1, [this](double /*t*/) {
    this->tether_release_->SetFromAvionicsPackets(this->avionics_packets());
  });

  // Build tether.
  sub_models_.push_back(&tether_);
  connections_.Add(3, [this](double /*t*/) {
    if (this->tether_release_->released()) {
      this->tether_.set_Xg_end(this->tether_.Xg_last_node());
      this->tether_.set_Vg_end(this->tether_.Vg_last_node());
    } else {
      Vec3 pos, vel;
      Wing::CalcBridlePoint(this->wing_.Xg(), this->wing_.Vb(),
                            this->wing_.omega(), this->wing_.dcm_g2b(),
                            system_params_.wing, this->tether_.Xg_last_node(),
                            this->tether_.Vg_last_node(), &pos, &vel);
      this->tether_.set_Xg_end(pos);
      this->tether_.set_Vg_end(vel);
    }
  });
  int32_t tether_force_index = wing_.AddExternalForce("tether");
  connections_.Add(4, [this, tether_force_index](double /*t*/) {
    ForceMomentPos tether_force;
    ForceMomentPos fmx_in = this->tether_.end_force_moment();
    ForceMomentPosPoseTransform(&this->wing_.dcm_g2b(), &this->wing_.Xg(),
                                &fmx_in, &tether_force);
    this->wing_.set_external_forces(tether_force_index, tether_force);
  });

  // Set up the constraint system
  if (*g_sim.sim_opt & kSimOptConstraintSystem) {
    constraint_ = std::unique_ptr<SpringConstraint>(
        new SpringConstraint(sim_params_.constraint_sim));
    sub_models_.push_back(constraint_.get());
    int32_t constraint_force_index = wing_.AddExternalForce("constraint");

    connections_.Add(3, [this](double /*t*/) {
      Vec3 pos;
      this->wing_.CalcProboscisPos(&pos);
      this->constraint_->set_Xg_end(pos);
    });

    connections_.Add(1, [this](double /*t*/) {
      const ControllerCommandMessage &command_message =
          this->avionics_packets().command_message;
      // TODO: Determine a better way to signal that the
      // slack should be taken up.
      bool motors_active = false;
      for (int32_t i = 0; i < kNumMotors; ++i) {
        if (command_message.motor_speed_upper_limit[i] > 60.0) {
          motors_active = true;
        }
      }
      this->constraint_->set_let_out_slack(motors_active);
    });

    connections_.Add(4, [this, constraint_force_index](double /*t*/) {
      ForceMomentPos constraint_force_moment;
      ForceMomentPos fmx_in = this->constraint_->end_force_moment();
      ForceMomentPosPoseTransform(&this->wing_.dcm_g2b(), &this->wing_.Xg(),
                                  &fmx_in, &constraint_force_moment);
      this->wing_.set_external_forces(constraint_force_index,
                                      constraint_force_moment);
    });
  }

  // Build rotors.
  bool use_hitl_motors = (hitl_config.motor_level == kActuatorHitlLevelReal);
  for (int32_t i = 0; i < kNumMotors; ++i) {
    const RotorParams &rotor_params = system_params_.rotors[i];
    const RotorSensorParams &rotor_sensor_params =
        system_params_.rotor_sensors[i];
    if (use_hitl_motors) {
      rotors_.emplace_back(new HitlRotor(
          static_cast<MotorLabel>(i), rotor_params, sim_params_.rotor_sim,
          rotor_sensor_params, environment_, system_params_.ts));
    } else {
      rotors_.emplace_back(new Rotor(static_cast<MotorLabel>(i), rotor_params,
                                     sim_params_.rotor_sim, rotor_sensor_params,
                                     environment_));
    }
    RotorBase *rotor = rotors_.back().get();
    sub_models_.push_back(rotor);
    connections_.Add(1, [this, rotor](double /*t*/) {
      rotor->set_parent_omega(this->wing_.omega());
    });
    connections_.Add(1, [this, rotor](double /*t*/) {
      rotor->SetFromAvionicsPackets(this->avionics_packets());
    });
    connections_.Add(3, [this, rotor, rotor_params](double /*t*/) {
      Vec3 local_apparent_wind_b;
      this->wing_.CalcLocalApparentWindB(rotor_params.pos, 0.0,
                                         &local_apparent_wind_b);
      rotor->set_local_apparent_wind_b(local_apparent_wind_b);
    });
    int32_t motor_force_index =
        wing_.AddExternalForce("motors[" + std::to_string(i) + "]");
    int32_t blown_wing_index =
        wing_.AddExternalForce("blown_wing[" + std::to_string(i) + "]");
    connections_.Add(
        5, [this, rotor, motor_force_index, blown_wing_index](double /*t*/) {
          ForceMomentPos rotor_force;
          rotor->CalcRotorForceMomentPos(&rotor_force);
          this->wing_.set_external_forces(motor_force_index, rotor_force);
          ForceMomentPos blown_wing_force;
          rotor->CalcBlownWingForceMomentPos(&blown_wing_force);
          this->wing_.set_external_forces(blown_wing_index, blown_wing_force);
        });
  }

  // Calculates total thrust coefficient to use in the next time step for the
  // rotor wake-on-tail interference model for forward flight.
  connections_.Add(6, [this](double /*t*/) {
    double sum_rotor_thrust_coeff = 0.0;
    for (int32_t i = 0; i < kNumMotors; ++i) {
      sum_rotor_thrust_coeff += rotors_[i]->RotorThrustCoeff();
    }
    this->wing_.set_total_rotor_thrust_coeff(sum_rotor_thrust_coeff /
                                             kNumMotors);
  });

  connections_.Add(4, [this](double /*t*/) {
    const DynoSimParams &params = sim_params_.dyno_sim;
    float max_speed = static_cast<float>(params.max_speed);

    DynoCommandMessage dyno_command = DynoCommandMessage();
    dyno_command.motor_command =
        MotorRunCommanded(avionics_packets_.val().command_message)
            ? kMotorCommandRun
            : kMotorCommandNone;

    for (int32_t i = 0; i < kNumMotors; ++i) {
      double torque =
          Saturate(this->rotors_[i]->aero_torque() * params.torque_scales[i],
                   -params.max_torque, params.max_torque);
      dyno_command.motor_torque[i] = static_cast<float>(torque);
      dyno_command.motor_speed_lower_limit[i] = -max_speed;
      dyno_command.motor_speed_upper_limit[i] = max_speed;
    }
    dyno_command_packet_.set_val(dyno_command);
  });

  // Build servos.
  for (int32_t i = 0; i < kNumServos; ++i) {
    ServoLabel servo_label = static_cast<ServoLabel>(i);

    switch (static_cast<ActuatorHitlLevel>(hitl_config.servo_levels[i])) {
      case kActuatorHitlLevelSimulated:
        servos_.emplace_back(new Servo(servo_label, system_params_.servos[i],
                                       sim_params_.servos_sim[i], faults_));
        break;
      case kActuatorHitlLevelReal:
        servos_.emplace_back(new HitlServo(servo_label,
                                           system_params_.servos[i],
                                           sim_params_.servos_sim[i], faults_));
        break;
      default:
        LOG(FATAL) << "Invalid ActuatorHitlLevel for servo "
                   << ServoLabelToString(servo_label);
        break;
    }

    ServoBase *servo = servos_.back().get();
    sub_models_.push_back(servo);
    connections_.Add(1, [this, servo](double /*t*/) {
      servo->SetFromAvionicsPackets(this->avionics_packets());
    });
    connections_.Add(3, [this, servo, servo_label](double /*t*/) {
      // In the simulation the rudder and elevator servos each
      // have control over half of the control surface.  They are
      // not mechanically linked.  Therefore, the area assigned to
      // each of the rudder and elevator servos is half of the
      // total control surface area.
      double surface_fraction = 1.0;
      if (servo_label == kServoE1 || servo_label == kServoE2 ||
          servo_label == kServoR1 || servo_label == kServoR2) {
        surface_fraction = 0.5;
      }
      servo->set_external_flap_torque(
          surface_fraction *
          this->wing_.CalcFlapMoment(ServoToFlap(servo_label)));
    });
  }

  // Converts the servo angles to equivalent flap deflections.  For
  // flaps that are controlled by multiple servos, this takes the
  // average of their reported deflections.
  connections_.Add(2, [this](double /*t*/) {
    this->wing_.set_flap_angles(kFlapA1, this->servos_[kServoA1]->flap_angle());
    this->wing_.set_flap_angles(kFlapA2, this->servos_[kServoA2]->flap_angle());
    this->wing_.set_flap_angles(kFlapA4, this->servos_[kServoA4]->flap_angle());
    this->wing_.set_flap_angles(kFlapA5, this->servos_[kServoA5]->flap_angle());
    this->wing_.set_flap_angles(kFlapA7, this->servos_[kServoA7]->flap_angle());
    this->wing_.set_flap_angles(kFlapA8, this->servos_[kServoA8]->flap_angle());
    this->wing_.set_flap_angles(kFlapEle,
                                0.5 * (this->servos_[kServoE1]->flap_angle() +
                                       this->servos_[kServoE2]->flap_angle()));
    this->wing_.set_flap_angles(kFlapRud,
                                0.5 * (this->servos_[kServoR1]->flap_angle() +
                                       this->servos_[kServoR2]->flap_angle()));
  });

  if (system_params_.offshore) {
    // Build sea.
    sea_.reset(new Sea(environment_, ground_frame_, sim_params_.sea_sim,
                       sim_params_.buoy_sim.msl_pos_z_g));
    sub_models_.push_back(sea_.get());
  }

  // Build the buoy. This is necessary in on-shore scenarios because of the need
  // for a vessel frame.
  buoy_.reset(new Buoy(environment_, ground_frame_, system_params_.buoy,
                       sim_params_.buoy_sim, sea_.get()));
  sub_models_.push_back(buoy_.get());
  connections_.Add(4, [this](double /*t*/) {
    this->buoy_->SetTetherForceMoment(this->tether_.start_force_moment());
  });

  // Build the ground station.
  if (system_params_.gs_model == kGroundStationModelGSv2) {
    InitGroundStationV2();
  } else {
    InitGroundStationV1();
    sensors_.emplace_back(new Gsg(perch_.get(), tether_, wing_,
                                  system_params_.gsg, sim_params_.gsg_sim,
                                  system_params_.perch, sim_params_.perch_sim,
                                  system_params_.levelwind, faults_));
  }

  if (*g_sim.sim_opt & kSimOptGroundContact) {
    for (int32_t i = 0; i < kNumGroundContactors; ++i) {
      int32_t contactor_force_index =
          wing_.AddExternalForce("ground_contactor[" + std::to_string(i) + "]");
      connections_.Add(2, [this, i, contactor_force_index](double /*t*/) {
        Contactor contactor(this->sim_params_.contact_sim.ground_contactors[i],
                            this->wing_.frame(), this->ground_frame_.surface());
        ForceMomentPos contact_force;
        contactor.CalcContactForceMomentPos(&contact_force);
        if (*g_sim.sim_opt & kSimOptExitOnCrash &&
            Vec3NormSquared(&contact_force.force) > 0.001) {
          throw std::domain_error("Wing has hit the ground.");
        }
        this->wing_.set_external_forces(contactor_force_index, contact_force);
      });
    }

    this->tether_.set_ground_surface(&this->ground_frame_.surface());
  }

  // Finish adding forces to the Wing.
  wing_.FinalizeExternalForces();

  // Build power system.
  if (!use_hitl_motors) {
    if (*g_sim.sim_opt & kSimOptStackedPowerSystem) {
      power_sys_ = std::unique_ptr<PowerSys>(new StackedPowerSys(
          rotors_, system_params_.rotor_sensors, system_params_.power_sys,
          sim_params_.power_sys_sim, faults_));
    } else {
      power_sys_ = std::unique_ptr<PowerSys>(new SimplePowerSys(
          rotors_, system_params_.rotor_sensors, system_params_.power_sys,
          sim_params_.power_sys_sim, faults_));
    }
    sub_models_.push_back(power_sys_.get());

    for (int32_t i = 0; i < kNumMotors; ++i) {
      connections_.Add(2, [this, i](double /*t*/) {
        rotors_[i].get()->SetMotorTorque(
            this->power_sys_->motor_torques(static_cast<MotorLabel>(i)));
      });
    }

    connections_.Add(0, [this](double /*t*/) {
      this->power_sys_->set_tether_released(tether_release_->released());
    });
  }

  // Add high voltage harness.
  if (power_sys_) {
    high_voltage_harness_.reset(
        new HighVoltageHarness(sim_params_.high_voltage_harness_sim));
    sub_models_.push_back(high_voltage_harness_.get());
    connections_.Add(2, [this](double /*t*/) {
      for (int32_t i = 0; i < kNumMotors; ++i) {
        MotorLabel m = static_cast<MotorLabel>(i);
        high_voltage_harness_->set_motor_current(m,
                                                 power_sys_->motor_current(m));
      }
    });
  }

  // Add wing IMU mount and IMUs.
  sub_models_.push_back(&wing_imu_mount_);

  for (int32_t i = 0; i < kNumWingImus; ++i) {
    Imu *imu = new Imu(
        "wing:" + std::to_string(i), environment_, wing_, wing_imu_mount_,
        system_params_.wing_imus[i], sim_params_.wing_imus_sim[i],
        &sim_telem.imus[i],
        &sensor_message_->control_input_messages_updated.flight_comp_imus[i],
        &sensor_message_->control_input_messages_updated.flight_comp_sensors[i],
        &sensor_message_->control_input_messages.flight_comp_imus[i],
        &sensor_message_->control_input_messages.flight_comp_sensors[i],
        faults_);
    connections_.Add(3, [this, imu](double /*t*/) {
      Vec3 induced_field_b = kVec3Zero;
      if (high_voltage_harness_) {
        high_voltage_harness_->CalcInducedMagneticField(imu->GetPos(),
                                                        &induced_field_b);
      }
      imu->set_external_magnetic_field(induced_field_b);
    });

    sensors_.emplace_back(imu);
  }

  Loadcell *loadcell =
      new Loadcell(system_params_.loadcells, sim_params_.loadcell_sim,
                   system_params_.wing, faults_);
  sensors_.emplace_back(loadcell);
  connections_.Add(2, [this, loadcell](double /*t*/) {
    loadcell->set_tether_released(tether_release_->released());
  });
  connections_.Add(4, [this, loadcell](double /*t*/) {
    Vec3 Fb_tether;
    Mat3Vec3Mult(&this->wing_.dcm_g2b(), &this->tether_.end_force(),
                 &Fb_tether);
    loadcell->set_Fb_tether(Fb_tether);
  });

  // Add Pitot tube.
  // These sensors are not currently installed on the wing.  We
  // deactivate them in the flight plans used during early testing.
  if (!(system_params_.flight_plan == kFlightPlanDisengageEngage ||
        system_params_.flight_plan == kFlightPlanHoverInPlace ||
        system_params_.flight_plan == kFlightPlanLaunchPerch)) {
    for (int32_t i = 0; i < kNumPitotSensors; ++i) {
      Pitot *pitot =
          new Pitot(environment_, wing_, static_cast<PitotSensorLabel>(i),
                    system_params_.sensor_layout.pitot_fc_labels[i],
                    system_params_.pitot, sim_params_.pitots_sim[i], faults_);
      sensors_.emplace_back(pitot);
      connections_.Add(5, [this, pitot, i](double /*t*/) {
        Vec3 rotor_force_b;
        this->wing_.CalcTotalRotorForce(&rotor_force_b);
        double thrust = Vec3Dot(&rotor_force_b,
                                &this->sim_params_.pitots_sim[i].rotor_axis);
        pitot->set_total_rotor_thrust(thrust);
      });
    }
  }

  // Add servos.
  sensors_.emplace_back(new ServoSensor(servos_, system_params_.servos,
                                        sim_params_.servos_sim, faults_));

  // Add joystick.
  if (hitl_config.use_software_joystick) {
    sensors_.emplace_back(
        new Joystick(system_params_.joystick, sim_params_.joystick_sim));
  }

  // Add GPS receivers.
  for (int32_t i = 0; i < kNumWingGpsReceivers; ++i) {
    auto gps_label = static_cast<WingGpsReceiverLabel>(i);
    GpsReceiverType gps_type = WingGpsReceiverLabelToGpsReceiverType(gps_label);
    if (gps_type == kGpsReceiverTypeNovAtel) {
      sensors_.emplace_back(new NovAtelGps(
          ground_frame_, wing_, gps_label, system_params_.wing_gps[gps_label],
          sim_params_.gps_sim[gps_type], faults_));
    } else if (gps_type == kGpsReceiverTypeSeptentrio) {
      sensors_.emplace_back(new SeptentrioGps(
          ground_frame_, wing_, gps_label, system_params_.wing_gps[gps_label],
          sim_params_.gps_sim[gps_type], faults_));
    } else {
      LOG(FATAL) << "Invalid GPS type: " << static_cast<int32_t>(gps_type);
    }
  }

  RotorSensor *rotor_sensor =
      new RotorSensor(rotors_, system_params_.rotor_sensors, faults_);
  sensors_.emplace_back(rotor_sensor);
  connections_.Add(1, [this, rotor_sensor, use_hitl_motors](double /*t*/) {
    for (int32_t i = 0; i < kNumMotors; ++i) {
      MotorLabel motor_label = static_cast<MotorLabel>(i);
      if (use_hitl_motors) {
        if (rotors_[motor_label]->motor_status_flag() & kMotorStatusRunning ||
            rotors_[motor_label]->motor_status_flag() & kMotorStatusWindDown) {
          rotor_sensor->set_is_faulted(motor_label, false);
        } else {
          rotor_sensor->set_is_faulted(motor_label, true);
        }
      } else {
        rotor_sensor->set_is_faulted(
            motor_label, !this->power_sys_->motor_connections(motor_label));
      }
    }
  });
  if (power_sys_) {
    connections_.Add(2, [this, rotor_sensor](double /*t*/) {
      for (int32_t i = 0; i < kNumMotors; i++) {
        MotorLabel m = static_cast<MotorLabel>(i);
        // The negative sign here was determined to be neccessary to report
        // positive current for consumption, as the motor controllers do.
        rotor_sensor->set_bus_current(m, -power_sys_->motor_current(m));
      }
    });
  } else if (use_hitl_motors) {
    connections_.Add(2, [this, rotor_sensor](double /*t*/) {
      for (int32_t i = 0; i < kNumMotors; i++) {
        MotorLabel m = static_cast<MotorLabel>(i);
        rotor_sensor->set_bus_current(m, rotors_[m]->bus_current());
      }
    });
  }

  for (std::unique_ptr<Sensor> &sensor : sensors_) {
    sub_models_.push_back(sensor.get());
  }

  AddAllInternalConnections(&connections_);
}

Tether::CalcBridlePointFunc FullSystem::GetCalcBridlePointCallback() const {
  return std::bind(&Wing::CalcBridlePoint, this->wing_.Xg(), this->wing_.Vb(),
                   this->wing_.omega(), this->wing_.dcm_g2b(),
                   system_params_.wing, std::placeholders::_1,
                   std::placeholders::_2, std::placeholders::_3,
                   std::placeholders::_4);
}

void FullSystem::InitGroundStationV1() {
  // Build winch and perch.
  if (*g_sim.sim_opt & kSimOptPerch) {
    winch_.reset(new Winch(system_params_.winch, sim_params_.winch_sim));

    WinchSensor *winch_sensor = new WinchSensor(system_params_.winch);
    connections_.Add(2, [this, winch_sensor](double /*t*/) {
      winch_sensor->set_theta_winch(this->perch_->theta_winch());
    });
    connections_.Add(2, [this, winch_sensor](double /*t*/) {
      winch_sensor->set_omega_winch(this->perch_->omega_winch());
    });
    connections_.Add(2, [this, winch_sensor](double /*t*/) {
      winch_sensor->set_winch_torque(this->winch_->CalcTorque(
          this->perch_->theta_winch(), this->perch_->omega_winch()));
    });
    sensors_.emplace_back(winch_sensor);

    sub_models_.push_back(winch_.get());
    connections_.Add(1, [this](double /*t*/) {
      this->winch_->SetFromAvionicsPackets(this->avionics_packets());
    });

    perch_ = std::unique_ptr<Perch>(new Perch(
        ground_frame_, *buoy_, system_params_.perch, system_params_.levelwind,
        system_params_.winch, sim_params_.perch_sim));
    sub_models_.push_back(perch_.get());
    // TODO: This is a hack that adds both the contact
    // forces and tether forces to the perch, by grabbing all of the
    // wing's external forces, when the wing is close to the perch.
    // When the wing is far enough from the perch that there are no
    // contact forces, this adds only the tether force.  Ideally, we
    // should just properly add up the individual tether and contact
    // forces and apply them at the appropriate positions.  See
    // b/19891678.
    connections_.Add(4, [this](double /*t*/) {
      Vec3 external_force;
      if (this->perch_->CalcTetherFreeLength() < 2.0) {
        this->wing_.CalcTotalExternalForce(&external_force);
        Vec3Scale(&external_force, -1.0, &external_force);
      } else {
        external_force = this->tether_.start_force();
      }
      this->perch_->set_external_force_g(external_force);
    });
    connections_.Add(1, [this](double /*t*/) {
      this->perch_->set_winch_torque(this->winch_->CalcTorque(
          this->perch_->theta_winch(), this->perch_->omega_winch()));
    });
    connections_.Add(2, [this](double /*t*/) {
      this->tether_.set_free_length(this->perch_->CalcTetherFreeLength());
      this->tether_.set_Xg_start(this->perch_->anchor_pos_g());
      this->tether_.set_Vg_start(this->perch_->anchor_vel_g());
    });

    InitPerchWinchTether(sim_params_.perch_sim.initialize_in_crosswind_config,
                         perch_.get(), winch_.get(), &tether_);

    if (*g_sim.sim_opt & kSimOptPerchContact) {
      for (int32_t i = 0; i < kNumPerchContactors; ++i) {
        int32_t contactor_force_index = wing_.AddExternalForce(
            "perch_contactor[" + std::to_string(i) + "]");
        connections_.Add(2, [this, i, contactor_force_index](double /*t*/) {
          Contactor contactor(this->sim_params_.contact_sim.perch_contactors[i],
                              this->wing_.frame(), this->perch_->surface());
          ForceMomentPos contact_force_g;
          contactor.CalcContactForceMomentPos(&contact_force_g);
          this->wing_.set_external_forces(contactor_force_index,
                                          contact_force_g);
        });
      }
    }
  } else {
    // If there is no perch, initialize the tether at full length.
    double full_tether_length = system_params_.tether.length;
    connections_.Add(2, [this, full_tether_length](double /*t*/) {
      this->tether_.set_free_length(full_tether_length);
      this->tether_.set_Xg_start(kVec3Zero);
      this->tether_.set_Vg_start(kVec3Zero);
    });

    tether_.Init(GetCalcBridlePointCallback(), kVec3Zero, kVec3Zero,
                 full_tether_length);
  }

  // Ground station GPS.
  bool has_compass = false;
  GsGps *gs_gps = new GsGps(ground_frame_, system_params_.gs_gps, has_compass,
                            sim_params_.gs_gps_sim);
  sensors_.emplace_back(gs_gps);
  connections_.Add(2, [this, gs_gps](double /*t*/) {
    gs_gps->set_parent_frame(perch_->frame());
  });

  // Wind sensor.
  WindSensor *wind_sensor = new WindSensor(
      system_params_.wind_sensor, sim_params_.wind_sensor_sim, faults_);
  sensors_.emplace_back(wind_sensor);
  connections_.Add(2, [this, wind_sensor](double /*t*/) {
    wind_sensor->set_parent_frame(this->perch_ &&
                                          system_params_.wind_sensor.on_perch
                                      ? this->perch_->frame()
                                      : this->buoy_->vessel_frame());
    wind_sensor->set_ground_frame(this->ground_frame_);
  });
  connections_.Add(4, [this, wind_sensor](double /*t*/) {
    Vec3 wind_g;
    this->environment_.CalcWind(wind_sensor->pos_g(), &wind_g);
    wind_sensor->set_wind_g(wind_g);
  });
}

void FullSystem::SetGsgAngles(const Buoy &buoy__, const Tether &tether__,
                              GroundStationV2Base *gs02__,
                              GroundStationV2Sensors *gs02_sensors_,
                              SimTelemetry *sim_telem_) {
  Vec3 tether_dir_v;
  Mat3Vec3Mult(&buoy__.dcm_g2v(), &tether__.tether_start_dir_g(),
               &tether_dir_v);

  double Xv_start_elevation, Xv_start_azimuth, gsg_yoke, gsg_termination;
  gs02__->TetherDirToAngles(tether_dir_v, &Xv_start_elevation,
                            &Xv_start_azimuth, &gsg_yoke, &gsg_termination);
  gs02__->SetLevelwindEngaged(Xv_start_elevation);

  gs02_sensors_->set_nominal_levelwind_ele(Xv_start_elevation);
  gs02_sensors_->set_nominal_gsg_termination(gsg_termination);
  gs02_sensors_->set_nominal_gsg_yoke(gsg_yoke);
  sim_telem_->tether.Xv_start_elevation = Xv_start_elevation;
  sim_telem_->tether.Xv_start_azimuth = Xv_start_azimuth;
}

void FullSystem::InitGroundStationV2() {
  if (system_params_.hitl.config.gs02_level == kActuatorHitlLevelReal) {
    gs02_.reset(new HitlGroundStationV2(
        environment_.ned_frame(), buoy_->vessel_frame(),
        system_params_.ground_station.gs02, sim_params_.gs02_sim));
  } else {
    gs02_.reset(new GroundStationV2(
        environment_.ned_frame(), buoy_->vessel_frame(),
        system_params_.ground_station.gs02, sim_params_.gs02_sim));
  }
  sub_models_.push_back(gs02_.get());

  connections_.Add(1, [this](double /*t*/) {
    gs02_->SetFromAvionicsPackets(this->avionics_packets());
  });

  // GS02 sensors.
  GroundStationV2Sensors *gs02_sensors = new GroundStationV2Sensors();
  sensors_.emplace_back(gs02_sensors);
  connections_.Add(1, [this, gs02_sensors](double /*t*/) {
    gs02_sensors->set_mode(gs02_->mode());
    gs02_sensors->set_mode_cmd(gs02_->mode_cmd());
    gs02_sensors->set_transform_stage(gs02_->transform_stage());
    gs02_sensors->set_platform_azi(gs02_->platform_azi());
    gs02_sensors->set_drum_angle(gs02_->drum_angle());
    gs02_sensors->set_detwist_angle(gs02_->detwist_angle());
    gs02_sensors->set_prox_sensor_active(gs02_->prox_sensor_active());
  });
  connections_.Add(5, [this, gs02_sensors](double /*t*/) {
    SetGsgAngles(*this->buoy_, this->tether_, this->gs02_.get(), gs02_sensors,
                 &sim_telem);
  });

  // Require that the g-frame equal NED.
  CHECK_LE(fabs(ground_frame_.heading()), DBL_EPSILON);

  // Supply length and anchor state to tether.
  connections_.Add(2, [this](double /*t*/) {
    tether_.set_free_length(gs02_->CalcTetherFreeLength());
    Vec3 anchor_pos_ned, anchor_vel_ned;
    gs02_->CalcTetherAnchorPosVelNed(&anchor_pos_ned, &anchor_vel_ned);
    tether_.set_Xg_start(anchor_pos_ned);
    tether_.set_Vg_start(anchor_vel_ned);
  });

  // Tether forces at the ground side attachment.
  connections_.Add(4, [this](double /*t*/) {
    gs02_->SetTetherForce(&this->tether_.start_force());
  });

  // Wing contact forces.
  for (int32_t i = 0; i < kNumPerchContactors; ++i) {
    int32_t contactor_force_index =
        wing_.AddExternalForce("perch_contactor[" + std::to_string(i) + "]");
    connections_.Add(2, [this, i, contactor_force_index](double /*t*/) {
      Contactor contactor(sim_params_.contact_sim.perch_contactors[i],
                          wing_.frame(), gs02_->panel_surface());
      ForceMomentPos contact_force;
      contactor.CalcContactForceMomentPos(&contact_force);
      this->wing_.set_external_forces(contactor_force_index, contact_force);
    });
  }

  // Ground station GPS.
  bool has_compass = true;
  GsGps *gs_gps = new GsGps(ground_frame_, system_params_.gs_gps, has_compass,
                            sim_params_.gs_gps_sim);
  sensors_.emplace_back(gs_gps);
  connections_.Add(2, [this, gs_gps](double /*t*/) {
    gs_gps->set_parent_frame(gs02_->platform_frame());
  });

  // Wind sensor.
  WindSensor *wind_sensor = new WindSensor(
      system_params_.wind_sensor, sim_params_.wind_sensor_sim, faults_);
  sensors_.emplace_back(wind_sensor);
  connections_.Add(2, [this, wind_sensor](double /*t*/) {
    wind_sensor->set_parent_frame(gs02_->platform_frame());
    wind_sensor->set_ground_frame(this->ground_frame_);
  });
  connections_.Add(4, [this, wind_sensor](double /*t*/) {
    Vec3 wind_g;
    this->environment_.CalcWind(wind_sensor->pos_g(), &wind_g);
    wind_sensor->set_wind_g(wind_g);
  });

  InitGs02Tether(sim_params_.gs02_sim, gs02_.get(), &tether_);

  // GS02 platform.
  platform_.reset(new Platform(buoy_.get(), gs02_.get()));
  sub_models_.push_back(platform_.get());

  // GS IMU mount and IMU.
  sub_models_.push_back(&gs_imu_mount_);

  for (int32_t i = 0; i < kNumGsImus; ++i) {
    Imu *imu = new Imu(
        "gs:" + std::to_string(i), environment_, *platform_, gs_imu_mount_,
        system_params_.gs_imus[i], sim_params_.gs_imus_sim[i],
        &sim_telem.imus[i],
        &sensor_message_->ground_input_messages_updated.ground_comp_imus[i],
        &sensor_message_->ground_input_messages_updated.ground_comp_sensors[i],
        &sensor_message_->ground_input_messages.ground_comp_imus[i],
        &sensor_message_->ground_input_messages.ground_comp_sensors[i],
        faults_);

    // TODO: Verify that there is no need to apply a correction
    // accounting for any external magnetic field
    // (see set_external_magnetic_field).
    connections_.Add(3, [this, imu](double /*t*/) {
      Vec3 induced_field_b = kVec3Zero;
      imu->set_external_magnetic_field(induced_field_b);
    });

    sensors_.emplace_back(imu);
  }
}

// Initializes the perch, winch, and tether by finding a reasonable
// initial winch drum angle.  Other initialization parameters are set
// in the configuration file; however, this parameter is set in the
// simulator because it depends on a complicated interaction between
// the perch, winch, and tether.
//
// When the perch starts in the crosswind configuration, the winch
// drum angle is, by definition, 0.0 rad.  Otherwise, the initial
// winch drum angle is determined by finding, through bisection, the
// angle that produces zero tether force.
//
// TODO: Remove this method when the v1 ground station model is
// eliminated.
void FullSystem::InitPerchWinchTether(bool initialize_in_crosswind_config,
                                      Perch *perch, Winch *winch,
                                      Tether *tether) const {
  if (initialize_in_crosswind_config) {
    perch->Init(0.0);
    winch->Init(0.0);
    tether->Init(GetCalcBridlePointCallback(), perch->anchor_pos_g(),
                 perch->anchor_vel_g(), perch->CalcTetherFreeLength());
  } else {
    // Set the minimum tension to some small fraction of the tether
    // weight and the maximum tension to half the tether weight.  The
    // exact values used here are not critical as long as the forces
    // involved are what could be expected during perching or hover.
    const double tension_min = system_params_.tether.length *
                               system_params_.tether.linear_density *
                               system_params_.phys.g / 100.0;
    const double tension_max = system_params_.tether.length *
                               system_params_.tether.linear_density *
                               system_params_.phys.g / 2.0;

    // The maximum winch drum angle is the crosswind location, 0.0
    // rad.  The minimum winch drum angle is found by wrapping the
    // tether completely around the drum and adding an extra half
    // rotation to account for the perch transformation as theta_winch
    // is measured relative to the perch platform.
    double theta_winch_min =
        -(system_params_.tether.length / system_params_.winch.r_drum + M_PI);
    double theta_winch_max = 0.0;
    bool bisection_done = false;
    do {
      double theta_winch_mid = (theta_winch_min + theta_winch_max) / 2.0;
      perch->Init(theta_winch_mid);
      winch->Init(theta_winch_mid);
      tether->Init(GetCalcBridlePointCallback(), perch->anchor_pos_g(),
                   perch->anchor_vel_g(), perch->CalcTetherFreeLength());
      double tether_force = Vec3Norm(&tether->start_force());
      if (tether_force < tension_min) {
        theta_winch_max = theta_winch_mid;
      } else if (tether_force > tension_max) {
        theta_winch_min = theta_winch_mid;
      } else {
        bisection_done = true;
      }
    } while (!bisection_done);
  }
}

// Initializes GS02 and the tether by finding a drum angle that produces a
// reasonable tether tension. Other initialization parameters are set in the
// configuration file; however, this parameter is set in the simulator because
// it depends on a complicated interaction between the drum and tether.
void FullSystem::InitGs02Tether(const Gs02SimParams &gs02_sim,
                                GroundStationV2Base *gs02,
                                Tether *tether) const {
  if (!gs02_sim.init_tether_tension) {
    gs02->Init(gs02_sim.initial_platform_azi, gs02_sim.initial_drum_angle);
    Vec3 anchor_pos_ned, anchor_vel_ned;
    gs02->CalcTetherAnchorPosVelNed(&anchor_pos_ned, &anchor_vel_ned);
    tether->Init(GetCalcBridlePointCallback(), anchor_pos_ned, anchor_vel_ned,
                 gs02->CalcTetherFreeLength());
  } else {
    // Set the minimum tension to some small fraction of the tether
    // weight and the maximum tension to half the tether weight.  The
    // exact values used here are not critical as long as the forces
    // involved are what could be expected during perching or hover.
    const double tension_min = system_params_.tether.length *
                               system_params_.tether.linear_density *
                               system_params_.phys.g / 100.0;
    const double tension_max = system_params_.tether.length *
                               system_params_.tether.linear_density *
                               system_params_.phys.g / 2.0;

    // The maximum winch drum angle is the crosswind location, 0.0
    // rad.  The minimum winch drum angle is found by wrapping the
    // tether completely around the drum and adding an extra half
    // rotation to account for the perch transformation as theta_winch
    // is measured relative to the perch platform.
    double theta_winch_min = gs02_->MinDrumAngle();
    double theta_winch_max = 0.0;
    bool bisection_done = false;
    do {
      CHECK_GT(theta_winch_max - theta_winch_min, 1e-8)
          << "Failed to converge.";
      double theta_winch_mid = (theta_winch_min + theta_winch_max) / 2.0;
      gs02->Init(gs02_sim.initial_platform_azi, theta_winch_mid);
      Vec3 anchor_pos_ned, anchor_vel_ned;
      gs02->CalcTetherAnchorPosVelNed(&anchor_pos_ned, &anchor_vel_ned);
      tether->Init(GetCalcBridlePointCallback(), anchor_pos_ned, anchor_vel_ned,
                   gs02->CalcTetherFreeLength());
      double tether_force = Vec3Norm(&tether->start_force());
      if (tether_force < tension_min) {
        theta_winch_max = theta_winch_mid;
      } else if (tether_force > tension_max) {
        theta_winch_min = theta_winch_mid;
      } else {
        bisection_done = true;
      }
    } while (!bisection_done);
  }
}

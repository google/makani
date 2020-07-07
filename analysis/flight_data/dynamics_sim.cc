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

#include "analysis/flight_data/dynamics_sim.h"
#include "control/avionics/avionics_interface.h"

DynamicsSystem::DynamicsSystem(const std::string &name,
                               const SystemParams &system_params,
                               const SimParams &sim_params,
                               FaultSchedule *faults)
    : BaseSystemModel(name, system_params, sim_params, faults),
      environment_(sim_params_.iec_sim, sim_params_.phys_sim,
                   system_params_.phys, system_params_.wind_sensor,
                   system_params_.ground_frame),
      ground_frame_(environment_.ned_frame(), system_params_.ground_frame,
                    sim_params_.ground_frame_sim),
      wing_(environment_, ground_frame_, system_params_.wing,
            sim_params_.aero_sim, sim_params_.wing_sim, faults_),
      rotors_(),
      flight_mode_() {
  sub_models_.push_back(&environment_);
  sub_models_.push_back(&wing_);

  // Set up rotor models.
  for (int32_t i = 0; i < kNumMotors; ++i) {
    const RotorParams &rotor_params = system_params_.rotors[i];
    const RotorSensorParams &rotor_sensor_params =
        system_params_.rotor_sensors[i];
    rotors_.emplace_back(new HitlRotor(
        static_cast<MotorLabel>(i), rotor_params, sim_params_.rotor_sim,
        rotor_sensor_params, environment_, system_params_.ts));
    sub_models_.push_back(rotors_.back().get());
  }

  // Set wing and rotor states from log data.
  connections_.Add(0, [this](double /*t*/) {
    wing_.InjectStates(Xg(), Vb(), pqr(), dcm_g2b());
    for (int32_t i = 0; i < kNumFlaps; ++i) {
      wing_.set_flap_angles(static_cast<FlapLabel>(i), flaps()[i]);
    }

    // In order to use HitlRotor, calibrated rotor velocities need to be stored
    // in both the command message and the rotor statuses. Note that the rotor
    // velocities are all positive in ControlInput, and they are converted to
    // motor conventions by ConvertRotors. Then the rotor model will convert
    // them to its own conventions in SetFromAvionicsPackets.
    AvionicsPackets avionics_packets;
    double motor_torques[kNumMotors] = {0};
    ConvertRotors(rotors(), rotors(), motor_torques, system_params_.rotors,
                  system_params_.rotor_sensors,
                  &avionics_packets.command_message);
    for (int32_t i = 0; i < kNumMotors; ++i) {
      avionics_packets.motor_statuses.emplace_back();
      avionics_packets.motor_statuses.back().omega =
          avionics_packets.command_message.motor_speed_upper_limit[i];
    }

    for (int32_t i = 0; i < kNumMotors; ++i) {
      rotors_[i]->SetFromAvionicsPackets(avionics_packets);
    }
  });

  // Add rotor connections.
  for (int32_t i = 0; i < kNumMotors; ++i) {
    RotorBase *rotor = rotors_[i].get();
    const RotorParams &rotor_params = system_params_.rotors[i];

    // The rotor acceleration is a discrete state, which is updated here.
    connections_.Add(1, [this, rotor](double t) { rotor->DiscreteUpdate(t); });

    // Standard rotor connections -- these are taken directly from FullSystem.
    connections_.Add(1, [this, rotor](double /*t*/) {
      rotor->set_parent_omega(this->wing_.omega());
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

  // Apply tether force/moment.
  int32_t tether_force_index = wing_.AddExternalForce("tether");
  connections_.Add(4, [this, tether_force_index](double /*t*/) {
    ForceMomentPos fmx_tether_b = {tether_force_b(), bridle_moment_b(),
                                   knot_pos_b()};
    this->wing_.set_external_forces(tether_force_index, fmx_tether_b);
  });

  // Set wind velocity in the g-frame by backing it out from apparent wind and
  // kite velocity in the b-frame. Note that the primary use of wind_g is for
  // the wing model to calculate apparent wind, at which point this
  // transformation is inverted.
  connections_.Add(2, [this](double /*t*/) {
    wing_.set_wind_g(wind_g());

    // Set wind vorticity to zero -- we don't have a good way to estimate
    // this.
    wing_.set_wind_omega_g(kVec3Zero);
  });

  // Finish wing_'s external force initialization.
  wing_.FinalizeExternalForces();

  AddAllInternalConnections(&connections_);

  SetupDone();
}

// Populates const Vec3 & wind_gmicReplayMessage.
DynamicsReplayMessage GenerateDynamics(const SystemParams &system_params,
                                       const SimTelemetry &sim_telemetry,
                                       const Vec3 &Ab, const Vec3 &omega_b,
                                       const Vec3 &omega_b_dot) {
  DynamicsReplayMessage out;

  // Calculate inertial force.
  Vec3Scale(&Ab, -system_params.wing.m, &out.fm_inertial_b.force);

  // Compute inertial moment:
  //     M_inertial = -(I \times \dot{\omega_b}
  //                    + \omega_b \times (I \omega_b)).
  {
    const Mat3 &I = system_params.wing.I;
    Vec3 tmp1, tmp2;
    Mat3Vec3Mult(&I, &omega_b_dot, &tmp1);
    Mat3Vec3Mult(&I, &omega_b, &tmp2);
    Vec3Cross(&omega_b, &tmp2, &tmp2);
    Vec3LinComb(-1.0, &tmp1, -1.0, &tmp2, &out.fm_inertial_b.moment);
  }

  // Copy modeled force-moment breakdown from WingTelemetry.
  const WingTelemetry &w = sim_telemetry.wing;
  out.fm_aero_b = w.fm_aero;
  out.fm_rotors_b = w.fm_rotors;
  out.fm_tether_b = w.fm_tether;
  out.fm_gravity_b = w.fm_gravity;
  out.fm_blown_wing_b = w.fm_blown_wing;

  // Compute fm_error_b = w.fm_total + fm_inertial_b.
  //
  // We're relying on w.fm_total containing the sum of all simulated
  // force-moments rather than re-adding them manually.
  Vec3Add(&w.fm_total.force, &out.fm_inertial_b.force, &out.fm_error_b.force);
  Vec3Add(&w.fm_total.moment, &out.fm_inertial_b.moment,
          &out.fm_error_b.moment);

  // Compute the body-to-wind DCM.
  Mat3 dcm_w2b;
  const ApparentWindSph apparent_wind = out.state_est.apparent_wind.sph_f;
  CalcDcmWToB(apparent_wind.alpha, apparent_wind.beta, &dcm_w2b);
  Mat3Trans(&dcm_w2b, &out.dcm_b2w);

  return out;
}

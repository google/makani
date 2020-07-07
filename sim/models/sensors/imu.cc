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

#include "sim/models/sensors/imu.h"

#include <math.h>

#include <string>
#include <vector>

#include "common/c_math/filter.h"
#include "common/c_math/mat3.h"
#include "common/c_math/vec3.h"
#include "control/system_types.h"
#include "sim/faults/faults.h"
#include "sim/math/util.h"
#include "sim/models/environment.h"
#include "sim/models/rigid_bodies/wing.h"
#include "sim/models/signals/delayed_signal.h"
#include "sim/models/signals/measurement.h"
#include "sim/physics/ground_frame.h"
#include "sim/sim_messages.h"
#include "sim/sim_telemetry.h"
#include "sim/state.h"

void MagnetometerMeasurement::DiscreteStepHelper(double t) {
  Measurement<Vec3>::DiscreteStepHelper(t);

  // Model periodic glitches.
  Vec3 corrupted__ = Measurement<Vec3>::recorded();
  if (*g_sim.sim_opt & kSimOptImperfectSensors) {
    if (fmod(t, params_.glitch_period) < params_.glitch_duration) {
      Vec3Add(&corrupted__, &params_.glitch_magnitudes, &corrupted__);
    }

    // Model harmonic noise on the magnetometers.
    for (int32_t i = 0; i < NUM_MAGNETOMETER_HARMONICS; ++i) {
      double harmonic = params_.harmonics_amplitudes[i] *
                        cos(2.0 * M_PI * params_.harmonics_frequencies[i] * t);
      Vec3LinComb(1.0, &corrupted__, harmonic, &kVec3Ones, &corrupted__);
    }
  }
  corrupted_.DiscreteUpdate(t, corrupted__);
}

Imu::Imu(const std::string &label, const Environment &environment,
         const RigidBody &parent, const ImuMount &imu_mount,
         const ImuParams &imu_params, const ImuSimParams &imu_sim_params,
         ImuTelemetry *imu_telem, bool *updated_flight_comp_imu_message,
         bool *updated_flight_comp_sensor_message,
         FlightComputerImuMessage *flight_comp_imu_message,
         FlightComputerSensorMessage *flight_comp_sensor_message,
         FaultSchedule *faults)
    : Sensor("IMU[" + label + "]", imu_sim_params.ts),
      imu_params_(imu_params),
      imu_sim_params_(imu_sim_params),
      dcm_parent2actual_(imu_params.dcm_parent2m),
      environment_(environment),
      parent_(parent),
      imu_mount_(imu_mount),
      rng_(full_name()),
      imu_telem_(imu_telem),
      updated_flight_comp_imu_message_(updated_flight_comp_imu_message),
      updated_flight_comp_sensor_message_(updated_flight_comp_sensor_message),
      flight_comp_imu_message_(flight_comp_imu_message),
      flight_comp_sensor_message_(flight_comp_sensor_message),
      external_magnetic_field_parent_(new_derived_value(),
                                      "external_magnetic_field_parent"),
      actual_acc_(new_discrete_state(), "actual_acc", 0.0, kVec3Zero),
      actual_gyro_(new_discrete_state(), "actual_gyro", 0.0, kVec3Zero),
      actual_mag_(new_discrete_state(), "actual_mag", 0.0, kVec3Zero),
      actual_P_stat_(new_discrete_state(), "actual_P_stat", 0.0),
      acc_random_walk_bias_(new_discrete_state(), "acc_random_walk_bias",
                            imu_sim_params.ts, kVec3Zero),
      gyro_random_walk_bias_(new_discrete_state(), "gyro_random_walk_bias",
                             imu_sim_params.ts, kVec3Zero),
      acc_markov_process_bias_(new_discrete_state(), "acc_markov_process_bias",
                               imu_sim_params.ts, kVec3Zero),
      gyro_markov_process_bias_(new_discrete_state(),
                                "gyro_markov_process_bias", imu_sim_params.ts,
                                kVec3Zero),
      acc_(full_name(), "acc", imu_sim_params.ts,
           {imu_sim_params_.acc_sensor[0], imu_sim_params_.acc_sensor[1],
            imu_sim_params_.acc_sensor[2]},
           actual_acc_, faults),
      gyro_(full_name(), "gyro", imu_sim_params.ts,
            {imu_sim_params_.gyro_sensor[0], imu_sim_params_.gyro_sensor[1],
             imu_sim_params_.gyro_sensor[2]},
            actual_gyro_, faults),
      mag_(full_name(), "mag", imu_sim_params.ts,
           {imu_sim_params_.mag_sensor[0], imu_sim_params_.mag_sensor[1],
            imu_sim_params_.mag_sensor[2]},
           imu_sim_params_.mag_noise, actual_mag_, faults),
      P_stat_(full_name(), "P_stat", imu_sim_params.ts,
              {imu_sim_params.stat_sensor}, actual_P_stat_, faults),
      delayed_acc_(full_name(), "delayed_acc", imu_sim_params.ts,
                   (*g_sim.sim_opt & kSimOptImperfectSensors)
                       ? imu_sim_params.delay
                       : 0.0,
                   acc_.recorded()),
      delayed_gyro_(full_name(), "delayed_gyro", imu_sim_params.ts,
                    (*g_sim.sim_opt & kSimOptImperfectSensors)
                        ? imu_sim_params.delay
                        : 0.0,
                    gyro_.recorded()),
      delayed_mag_(full_name(), "delayed_mag", imu_sim_params.ts,
                   (*g_sim.sim_opt & kSimOptImperfectSensors)
                       ? imu_sim_params.delay
                       : 0.0,
                   mag_.recorded()),
      delayed_P_stat_(full_name(), "delayed_P_stat", imu_sim_params.ts,
                      (*g_sim.sim_opt & kSimOptImperfectSensors)
                          ? imu_sim_params.delay
                          : 0.0,
                      P_stat_.recorded()) {
  // Compute parent frame to actual DCM.
  if (*g_sim.sim_opt & kSimOptImperfectSensors) {
    QuatToDcm(&imu_sim_params_.q_m2actual, &dcm_parent2actual_);
    Mat3Mat3Mult(&dcm_parent2actual_, &imu_params_.dcm_parent2m,
                 &dcm_parent2actual_);
  }

  // Note that the DelayedInterpolatingSignal sub-models must come after the
  // corresponding Measurement sub-models in this list to ensure the
  // measurements are updated first.
  set_sub_models({&acc_, &gyro_, &mag_, &P_stat_, &delayed_acc_, &delayed_gyro_,
                  &delayed_mag_, &delayed_P_stat_});

  delayed_acc_.set_val_func([this](Vec3 *v) { *v = acc_.recorded(); });
  delayed_gyro_.set_val_func([this](Vec3 *v) { *v = gyro_.recorded(); });
  delayed_mag_.set_val_func([this](Vec3 *v) { *v = mag_.recorded(); });
  delayed_P_stat_.set_val_func([this](double *v) { *v = P_stat_.recorded(); });

  SetupDone();
}

namespace {

void UpdateBias(const Vec3 &random_walk_bias_z1,
                const Vec3 &markov_process_bias_z1, const BiasParams &params,
                double dt, NamedRandomNumberGenerator *rng,
                Vec3 *random_walk_bias, Vec3 *markov_process_bias) {
  Vec3 tmp = {rng->GetNormal(), rng->GetNormal(), rng->GetNormal()};
  Vec3LinComb(1.0, &random_walk_bias_z1, params.random_walk_scale * Sqrt(dt),
              &tmp, random_walk_bias);

  tmp = {rng->GetNormal(), rng->GetNormal(), rng->GetNormal()};
  Vec3LinComb(exp(-dt * 2.0 * M_PI * params.markov_process_cutoff_freq),
              &markov_process_bias_z1, params.markov_process_scale * Sqrt(dt),
              &tmp, markov_process_bias);
}

}  // namespace

void Imu::DiscreteStepHelper(double t) {
  // Update bias random walks.
  Vec3 random_walk_bias, markov_process_bias;
  UpdateBias(acc_random_walk_bias_.val(), acc_markov_process_bias_.val(),
             imu_sim_params_.acc_bias, acc_random_walk_bias_.ts(), &rng_,
             &random_walk_bias, &markov_process_bias);

  acc_random_walk_bias_.DiscreteUpdate(t, random_walk_bias);
  acc_markov_process_bias_.DiscreteUpdate(t, markov_process_bias);

  UpdateBias(gyro_random_walk_bias_.val(), gyro_markov_process_bias_.val(),
             imu_sim_params_.gyro_bias, gyro_random_walk_bias_.ts(), &rng_,
             &random_walk_bias, &markov_process_bias);

  gyro_random_walk_bias_.DiscreteUpdate(t, random_walk_bias);
  gyro_markov_process_bias_.DiscreteUpdate(t, markov_process_bias);

  ReferenceFrame parent_accelerating_frame;
  parent_.CalcAcceleratingFrame(&parent_accelerating_frame);
  ReferenceFrame imu_frame(parent_accelerating_frame, imu_params_.pos,
                           dcm_parent2actual_);

  // Update accelerometer.
  Vec3 acc__;
  imu_frame.TransformFrom(environment_.ned_frame(),
                          ReferenceFrame::kAcceleration, environment_.g_ned(),
                          &acc__);
  Vec3Scale(&acc__, -1.0, &acc__);

  // Update gyro.
  Vec3 gyro__;
  imu_frame.TransformOriginFrom(environment_.ned_frame(),
                                ReferenceFrame::kAngularVelocity, &gyro__);
  Vec3Scale(&gyro__, -1.0, &gyro__);

  // Update magnetometer.
  Vec3 mag__;
  imu_frame.RotateFrom(environment_.ned_frame(), environment_.mag_ned(),
                       &mag__);

  // If imperfect sensors enabled, add vibration and biases.
  if (*g_sim.sim_opt & kSimOptImperfectSensors) {
    Vec3 acc_vibration_parent = imu_mount_.GetAccelVibration();
    Vec3 tmp;
    Mat3Vec3Mult(&dcm_parent2actual_, &acc_vibration_parent, &tmp);
    Vec3Add(&tmp, &acc__, &acc__);
    Vec3Add(&acc__, &acc_random_walk_bias_.val(), &acc__);
    Vec3Add(&acc__, &acc_markov_process_bias_.val(), &acc__);

    Vec3 gyro_vibration_parent = imu_mount_.GetGyroVibration();
    Mat3Vec3Mult(&dcm_parent2actual_, &gyro_vibration_parent, &tmp);
    Vec3Add(&tmp, &gyro__, &gyro__);
    Vec3Add(&gyro__, &gyro_random_walk_bias_.val(), &gyro__);
    Vec3Add(&gyro__, &gyro_markov_process_bias_.val(), &gyro__);

    // Add the external magnetic field.
    Mat3Vec3Axpby(&dcm_parent2actual_, kNoTrans,
                  &external_magnetic_field_parent(), 1.0, &mag__, &mag__);
  }
  actual_acc_.DiscreteUpdate(t, acc__);
  actual_gyro_.DiscreteUpdate(t, gyro__);
  actual_mag_.DiscreteUpdate(t, mag__);

  Vec3 imu_pos_ned;
  imu_frame.TransformOriginTo(environment_.ned_frame(),
                              ReferenceFrame::kPosition, &imu_pos_ned);

  double P_stat__ = environment_.pressure(-imu_pos_ned.z);
  actual_P_stat_.DiscreteUpdate(t, P_stat__);
}

void Imu::UpdateSensorOutputs(SimSensorMessage * /*sensor_message*/,
                              TetherUpMessage * /*tether_up*/) const {
  *updated_flight_comp_imu_message_ = true;
  *updated_flight_comp_sensor_message_ = true;
  UpdateImuPacket(flight_comp_imu_message_);
  UpdateSensorsPacket(flight_comp_sensor_message_);
}

void Imu::Publish() const {
  imu_telem_->actual_acc = actual_acc_.val();
  imu_telem_->actual_gyro = actual_gyro_.val();
  imu_telem_->actual_mag = actual_mag_.val();

  Vec3Add(&acc_random_walk_bias_.val(), &acc_markov_process_bias_.val(),
          &imu_telem_->acc_bias_parent);
  imu_telem_->acc_bias_parent.x += imu_sim_params_.acc_sensor[0].bias;
  imu_telem_->acc_bias_parent.y += imu_sim_params_.acc_sensor[1].bias;
  imu_telem_->acc_bias_parent.z += imu_sim_params_.acc_sensor[2].bias;
  Mat3TransVec3Mult(&imu_params_.dcm_parent2m, &imu_telem_->acc_bias_parent,
                    &imu_telem_->acc_bias_parent);

  Vec3Add(&gyro_random_walk_bias_.val(), &gyro_markov_process_bias_.val(),
          &imu_telem_->gyro_bias_parent);
  imu_telem_->gyro_bias_parent.x += imu_sim_params_.gyro_sensor[0].bias;
  imu_telem_->gyro_bias_parent.y += imu_sim_params_.gyro_sensor[1].bias;
  imu_telem_->gyro_bias_parent.z += imu_sim_params_.gyro_sensor[2].bias;
  Mat3TransVec3Mult(&imu_params_.dcm_parent2m, &imu_telem_->gyro_bias_parent,
                    &imu_telem_->gyro_bias_parent);

  imu_telem_->acc = acc();
  imu_telem_->gyro = gyro();
  imu_telem_->mag = mag();

  imu_telem_->actual_P_stat = actual_P_stat_.val();
  imu_telem_->P_stat = P_stat();
}

void Imu::UpdateImuPacket(FlightComputerImuMessage *imu) const {
  imu->raw.acc[0] =
      static_cast<float>(InvertCal(acc().x, &imu_params_.acc_cal[0]));
  imu->raw.acc[1] =
      static_cast<float>(InvertCal(acc().y, &imu_params_.acc_cal[1]));
  imu->raw.acc[2] =
      static_cast<float>(InvertCal(acc().z, &imu_params_.acc_cal[2]));

  imu->raw.gyro[0] =
      static_cast<float>(InvertCal(gyro().x, &imu_params_.gyro_cal[0]));
  imu->raw.gyro[1] =
      static_cast<float>(InvertCal(gyro().y, &imu_params_.gyro_cal[1]));
  imu->raw.gyro[2] =
      static_cast<float>(InvertCal(gyro().z, &imu_params_.gyro_cal[2]));

  // TODO: Set latency fields.
  imu->raw.latency = 0;
}

void Imu::UpdateSensorsPacket(FlightComputerSensorMessage *sensors) const {
  sensors->aux.mag[0] =
      static_cast<float>(InvertCal(mag().x, &imu_params_.mag_cal[0]));
  sensors->aux.mag[1] =
      static_cast<float>(InvertCal(mag().y, &imu_params_.mag_cal[1]));
  sensors->aux.mag[2] =
      static_cast<float>(InvertCal(mag().z, &imu_params_.mag_cal[2]));

  // TODO: Set latency fields.
  // TODO: Set cs[].
  sensors->aux.mag_latency = 0;

  sensors->aux.pressure =
      static_cast<float>(InvertCal(P_stat(), &imu_params_.pressure_cal));
  sensors->aux.pressure_latency = 0;
}

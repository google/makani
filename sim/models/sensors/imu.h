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

#ifndef SIM_MODELS_SENSORS_IMU_H_
#define SIM_MODELS_SENSORS_IMU_H_

#include <string>
#include <vector>

#include "common/c_math/vec3.h"
#include "common/macros.h"
#include "control/system_types.h"
#include "sim/faults/faults.h"
#include "sim/math/util.h"
#include "sim/models/environment.h"
#include "sim/models/imu_mount.h"
#include "sim/models/rigid_bodies/rigid_body.h"
#include "sim/models/sensors/sensor.h"
#include "sim/models/signals/delayed_signal.h"
#include "sim/models/signals/measurement.h"
#include "sim/physics/ground_frame.h"
#include "sim/sim_messages.h"
#include "sim/sim_telemetry.h"
#include "sim/state.h"
#include "system/labels.h"

// The magnetometer on the current M600 sensor interface board has
// several additional noise sources modeled by this class.  These
// include low-level harmonic noise as well as periodic glitches when
// certain GPS receivers are present.
//
// The frequencies of these harmonics and glitches are currently
// hard-coded in the class whereas the amplitudes are set by
// parameters passed to the constructor.
class MagnetometerMeasurement : public Measurement<Vec3> {
 public:
  MagnetometerMeasurement(const std::string &parent_full_name,
                          const std::string &name__, double ts,
                          const std::vector<SensorModelParams> &sensor_params,
                          const MagnetometerNoiseSimParams &params,
                          const DiscreteState<Vec3> &tracked,
                          FaultSchedule *faults)
      : Measurement(parent_full_name, name__, ts, sensor_params, tracked,
                    faults),
        params_(params),
        corrupted_(new_discrete_state(), "corrupted", 0.0, kVec3Zero) {}
  ~MagnetometerMeasurement() {}

  const Vec3 &recorded() const { return corrupted_.val(); }

 private:
  void DiscreteStepHelper(double t) override;

  // Parameters.
  const MagnetometerNoiseSimParams &params_;

  // Discrete state.
  DiscreteState<Vec3> corrupted_;
};

class Imu : public Sensor {
 public:
  Imu(const std::string &label, const Environment &environment,
      const RigidBody &parent, const ImuMount &imu_mount,
      const ImuParams &imu_params, const ImuSimParams &imu_sim_params,
      ImuTelemetry *imu_telem, bool *updated_flight_comp_imu_message,
      bool *updated_flight_comp_sensor_message,
      FlightComputerImuMessage *flight_comp_imu_message,
      FlightComputerSensorMessage *flight_comp_sensor_message,
      FaultSchedule *faults);
  ~Imu() __attribute__((noinline)) {}

  void UpdateSensorOutputs(SimSensorMessage * /*sensor_message*/,
                           TetherUpMessage * /*tether_up*/) const override;
  void Publish() const override;
  const Vec3 &GetPos() const { return imu_params_.pos; }

  void set_external_magnetic_field(const Vec3 &field_parent) {
    external_magnetic_field_parent_.set_val(field_parent);
  }

 protected:
  // IMU parameters.
  const ImuParams &imu_params_;
  const ImuSimParams &imu_sim_params_;

  // DCM for converting body coordinates to the actual measurement
  // coordinates.
  Mat3 dcm_parent2actual_;

  const Environment &environment_;
  const RigidBody &parent_;
  const ImuMount &imu_mount_;

 private:
  void DiscreteStepHelper(double t) override;

  const Vec3 &acc() const { return delayed_acc_.output(); }
  const Vec3 &gyro() const { return delayed_gyro_.output(); }
  const Vec3 &mag() const { return delayed_mag_.output(); }
  double P_stat() const { return delayed_P_stat_.output(); }

  const Vec3 &external_magnetic_field_parent() const {
    return external_magnetic_field_parent_.val();
  }

  void UpdateImuPacket(FlightComputerImuMessage *imu) const;
  void UpdateSensorsPacket(FlightComputerSensorMessage *sensor) const;

  NamedRandomNumberGenerator rng_;

  // Outputs.
  ImuTelemetry *imu_telem_;
  bool *updated_flight_comp_imu_message_;
  bool *updated_flight_comp_sensor_message_;
  FlightComputerImuMessage *flight_comp_imu_message_;
  FlightComputerSensorMessage *flight_comp_sensor_message_;

  // Derived values.
  State<Vec3> external_magnetic_field_parent_;

  // Discrete state.
  DiscreteState<Vec3> actual_acc_, actual_gyro_, actual_mag_;
  DiscreteState<double> actual_P_stat_;
  DiscreteState<Vec3> acc_random_walk_bias_, gyro_random_walk_bias_;
  DiscreteState<Vec3> acc_markov_process_bias_, gyro_markov_process_bias_;

  // Sub-models.
  Measurement<Vec3> acc_, gyro_;
  MagnetometerMeasurement mag_;
  Measurement<double> P_stat_;
  DelayedInterpolatingSignal<Vec3> delayed_acc_, delayed_gyro_, delayed_mag_;
  DelayedInterpolatingSignal<double> delayed_P_stat_;

  DISALLOW_COPY_AND_ASSIGN(Imu);
};

#endif  // SIM_MODELS_SENSORS_IMU_H_

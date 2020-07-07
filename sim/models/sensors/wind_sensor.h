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

#ifndef SIM_MODELS_SENSORS_WIND_SENSOR_H_
#define SIM_MODELS_SENSORS_WIND_SENSOR_H_

#include <stdint.h>

#include "common/c_math/vec3.h"
#include "common/macros.h"
#include "control/system_types.h"
#include "sim/faults/faults.h"
#include "sim/models/environment.h"
#include "sim/models/perch.h"
#include "sim/models/sensors/sensor.h"
#include "sim/models/signals/measurement.h"
#include "sim/physics/ground_frame.h"
#include "sim/sim_messages.h"
#include "sim/sim_types.h"
#include "sim/state.h"

// Models a generic wind sensor, which may be attached to a moving
// rigid body.
class WindSensor : public Sensor {
  friend class WindSensorTest;

 public:
  WindSensor(const WindSensorParams &wind_sensor_params,
             const WindSensorSimParams &wind_sensor_sim_params,
             FaultSchedule *faults);
  ~WindSensor() {}

  void UpdateSensorOutputs(SimSensorMessage * /*message*/,
                           TetherUpMessage *tether_up) const override;
  void Publish() const override;

  const Vec3 &pos_g() const { return pos_g_.val(); }

  void set_parent_frame(const ReferenceFrame &frame) {
    parent_frame_.set_val(frame);
  }
  void set_ground_frame(const ReferenceFrame &frame) {
    ground_frame_.set_val(frame);
  }
  void set_wind_g(const Vec3 &val) { wind_g_.set_val(val); }

 private:
  void AddInternalConnections(ConnectionStore *connections) override;
  void UpdateDerivedStates();
  void DiscreteStepHelper(double t) override;

  const ReferenceFrame &parent_frame() const { return parent_frame_.val(); }
  const ReferenceFrame &ground_frame() const { return ground_frame_.val(); }
  const ReferenceFrame &wind_sensor_frame() const {
    return wind_sensor_frame_.val();
  }

  const Vec3 &wind_g() const { return wind_g_.val(); }
  const Vec3 &measured_wind_ws() const { return measured_wind_ws_.recorded(); }

  // Wind sensor parameters.
  const WindSensorParams &wind_sensor_params_;
  const WindSensorSimParams &wind_sensor_sim_params_;

  // Inputs.

  // Wind velocity [m/s] in the ground frame.
  State<Vec3> wind_g_;

  // Ground reference frame.
  State<ReferenceFrame> ground_frame_;

  // Frame the wind sensor is rigidly attached to.  This is typically
  // the perch if we are simulating the perch, but it may also be the
  // vessel frame if there is no perch.
  State<ReferenceFrame> parent_frame_;

  // Output states.

  // Reference frame describing the sensor position and orientation.
  State<ReferenceFrame> wind_sensor_frame_;

  // Position [m] of wind sensor in ground coordinates.
  State<Vec3> pos_g_;

  // Discrete state.

  // Sampled wind speed [m/s] in the wind sensor frame.
  DiscreteState<Vec3> sampled_wind_ws_;

  // Sub-models.

  // Measured wind speed [m/s] in the wind sensor frame.
  //
  // TODO: The Measurement class is only used for its
  // fault scheduling capabilities.  We should separate out this
  // functionality.
  Measurement<Vec3> measured_wind_ws_;

  DISALLOW_COPY_AND_ASSIGN(WindSensor);
};

#endif  // SIM_MODELS_SENSORS_WIND_SENSOR_H_

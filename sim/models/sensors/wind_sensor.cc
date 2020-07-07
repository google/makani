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

#include "sim/models/sensors/wind_sensor.h"

#include <glog/logging.h>
#include <math.h>
#include <stdint.h>

#include "avionics/common/tether_message_types.h"
#include "common/c_math/vec3.h"
#include "common/macros.h"
#include "control/system_types.h"
#include "sim/faults/faults.h"
#include "sim/models/sensors/sensor.h"
#include "sim/models/signals/measurement.h"
#include "sim/physics/reference_frame.h"
#include "sim/sim_messages.h"
#include "sim/sim_telemetry.h"
#include "sim/sim_types.h"
#include "sim/state.h"

WindSensor::WindSensor(const WindSensorParams &wind_sensor_params,
                       const WindSensorSimParams &wind_sensor_sim_params,
                       FaultSchedule *faults)
    : Sensor("WindSensor"),
      wind_sensor_params_(wind_sensor_params),
      wind_sensor_sim_params_(wind_sensor_sim_params),
      wind_g_(new_input_value(), "wind_g"),
      ground_frame_(new_input_value(), "ground_frame"),
      parent_frame_(new_input_value(), "parent_frame"),
      wind_sensor_frame_(new_derived_value(), "wind_sensor_frame"),
      pos_g_(new_derived_value(), "pos_g"),
      sampled_wind_ws_(new_discrete_state(), "sampled_wind_ws", 0.0, kVec3Zero),
      measured_wind_ws_(full_name(), "measured_wind_ws",
                        wind_sensor_sim_params.sample_time, sampled_wind_ws_,
                        faults) {
  set_sub_models({&measured_wind_ws_});

  SetupDone();
}

void WindSensor::UpdateSensorOutputs(SimSensorMessage * /*message*/,
                                     TetherUpMessage *tether_up) const {
  TetherWind *wind = &tether_up->wind;
  wind->sequence =
      static_cast<uint16_t>((wind->sequence + 1) % TETHER_SEQUENCE_ROLLOVER);
  wind->no_update_count = 0;
  wind->status = kTetherWindStatusGood;
  wind->velocity[0] = static_cast<float>(measured_wind_ws().x);
  wind->velocity[1] = static_cast<float>(measured_wind_ws().y);
  wind->velocity[2] = static_cast<float>(measured_wind_ws().z);

  // TODO: Move this somewhere else.
  // TODO: Don't use hardcoded values.
  TetherWeather *weather = &tether_up->weather;
  weather->sequence =
      static_cast<uint16_t>((weather->sequence + 1) % TETHER_SEQUENCE_ROLLOVER);
  weather->no_update_count = 0;
  weather->flags = 0;
  weather->temperature = 25.0f;     // [C]
  weather->pressure_pa = 92060.0f;  // [Pa]
  weather->humidity = 7.0f;         // [%]
}

void WindSensor::Publish() const {
  sim_telem.wind_sensor.wind_g = wind_g();
  sim_telem.wind_sensor.measured_wind_ws = measured_wind_ws();
}

void WindSensor::UpdateDerivedStates() {
  wind_sensor_frame_.set_val(ReferenceFrame(parent_frame(),
                                            wind_sensor_params_.pos_parent,
                                            wind_sensor_params_.dcm_parent2ws));
  Vec3 pos_g__;
  wind_sensor_frame().TransformTo(ground_frame(), ReferenceFrame::kPosition,
                                  kVec3Zero, &pos_g__);
  pos_g_.set_val(pos_g__);
}

void WindSensor::AddInternalConnections(ConnectionStore *connections) {
  connections->Add(3, [this](double /*t*/) { UpdateDerivedStates(); });
}

void WindSensor::DiscreteStepHelper(double t) {
  // Transform the input wind velocity in the ground frame to
  // the wind sensor's local frame.  This accounts for the added
  // apparent wind from the moving perch.
  Vec3 wind_ws;
  wind_sensor_frame().TransformFrom(ground_frame(), ReferenceFrame::kVelocity,
                                    wind_g(), &wind_ws);

  sampled_wind_ws_.DiscreteUpdate(t, wind_ws);
}

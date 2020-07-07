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

#ifndef SIM_MODELS_SENSORS_SENSOR_H_
#define SIM_MODELS_SENSORS_SENSOR_H_

#include <string>

#include "common/macros.h"
#include "sim/models/model.h"
#include "sim/sim_messages.h"

class Sensor : public Model {
 public:
  Sensor(const std::string &name__, double ts__) : Model(name__, ts__) {}
  explicit Sensor(const std::string &name__) : Sensor(name__, 0.0) {}
  ~Sensor() __attribute__((noinline)) {}

  virtual void UpdateSensorOutputs(SimSensorMessage *sensor_message,
                                   TetherUpMessage *tether_up) const = 0;
  bool UpdatedSince(double t) const { return t_z1() >= t; }

 private:
  DISALLOW_COPY_AND_ASSIGN(Sensor);
};

#endif  // SIM_MODELS_SENSORS_SENSOR_H_

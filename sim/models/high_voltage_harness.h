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

// Models magnetic fields induced by the high voltage harness over a
// limited range of positions.
#ifndef SIM_MODELS_HIGH_VOLTAGE_HARNESS_H_
#define SIM_MODELS_HIGH_VOLTAGE_HARNESS_H_

#include <vector>

#include "common/c_math/vec3.h"
#include "common/macros.h"
#include "sim/models/model.h"
#include "sim/sim_types.h"
#include "sim/state.h"

class HighVoltageHarness : public Model {
 public:
  explicit HighVoltageHarness(const HighVoltageHarnessSimParams &params);
  ~HighVoltageHarness() {}

  void set_motor_current(MotorLabel i, double current) {
    motor_currents_[i].set_val(current);
  }
  void CalcInducedMagneticField(const Vec3 &pos_b, Vec3 *induced_field_b);

 private:
  void DiscreteStepHelper(double /*t*/) override {}

  double motor_current(MotorLabel i) const { return motor_currents_[i].val(); }

  // Parameters.
  const HighVoltageHarnessSimParams &params_;

  // Derived values.

  // Motor currents [A] (positive when generating).
  std::vector<State<double>> motor_currents_;

  DISALLOW_COPY_AND_ASSIGN(HighVoltageHarness);
};

#endif  // SIM_MODELS_HIGH_VOLTAGE_HARNESS_H_

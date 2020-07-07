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

#ifndef SIM_SIM_DYNAMIC_SYSTEM_H_
#define SIM_SIM_DYNAMIC_SYSTEM_H_

#include <stdint.h>

#include <string>
#include <vector>

#include "common/macros.h"
#include "sim/math/ode_solver.h"
#include "sim/models/model.h"

namespace sim {

// Class for interfacing a Model with an ODE solver or linearization
// function.  Conceptually, an ODE solver wants to evaluate an ODE as
// a function of time and a state, i.e.:
//   .
//   x = f(t, x).
//
// In the simulator the state x, is a vector with components for each
// ContinuousState member of each Model.  Evaluating "f" consists of
// updating the continuous states and derived values of each model,
// then calculating their derivatives (see UpdateModelsFromStateVector).
// Likewise, linearizing a Model involves evaluating:
//   .
//   x = f(t, x, u)
//   y = g(t, x, u)
//
class SimDynamicSystem : public OdeSystem {
 public:
  explicit SimDynamicSystem(Model *model);
  ~SimDynamicSystem() {}

  int32_t num_states() const { return num_states_; }
  int32_t num_inputs() const { return num_inputs_; }
  int32_t num_outputs() const { return num_outputs_; }

  const std::string &state_names(int32_t i) const { return state_names_[i]; }
  const std::string &input_names(int32_t i) const { return input_names_[i]; }
  const std::string &output_names(int32_t i) const { return output_names_[i]; }

  void UpdateFromStateVector(double t, const std::vector<double> &x) const;
  void CalcDerivatives(double t, const std::vector<double> &x,
                       std::vector<double> *dx) const;

 private:
  // Number of serializable states, inputs, and outputs of the system.
  int32_t num_states_;
  int32_t num_inputs_;
  int32_t num_outputs_;

  std::vector<std::string> state_names_;
  std::vector<std::string> input_names_;
  std::vector<std::string> output_names_;

  // In the simulator, models are the fundamental unit of dynamic
  // systems.  They contain state, sub-models, and their connections.
  // SimDynamicSystem contains a pointer to the parent model of the
  // system.
  Model *model_;

  DISALLOW_COPY_AND_ASSIGN(SimDynamicSystem);
};

}  // namespace sim

#endif  // SIM_SIM_DYNAMIC_SYSTEM_H_

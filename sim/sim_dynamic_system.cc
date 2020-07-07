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

#include "sim/sim_dynamic_system.h"

#include <stdint.h>

#include <string>
#include <vector>

#include "sim/models/model.h"
#include "sim/state.h"

namespace sim {

namespace {

int32_t AddSerializableNames(const std::vector<StateInterface *> &states,
                             std::vector<std::string> *names) {
  int32_t count = 0;
  for (const StateInterface *state : states) {
    if (state->IsSerializable()) {
      for (int32_t i = 0; i < state->Size(); ++i) {
        names->push_back(state->name() + "[" + std::to_string(i) + "]");
        ++count;
      }
    }
  }
  return count;
}

}  // namespace

SimDynamicSystem::SimDynamicSystem(Model *model)
    : num_states_(0),
      num_inputs_(0),
      num_outputs_(0),
      state_names_(),
      input_names_(),
      output_names_(),
      model_(model) {
  std::vector<StateInterface *> continuous_states(
      model->continuous_states().begin(), model->continuous_states().end());
  num_states_ += AddSerializableNames(continuous_states, &state_names_);
  num_inputs_ += AddSerializableNames(model->input_values(), &input_names_);
  num_outputs_ += AddSerializableNames(model->derived_values(), &output_names_);

  for (const Model *m : model->sub_models()) {
    std::vector<StateInterface *> m_continuous_states(
        m->continuous_states().begin(), m->continuous_states().end());
    num_states_ += AddSerializableNames(m_continuous_states, &state_names_);
    num_inputs_ += AddSerializableNames(m->input_values(), &input_names_);
    num_outputs_ += AddSerializableNames(m->derived_values(), &output_names_);
  }
}

void SimDynamicSystem::UpdateFromStateVector(
    double t, const std::vector<double> &x) const {
  model_->SetStateFromVector(x.data());
  model_->RunConnections(t);
  model_->CalcDeriv(t);
}

void SimDynamicSystem::CalcDerivatives(double t, const std::vector<double> &x,
                                       std::vector<double> *dx) const {
  UpdateFromStateVector(t, x);
  model_->GetDerivToVector(dx->data());
}

}  // namespace sim

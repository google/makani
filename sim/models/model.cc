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

#include "sim/models/model.h"

#include <glog/logging.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include <limits>
#include <map>
#include <string>
#include <vector>

#include "sim/interfaces.h"
#include "sim/state.h"

void Model::ClearIntegrationState() {
  ClearContinuousStates();
  ClearInputValues();
  ClearDerivedValues();
}

void Model::SetStateFromVector(const double state_vec[]) {
  int32_t state_ind = 0;
  for (ContinuousStateInterface *continuous_state : continuous_states_) {
    continuous_state->Clear();
    state_ind += continuous_state->Deserialize(&state_vec[state_ind]);
  }
  ClearInputValues();
  ClearDerivedValues();

  for (Model *sub_model : sub_models_) {
    sub_model->SetStateFromVector(&state_vec[state_ind]);
    state_ind += sub_model->num_states();
  }
}

void Model::GetStateToVector(double state_vec[]) const {
  int32_t state_ind = 0;
  for (const ContinuousStateInterface *continuous_state : continuous_states_) {
    state_ind += continuous_state->Serialize(&state_vec[state_ind]);
  }
  for (const Model *sub_model : sub_models_) {
    sub_model->GetStateToVector(&state_vec[state_ind]);
    state_ind += sub_model->num_states();
  }
}

void Model::GetDerivToVector(double deriv_vec[]) const {
  int32_t state_ind = 0;
  for (const ContinuousStateInterface *continuous_state : continuous_states_) {
    state_ind += continuous_state->SerializeDeriv(&deriv_vec[state_ind]);
  }
  for (const Model *sub_model : sub_models_) {
    sub_model->GetDerivToVector(&deriv_vec[state_ind]);
    state_ind += sub_model->num_states();
  }
}

int32_t Model::Save(DictFormatterInterface *out) const {
  int32_t error = 0;
  DictFormatterInterface *child = out->CreateChild(name(), &error);
  if (error) return -1;
  if (child == nullptr) return -1;

  for (const ContinuousStateInterface *continuous_state : continuous_states_) {
    if (continuous_state->Save(child)) return -1;
  }

  for (const DiscreteStateInterface *discrete_state : discrete_states_) {
    if (discrete_state->Save(child)) return -1;
  }

  for (const Model *sub_model : sub_models_) {
    if (sub_model->Save(child)) return -1;
  }

  return 0;
}

int32_t Model::Load(const DictFormatterInterface &in) {
  int32_t error = 0;
  const DictFormatterInterface *child = in.GetChild(name(), &error);
  if (error) return -1;
  if (child == nullptr) return -1;

  for (ContinuousStateInterface *continuous_state : continuous_states_) {
    continuous_state->Clear();
    error = continuous_state->Load(*child);
    if (error) return -1;
  }
  ClearInputValues();
  ClearDerivedValues();

  for (DiscreteStateInterface *discrete_state : discrete_states_) {
    discrete_state->Clear();
    if (discrete_state->Load(*child)) return -1;
  }

  for (Model *sub_model : sub_models_) {
    if (sub_model->Load(*child)) return -1;
  }

  return 0;
}

void Model::CalcDeriv(double t) {
  // Check that all derived values have been updated before
  // calculating the derivative of a model.
  for (const StateInterface *derived_value : derived_values_) {
    DCHECK(derived_value->IsUpdated()) << derived_value->name()
                                       << " was not updated.";
  }

  CalcDerivHelper(t);
  for (Model *sub_model : sub_models_) {
    sub_model->CalcDeriv(t);
  }
}

void Model::DiscreteUpdate(double t) {
  double t_cmp = t * (1.0 + std::numeric_limits<double>::epsilon());
  if (t_cmp - t_z1_ >= ts_) {
    DiscreteStepHelper(t);
    if (ts_ > 0.0) {
      t_z1_ = t_cmp - fmod(t_cmp, ts_);
    } else {
      t_z1_ = t;
    }
  }

  for (Model *sub_model : sub_models_) {
    sub_model->DiscreteUpdate(t);
  }
}

void Model::Disp() const {
  printf("\n\n***** %s *****", full_name_.c_str());
  if (!continuous_states_.empty()) {
    printf("\nContinuous state:");
  }
  for (const ContinuousStateInterface *continuous_state : continuous_states_) {
    continuous_state->Disp();
  }

  if (!continuous_states_.empty()) {
    printf("\nContinuous state derivatives:");
  }
  for (const ContinuousStateInterface *continuous_state : continuous_states_) {
    continuous_state->DispDeriv();
  }

  if (!discrete_states_.empty()) {
    printf("\nDiscrete state:");
  }
  for (const DiscreteStateInterface *discrete_state : discrete_states_) {
    discrete_state->Disp();
  }

  if (!derived_values_.empty()) {
    printf("\nDerived values:");
  }
  for (const StateInterface *derived_value : derived_values_) {
    derived_value->Disp();
  }

  if (!sub_models_.empty()) {
    printf("\nSub-Models:");
  }
  for (const Model *sub_model : sub_models_) {
    sub_model->Disp();
  }
}

void Model::ClearInputValues() {
  for (StateInterface *input_value : input_values_) {
    input_value->Clear();
  }
  for (Model *sub_model : sub_models_) {
    sub_model->ClearInputValues();
  }
}

void Model::ClearContinuousStates() {
  for (ContinuousStateInterface *continuous_state : continuous_states_) {
    continuous_state->Clear();
  }
  for (Model *sub_model : sub_models_) {
    sub_model->ClearContinuousStates();
  }
}

void Model::ClearDiscreteStates() {
  for (DiscreteStateInterface *discrete_state : discrete_states_) {
    discrete_state->Clear();
  }
  for (Model *sub_model : sub_models_) {
    sub_model->ClearDiscreteStates();
  }
}

void Model::ClearDerivedValues() {
  for (StateInterface *derived_value : derived_values_) {
    derived_value->Clear();
  }
  for (Model *sub_model : sub_models_) {
    sub_model->ClearDerivedValues();
  }
}

int32_t Model::CalcNumStates() {
  int32_t count = 0;
  for (const ContinuousStateInterface *continuous_state : continuous_states_) {
    count += continuous_state->Size();
  }
  for (const Model *sub_model : sub_models_) {
    count += sub_model->num_states();
  }
  return count;
}

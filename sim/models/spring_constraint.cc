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

#include "sim/models/spring_constraint.h"
#include <glog/logging.h>
#include "sim/sim_telemetry.h"

SpringConstraint::SpringConstraint(const ConstraintSimParams &params)
    : Model("SpringConstraint"),
      params_(params),
      length_(new_continuous_state(), "length", params_.initial_length),
      Xg_end_(new_derived_value(), "Xg_end"),
      let_out_slack_(new_derived_value(), "let_out_slack", false),
      end_force_moment_(new_derived_value(), "end_force_moment") {
  SetupDone();
}

void SpringConstraint::Publish() const {
  sim_telem.constraint.tension = Vec3Norm(&end_force());
  sim_telem.constraint.length = length();
}

void SpringConstraint::UpdateDerivedStates() {
  ForceMomentPos end_force_moment__;
  CalcEndForceMomentPos(&end_force_moment__);
  end_force_moment_.set_val(end_force_moment__);
}

void SpringConstraint::AddInternalConnections(ConnectionStore *connections) {
  connections->Add(3, [this](double /*t*/) { UpdateDerivedStates(); });
}

void SpringConstraint::CalcDerivHelper(double /* t */) {
  double length_command = 0.0;
  if (let_out_slack()) {
    Vec3 dx;
    Vec3Sub(&params_.anchor_pos_g, &Xg_end(), &dx);
    length_command =
        fmin(params_.maximum_length, params_.slack_command + Vec3Norm(&dx));
  } else if (params_.tension_command <= 0.0) {
    length_command = length();
  } else {
    double tension_error = params_.tension_command - Vec3Norm(&end_force());
    length_command = length() - tension_error / params_.spring_constant;
  }

  length_.set_deriv(Saturate(
      2.0 * M_PI * params_.fc_pilot_response * (length_command - length()),
      -params_.maximum_rate, params_.maximum_rate));
}

void SpringConstraint::CalcEndForceMomentPos(ForceMomentPos *fmx) const {
  // Compute the displacement vector from one end of the line to the other.
  Vec3 dx;
  Vec3Sub(&params_.anchor_pos_g, &Xg_end(), &dx);
  double dx_norm = Vec3Norm(&dx);
  double length_elongation = dx_norm < length() ? 0.0 : dx_norm - length();
  double force_mag = params_.spring_constant * length_elongation;

  if (force_mag > 0.0) {
    DCHECK_GE(dx_norm, 0.0);
    Vec3Scale(&dx, force_mag / dx_norm, &fmx->force);
  } else {
    fmx->force = kVec3Zero;
  }

  fmx->moment = kVec3Zero;
  fmx->pos = Xg_end();
}

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

#ifndef SIM_MODELS_SPRING_CONSTRAINT_H_
#define SIM_MODELS_SPRING_CONSTRAINT_H_

#include <stdbool.h>
#include <stdint.h>

#include "common/c_math/vec3.h"
#include "control/system_types.h"
#include "sim/models/model.h"

// The SpringConstraint class models a section of line (e.g. rope) that
// exerts no force when slack, and acts as a spring when enlongated
// beyond its static length.  It is used to implement the constraint
// system used for hover testing.
class SpringConstraint : public Model {
 public:
  explicit SpringConstraint(const ConstraintSimParams &params);
  ~SpringConstraint() {}

  void Publish() const override;

  void set_Xg_end(const Vec3 &val) { Xg_end_.set_val(val); }
  void set_let_out_slack(const bool val) { let_out_slack_.set_val(val); }

  double length() const { return length_.val(); }
  bool let_out_slack() const { return let_out_slack_.val(); }
  const ForceMomentPos &end_force_moment() const {
    return end_force_moment_.val();
  }
  const Vec3 &end_force() const { return end_force_moment_.val().force; }

  void CalcEndForceMomentPos(ForceMomentPos *fmx) const;

 private:
  void AddInternalConnections(ConnectionStore *connections) override;
  void UpdateDerivedStates();
  void CalcDerivHelper(double t) override;
  void DiscreteStepHelper(double /*t*/) override {}

  const Vec3 &Xg_end() const { return Xg_end_.val(); }

  // Parameters.
  const ConstraintSimParams &params_;

  // Continuous state.

  // Length [m] of the constraint varied by the pilot.
  ContinuousState<double> length_;

  // Derived values.

  // Positions [m] in ground-station coordinates of the two end-points
  // of the constraint.  The start position is the fixed anchor point,
  // and the end position is on the wing.
  State<Vec3> Xg_end_;
  State<bool> let_out_slack_;

  // Force-moment exerted by the constraint system on the wing.
  State<ForceMomentPos> end_force_moment_;

  DISALLOW_COPY_AND_ASSIGN(SpringConstraint);
};

#endif  // SIM_MODELS_SPRING_CONSTRAINT_H_

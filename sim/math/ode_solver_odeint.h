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

#ifndef SIM_MATH_ODE_SOLVER_ODEINT_H_
#define SIM_MATH_ODE_SOLVER_ODEINT_H_

#include <vector>

#include "common/macros.h"
#include "sim/math/ode_solver.h"
#include "sim/sim_types.h"

namespace sim {

// Wrapper for the odeint ODE library.
class OdeIntOdeSolver : public OdeSolver {
 public:
  explicit OdeIntOdeSolver(const OdeSystem &ode_system,
                           const SimOdeSolverParams &params)
      : params_(params), ode_system_(ode_system) {}
  ~OdeIntOdeSolver() {}

  OdeSolverStatus Integrate(double t0, double tf, const std::vector<double> &x0,
                            double *t_int, std::vector<double> *x) override;

  // This function is used by the odeint library.
  void operator()(const std::vector<double> &x,
                  std::vector<double> &dx,  // NOLINT(runtime/references)
                  double t);

 private:
  // Solver parameters.
  const SimOdeSolverParams &params_;

  // ODE system to simulate.
  const OdeSystem &ode_system_;

  DISALLOW_COPY_AND_ASSIGN(OdeIntOdeSolver);
};

}  // namespace sim

#endif  // SIM_MATH_ODE_SOLVER_ODEINT_H_

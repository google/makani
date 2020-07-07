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

#ifndef SIM_MATH_ODE_SOLVER_GSL_H_
#define SIM_MATH_ODE_SOLVER_GSL_H_

#include <gsl/gsl_errno.h>
#include <gsl/gsl_odeiv2.h>
#include <stdint.h>

#include <vector>

#include "common/macros.h"
#include "sim/math/ode_solver.h"
#include "sim/sim_types.h"

namespace sim {

// Wrapper for the GSL ODE library.
class GslOdeSolver : public OdeSolver {
 public:
  explicit GslOdeSolver(const OdeSystem &ode_system,
                        const SimOdeSolverParams &params);
  ~GslOdeSolver() {
    if (ode_driver_ != nullptr) gsl_odeiv2_driver_free(ode_driver_);
  }

  OdeSolverStatus Integrate(double t0, double tf, const std::vector<double> &x0,
                            double *t_int, std::vector<double> *x) override;

 private:
  // Static callback function for the GSL ODE solver.
  //
  // Args:
  //   t: Time at which derivative will be evaluated.
  //   x: Array of length num_states() containing the state at which
  //       the derivative will be evaluated.
  //   dx: Array of length num_states() into which the derivative is stored.
  //   context: Pointer to the OdeSolver class containing the OdeSystem.
  //
  // Returns:
  //   GSL_SUCCESS if the derivative was calculated successfully,
  //   GSL_FAILURE if the time step was too large, or
  //   GSL_EBADFUNC if the integration should be aborted immediately.
  static int32_t GslCallback(double t, const double x[], double dx[],
                             void *context);

  // Solver parameters.
  const SimOdeSolverParams &params_;

  // ODE to be solved.
  const OdeSystem &ode_system_;

  // Parameters for GSL.
  gsl_odeiv2_system sys_;
  gsl_odeiv2_driver *ode_driver_;

  DISALLOW_COPY_AND_ASSIGN(GslOdeSolver);
};

}  // namespace sim

#endif  // SIM_MATH_ODE_SOLVER_GSL_H_

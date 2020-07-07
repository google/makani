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

#ifndef SIM_MATH_ODE_SOLVER_H_
#define SIM_MATH_ODE_SOLVER_H_

#include <stdint.h>

#include <gsl/gsl_errno.h>
#include <gsl/gsl_odeiv2.h>

#include <vector>

namespace sim {

// Enumeration for return codes.
enum class OdeSolverStatus { kSuccess, kError, kOutOfPhysics };

// Interface representing a system of ordinary differential equations.
class OdeSystem {
 public:
  virtual ~OdeSystem() {}

  // Get the number of states for the ODE.
  virtual int32_t num_states() const = 0;

  // Calculate the state derivative for a given time and state.
  virtual void CalcDerivatives(double t, const std::vector<double> &x,
                               std::vector<double> *dx) const = 0;
};

// Interface for ODE solvers.
class OdeSolver {
 public:
  virtual ~OdeSolver() {}

  // Integrate the ODE with initial condition x0 at time t0 until time tf.
  //
  // Args:
  //   t0: Initial time.
  //   tf: Final time (>= t0).
  //   x0: Initial state.
  //   t_int: Null pointer or output time at which the integrator terminated.
  //   x: Terminal state.  It should be safe to reuse x0 as x.
  //
  // Returns:
  //   kSuccess on success (indicating *t_int = tf) and kError otherwise.
  virtual OdeSolverStatus Integrate(double t0, double tf,
                                    const std::vector<double> &x0,
                                    double *t_int, std::vector<double> *x) = 0;
};

}  // namespace sim

#endif  // SIM_MATH_ODE_SOLVER_H_

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

#include "sim/math/ode_solver_odeint.h"

#include <glog/logging.h>
#include <boost/numeric/odeint.hpp>
#include <boost/ref.hpp>

#include <algorithm>
#include <exception>
#include <vector>

#include "sim/math/ode_solver.h"

namespace odeint = boost::numeric::odeint;

namespace sim {

OdeSolverStatus OdeIntOdeSolver::Integrate(double t0, double tf,
                                           const std::vector<double> &x0,
                                           double *t_int,
                                           std::vector<double> *x) {
  std::copy(x0.begin(), x0.end(), x->begin());
  try {
    odeint::runge_kutta_cash_karp54<std::vector<double>> stepper;
    odeint::integrate_adaptive(
        odeint::make_controlled(params_.abs_tolerance, params_.rel_tolerance,
                                stepper),
        boost::ref(*this), *x, t0, tf, params_.initial_time_step);
    if (t_int != nullptr) {
      *t_int = tf;
    }
  } catch (std::exception &ex) {
    LOG(ERROR) << "odeint reported an error: " << ex.what();
    return OdeSolverStatus::kError;
  }
  return OdeSolverStatus::kSuccess;
}

void OdeIntOdeSolver::operator()(
    const std::vector<double> &x,
    std::vector<double> &dx,  // NOLINT(runtime/references)
    double t) {
  ode_system_.CalcDerivatives(t, x, &dx);
}

}  // namespace sim

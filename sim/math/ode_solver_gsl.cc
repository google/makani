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

#include "sim/math/ode_solver_gsl.h"

#include <glog/logging.h>
#include <gsl/gsl_errno.h>
#include <gsl/gsl_odeiv2.h>
#include <stdint.h>

#include <algorithm>
#include <exception>
#include <vector>

#include "sim/math/ode_solver.h"
#include "sim/sim_types.h"

namespace sim {

GslOdeSolver::GslOdeSolver(const OdeSystem &ode_system,
                           const SimOdeSolverParams &params)
    : params_(params), ode_system_(ode_system), sys_(), ode_driver_(nullptr) {
  sys_.function = GslCallback;
  sys_.jacobian = nullptr;
  sys_.dimension = ode_system_.num_states();
  // Since params is a non-const pointer, we pass in the
  // GslOdeSolver's 'this' pointer rather than a pointer to the
  // ode_system itself.
  sys_.params = this;

  const gsl_odeiv2_step_type *step_type = nullptr;

  if (params_.type == kSimOdeSolverGslRk2) {
    step_type = gsl_odeiv2_step_rk2;
  } else if (params_.type == kSimOdeSolverGslRkck) {
    step_type = gsl_odeiv2_step_rkck;
  } else if (params_.type == kSimOdeSolverGslRkf45) {
    step_type = gsl_odeiv2_step_rkf45;
  } else if (params_.type == kSimOdeSolverGslMsadams) {
    step_type = gsl_odeiv2_step_msadams;
  } else {
    CHECK(false) << "Unknown GSL solver type.";
  }

  ode_driver_ = gsl_odeiv2_driver_alloc_standard_new(
      &sys_, step_type, params_.initial_time_step, params_.abs_tolerance,
      params_.rel_tolerance, 1.0, 1.0);
}

OdeSolverStatus GslOdeSolver::Integrate(double t0, double tf,
                                        const std::vector<double> &x0,
                                        double *t_int, std::vector<double> *x) {
  double t_int_value = t0;
  std::copy(x0.begin(), x0.end(), x->begin());
  int32_t status;
  try {
    status = gsl_odeiv2_driver_apply(ode_driver_, &t_int_value, tf, x->data());
  } catch (std::domain_error &e) {
    LOG(WARNING) << "Exiting early due to sim exiting physical model: "
                 << e.what();
    return OdeSolverStatus::kOutOfPhysics;
  }
  if (t_int != nullptr) {
    *t_int = t_int_value;
  }

  if (status != GSL_SUCCESS) {
    LOG(WARNING) << "gsl_odeiv2_driver_apply error: return value = " << status;
    return OdeSolverStatus::kError;
  }

  return OdeSolverStatus::kSuccess;
}

int32_t GslOdeSolver::GslCallback(double t, const double x[], double dx[],
                                  void *context) {
  GslOdeSolver *ode_solver = reinterpret_cast<GslOdeSolver *>(context);

  std::vector<double> x_vec(x, x + ode_solver->ode_system_.num_states());
  std::vector<double> dx_vec(ode_solver->ode_system_.num_states());

  // The CalcDerivatives function may throw runtime_error if the time
  // step was too large and should be retried.
  try {
    ode_solver->ode_system_.CalcDerivatives(t, x_vec, &dx_vec);
  } catch (std::runtime_error &e) {
    LOG(WARNING) << "GslCallback: " << e.what() << " at t = " << t;
    return GSL_FAILURE;
  }

  std::copy(dx_vec.begin(), dx_vec.end(), dx);

  return GSL_SUCCESS;
}

}  // namespace sim

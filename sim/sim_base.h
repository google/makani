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

#ifndef SIM_SIM_BASE_H_
#define SIM_SIM_BASE_H_

#include <stdint.h>
#include <sys/time.h>

#include <memory>
#include <string>
#include <vector>

#include "common/macros.h"
#include "control/system_types.h"
#include "sim/faults/faults.h"
#include "sim/math/ode_solver.h"
#include "sim/models/actuators/actuator.h"
#include "sim/models/base_system_model.h"
#include "sim/models/full_system.h"
#include "sim/sim_comms.h"
#include "sim/sim_dynamic_system.h"
#include "sim/sim_messages.h"
#include "sim/sim_types.h"

namespace sim {

class SimBase {
 public:
  SimBase(double sim_time, const SystemParams &system_params,
          const SimParams &sim_params);
  virtual ~SimBase() {}

  static bool AioCallback(void *context);

  int32_t RunAio();
  int32_t Run();

 protected:
  double t() const { return static_cast<double>(sim_time_step_) * ts_; }

  // Populates the simulation state vector and state derivative with
  // their initial values.
  void InitStateVector();

  // Initializes the ODE driver.
  void InitOdeDriver();

  // Initializes values used in SleepForRealtime.
  void InitRealtime();

  // Initializes the random number generator.
  void InitRandom();

  virtual void PublishMessages() = 0;
  virtual void ReceiveMessages() = 0;

  // Parameters.
  const SystemParams &system_params_;
  const SimParams &sim_params_;

  FaultSchedule faults_;

  // Timing information.
  const double ts_;
  double last_publish_time_;

  // Objects for defining the system.
  std::unique_ptr<BaseSystemModel> system_model_;
  std::unique_ptr<SimDynamicSystem> dynamic_system_;

  // Reset the state vector when starting the simulator or loading state.
  void ResetStateVector();
  virtual void InitRun();
  virtual OdeSolverStatus RunStep() = 0;
  virtual void FinishRun() {}
  const double sim_time_;
  uint64_t sim_time_step_;
  struct timeval last_time_, start_time_;

  // Interface to the ODE solver.
  std::unique_ptr<OdeSolver> ode_solver_;
  std::vector<double> state_vec_;

 private:
  DISALLOW_COPY_AND_ASSIGN(SimBase);
};

}  // namespace sim

#endif  // SIM_SIM_BASE_H_

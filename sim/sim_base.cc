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

#include "sim/sim_base.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "avionics/common/avionics_messages.h"
#include "avionics/common/pack_tether_message.h"
#include "avionics/linux/aio.h"
#include "avionics/linux/clock.h"
#include "avionics/network/aio_node.h"
#include "avionics/network/message_type.h"
#include "control/system_params.h"
#include "control/system_types.h"
#include "sim/math/ode_solver.h"
#include "sim/math/ode_solver_gsl.h"
#include "sim/math/ode_solver_odeint.h"
#include "sim/sim_comms.h"
#include "sim/sim_params.h"
#include "sim/sim_telemetry.h"
#include "sim/sim_types.h"
#include "system/labels.h"

namespace sim {

SimBase::SimBase(double sim_time, const SystemParams &system_params,
                 const SimParams &sim_params)
    : system_params_(system_params),
      sim_params_(sim_params),
      faults_((*g_sim.sim_opt & kSimOptFaults) ? &sim_params_.faults_sim
                                               : nullptr),
      ts_(system_params.ts),
      last_publish_time_(0.0),
      system_model_(nullptr),
      dynamic_system_(nullptr),
      sim_time_(sim_time),
      sim_time_step_(0U),
      last_time_({0L, 0L}),
      start_time_({0L, 0L}),
      ode_solver_(nullptr),
      state_vec_() {
  CHECK_GE(sim_params_.telemetry_sample_period, 0.0);
}

bool SimBase::AioCallback(void *context) {
  SimBase *sim = reinterpret_cast<SimBase *>(context);
  sim->RunStep();
  sim_telem.aio_idle_usec = AioGetIdleUsec();
  if (sim_telem.aio_idle_usec == 0L) {
    LOG(WARNING) << "aio_idle_usec==0; the simulator may not have read all "
                 << "incoming AIO messages.";
  }
  return sim->t() <= sim->sim_time_;
}

// Invokes AIO to run the simulator at a fixed rate.
//
// Returns:
//   This function should not be expected to return.
int32_t SimBase::RunAio() {
  InitRun();

  // Configure AIO message subscriptions.
  std::vector<MessageType> subscribe_types = {
      kMessageTypeControllerCommand, kMessageTypeSmallControlTelemetry};
  if (system_params_.hitl.config.use_software_joystick) {
    subscribe_types.push_back(kMessageTypeJoystickStatus);
  }
  if (system_params_.hitl.config.gs02_level == kActuatorHitlLevelReal) {
    subscribe_types.push_back(kMessageTypeGroundStationStatus);
  }
  if (system_params_.hitl.config.motor_level == kActuatorHitlLevelReal) {
    subscribe_types.push_back(kMessageTypeMotorStatus);
  }
  if (system_params_.hitl.config.tether_release_level ==
      kActuatorHitlLevelReal) {
    subscribe_types.push_back(kMessageTypeLoadcell);
  }
  bool any_hitl_servos = false;
  for (int32_t i = 0; i < kNumServos; ++i) {
    any_hitl_servos |=
        (system_params_.hitl.config.servo_levels[i] == kActuatorHitlLevelReal);
  }
  if (any_hitl_servos) {
    subscribe_types.push_back(kMessageTypeServoDebug);
  }

  // Stepping of the simulator at a fixed rate is handled by AIO.
  AioLoopStart(
      kAioNodeSimulator, system_params_.comms.aio_port, subscribe_types.data(),
      static_cast<int32_t>(subscribe_types.size()), sim::SimBase::AioCallback,
      this, static_cast<int32_t>(system_params_.ts * 1000000));
  AioClose();
  return 0;
}

// Runs the simulation loop until t() exceeds sim_time.
//
// Args:
//   sim_time: Completion time [s] of the simulation.
//
// Returns:
//   0 if the simulation ran without error, 1 otherwise.
int32_t SimBase::Run() {
  InitRun();

  OdeSolverStatus err = OdeSolverStatus::kSuccess;
  while (t() <= sim_time_) {
    err = RunStep();
    if (err != OdeSolverStatus::kSuccess) break;
  }

  FinishRun();

  // TODO: Treat OutOfPhysics as an error.  We are treating it as a
  // non-error because we want to process log data.  In the future, consider
  // processing log data on all exits, including asserts, but also track the
  // assert error in the telemetry.
  return (err == OdeSolverStatus::kError) ? 1 : 0;
}

void SimBase::ResetStateVector() {
  system_model_->GetStateToVector(state_vec_.data());
  dynamic_system_->UpdateFromStateVector(t(), state_vec_);
}

void SimBase::InitStateVector() {
  state_vec_.resize(dynamic_system_->num_states());
  ResetStateVector();
}

void SimBase::InitOdeDriver() {
  switch (sim_params_.ode_solver.type) {
    case kSimOdeSolverGslRk2:
    case kSimOdeSolverGslRkck:
    case kSimOdeSolverGslRkf45:
    case kSimOdeSolverGslMsadams:
      ode_solver_.reset(
          new GslOdeSolver(*dynamic_system_, sim_params_.ode_solver));
      break;

    case kSimOdeSolverOdeintRkck:
      ode_solver_.reset(
          new OdeIntOdeSolver(*dynamic_system_, sim_params_.ode_solver));
      break;

    default:
      CHECK(false) << "Unknown ODE solver type.";
      break;
  }
}

void SimBase::InitRealtime() {
  gettimeofday(&last_time_, nullptr);
  gettimeofday(&start_time_, nullptr);
}

void SimBase::InitRandom() {
  NamedRandomNumberGenerator::SetSeedOffset(sim_params_.random_seed_offset);
}

// Initialize variables before a simulation is run.
void SimBase::InitRun() { last_publish_time_ = 0.0; }

}  // namespace sim

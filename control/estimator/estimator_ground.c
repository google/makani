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

#include "control/estimator/estimator_ground.h"

#include "avionics/common/avionics_messages.h"
#include "avionics/common/cvt_avionics_messages.h"
#include "avionics/common/pack_avionics_messages.h"
#include "avionics/linux/aio.h"
#include "avionics/network/message_type.h"
#include "common/macros.h"
#include "control/avionics/avionics_interface.h"
#include "control/common.h"
#include "control/estimator/estimator_nav_ground.h"
#include "control/ground_telemetry.h"
#include "control/pack_ground_telemetry.h"

// Test that valid messages are simultaneously available for a critical
// list of sensors.  This must return true before the control system will
// advance out of the kInitializationStateWaitForValidData.
static bool EstimatorGroundIsDataReady(const FaultMask faults[]) {
  if (HasAnyFault(&faults[kSubsysGsGpsPos])) return false;
  if (HasAnyFault(&faults[kSubsysGsGpsVel])) return false;
  if (HasAnyFault(&faults[kSubsysGsCompassAngles])) return false;
  if (HasAnyFault(&faults[kSubsysGsAcc])) return false;
  if (HasAnyFault(&faults[kSubsysGsGyro])) return false;
  return true;
}

// Compute the next initialization state from the previous state.
static InitializationState EstimatorGroundGetNextInitState(
    const FaultMask faults[], InitializationState init_state) {
  // Update init_state.
  switch (init_state) {
    case kInitializationStateFirstEntry:
      return kInitializationStateWaitForValidData;

    case kInitializationStateWaitForValidData:
      return EstimatorGroundIsDataReady(faults) ? kInitializationStateFirstLoop
                                                : init_state;

    case kInitializationStateFirstLoop:
      return kInitializationStateRunning;

    default:
      assert(false);  // Fall-through intentional.
    case kInitializationStateRunning:
      return init_state;
  }
}

// Initialize the ground estimator state.
void EstimatorGroundInit(GroundEstimatorState *state) {
  assert(state != NULL);
  memset(state, 0, sizeof(*state));
  state->init_state = kInitializationStateFirstEntry;
  state->time = 0.0;
}

// Run the ground estimator.
bool EstimatorGroundStep(const SystemParams *system_params,
                         const ControlParams *control_params,
                         GroundEstimatorState *state) {
  // Handle initialization.

  if (state->init_state == kInitializationStateFirstEntry) {
    GroundvionicsInterfaceInit(&state->avionics_interface);
  }

  InitializationState next_init_state =
      EstimatorGroundGetNextInitState(state->faults, state->init_state);

  GroundEstimatorInput input;
  ConvertGroundvionicsToGroundEstimator(
      system_params, &control_params->ground_sensor_limits,
      &control_params->fault_detection, &state->avionics_interface,
      &state->input_messages, &input, state->faults);

  bool running = IsControlSystemRunning(state->init_state);
  if (running) {
    if (state->init_state == kInitializationStateFirstLoop) {
      EstimatorNavGroundInit(system_params, &control_params->estimator.nav,
                             &state->estimator);
    }
    bool initializing = state->time < control_params->estimator.t_initialize;
    EstimatorNavGroundStep(initializing, &input.imu, &input.gs_gps,
                           &input.gps_compass, state->faults, system_params,
                           &control_params->estimator.nav, &state->estimator,
                           &state->ground_estimate);

    state->ground_estimate.time = state->time;
  }
  GroundTelemetry *gt = GetGroundTelemetryMessage();
  gt->input = input;
  gt->estimate = state->ground_estimate;
  gt->estimator.init_state = state->init_state;

  state->init_state = next_init_state;
  state->time += system_params->ts;

  return running;
}

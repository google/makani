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

#ifndef ANALYSIS_CONTROL_REPLAY_ESTIMATOR_REPLAY_H_
#define ANALYSIS_CONTROL_REPLAY_ESTIMATOR_REPLAY_H_

#include <stdint.h>

#include "control/control_params.h"
#include "control/control_telemetry.h"
#include "control/control_types.h"
#include "control/estimator/estimator_types.h"

#ifdef __cplusplus
extern "C" {
#endif

// This file provides a python ctypes abstraction for replaying the estimator
// from within python. Example:
//
// from makani.analysis.control.replay import estimator_replay as replay
// import ctypes
//
// filename = '/path/to/file.h5'
// path = '/messages/kAioNodeControllerA/kMessageTypeControlTelemetry'
// num_messages = replay.H5GetNumMessages(filename, path)
// messages = (replay.ControlTelemetry * num_messages)()
// num_messages = replay.H5GetControlTelemetryMessages(filename, path, None,
//                                                     num_messages, messages)
//
// params = replay.GetControlParams().contents.estimator
// flight_mode = replay.FlightMode()
// state = replay.EstimatorState()
// replay.EstimatorReplayInit(ctypes.byref(params), ctypes.byref(flight_mode),
//                            ctypes.byref(state))
//
// estimate = replay.StateEstimate
// for i in range(num_messages):
//   replay.EstimatorReplayIterate(ctypes.byref(params),
//                                 ctypes.byref(messages[i]),
//                                 ctypes.byref(flight_mode),
//                                 ctypes.byref(state), ctypes.byref(estimate))

// Initialize the estimator.
void EstimatorReplayInit(const EstimatorParams *params, FlightMode *flight_mode,
                         EstimatorState *state);

// Iterate the estimator given input message and initial state (flight_mode_z1,
// state_z1). This function updates flight_mode_z1 and state_z1 with their
// current value and outputs the estimate as estimate.
void EstimatorReplayIterate(const EstimatorParams *params,
                            const ControlTelemetry *message,
                            FlightMode *flight_mode_z1,
                            EstimatorState *state_z1,
                            StateEstimate *estimate);

// Iterate the estimator over input message interval [first_index, last_index],
// and output the corresponding state and estimate in their respective array.
void EstimatorReplayIterateArray(const EstimatorParams *params,
                                 int64_t first_index, int64_t last_index,
                                 const ControlTelemetry messages[],
                                 FlightMode *flight_mode_z1,
                                 EstimatorState *state_z1,
                                 EstimatorState states[],
                                 StateEstimate estimates[]);

// Set the subsystem fault mask from first_index to last_index in the messages[]
// array for subsystem subsys. This function ORs the fault_mask with any
// existing faults.
void SetControlTelemetryFaults(int64_t first_index, int64_t last_index,
                               int32_t num_subsys,
                               const SubsystemLabel subsys[],
                               int32_t fault_mask,
                               ControlTelemetry messages[]);

// Clear the subsystem fault mask from first_index to last_index in the
// messages[] array for subsystem subsys. This function clears the associated
// fault_mask, then ORs the original faults from original_messages[].
void ClearControlTelemetryFaults(int64_t first_index, int64_t last_index,
                                 const ControlTelemetry original_messages[],
                                 int32_t num_subsys,
                                 const SubsystemLabel subsys[],
                                 int32_t fault_mask,
                                 ControlTelemetry messages[]);

// Get the number of messages in an HDF5 dataset.
int64_t H5GetNumMessages(const char *filename, const char *path);

// Export the ControlTelemetry messages to an array. Returns the number of
// messages exported.
int64_t H5GetControlTelemetryMessages(const char *filename, const char *path,
                                      const char *flight, int64_t num_messages,
                                      ControlTelemetry messages[]);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // ANALYSIS_CONTROL_REPLAY_ESTIMATOR_REPLAY_H_

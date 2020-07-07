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

#ifndef CONTROL_AVIONICS_AVIONICS_INTERFACE_H_
#define CONTROL_AVIONICS_AVIONICS_INTERFACE_H_

#include <stdbool.h>

#include "avionics/common/avionics_messages.h"
#include "control/avionics/avionics_interface_types.h"
#include "control/control_types.h"
#include "control/fault_detection/fault_detection_types.h"
#include "control/system_types.h"

#ifdef __cplusplus
extern "C" {
#endif

void AvionicsInterfaceInit(AvionicsInterfaceState *state);

void GroundvionicsInterfaceInit(GroundvionicsInterfaceState *state);

// Reads avionics messages from the CVT and converts them to controller data
// structures. Also sets the following faults:
//   - kFaultTypeNoUpdate,
//   - kFaultTypeThrownError,
//   - kFaultTypeImplausible,
//   - kFaultTypeOutOfRange.
void ConvertAvionicsToControl(const SystemParams *system_params,
                              const SensorLimitsParams *limits_params,
                              const FaultDetectionParams *fault_params,
                              AvionicsInterfaceState *state,
                              ControlInputMessages *input_messages,
                              ControlInput *control_input, FaultMask faults[]);

void ConvertRotors(const double rotors_omega_upper[],
                   const double rotors_omega_lower[],
                   const double rotors_torque[],
                   const RotorParams rotor_params[],
                   const RotorSensorParams rotor_sensor_params[],
                   ControllerCommandMessage *command_message);

void ConvertControlToAvionics(const ControlOutput *control_output,
                              const SystemParams *params,
                              ControllerCommandMessage *command_message,
                              ControllerSyncMessage *sync_message);

// Reads groundvionics messages from the CVT and converts them to estimator
// data structures. Also sets the following faults:
//   - kFaultTypeNoUpdate,
//   - kFaultTypeThrownError,
//   - kFaultTypeImplausible,
//   - kFaultTypeOutOfRange.
void ConvertGroundvionicsToGroundEstimator(
    const SystemParams *system_params,
    const GroundSensorLimitsParams *limits_params,
    const FaultDetectionParams *fault_params,
    GroundvionicsInterfaceState *state,
    GroundEstimatorInputMessages *input_messages,
    GroundEstimatorInput *ground_estimator_input, FaultMask faults[]);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // CONTROL_AVIONICS_AVIONICS_INTERFACE_H_

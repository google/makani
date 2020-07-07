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

#ifndef CONTROL_AVIONICS_AVIONICS_SIM_H_
#define CONTROL_AVIONICS_AVIONICS_SIM_H_

#include <stdbool.h>
#include <stdint.h>

#include "control/system_types.h"
#include "sim/sim_messages.h"

#ifdef __cplusplus
extern "C" {
#endif

// Returns subscribe types needed for a simulated run of the controller.
//
// Args:
//   hitl_config: HitlConfiguration of the current run.
//   num_subscribe_types: On output, provides the length of the returned array.
const MessageType *GetSubscribeTypesForSimulation(
    const HitlConfiguration *hitl_config, int32_t *num_subscribe_types);

// Indicates whether two HitlConfigurations are equivalent, to the extent needed
// for agreement between the simulator and controller.
bool HitlConfigurationsEquivalent(const HitlConfiguration *a,
                                  const HitlConfiguration *b);

// Spoof updates to the CVT from a SimSensorMessage.
//
// This function is used during simulations to place message from the
// simulator into the CVT.  These messages are then read back out by
// UpdateSimSensorMessageFromCvt below.
//
// Args:
//   sensor_message: Data to write to the CVT.
//   hitl_config: HitlConfiguration of the current run.
//   sequence: Sequence number used for all writes to the CVT.
//   timestamp: Timestamp used for all writes to the CVT.
void UpdateControllerCvtFromSimSensorMessage(
    const SimSensorMessage *sensor_message,
    const HitlConfiguration *hitl_config, uint16_t sequence, int64_t timestamp);

// Spoof updates to the CVT from a SimSensorMessage.
//
// This function is used during simulations to place message from the
// simulator into the CVT.  These messages are then read back out by
// UpdateSimSensorMessageFromCvt below.
//
// Args:
//   sensor_message: Data to write to the CVT.
//   sequence: Sequence number used for all writes to the CVT.
//   timestamp: Timestamp used for all writes to the CVT.
void UpdateGroundEstimatorCvtFromSimSensorMessage(
    const SimSensorMessage *sensor_message, uint16_t sequence,
    int64_t timestamp);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // CONTROL_AVIONICS_AVIONICS_SIM_H_

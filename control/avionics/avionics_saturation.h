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

#ifndef CONTROL_AVIONICS_AVIONICS_SATURATION_H_
#define CONTROL_AVIONICS_AVIONICS_SATURATION_H_

#include <stdbool.h>

#include "control/avionics/avionics_interface_types.h"
#include "control/control_types.h"
#include "control/fault_detection/fault_detection_types.h"
#include "control/system_types.h"

#ifdef __cplusplus
extern "C" {
#endif

// Signal conditioning on incoming sensor measurements.
void AvionicsSaturateSensors(const ControlInput *input_raw,
                             const SensorLimitsParams *limits,
                             FaultMask faults[], ControlInput *input);

void GroundvionicsSaturateSensors(const GroundEstimatorInput *input_raw,
                                  const GroundSensorLimitsParams *limits,
                                  FaultMask faults[],
                                  GroundEstimatorInput *input);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // CONTROL_AVIONICS_AVIONICS_SATURATION_H_

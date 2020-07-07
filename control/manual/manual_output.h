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

#ifndef CONTROL_MANUAL_MANUAL_OUTPUT_H_
#define CONTROL_MANUAL_MANUAL_OUTPUT_H_

#include "control/actuator_util.h"
#include "control/control_types.h"

#ifdef __cplusplus
extern "C" {
#endif

// Updates the state of the output stage by a single time step.
// Returns the final control output.
void ManualOutputStep(const Deltas *deltas, const ThrustMoment *thrust_moment,
                      const ManualOutputParams *params,
                      const double *flight_mode_time,
                      ControlOutput *control_output);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // CONTROL_MANUAL_MANUAL_OUTPUT_H_

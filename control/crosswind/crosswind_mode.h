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

#ifndef CONTROL_CROSSWIND_CROSSWIND_MODE_H_
#define CONTROL_CROSSWIND_CROSSWIND_MODE_H_

#include <stdbool.h>

#include "common/c_math/vec3.h"
#include "control/control_types.h"
#include "control/crosswind/crosswind_types.h"
#include "control/estimator/estimator_types.h"

#ifdef __cplusplus
extern "C" {
#endif

// Determines whether the crosswind controller is ready to switch into
// any of the crosswind modes.
bool CrosswindModeIsReadyFor(FlightMode proposed_flight_mode,
                             const FlightStatus *flight_status,
                             const StateEstimate *state_est,
                             double time_in_flare, CrosswindPathType path_type,
                             const CrosswindModeParams *params);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // CONTROL_CROSSWIND_CROSSWIND_MODE_H_

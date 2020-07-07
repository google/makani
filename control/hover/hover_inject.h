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

#ifndef CONTROL_HOVER_HOVER_INJECT_H_
#define CONTROL_HOVER_HOVER_INJECT_H_

#include <stdbool.h>

#include "common/c_math/vec3.h"
#include "control/control_types.h"
#include "control/hover/hover_types.h"

#ifdef __cplusplus
extern "C" {
#endif

// Initializes the hover signal injection module.  Note that this
// module currently has no state, and this function just validates the
// parameters.
void HoverInjectInit(const HoverInjectParams *params);

// Returns true if the parameters are valid.
bool HoverInjectValidateParams(const HoverInjectParams *params);

// Returns true if signal injection is enabled and will be used in the
// current flight plan and mode.
bool HoverInjectIsEnabled(const FlightStatus *flight_status,
                          const HoverInjectParams *params);

// Injects an additional position command.  This is intended to be
// called immediately before the position loop.
void HoverInjectPositionSignal(const FlightStatus *flight_status,
                               const Vec3 *wing_pos_g_cmd,
                               const Vec3 *hover_origin_g,
                               const HoverInjectParams *params,
                               Vec3 *wing_pos_g_cmd_out);

// Injects an additional Euler vector command.  This is intended to be
// called after the position loop and before the attitude loop.
void HoverInjectAnglesSignal(const FlightStatus *flight_status,
                             const Vec3 *angles_cmd,
                             const HoverInjectParams *params,
                             Vec3 *angles_cmd_out);

// Injects flap commands directly at the output.
void HoverInjectOutputSignal(const FlightStatus *flight_status,
                             const ControlOutput *control_output,
                             const HoverInjectParams *params,
                             ControlOutput *control_output_out);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // CONTROL_HOVER_HOVER_INJECT_H_

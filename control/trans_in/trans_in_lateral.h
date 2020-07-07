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

#ifndef CONTROL_TRANS_IN_TRANS_IN_LATERAL_H_
#define CONTROL_TRANS_IN_TRANS_IN_LATERAL_H_

#include <stdbool.h>

#include "control/trans_in/trans_in_types.h"

#ifdef __cplusplus
extern "C" {
#endif

// Validate parameters.
bool TransInLateralValidateParams(const TransInLateralParams *params);

// Run a single step of the lateral controller.
void TransInLateralStep(double wing_pos_ti_y_cmd, double wing_pos_ti_y,
                        double wing_vel_ti_y_cmd, double wing_vel_ti_y,
                        double airspeed, double aero_climb_angle,
                        const TransInLateralParams *params,
                        double *angle_of_sideslip_cmd, double *roll_ti_cmd,
                        double *yaw_ti_cmd, double *roll_rate_b_cmd,
                        double *yaw_rate_b_cmd);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // CONTROL_TRANS_IN_TRANS_IN_LATERAL_H_

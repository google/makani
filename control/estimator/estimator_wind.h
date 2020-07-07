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

#ifndef CONTROL_ESTIMATOR_ESTIMATOR_WIND_H_
#define CONTROL_ESTIMATOR_ESTIMATOR_WIND_H_

#include <stdbool.h>

#include "common/c_math/vec3.h"
#include "control/estimator/estimator_types.h"
#include "control/fault_detection/fault_detection_types.h"
#include "control/system_types.h"

#ifdef __cplusplus
extern "C" {
#endif

// Initialize wind estimate.
void EstimatorWindInit(EstimatorWindState *state);

// Estimate the current ground wind speed.
//
// Computes the estimated wind speed at the wind sensor height.  This
// value can be overridden by the kControlOptHardCodeWind flag.  The
// estimate is marked as valid if the perch azimuth estimate is valid
// and no faults are declared on the wind sensor.  Otherwise, the last
// valid value is held.
void EstimatorWindStep(bool initializing, const Vec3 *wind_ws,
                       const FaultMask *wind_sensor_fault,
                       const VesselEstimate *vessel,
                       const WindSensorParams *wind_sensor_params,
                       const EstimatorWindParams *params,
                       EstimatorWindState *state, WindEstimate *wind_g);

// Estimate the current wind speed at the kite.
void EstimatorWindAloftStep(bool initializing,
                            const ApparentWindEstimate *apparent_wind,
                            const ApparentWindSph *apparent_wind_pitot,
                            const Vec3 *Vg, const Mat3 *dcm_g2b,
                            const WindEstimate *wind_g,
                            const EstimatorWindParams *params,
                            EstimatorWindState *state,
                            WindEstimate *wind_aloft_g);
#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // CONTROL_ESTIMATOR_ESTIMATOR_WIND_H_

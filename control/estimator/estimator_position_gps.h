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

#ifndef CONTROL_ESTIMATOR_ESTIMATOR_POSITION_GPS_H_
#define CONTROL_ESTIMATOR_ESTIMATOR_POSITION_GPS_H_

#include <stdbool.h>

#include "common/c_math/mat3.h"
#include "common/c_math/vec3.h"
#include "control/control_types.h"
#include "control/estimator/estimator_types.h"
#include "control/fault_detection/fault_detection_types.h"
#include "control/sensor_types.h"

#ifdef __cplusplus
extern "C" {
#endif

// Validate the GPS module parameters.
bool EstimatorPositionGpsValidateParams(
    const EstimatorPositionGpsParams *params);

// Initialize the GPS module.
void EstimatorPositionGpsInit(EstimatorPositionGpsState *state);

// Update the bias of the GPS position measurement to agree with a
// trusted position value.  This update is not immediate (see the bias_fc
// field in EstimatorPositionGpsParams).
void EstimatorPositionGpsUpdateBias(const Vec3 *Xg,
                                    const EstimatorPositionGpsEstimate *gps,
                                    const EstimatorPositionGpsParams *params,
                                    EstimatorPositionGpsState *state);

// Select between the available center GPS estimates.
WingGpsReceiverLabel EstimatorPositionGpsSelectCenterEstimate(
    FlightMode flight_mode, WingGpsReceiverLabel last_valid,
    const EstimatorPositionGpsEstimate gps_estimates[], const Vec3 *Xg,
    const EstimatorPositionGpsParams *params);

// Run one step of the GPS position estimation.
void EstimatorPositionGpsStep(
    const GpsData *gps_data, const GroundStationPoseEstimate *ground_station,
    const Mat3 *dcm_g2b, const Vec3 *pqr, const FaultMask *wing_gps_pos_fault,
    const FaultMask *wing_gps_vel_fault, const GpsParams *gps_params,
    const EstimatorPositionGpsParams *params, EstimatorPositionGpsState *state,
    EstimatorPositionGpsEstimate *gps);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // CONTROL_ESTIMATOR_ESTIMATOR_POSITION_GPS_H_

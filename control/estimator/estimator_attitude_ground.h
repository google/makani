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

#ifndef CONTROL_ESTIMATOR_ESTIMATOR_ATTITUDE_GROUND_H_
#define CONTROL_ESTIMATOR_ESTIMATOR_ATTITUDE_GROUND_H_

#include <stdbool.h>

#include "common/c_math/mat3.h"
#include "common/c_math/quaternion.h"
#include "common/c_math/vec3.h"
#include "control/control_types.h"
#include "control/estimator/estimator_attitude.h"
#include "control/estimator/estimator_types.h"
#include "control/fault_detection/fault_detection_types.h"
#include "control/sensor_types.h"
#include "control/system_types.h"

#ifdef __cplusplus
extern "C" {
#endif

// Calculate attitude, platform rate, and gyro bias estimates from IMU
// data.
//
// Applies a six-state Kalman filter to estimate attitude.
//
// Two vector measurements are used:
//   1) Accelerometer readings are compared to expected specific force
//      (calculated assuming a fixed gravitational constant).
//   2) Magnetometer readings are compared to expected magnetic field.
//
//  - Accelerometer comparisons are weighted according to how nearly
//    the accelerometer magnitude matches the physical constant g.
//
// Args:
//   initialize: When true, applies larger proportional gains to get a coarse
//       alignment and disables bias estimation.
//   imu: IMU data structures whose signals are resolved in p coordinates.
//   gps_compass: GPS compass data.
//   gps_compass_params: GPS compass parameters.
//   gps_compass_faults: GPS compass faults..
//   imu_params: IMU parameters.
//   params: Attitude estimation parameters.
//   state: State.
//   pqr: Output estimate of the platform rate estimate resolved in p
//   coordinates.
//   q_g2p: Output estimate quaternion transforming vectors in ground
//       coordinates to platform coordinates.
//   acc_p: Output estimate of the equivalent accelerometer reading that
//       would be taken at the platform coordinate origin.
//   correct: The filter corrections.
void EstimatorAttitudeGroundStep(
    bool initialize, const ImuData *imu, const GpsCompassData *gps_compass,
    const ImuParams *imu_params, const EstimatorAttitudeParams *params,
    const FaultMask *imu_faults, const GsGpsParams *gps_compass_params,
    const FaultMask *gps_compass_faults, EstimatorAttitudeState *state,
    Vec3 *pqr, Quat *q_g2p, Vec3 *acc_p, EstimatorAttitudeCorrections *correct);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // CONTROL_ESTIMATOR_ESTIMATOR_ATTITUDE_GROUND_H_

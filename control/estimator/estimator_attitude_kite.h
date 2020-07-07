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

#ifndef CONTROL_ESTIMATOR_ESTIMATOR_ATTITUDE_KITE_H_
#define CONTROL_ESTIMATOR_ESTIMATOR_ATTITUDE_KITE_H_

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

// Calculate attitude, body rate, and gyro bias estimates from IMU
// data.
//
// Applies a six-state Kalman filter to estimate attitude.
//
// Three vector measurements are used:
//   1) Accelerometer readings are compared to expected specific force
//      (calculated assuming a fixed gravitational constant).
//   2) Magnetometer readings are compared to expected magnetic field.
//   3) Apparent wind estimates (calculated from estimated velocity
//      and wind speed) are compared to expected apparent wind
//      (calculated by assuming nominal angle-of-attack and side-slip
//      angle from crosswind flight).
//
//  - Accelerometer comparisons are weighted according to how nearly
//    the accelerometer magnitude matches the physical constant g.
//
// Args:
//   initialize: When true, applies larger proportional gains to get a coarse
//       alignment and disables bias estimation.
//   imu: IMU data structures whose signals are resolved in b coordinates.
//   vel_type: Previous velocity solution type.
//   Vg_z1: Previous velocity estimate.
//   wind_g: Expected wind vector at the b coordinate origin resolved
//       in g coordinates.
//   ground_station: Ground station frame estimate.
//   mag_g: Expected magnetometer measurement [Gauss] in g-coordinates.
//   gps_center_data: GPS data from the center position.
//   gps_center_params: GPS parameters for the center position.
//   gps_center_faults: GPS position faults for the center position.
//   gps_port_data: GPS data from the port wingtip.
//   gps_port_params: GPS parameters for the port wingtip.
//   gps_port_faults: GPS position faults for the port wingtip.
//   gps_star_data: GPS data from the starboard wingtip.
//   gps_star_params: GPS parameters for the starboard wingtip.
//   gps_star_faults: GPS position faults for the starboard wingtip.
//   flight_mode: Current flight mode.
//   imu_params: IMU parameters.
//   params: Attitude estimation parameters.
//   state: State.
//   pqr: Output estimate of the body rate estimate resolved in b coordinates.
//   q_g2b: Output estimate quaternion transforming vectors in ground
//       coordinates to body coordinates.
//   acc_b: Output estimate of the equivalent accelerometer reading that
//       would be taken at the body coordinate origin.
//   correct: The filter corrections.
void EstimatorAttitudeKiteStep(
    bool initialize, const ImuData *imu, EstimatorVelocitySolutionType vel_type,
    const Vec3 *Vg_z1, const WindEstimate *wind_g,
    const GroundStationPoseEstimate *ground_station, const Vec3 *mag_g,
    const GpsData *gps_center_data, const GpsParams *gps_center_params,
    const FaultMask *gps_center_faults, const GpsData *gps_port_data,
    const GpsParams *gps_port_params, const FaultMask *gps_port_faults,
    const GpsData *gps_star_data, const GpsParams *gps_star_params,
    const FaultMask *gps_star_faults, FlightMode flight_mode,
    const ImuParams *imu_params, const EstimatorAttitudeParams *params,
    const FaultMask *imu_faults, EstimatorAttitudeState *state, Vec3 *pqr,
    Quat *q_g2b, Vec3 *acc_b, EstimatorAttitudeCorrections *correct);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // CONTROL_ESTIMATOR_ESTIMATOR_ATTITUDE_KITE_H_

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

#ifndef CONTROL_ESTIMATOR_ESTIMATOR_NAV_KITE_H_
#define CONTROL_ESTIMATOR_ESTIMATOR_NAV_KITE_H_

#include <stdbool.h>

#include "common/c_math/mat3.h"
#include "common/c_math/vec3.h"
#include "control/control_types.h"
#include "control/estimator/estimator_nav.h"
#include "control/estimator/estimator_types.h"
#include "control/fault_detection/fault_detection_types.h"
#include "control/sensor_types.h"
#include "control/system_types.h"

#ifdef __cplusplus
extern "C" {
#endif

// Initialize the kite navigation estimator state.
void EstimatorNavKiteInit(const EstimatorNavParams *params, double g_heading,
                          EstimatorNavKiteState *state);

// Run the inertial navigation portion of the kite estimator.
//
// Redundant IMUs are handled by applying an independent filter to
// each set of IMU data, then selecting among the outputs by applying
// median voting.
void EstimatorNavKiteStep(
    bool initializing, const ImuData imus[], const GpsData wing_gps[],
    const GsGpsData *gs_gps, const PitotData pitots[],
    const GroundStationPoseEstimate *ground_station, const WindEstimate *wind_g,
    const EncodersEstimate *encoders, const PerchAziEstimate *perch_azi,
    const TetherForceEstimate *tether_force_b, const WinchEstimate *winch,
    FlightMode flight_mode, const FaultMask faults[],
    const SystemParams *system_params, const EstimatorNavParams *params,
    EstimatorNavKiteState *state, Vec3 *pqr, Mat3 *dcm_g2b, Vec3 *Ag,
    bool *gps_active, Vec3 *Vg, Vec3 *Xg, Vec3 *pqr_f, Vec3 *Ab_f, Vec3 *acc_b,
    double *acc_norm_f, Vec3 *Vb, Vec3 *Vb_f, Vec3 *Vg_f);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // CONTROL_ESTIMATOR_ESTIMATOR_NAV_KITE_H_

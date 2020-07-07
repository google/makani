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

#ifndef CONTROL_ESTIMATOR_ESTIMATOR_GROUND_STATION_H_
#define CONTROL_ESTIMATOR_ESTIMATOR_GROUND_STATION_H_

#include <stdbool.h>

#include "avionics/common/avionics_messages.h"
#include "control/estimator/estimator_types.h"
#include "control/fault_detection/fault_detection_types.h"

#ifdef __cplusplus
extern "C" {
#endif

// Initialize ground station estimate.
void EstimatorGroundStationInit(EstimatorGroundStationState *state);

// Estimate the ground station's ECEF position and heading.
//
// When run_filters is true, an estimate ground station's ECEF position
// is calculated from the GS GPS's reported location and the known
// vertical offset from the GS origin to the GPS antenna's location.
// That value is incorporated into an estimate of the GS ECEF position
// by a simple low pass filter.
//
// When run_filter is false, this value is fixed.
//
// The ground station heading is determined as a fixed parameter.
//
// Args:
//   gs_gps: Current GPS data reported by the ground station.
//   gs_gps_fault: Fault code for the GS GPS.
//   ground_frame_params: Ground frame parameters.
//   buoy_params: Buoy parameters.
//   state: State.
//   ground_station: Output estimate.
void EstimatorGroundStationStep(
    const GsGpsData *gs_gps, const FaultMask *gs_gps_fault,
    GroundStationMode gs_mode, const FaultMask *ground_station_faults,
    double detwist_angle, const FaultMask *detwist_fault,
    uint8_t gs_transform_stage, const GroundFrameParams *ground_frame_params,
    const EstimatorGroundStationParams *est_params,
    EstimatorGroundStationState *state, GroundStationEstimate *ground_station);

// Initialize the vessel estimate state.
void EstimatorVesselInit(EstimatorVesselState *state);

// Populate the estimate of the vessel rigid body states.
//
// This is mainly a pass-through using the estimate from the ground
// estimator. We may also want to add a simplified, kite-based
// fallback estimator here.
void EstimatorVesselStep(const GroundEstimateMessage *ground_estimate,
                         const PerchAziEstimate *platform_azi,
                         const FaultMask *ground_estimator_fault,
                         EstimatorVesselState *state, VesselEstimate *vessel);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // CONTROL_ESTIMATOR_ESTIMATOR_GROUND_STATION_H_

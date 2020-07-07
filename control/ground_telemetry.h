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

#ifndef CONTROL_GROUND_TELEMETRY_H_
#define CONTROL_GROUND_TELEMETRY_H_

#include "avionics/common/avionics_messages.h"
#include "control/control_types.h"
#include "control/estimator/estimator_types.h"
#include "system/labels.h"

typedef struct {
  int32_t init_state;
  int32_t initializing;
  EstimatorPositionCorrections position_corrections;
  EstimatorAttitudeCorrections attitude_corrections;
  EstimatorPositionGpsEstimate gps;
  Vec3 cov_vel_g;
  Vec3 cov_pos_g;
  Vec3 gyro_biases;
  Vec3 cov_attitude_err;
  Vec3 cov_gyro_bias;
  // Vec3 acc_b_estimates[kNumWingImus];
} GroundEstimatorTelemetry;

typedef struct {
  GroundEstimatorTelemetry estimator;
  GroundEstimatorInput input;
  GroundEstimateMessage estimate;
} GroundTelemetry;

#ifdef __cplusplus
extern "C" {
#endif

GroundTelemetry *GetGroundTelemetryMessage(void);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // CONTROL_GROUND_TELEMETRY_H_

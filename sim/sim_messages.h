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

#ifndef SIM_SIM_MESSAGES_H_
#define SIM_SIM_MESSAGES_H_

#include <stdint.h>

#include "common/c_math/force_moment.h"
#include "control/avionics/avionics_interface_types.h"
#include "control/control_telemetry.h"
#include "control/control_types.h"
#include "control/estimator/estimator_types.h"
#include "control/system_types.h"
#include "system/labels.h"

#ifdef __cplusplus
extern "C" {
#endif

extern const uint32_t kControllerHandshakeSignals[kNumControllers];
extern const uint32_t kGroundStationEstimatorHandshakeSignal;
extern const uint32_t kSimHandshakeSignal;

typedef enum {
  kSimRecordStateCommandDont,
  kSimRecordStateCommandOverwrite,
  kSimRecordStateCommandLoad
} SimRecordStateCommand;

typedef struct {
  uint8_t record_mode;
  uint32_t handshake;
  bool stop;
} SimCommandMessage;

typedef struct {
  HitlConfiguration hitl_config;
  ControlInputMessages control_input_messages;
  ControlInputMessagesUpdated control_input_messages_updated;
  GroundEstimatorInputMessages ground_input_messages;
  GroundEstimatorInputMessagesUpdated ground_input_messages_updated;
} SimSensorMessage;

typedef struct {
  TetherDownMessage messages[kNumTetherDownSources];
  bool updated[kNumTetherDownSources];
} SimTetherDownMessage;

typedef struct {
  WingParams wing_params;
  ControlInput control_input;
  StateEstimate state_est;
  double time;
  int32_t flight_mode;

  ForceMoment fm_aero_b;
  ForceMoment fm_rotors_b;
  ForceMoment fm_tether_b;
  ForceMoment fm_gravity_b;
  ForceMoment fm_inertial_b;
  ForceMoment fm_error_b;
  ForceMoment fm_blown_wing_b;

  Mat3 dcm_b2w;
} DynamicsReplayMessage;

typedef struct {
  double time;
  ControlInput control_input;
  FlightMode flight_mode;
  EstimatorTelemetry estimator_telemetry;
  EstimatorState estimator_state;
  StateEstimate state_est;
  Vec3 hover_angles;
} EstimatorReplayMessage;

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // SIM_SIM_MESSAGES_H_

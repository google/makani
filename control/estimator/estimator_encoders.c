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

#include "control/estimator/estimator_encoders.h"

#include <assert.h>
#include <string.h>

#include "control/estimator/estimator_types.h"
#include "control/fault_detection/fault_detection_types.h"
#include "control/sensor_types.h"

void EstimatorEncodersInit(EstimatorEncodersState *state) {
  assert(state != NULL);
  memset(state, 0, sizeof(*state));
  state->last_valid_gsg.azi = 0.0;
  state->last_valid_gsg.ele = 0.0;
  state->last_valid_levelwind_ele = 0.0;
  state->last_valid_perch_azi = 0.0;
}

static void SelectUnfaultedEncoder(double value_a, const FaultMask *fault_a,
                                   double value_b, const FaultMask *fault_b,
                                   bool *valid, double *last_valid_value) {
  // TODO: Set encoders to be invalid if neither has a fault, but they
  // disagree by more than a threshold amount.
  if (!HasAnyFault(fault_a)) {
    *valid = true;
    *last_valid_value = value_a;
  } else if (!HasAnyFault(fault_b)) {
    *valid = true;
    *last_valid_value = value_b;
  } else {
    *valid = false;
  }
}

void EstimatorEncodersStep(
    const GsgData gsg[], const FaultMask *gsg_a_faults,
    const FaultMask *gsg_b_faults, const double levelwind_ele[],
    const FaultMask *levelwind_ele_a_fault,
    const FaultMask *levelwind_ele_b_fault, const double perch_azi[],
    const FaultMask *perch_azi_a_fault, const FaultMask *perch_azi_b_fault,
    EstimatorEncodersState *state, EncodersEstimate *encoders) {
  SelectUnfaultedEncoder(
      gsg[kDrumSensorsA].azi, &gsg_a_faults[kFaultDetectionGsgSignalAzi],
      gsg[kDrumSensorsB].azi, &gsg_b_faults[kFaultDetectionGsgSignalAzi],
      &encoders->gsg_azi_valid, &state->last_valid_gsg.azi);
  encoders->gsg.azi = state->last_valid_gsg.azi;

  SelectUnfaultedEncoder(
      gsg[kDrumSensorsA].ele, &gsg_a_faults[kFaultDetectionGsgSignalEle],
      gsg[kDrumSensorsB].ele, &gsg_b_faults[kFaultDetectionGsgSignalEle],
      &encoders->gsg_ele_valid, &state->last_valid_gsg.ele);
  encoders->gsg.ele = state->last_valid_gsg.ele;

  SelectUnfaultedEncoder(
      levelwind_ele[kPlatformSensorsA], levelwind_ele_a_fault,
      levelwind_ele[kPlatformSensorsB], levelwind_ele_b_fault,
      &encoders->levelwind_ele_valid, &state->last_valid_levelwind_ele);
  encoders->levelwind_ele = state->last_valid_levelwind_ele;

  SelectUnfaultedEncoder(perch_azi[kPlatformSensorsA], perch_azi_a_fault,
                         perch_azi[kPlatformSensorsB], perch_azi_b_fault,
                         &encoders->perch_azi_valid,
                         &state->last_valid_perch_azi);
  encoders->perch_azi = state->last_valid_perch_azi;
}

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

#ifndef CONTROL_ACTUATOR_TYPES_H_
#define CONTROL_ACTUATOR_TYPES_H_

#include "avionics/network/aio_labels.h"
#include "common/c_math/vec3.h"
#include "common/macros.h"
#include "control/simple_aero_types.h"

typedef enum {
  kStackingStateForceSigned = -1,
  kStackingStateNormal,
  kStackingStateFaultBlock1,
  kStackingStateFaultBlock2,
  kStackingStateFaultBlock3,
  kStackingStateFaultBlock4,
  kNumStackingStates
} StackingState;

typedef struct {
  double aileron;
  double inboard_flap;
  double midboard_flap;
  double outboard_flap;
  double elevator;
  double rudder;
} Deltas;

#define NUM_DELTAS 6
COMPILE_ASSERT((sizeof(Deltas) / sizeof(double) == NUM_DELTAS),
               NUM_DELTAS_consistency_check);

typedef struct {
  double thrust;
  Vec3 moment;
} ThrustMoment;

#define FREESTREAM_VEL_TABLE_LENGTH 5
typedef struct {
  double idle_speed;
  double max_speeds[kNumMotors];
  double max_torque_command;
  double min_torque_command;
  double min_aero_power;
  bool penalize_symmetric_torsion_mode;
  double regularization_weight;
  double symmetric_torsion_weight;
  double thrust_moment_to_thrusts[kNumMotors][4];
  double thrusts_to_thrust_moment[4][kNumMotors];
  double comm_and_diff_thrusts_to_thrusts[kNumMotors][5];
  double comm_and_diff_thrusts_to_thrust_moment[kNumStackingStates][4][5];
  double constraint_matrix[kNumStackingStates][kNumMotors + 1][5];
  double freestream_vel_table[FREESTREAM_VEL_TABLE_LENGTH];
  double max_thrusts[2][kNumMotors][FREESTREAM_VEL_TABLE_LENGTH];
  double motor_mount_thrust_limit[2];
  double total_power_limit_thrusts[FREESTREAM_VEL_TABLE_LENGTH];
  SimpleRotorModelParams simple_models[kNumMotors];
} RotorControlParams;

#endif  // CONTROL_ACTUATOR_TYPES_H_

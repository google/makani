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

#ifndef CONTROL_MANUAL_MANUAL_TYPES_H_
#define CONTROL_MANUAL_MANUAL_TYPES_H_

#include <stdbool.h>

#include "common/c_math/mat3.h"
#include "control/actuator_types.h"
#include "control/system_types.h"

typedef struct {
  double pitch_angle;
  double roll_angle;
  double angle_of_attack;
  PidParams pitch_pid;
  PidParams roll_pid;
  double yaw_rate_gain;
  double roll_crossfeed_gain;
} ManualAutoGlideParams;

typedef struct {
  ThrustMoment thrust_moment_weights;
  double flap_offsets[kNumFlaps];
  double lower_flap_limits[kNumFlaps];
  double upper_flap_limits[kNumFlaps];
  double kes_delay;
  double kes_torque;
} ManualOutputParams;

typedef struct {
  Deltas joystick_flap_gains;
  ThrustMoment joystick_motor_gains;
  ManualAutoGlideParams auto_glide;
  ManualOutputParams output;
} ManualParams;

typedef struct { bool release_latched; } ManualState;

#endif  // CONTROL_MANUAL_MANUAL_TYPES_H_

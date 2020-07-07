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

#ifndef AVIONICS_MOTOR_FIRMWARE_ANGLE_MEAS_H_
#define AVIONICS_MOTOR_FIRMWARE_ANGLE_MEAS_H_

#include <stdbool.h>
#include <stdint.h>

typedef struct {
  float cos_offset;
  float sin_offset;
  float sin_scale;
  float angle_meas_lower_warn_lim;
  float angle_meas_upper_warn_lim;
} MotorAngleMeasParam;

typedef struct {
  float sin_m;
  float cos_m;
  bool updated;
  bool valid;
} MotorAngleMeas;

// Initialize communication with the angle sensor and read the current state.
// Note that this function is blocking and is guaranteed to update all fields in
// meas; meas->updated is always true after the call.
void MotorAngleMeasInit(MotorAngleMeas *meas);

// Read the current state of the angle sensor. If information is not available
// when called, meas->updated will be set to false and all other fields will be
// zeroed.
void MotorAngleMeasGet(MotorAngleMeas *meas);

extern MotorAngleMeasParam g_motor_angle_meas_param;

#endif  // AVIONICS_MOTOR_FIRMWARE_ANGLE_MEAS_H_

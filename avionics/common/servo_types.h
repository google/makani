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

#ifndef AVIONICS_COMMON_SERVO_TYPES_H_
#define AVIONICS_COMMON_SERVO_TYPES_H_

#include <stdbool.h>
#include <stdint.h>

#include "avionics/common/actuator_types.h"
#include "avionics/common/faults.h"

typedef enum {
  kServoStatusPaired      = (1 << 0),
  kServoStatusCommanded   = (1 << 1),
  kServoStatusArmed       = (1 << 2),
  kServoStatusReset       = (1 << 3),
  kServoStatusPairSynced  = (1 << 4),
  kServoStatusOutputClamp = (1 << 5),
} ServoStatusFlag;

typedef enum {
  kServoWarningPairTimeout      = (1 << 0),
  kServoWarningPairFailed       = (1 << 1),
  kServoWarningOutputClampStuck = (1 << 2),
  kServoWarningR22              = (1 << 3),
  kServoWarningScuttle          = (1 << 4),
  kServoWarningR22Reinitialized = (1 << 5),
} ServoWarningFlag;

typedef enum {
  kServoErrorMotorFailure    = (1 << 0),
  kServoErrorResolverFailure = (1 << 1),
  kServoErrorHallFailure     = (1 << 2),
  kServoErrorR22Comm         = (1 << 3),
  kServoErrorR22Fault        = (1 << 4),
  kServoErrorR22             = (1 << 5),
  kServoErrorR22Temperature  = (1 << 6),
  kServoErrorR22OverVoltage  = (1 << 7),
} ServoErrorFlag;

typedef struct {
  int32_t raw;
  float value;
  int32_t repeated;
  int64_t timestamp;
} ServoMeasurement;

typedef enum {
  kServoModePositionCommand,
  kServoModeVelocityCommand,
  kServoModeTorqueCommand,
} ServoMode;

typedef struct {
  int8_t valid;
  ServoMode mode;
  float desired_angle;
  float desired_velocity;
  float desired_torque;
} ServoControllerCommand;

typedef struct {
  uint32_t r22_status_bits;
  float angle;
  uint16_t angle_raw;
  ServoMeasurement velocity;
  ServoMeasurement current;
  int16_t temperature;
} ServoR22Input;

typedef struct {
  ServoControllerCommand cmd;
  ServoR22Input r22;
  uint8_t controllers_used;  // See ControllerBitFlag in avionics_messages.h
  bool tether_released;
  bool scuttle_command;
} ServoInputState;

typedef struct {
  // TODO: Implement control which does not require init flags.
  int8_t init_estimate;
  int8_t init_alignment;
  int64_t pair_timeout;
  int16_t jitter;
  StatusFlags flags;
  float angle_bias;
  float angle_feedback;
  float angle_estimate;
  float angle_variance;
  float velocity_prev;
  float desired_angle;
  float current_limit;
  bool valid;
} ServoControlState;

#endif  // AVIONICS_COMMON_SERVO_TYPES_H_

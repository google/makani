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

#ifndef AVIONICS_COMMON_WINCH_MESSAGES_H_
#define AVIONICS_COMMON_WINCH_MESSAGES_H_

#include <stdint.h>

#include "avionics/common/faults.h"
#include "system/labels.h"

// This file specifies message types communicated between the winch PLC and
// core switch. It does not specify AIO message types. Updating this file
// means updating the winch PLC.

// Communicated from core switch to winch PLC (not an AIO message).
typedef struct {
  uint8_t source;     // Flight controller number.
  uint16_t sequence;  // AIO sequence number.
  float velocity;     // Commanded winch velocity [rad/s].
} PlcWinchCommandMessage;

typedef struct {
  double position;    // [rad]
  float velocity;     // [rad/s]
  float torque;       // [N-m]
  StatusFlags flags;  // TODO: define.
} WinchDrumStatus;

typedef struct {
  double position;    // [m]
  float velocity;     // [m/s]
  float torque;       // [N-m]
  StatusFlags flags;  // TODO: define.
} WinchLevelwindStatus;

typedef enum {
  kWinchProximityEarlyA = (1 << kProximitySensorEarlyA),
  kWinchProximityEarlyB = (1 << kProximitySensorEarlyB),
  kWinchProximityFinalA = (1 << kProximitySensorFinalA),
  kWinchProximityFinalB = (1 << kProximitySensorFinalB)
} WinchProximityFlag;

typedef struct {
  uint16_t sequence;       // AIO sequence number.
  uint32_t state_command;  // See ActuatorStateCommand.
  uint32_t arming_signal;  // Arming/disarm signal, if applicable.
} PlcWinchSetStateMessage;

// Communicated from winch PLC to core switch (not an AIO message).
typedef struct {
  uint16_t sequence;     // AIO sequence number.
  WinchLevelwindStatus levelwind;
  WinchDrumStatus winch_drum;
  double drum_position;  // [rad]
  StatusFlags flags;     // TODO: define.
  uint16_t state;        // See ActuatorState.
  uint16_t proximity;    // See WinchProximitySensorFlags.
} PlcWinchStatusMessage;

#endif  // AVIONICS_COMMON_WINCH_MESSAGES_H_

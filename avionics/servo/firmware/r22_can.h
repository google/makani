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

#ifndef AVIONICS_SERVO_FIRMWARE_R22_CAN_H_
#define AVIONICS_SERVO_FIRMWARE_R22_CAN_H_

#include <stdbool.h>
#include <stdint.h>

#include "avionics/firmware/drivers/canopen_sdo.h"
#include "avionics/servo/firmware/r22_param.h"

// See Copley CANopen Programmer's Manual - Control Word (0x6040) pg. 58.
typedef enum {
  kControlWordDisable = 0,
  kControlWordSwitchOn = 1 << 0,
  kControlWordEnableVoltage = 1 << 1,
  kControlWordQuickStopN = 1 << 2,
  kControlWordEnableOperation = 1 << 3,
  kControlWordResetFault = 1 << 7,
  kControlWordHalt = 1 << 8,

  // Specific to profile position mode (pg. 194).
  kControlWordProfilePositionNewSetpoint = 1 << 4,
  kControlWordProfilePositionChangeSetImmediately = 1 << 5,
  kControlWordProfilePositionRelative = 1 << 6,
} ControlWord;

// See Copley CANopen Programmer's Manual - Mode of Operation (0x6060) pg. 64.
typedef enum {
  kModeOfOperationProfilePosition = 1,
  kModeOfOperationProfileVelocity = 3,
  kModeOfOperationProfileTorque = 4,
  kModeOfOperationHoming = 6,
  kModeOfOperationInterpolatedPosition = 7,
  kModeOfOperationCyclicSynchronousPosition = 8,
  kModeOfOperationCyclicSynchronousVelocity = 9,
  kModeOfOperationCyclicSynchronousTorque = 10,
} ModeOfOperation;

typedef struct {
  uint16_t status_word;
  uint32_t event_status;
  ModeOfOperation mode_of_operation;
  int32_t actual_position;
  int32_t actual_velocity;
  int16_t actual_current;
  int16_t actual_torque;
  int16_t temperature;
} ServoReceiveParams;

typedef struct {
  uint16_t control_word;
  ModeOfOperation mode_of_operation;
  int32_t target_position;
  int32_t target_velocity;
  int32_t target_torque;
  uint16_t current_limit;
} ServoTransmitParams;

void R22CanInit(void);
void R22CanPdoInitPoll(void);
bool R22CanPdoConfigured(void);
void R22CanPoll(void);
bool R22CanGetValue(R22Parameter param);
bool R22CanIsIdle(void);
bool R22CanHadError(void);
bool R22CanReadUint8(int32_t data_length, uint8_t *data);
bool R22CanReadInt8(int32_t data_length, int8_t *data);
bool R22CanReadUint16(int32_t data_length, uint16_t *data);
bool R22CanReadInt16(int32_t data_length, int16_t *data);
bool R22CanReadUint32(int32_t data_length, uint32_t *data);
bool R22CanReadInt32(int32_t data_length, int32_t *data);
bool R22CanCompare(int32_t data_length, const void *data);
void R22CanSetUint8Be(R22Parameter param, int32_t in_length, const uint8_t *in);
void R22CanSetUint8Le(R22Parameter param, int32_t in_length, const uint8_t *in);
void R22CanSetUint8(R22Parameter param, uint8_t value);
void R22CanSetInt8(R22Parameter param, int8_t value);
void R22CanSetUint16(R22Parameter param, uint16_t value);
void R22CanSetInt16(R22Parameter param, int16_t value);
void R22CanSetUint32(R22Parameter param, uint32_t value);
void R22CanSetInt32(R22Parameter param, int32_t value);
bool R22CanUpdateRxParams1(ServoReceiveParams *params);
bool R22CanUpdateRxParams2(ServoReceiveParams *params);
bool R22CanUpdateRxParams3(ServoReceiveParams *params);
bool R22CanUpdateTxParams1(const ServoTransmitParams *params);
bool R22CanUpdateTxParams2(const ServoTransmitParams *params);
bool R22CanUpdateTxParams3(const ServoTransmitParams *params);
void R22CanCommandReset(void);
void R22CanCommandPreOperational(void);
void R22CanCommandOperational(void);
void R22CanSendSync(void);
void R22CanClearErrorLog(void);
void R22CanClearErrors(void);
int64_t R22CanReadTimestamp(void);
CanopenSdoAbort R22CanGetAbort(void);

#endif  // AVIONICS_SERVO_FIRMWARE_R22_CAN_H_

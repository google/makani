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

#ifndef AVIONICS_SERVO_FIRMWARE_R22_SERIAL_H_
#define AVIONICS_SERVO_FIRMWARE_R22_SERIAL_H_

#include <stdbool.h>
#include <stdint.h>

#include "avionics/servo/firmware/r22_param.h"
#include "avionics/servo/firmware/r22_types.h"

// See Copley Control's Binary Serial Interface v1.2 document (ref AN112).
// http://www.copleycontrols.com/Motion/pdf/Binary-Serial.pdf

// Write no operation command (useful to verify communications).
void R22SerialWriteNoOperation(void);

// Retrieve operating mode (useful to verify boot/normal mode during
// initialization).
void R22SerialWriteRetrieveOperatingMode(void);

// Swap operating modes (useful to switch to normal mode during initialization).
void R22SerialWriteSwapOperatingModes(void);

// Get variable value (useful to read measurements and parameter values).
void R22SerialWriteGetVariableValue(R22Parameter param, R22Memory mem);

// Set variable value, various flavors (useful to write parameter values).
void R22SerialWriteSetVariableUint8(R22Parameter param, R22Memory mem,
                                    int32_t in_length, const uint8_t *in);
void R22SerialWriteSetVariableUint16(R22Parameter param, R22Memory mem,
                                     int32_t in_length, const uint16_t *in);
void R22SerialWriteSetVariableInt16(R22Parameter param, R22Memory mem,
                                    int32_t in_length, const int16_t *in);
void R22SerialWriteSetVariableUint32(R22Parameter param, R22Memory mem,
                                     int32_t in_length, const uint32_t *in);
void R22SerialWriteSetVariableInt32(R22Parameter param, R22Memory mem,
                                    int32_t in_length, const int32_t *in);

// Software reset.
void R22SerialWriteReset(void);

// Set trajectory mode.
void R22SerialWriteTrajectory(R22TrajectorySubcommand traj);

// Issue error log command.
void R22SerialWriteErrorLog(R22ErrorLogSubcommand subcmd, uint16_t param);

// Get error code of last transfer.
R22Error R22SerialReadError(void);

// Get timestamp of last transfer.
int64_t R22SerialReadTimestamp(void);

// Check for successful completion of last transfer.
bool R22SerialReadReturnSuccess(void);

// Read data from last transfer, various flavors.
bool R22SerialReadUint8(int32_t data_length, uint8_t *data);
bool R22SerialReadInt8(int32_t data_length, int8_t *data);
bool R22SerialReadUint16(int32_t data_length, uint16_t *data);
bool R22SerialReadInt16(int32_t data_length, int16_t *data);
bool R22SerialReadUint32(int32_t data_length, uint32_t *data);
bool R22SerialReadInt32(int32_t data_length, int32_t *data);

// Compare data of last transfer to arbitrary buffer (useful to verify
// parameters without dealing with variable types).
bool R22SerialCompare(int32_t data_length, const void *data);

// Send serial data. Call until return value is true.
bool R22SerialSend(void);

// Receive serial data. Call until return value is true.
bool R22SerialReceive(void);

// Send/receive serial data. Call until return value is true.
bool R22SerialTransfer(void);

#endif  // AVIONICS_SERVO_FIRMWARE_R22_SERIAL_H_

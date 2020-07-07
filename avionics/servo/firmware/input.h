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

#ifndef AVIONICS_SERVO_FIRMWARE_INPUT_H_
#define AVIONICS_SERVO_FIRMWARE_INPUT_H_

#include <stdbool.h>
#include <stdint.h>

#include "avionics/common/servo_types.h"
#include "avionics/network/aio_node.h"
#include "avionics/servo/firmware/r22_param.h"
#include "avionics/servo/firmware/types.h"

typedef float (*ServoCalFunction)(int32_t value);

float ServoApplyAngleCal(int32_t raw);
int32_t ServoInvertAngleCal(float value);
float ServoApplyVelocityCal(int32_t raw);
int32_t ServoInvertVelocityCal(float value);
float ServoApplyCurrentCal(int32_t raw);
int32_t ServoInvertCurrentCal(float value);

AioNode ServoPairedNode(void);
bool ServoIsPaired(void);
int32_t ServoDirectionSign(void);

void ServoMeasureUpdate(ServoMeasurement *measure, ServoCalFunction fn,
                        int32_t value, int64_t timestamp);
bool ServoInputQueryPaired(ServoState *paired);
bool ServoInputQueryControllerCommand(int64_t now, ServoInputState *input);
void ServoInputInit(int64_t now);
void ServoInputQueryOperatorCommand(ServoOperatorCommand *cmd);
void ServoInputQueryLoadcellTetherRelease(bool *tether_released);
void ServoInputQueryJoystickScuttle(bool *scuttle);

#endif  // AVIONICS_SERVO_FIRMWARE_INPUT_H_

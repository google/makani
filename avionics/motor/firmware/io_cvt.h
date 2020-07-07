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

// CVT abstraction to switch between motors and motor dynos.

#ifndef AVIONICS_MOTOR_FIRMWARE_IO_CVT_H_
#define AVIONICS_MOTOR_FIRMWARE_IO_CVT_H_

#include <stdbool.h>
#include <stdint.h>

#include "avionics/common/avionics_messages.h"
#include "avionics/common/controller_arbitration.h"
#include "avionics/network/aio_node.h"

typedef struct {
  int16_t command;          // See MotorCommandFlag.
  float speed_upper_limit;  // [rad/s]
  float speed_lower_limit;  // [rad/s]
  float torque;             // [N-m]
} MotorCommand;

void CvtInit(int64_t now, int32_t motor_index);
bool CvtGetGetParamMessage(AioNode source, MotorGetParamMessage *msg,
                           uint16_t *sequence, int64_t *timestamp);
bool CvtGetSetParamMessage(AioNode source, MotorSetParamMessage *msg,
                           uint16_t *sequence, int64_t *timestamp);
bool CvtGetSetStateMessage(AioNode source, MotorSetStateMessage *msg,
                           uint16_t *sequence, int64_t *timestamp);
bool CvtGetStackingMessage(AioNode source, MotorStackingMessage *msg,
                           uint16_t *sequence, int64_t *timestamp);
bool CvtGetMotorCommand(int64_t now, MotorCommand* cmd,
                        ControllerLabel *source);

#endif  // AVIONICS_MOTOR_FIRMWARE_IO_CVT_H_

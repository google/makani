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

#include "avionics/motor/firmware/io_cvt.h"

#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "avionics/common/avionics_messages.h"
#include "avionics/common/cvt_avionics_messages.h"
#include "avionics/network/aio_node.h"

static int32_t g_motor_index = 0;

void CvtInit(int64_t now, int32_t motor_index) {
  (void)now;

  assert(0 <= motor_index && motor_index < kNumDynoMotors);
  g_motor_index = motor_index;
}

bool CvtGetGetParamMessage(AioNode source, MotorGetParamMessage *msg,
                           uint16_t *sequence, int64_t *timestamp) {
  return CvtGetDynoMotorGetParamMessage(source, msg, sequence, timestamp);
}

bool CvtGetSetParamMessage(AioNode source, MotorSetParamMessage *msg,
                           uint16_t *sequence, int64_t *timestamp) {
  return CvtGetDynoMotorSetParamMessage(source, msg, sequence, timestamp);
}

bool CvtGetSetStateMessage(AioNode source, MotorSetStateMessage *msg,
                           uint16_t *sequence, int64_t *timestamp) {
  return CvtGetDynoMotorSetStateMessage(source, msg, sequence, timestamp);
}

bool CvtGetStackingMessage(AioNode source, MotorStackingMessage *msg,
                           uint16_t *sequence, int64_t *timestamp) {
  (void)source;
  (void)msg;
  (void)sequence;
  (void)timestamp;
  return false;
}

static inline bool ValidateDynoCommandMessage(DynoCommandMessage *msg) {
  return isfinite(msg->motor_torque[g_motor_index])
      && isfinite(msg->motor_speed_upper_limit[g_motor_index])
      && isfinite(msg->motor_speed_lower_limit[g_motor_index]);
}

static inline MotorCommand* UnpackDynoCommandMessage(DynoCommandMessage *msg,
                                                     MotorCommand *cmd) {
  cmd->command = msg->motor_command;
  cmd->speed_upper_limit = msg->motor_speed_upper_limit[g_motor_index];
  cmd->speed_lower_limit = msg->motor_speed_lower_limit[g_motor_index];
  cmd->torque = msg->motor_torque[g_motor_index];
  return cmd;
}

bool CvtGetMotorCommand(int64_t now, MotorCommand* cmd,
                        ControllerLabel *source) {
  (void)now;

  static DynoCommandMessage msg;
  if ((CvtGetDynoCommandMessage(kAioNodeSimulator, &msg, NULL, NULL) &&
       ValidateDynoCommandMessage(&msg)) ||
      (CvtGetDynoCommandMessage(kAioNodeOperator, &msg, NULL, NULL) &&
       ValidateDynoCommandMessage(&msg))) {
    UnpackDynoCommandMessage(&msg, cmd);
    *source = kControllerA;
    return true;
  }
  return false;
}

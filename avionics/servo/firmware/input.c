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

#include "avionics/servo/firmware/input.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "avionics/common/actuator_types.h"
#include "avionics/common/avionics_messages.h"
#include "avionics/common/controller_arbitration.h"
#include "avionics/common/cvt.h"
#include "avionics/common/cvt_avionics_messages.h"
#include "avionics/common/fast_math/fast_math.h"
#include "avionics/common/safety_codes.h"
#include "avionics/common/servo_types.h"
#include "avionics/common/tether_convert.h"
#include "avionics/common/tether_message.h"
#include "avionics/common/tether_message_types.h"
#include "avionics/firmware/cpu/clock.h"
#include "avionics/firmware/drivers/log.h"
#include "avionics/firmware/identity/identity.h"
#include "avionics/firmware/network/net_send.h"
#include "avionics/network/aio_labels.h"
#include "avionics/network/message_type.h"
#include "avionics/servo/firmware/def.h"
#include "avionics/servo/firmware/calib_params.h"
#include "avionics/servo/firmware/config_params.h"
#include "avionics/servo/firmware/input_cvt.h"
#include "avionics/servo/firmware/r22_param.h"
#include "common/c_math/util.h"

#define CONTROLLER_TIMEOUT_CYCLES CLOCK32_MSEC_TO_CYCLES(100)
#define AUTO_SCUTTLE_TIMEOUT_MS 5000  // 5 seconds.

static ControllerArbitrationState g_controller_arbitration_state;

static const CalParams kAngleCalib = {
  // Convert encoder counts to radians.
  .scale = 2.0 * PI / SERVO_ENCODER_COUNTS,
  .bias = -PI,
  .bias_count = 0
};

static const CalParams kVelocityCalib = {
  // Convert hall effect encoder ticks to radians/second.
  // The gear ratio of the drive motor to load is 160.
  // There are 36 encoder counts per drive motor revolution.
  // Velocity output is in 0.1 encoder counts per second.
  .scale = 2.0 * PI / (10.0 * 160 * 36.0),
  .bias = 0.0,
  .bias_count = 0
};

static const CalParams kCurrentCalib = {
  .scale = 0.01,
  .bias = 0.0,
  .bias_count = 0
};

float ServoApplyAngleCal(int32_t raw) {
  // R22 may output negative values when near zero.
  raw %= SERVO_ENCODER_COUNTS;  // (-counts, counts).
  raw += SERVO_ENCODER_COUNTS;  // (0, 2*counts].
  raw %= SERVO_ENCODER_COUNTS;  // [0, counts).
  return WrapAngle(ApplyCal(raw, &kAngleCalib)
                   - kServoCalibParams->resolver_zero);
}

int32_t ServoInvertAngleCal(float value) {
  int32_t raw = InvertCal(value + kServoCalibParams->resolver_zero,
                          &kAngleCalib);
  raw %= SERVO_ENCODER_COUNTS;  // (-counts, counts).
  raw += SERVO_ENCODER_COUNTS;  // (0, 2*counts].
  raw %= SERVO_ENCODER_COUNTS;  // [0, counts).
  return raw;
}

float ServoApplyVelocityCal(int32_t raw) {
  return ApplyCal(raw, &kVelocityCalib);
}

int32_t ServoInvertVelocityCal(float value) {
  return InvertCal(value, &kVelocityCalib);
}

float ServoApplyCurrentCal(int32_t raw) {
  return ApplyCal(raw, &kCurrentCalib);
}

int32_t ServoInvertCurrentCal(float value) {
  return InvertCal(value, &kCurrentCalib);
}

AioNode ServoPairedNode(void) {
  AioNode node = AppConfigGetAioNode();
  AioNode paired;
  switch (node) {
    case kAioNodeServoE1:
      paired = kAioNodeServoE2;
      break;
    case kAioNodeServoE2:
      paired = kAioNodeServoE1;
      break;
    case kAioNodeServoR1:
      paired = kAioNodeServoR2;
      break;
    case kAioNodeServoR2:
      paired = kAioNodeServoR1;
      break;
    default:
      paired = node;
      break;
  }
  return paired;
}

bool ServoIsPaired(void) {
  return AppConfigGetAioNode() != ServoPairedNode();
}

int32_t ServoDirectionSign(void) {
  return kServoCalibParams->direction;
}

void ServoMeasureUpdate(ServoMeasurement *measure, ServoCalFunction fn,
                        int32_t value, int64_t timestamp) {
  if (measure->raw != value) {
    measure->raw = value;
    measure->repeated = 0;
  } else {
    ++measure->repeated;
  }
  measure->value = fn(value);
  measure->timestamp = timestamp;
}

static bool CvtGetServoPairedStatusMessage(AioNode node,
                                           ServoPairedStatusMessage *msg,
                                           uint16_t *seq,
                                           int64_t *timestamp) {
  switch (node) {
    case kAioNodeServoE1:
    case kAioNodeServoE2:
      return CvtGetServoPairedStatusElevatorMessage(node, msg, seq, timestamp);
    case kAioNodeServoR1:
    case kAioNodeServoR2:
      return CvtGetServoPairedStatusRudderMessage(node, msg, seq, timestamp);
    default:
      return false;
  }
}

bool ServoInputQueryPaired(ServoState *paired) {
  // Validate raw data to prevent asserts in calibration routines.
  ServoPairedStatusMessage msg;
  int64_t timestamp;
  if (CvtGetServoPairedStatusMessage(ServoPairedNode(), &msg, NULL, &timestamp)
      && isfinite(msg.input.r22.angle)
      && isfinite(msg.input.r22.velocity.value)
      && isfinite(msg.control_state.angle_estimate)
      && isfinite(msg.control_state.angle_variance)
      && -PI <= msg.input.r22.angle
      && msg.input.r22.angle <= PI
      && -PI <= msg.control_state.angle_estimate
      && msg.control_state.angle_estimate <= PI
      && msg.control_state.angle_variance > 0) {
    // Paired servo measures latency. This measurement should be less than
    // the control loop period.
    int64_t latency = msg.latency_usec;
    timestamp -= latency;
    if (0 <= latency && latency <= SERVO_CONTROL_PERIOD_US) {
      paired->input = msg.input;
      paired->control = msg.control_state;
      paired->state = msg.state;
      paired->sync_timestamp = timestamp;
      paired->updated = true;
      return true;
    }
  }
  return false;
}

static float SaturateControllerCommand(float input_angle) {
  // Apply flash param angle limits.  These should be equal to the Copley
  // software limits, but those require a homing sequence to function.  Ensure
  // that they are applied here.
  float output_angle = input_angle;
  if (output_angle < kServoConfigParams->servo_min_limit) {
    output_angle = kServoConfigParams->servo_min_limit;
  }
  if (output_angle > kServoConfigParams->servo_max_limit) {
    output_angle = kServoConfigParams->servo_max_limit;
  }

  return output_angle;
}

static bool ValidateControllerCommand(const ControllerCommandMessage *msg) {
  float angle = msg->servo_angle[AppConfigGetIndex()];
  return isfinite(angle) && -PI <= angle && angle <= PI;
}

static bool QueryControllerCommand(int64_t now, float *angle,
                                   ControllerLabel *source) {
  ControllerArbitrationUpdateFromCvt(&g_controller_arbitration_state);
  const ControllerCommandMessage *msg
      = ControllerArbitrationGetCommand(now, ValidateControllerCommand,
                                        &g_controller_arbitration_state,
                                        source);
  if (msg != NULL) {
    *angle = SaturateControllerCommand(msg->servo_angle[AppConfigGetIndex()]);
    return true;
  }
  return false;
}

bool ServoInputQueryControllerCommand(int64_t now,
                                      ServoInputState *input) {
  float angle;
  ControllerLabel source;

  static uint32_t valid_expire = 0;
  ServoControllerCommand *ref = &input->cmd;
  if (QueryControllerCommand(now, &angle, &source)) {
    valid_expire = Clock32GetCycles() + CONTROLLER_TIMEOUT_CYCLES;
    ref->mode = kServoModePositionCommand;
    ref->desired_angle = angle;
    ref->valid = true;
    input->controllers_used |= 1 << source;
  } else {
    // Disable servo after specified command silence.
    if (ref->valid && CLOCK32_GE(Clock32GetCycles(), valid_expire)) {
      LOG_PRINTF("Controller timeout.\n");
      ref->valid = false;
    }
  }
  return ref->valid;
}

static bool ServoSelected(uint16_t selected) {
  return (selected & (1 << AppConfigGetIndex())) != 0;
}

static bool ArmingSignalValid(const ServoSetStateMessage *msg) {
  if (msg->state_command == kActuatorStateCommandArm) {
    return msg->servo_arming_signal == SERVO_ARMING_SIGNAL;
  } else if (msg->state_command == kActuatorStateCommandDisarm) {
    return msg->servo_arming_signal == SERVO_DISARM_SIGNAL;
  } else {
    return true;
  }
}

static ActuatorStateCommand QuerySetStateCommand(AioNode fc) {
  ServoSetStateMessage msg;
  if (CvtGetSetStateMessage(fc, &msg, NULL, NULL)
      && ArmingSignalValid(&msg)
      && ServoSelected(msg.selected_servos)) {
    return msg.state_command;
  }
  return kActuatorStateCommandNone;
}

static void ServoInputQuerySetStateCommand(ServoOperatorCommand *cmd) {
  cmd->clear_errors = false;
  ActuatorStateCommand state_cmd = QuerySetStateCommand(kAioNodeOperator);
  switch (state_cmd) {
    case kActuatorStateCommandArm:
      if (!cmd->armed) {
        LOG_PRINTF("Received arm signal.\n");
      }
      cmd->armed = true;
      break;
    case kActuatorStateCommandDisarm:
      if (cmd->armed) {
        LOG_PRINTF("Received disarm signal.\n");
      }
      cmd->armed = false;
      break;
    case kActuatorStateCommandClearErrors:
      LOG_PRINTF("Received clear errors command.\n");
      cmd->clear_errors = true;
      break;
    default:
      break;
  }
}

static void ServoInputQueryClearErrorLogCommand(ServoOperatorCommand *cmd) {
  ServoClearErrorLogMessage msg;
  cmd->clear_error_log = false;
  if (CvtGetClearErrorLogMessage(kAioNodeOperator, &msg, NULL, NULL)
      && ServoSelected(msg.selected_servos)) {
    cmd->clear_error_log = true;
  }
}

static void HandleGetParam(AioNode source, ServoOperatorCommand *cmd) {
  ServoGetParamMessage message;
  if (CvtGetGetParamMessage(source, &message, NULL, NULL)
      && ServoSelected(message.selected_servos)) {
    R22Parameter param = (R22Parameter) message.param;
    if (message.param < 0) {
      // The user specified an invalid param ID.
      LOG_PRINTF("Unable to get param ID %d.\n", param);
      return;
    }
    cmd->get_parameter = true;
    cmd->set_parameter = false;
    cmd->parameter_type = param;
  }
}

static void HandleSetParam(AioNode source, ServoOperatorCommand *cmd) {
  ServoSetParamMessage message;
  if (CvtGetSetParamMessage(source, &message, NULL, NULL)
      && ServoSelected(message.selected_servos)) {
    LOG_PRINTF("Received set param.\n");
    R22Parameter param = (R22Parameter)message.param;
    const R22ParamInfo *info = R22ParamGetInfo(param);
    if (!info || !info->can_index) {
      // The user specified a param which is not set through CAN.
      LOG_PRINTF("Unable to set param ID %d.\n", param);
      return;
    }
    // Store the parameter in RAM so it isn't reverted.
    R22ParamSetValue(param, message.value);
    cmd->get_parameter = false;
    cmd->set_parameter = true;
    cmd->set_parameter_value = message.value;
    cmd->parameter_type = param;
  }
}

static void ServoInputQueryParamCommand(ServoOperatorCommand *cmd) {
  HandleGetParam(kAioNodeOperator, cmd);
  HandleSetParam(kAioNodeOperator, cmd);
}

void ServoInputInit(int64_t now) {
  ControllerArbitrationInit(now, &g_controller_arbitration_state);
}

void ServoInputQueryOperatorCommand(ServoOperatorCommand *cmd) {
  ServoInputQuerySetStateCommand(cmd);
  ServoInputQueryParamCommand(cmd);
  ServoInputQueryClearErrorLogCommand(cmd);
}

// The tether released flag will latch on once a loadcell has indicated firing
// the release.
void ServoInputQueryLoadcellTetherRelease(bool *tether_released) {
  LoadcellMessage loadcell_message;
  for (LoadcellNodeLabel i = kLoadcellNodePortA; i < kNumLoadcellNodes; i++) {
    if (CvtGetLoadcellMessage(LoadcellNodeLabelToLoadcellNodeAioNode(i),
                              &loadcell_message, NULL, NULL)) {
      if (loadcell_message.tether_released) {
        *tether_released = true;
      }
    }
  }
}

void ServoInputQueryJoystickScuttle(bool *scuttle) {
  static TetherUpMergeState merge_state;
  static bool initialized = false;

  if (!initialized) {
    initialized = true;
    TetherUpMergeStateInit(&merge_state);
  }
  const TetherUpMessage *out = TetherUpMergeCvtGet(&merge_state);

  int32_t age =
      TetherNoUpdateCountToMilliseconds(out->joystick.no_update_count);
  if (0 <= age && age < AUTO_SCUTTLE_TIMEOUT_MS) {
    *scuttle = (out->joystick.scuttle_code == SCUTTLE_SAFETY_CODE);
  } else {
    *scuttle = true;
  }
}

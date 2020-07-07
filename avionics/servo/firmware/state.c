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

#include "avionics/servo/firmware/state.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "avionics/common/avionics_messages.h"
#include "avionics/common/endian.h"
#include "avionics/common/faults.h"
#include "avionics/firmware/cpu/clock.h"
#include "avionics/firmware/cpu/sci.h"
#include "avionics/firmware/drivers/led.h"
#include "avionics/firmware/identity/identity.h"
#include "avionics/firmware/network/net_send.h"
#include "avionics/firmware/util/timer.h"
#include "avionics/servo/firmware/control.h"
#include "avionics/servo/firmware/def.h"
#include "avionics/servo/firmware/init.h"
#include "avionics/servo/firmware/input.h"
#include "avionics/servo/firmware/output.h"
#include "avionics/servo/firmware/r22.h"
#include "avionics/servo/firmware/r22_can.h"
#include "avionics/servo/firmware/r22_def.h"
#include "avionics/servo/firmware/r22_param.h"
#include "avionics/servo/firmware/r22_serial.h"
#include "common/c_math/util.h"

#define SERVO_FAILURE_LIMIT        5
#define SERVO_GET_EXTRA_DECIMATION 100
#define SERVO_MAX_JITTER_US        500
#define SERVO_TPDO_TIMEOUT_US      800

typedef enum {
  kServoIdle,
  kServoR22Read,
  kServoSendState,
  kServoGetPairedState,
  kServoRunController,
  kServoR22Write,
  kServoCompleteTask,
  kServoCheckParameter,
  kServoSetParameter,
  kServoSetParameterCommand,
  kServoGetParameterCommand,
  kServoClearErrorLog,
  kServoClearErrors,
  kServoInit
} ServoProcessState;

typedef void (*ServoTransferCompleteFunction)(void);

static struct {
  ServoProcessState state;
  int64_t timeout;
  int32_t sign;
  int16_t num_failures;
  bool first_entry;
  bool clear_entry;
  bool set_parameter;
  int32_t get_extra_timer;
  int64_t frame_timeout;
  uint8_t read_index;
  bool reset_status;
  bool reinitialized;
  Timer clamp_timer;
} g_servo;

static struct {
  R22Parameter param;
  const R22ParamInfo *info;
  R22Memory mem;
  int32_t num_tries;
  const uint8_t *data;
} g_param;

static struct {
  uint32_t get_parameter;
  uint32_t pdo_read;
  uint32_t pdo_write;
  uint32_t set_parameter;
  uint32_t paired_state;
  uint32_t clear_error_log;
  uint32_t clear_errors;
  uint32_t loop_timing;
} g_bit;

static ServoState g_self;
static ServoState g_paired;
static ServoOperatorCommand g_operator_command;

static ServoTransmitParams g_r22_write;
static ServoReceiveParams g_r22_read;

// See generate_r22_param.py for configuration.
static void PatchConfiguration(AioNode node) {
  switch (node) {
    case kAioNodeServoA1:
    case kAioNodeServoA8:
      R22ParamApplyPatchAileron1();
      break;
    case kAioNodeServoA2:
    case kAioNodeServoA7:
      R22ParamApplyPatchAileron2();
      break;
    case kAioNodeServoA4:
    case kAioNodeServoA5:
      R22ParamApplyPatchAileron4();
      break;
    case kAioNodeServoE1:
    case kAioNodeServoE2:
      R22ParamApplyPatchElevator();
      break;
    case kAioNodeServoR1:
    case kAioNodeServoR2:
      R22ParamApplyPatchRudder();
      break;
    default:
      break;
  }
}

static void ServoControlHeartbeat(void) {
  // Heartbeat.
  static uint32_t count = 0U;
  ++count;
  LedSet((count & 0x80), (count & 0x1));
}

// Read function for ServoCheckParameter state.
static void ServoCheckParameterRead(void) {
  g_servo.set_parameter = !R22CanHadError() &&
      !R22CanCompare(g_param.info->addr_length, g_param.data);
}

// Read function for ServoGetParameterCommand state.
static void ServoGetParameterRead(void) {
  if (R22CanHadError()) {
    return;
  }
  const R22ParamInfo *info = R22ParamGetInfo(g_operator_command.parameter_type);
  if (!info) {
    assert(false);
    return;
  }
  int32_t length = info->addr_length;

  ServoAckParamMessage ack_param;
  ack_param.param = (uint16_t)g_operator_command.parameter_type;

  assert(0 < length && length <= 4);
  uint8_t data[4];
  uint8_t v8;
  uint16_t v16;
  uint32_t v32;
  R22CanReadUint8(length, data);
  switch (length) {
    case 1:
      ReadUint8Le(data, &v8);
      ack_param.value = v8;
      break;
    case 2:
      ReadUint16Le(data, &v16);
      ack_param.value = v16;
      break;
    case 4:
      ReadUint32Le(data, &v32);
      ack_param.value = v32;
      break;
    default:
      assert(false);
      return;
  }
  NetSendAioServoAckParamMessage(&ack_param);
}

static void ServoUpdateParameters(void) {
  g_self.input.r22.r22_status_bits = g_r22_read.event_status;
  // Current and velocity can be sign-corrected before or after calibration as
  // there is no offset.  Angle must be corrected after calibration because
  // there is a calibration offset.
  ServoMeasureUpdate(&g_self.input.r22.current, ServoApplyCurrentCal,
                     g_r22_read.actual_current * g_servo.sign,
                     g_self.sync_timestamp);
  g_self.input.r22.angle =
      g_servo.sign * ServoApplyAngleCal(g_r22_read.actual_position);
  g_self.input.r22.angle_raw = g_r22_read.actual_position;
  ServoMeasureUpdate(&g_self.input.r22.velocity, ServoApplyVelocityCal,
                     g_r22_read.actual_velocity * g_servo.sign,
                     g_self.sync_timestamp);
  g_self.input.r22.temperature = g_r22_read.temperature;
}

// This is the only part of the controller which is called BEFORE updating
// paired status messages.
static void ServoUpdateLocalStatus(ServoState *self,
                                   ServoOperatorCommand *oper) {
  // Input state change.
  switch (self->state) {
    case kActuatorStateInit:
      self->state = kActuatorStateReady;
      break;
    case kActuatorStateReady:
      if (oper->armed) {
        self->state = kActuatorStateArmed;
      }
      break;
    case kActuatorStateArmed:
    case kActuatorStateRunning:
      if (!oper->armed) {
        self->state = kActuatorStateReady;
      } else if (self->control.valid) {
        self->state = kActuatorStateRunning;
      }
      break;
    case kActuatorStateError:
      if (!HasError(&self->control.flags)) {
        self->state = kActuatorStateReady;
      }
      break;
    default:
      self->state = kActuatorStateError;
      oper->armed = false;
      break;
  }

  SetStatus(kServoStatusArmed,
            self->state == kActuatorStateArmed ||
            self->state == kActuatorStateRunning, &self->control.flags);
  SetStatus(kServoStatusCommanded, self->control.valid, &self->control.flags);
  SetStatus(kServoStatusReset, g_servo.reset_status, &self->control.flags);

  // R22 flags.
  SetStatus(kServoStatusOutputClamp, R22GetClampMon(),
            &self->control.flags);
  if (!R22GetClampMon()) {
    TimerStartMsec(500, &g_servo.clamp_timer);
  }
  SignalWarning(kServoWarningOutputClampStuck,
                R22GetClampMon() && TimerExpired(&g_servo.clamp_timer),
                &self->control.flags);
  SignalWarning(kServoWarningR22Reinitialized, g_servo.reinitialized,
                &self->control.flags);
  SignalError(kServoErrorR22Fault, R22GetFaultMon(), &self->control.flags);
  SignalWarning(kServoWarningR22,
                self->input.r22.r22_status_bits & R22_STATUS_BIT_WARNING_MASK,
                &self->control.flags);
  SignalError(kServoErrorR22,
              self->input.r22.r22_status_bits & R22_STATUS_BIT_ERROR_MASK,
              &self->control.flags);
  SignalError(kServoErrorR22Temperature, self->input.r22.r22_status_bits &
              kR22StatusBitDriveOverTemperature, &self->control.flags);
  SignalError(kServoErrorR22OverVoltage, self->input.r22.r22_status_bits &
              kR22StatusBitOverVoltage, &self->control.flags);
  // TODO: Move rest of R22 flags in here.

  if (HasError(&self->control.flags)) {
    self->state = kActuatorStateError;
    oper->armed = false;
  }
}

static void ServoCheckTimeout(ServoProcessState next,
                              int64_t now, uint32_t *failures) {
  if (now >= g_servo.timeout) {
    if (failures != NULL) {
      ++(*failures);
    }
    // TODO: Should this compare against the type-specific failures?
    ++g_servo.num_failures;
    if (g_servo.num_failures >= SERVO_FAILURE_LIMIT) {
      g_servo.num_failures = 0;
      ServoStateInit();
      g_servo.reinitialized = true;
    } else {
      g_servo.clear_entry = true;
      g_servo.state = next;
    }
  }
}

static void ServoCheckPairTimeout(ServoProcessState next,
                                  int64_t now) {
  if (now >= g_servo.timeout) {
    g_servo.clear_entry = true;
    g_servo.state = next;
  }
}

// Common function to handle R22 transfers and timeouts.
static void ServoTransfer(ServoTransferCompleteFunction fn,
                          ServoProcessState next, int64_t now,
                          uint32_t *failures) {
  if (R22CanIsIdle() || now > g_servo.timeout) {
    if (!R22CanHadError() && now <= g_servo.timeout) {
      if (fn) {
        fn();
      }
      g_servo.num_failures = 0;
      g_servo.state = next;
    } else {
      CanopenSdoAbort abort = R22CanGetAbort();
      // TODO: Do not log these abort codes as they are related to
      // R22 behavior inconsistent with the documentation.
      if (abort == kCanopenSdoAbortInvalidDictionaryObject ||
          abort == kCanopenSdoAbortCommandSpecifierNotValid ||
          abort == kCanopenSdoAbortInvalidSubindex ||
          abort == kCanopenSdoAbortNone) {
        g_servo.clear_entry = true;
        g_servo.state = next;
        return;
      }
      if (failures) {
        ++(*failures);
      }
      ++g_servo.num_failures;
      if (g_servo.num_failures >= SERVO_FAILURE_LIMIT) {
        g_servo.num_failures = 0;
        ServoStateInit();
        g_servo.reinitialized = true;
      } else {
        g_servo.clear_entry = true;
        g_servo.state = next;
      }
    }
  }
}

// Wait until process iteration time.
static void ServoIdle(ServoProcessState next, int64_t now) {
  if (now >= g_servo.frame_timeout) {
    if (g_servo.first_entry) {
      // Failed to meet control loop timing requirements.
      ++g_bit.loop_timing;
    }
    ServoControlHeartbeat();
    // Clear out any TPDOs already received.
    R22CanUpdateRxParams1(&g_r22_read);
    R22CanUpdateRxParams2(&g_r22_read);
    R22CanUpdateRxParams3(&g_r22_read);
    // Clear out any status message already received.
    ServoInputQueryPaired(&g_paired);
    // Servo synchronization.
    R22CanSendSync();
    g_self.sync_timestamp = now;
    g_servo.frame_timeout += SERVO_CONTROL_PERIOD_US;
    g_servo.state = next;
  }
}

static void ServoR22Read(ServoProcessState next, int64_t now) {
  if (g_servo.first_entry) {
    g_servo.read_index = 0;
    g_servo.timeout = now + SERVO_TPDO_TIMEOUT_US;
  }
  bool success;
  switch (g_servo.read_index) {
    case 0:
      success = R22CanUpdateRxParams1(&g_r22_read);
      break;
    case 1:
      success = R22CanUpdateRxParams2(&g_r22_read);
      break;
    case 2:
      success = R22CanUpdateRxParams3(&g_r22_read);
      break;
    default:
      ServoUpdateParameters();
      g_servo.state = next;
      return;
  }
  if (success) {
    g_servo.read_index++;
  } else {
    ServoCheckTimeout(next, now, &g_bit.pdo_read);
  }
}

static void ServoSendState(int64_t now, ServoProcessState next) {
  // TODO: Send all input commands in status packet to enable 6-way
  // arbitration.
  ServoInputQueryControllerCommand(now, &g_self.input);
  ServoInputQueryOperatorCommand(&g_operator_command);
  ServoInputQueryLoadcellTetherRelease(&g_self.input.tether_released);
  ServoInputQueryJoystickScuttle(&g_self.input.scuttle_command);
  ServoUpdateLocalStatus(&g_self, &g_operator_command);
  ServoOutputSendStatusMessage(&g_self);
  g_servo.state = next;
}

static void ServoGetPairedState(ServoProcessState next, int64_t now) {
  if (g_servo.first_entry) {
    g_servo.timeout = now + SERVO_NET_TIMEOUT_US;
  }
  if (!ServoIsPaired() || ServoInputQueryPaired(&g_paired)) {
    g_servo.state = next;
  } else {
    ServoCheckPairTimeout(next, now);
    // TODO: This means sync is bad or pair is missing.
  }
}

// Control function for ServoRunController state.
static void ServoRunController(ServoProcessState next) {
  g_self.updated = true;
  // Only run the control loop if we are in position mode.
  if (g_self.input.cmd.mode != kServoModePositionCommand) {
    g_servo.state = next;
  }
  ServoControlPaired(&g_self, &g_paired);
  int64_t timing_offset = 0;
  if (ServoIsPaired()) {
    timing_offset = g_paired.sync_timestamp - g_self.sync_timestamp;
  }
  // Adjust next loop timing.
  if (abs(timing_offset) < SERVO_CONTROL_PERIOD_US) {
    if (abs(timing_offset) > SERVO_CONTROL_PERIOD_US / 2) {
      timing_offset += SERVO_CONTROL_PERIOD_US * (timing_offset > 0 ? -1 : 1);
    }
    g_self.control.jitter = Saturate(timing_offset / 2, -SERVO_MAX_JITTER_US,
                                     SERVO_MAX_JITTER_US);
    g_servo.frame_timeout += g_self.control.jitter;
  } else {
    g_self.control.jitter = 0;
  }
  g_paired.updated = false;
  g_servo.state = next;
}

static void ServoR22Write(ServoProcessState next) {
  g_r22_write.target_position = ServoInvertAngleCal(
      g_servo.sign * g_self.control.desired_angle);
  g_r22_write.target_velocity = g_self.input.cmd.desired_velocity;
  g_r22_write.target_torque = g_self.input.cmd.desired_torque;
  switch (g_self.input.cmd.mode) {
    case kServoModePositionCommand:
      g_r22_write.mode_of_operation = kModeOfOperationProfilePosition;
      break;
    case kServoModeVelocityCommand:
      assert(!ServoIsPaired());
      g_r22_write.mode_of_operation = kModeOfOperationProfileVelocity;
      break;
    case kServoModeTorqueCommand:
      assert(!ServoIsPaired());
      g_r22_write.mode_of_operation = kModeOfOperationProfileTorque;
      break;
    default:
      assert(false);
      g_servo.state = next;
      return;
  }
  if (g_self.control.valid && g_operator_command.armed) {
    g_r22_write.current_limit = ServoInvertCurrentCal(
        g_self.control.current_limit);
    g_r22_write.control_word = (
        kControlWordSwitchOn | kControlWordEnableVoltage |
        kControlWordQuickStopN | kControlWordEnableOperation |
        kControlWordProfilePositionChangeSetImmediately);
  } else {
    g_r22_write.current_limit = 0;
    g_r22_write.control_word = kControlWordDisable;
  }
  if (!R22CanUpdateTxParams1(&g_r22_write)) {
    g_bit.pdo_write++;
  }
  if (g_self.control.valid && g_operator_command.armed) {
    g_r22_write.control_word |= kControlWordProfilePositionNewSetpoint;
  }
  if (!R22CanUpdateTxParams2(&g_r22_write)) {
    g_bit.pdo_write++;
  }
  if (!R22CanUpdateTxParams3(&g_r22_write)) {
    g_bit.pdo_write++;
  }
  g_servo.state = next;
}

// Process ServoCompleteTask state. Called after handling the required control
// loop tasks.
static void ServoCompleteTask(ServoProcessState next) {
  g_servo.reset_status = false;
  g_servo.reinitialized = false;
  if (g_operator_command.clear_errors) {
    g_operator_command.clear_errors = false;
    g_servo.state = kServoClearErrors;
  } else {
    ++g_servo.get_extra_timer;
    g_servo.get_extra_timer %= SERVO_GET_EXTRA_DECIMATION;
    if (g_servo.get_extra_timer == 0) {
      g_servo.state = kServoCheckParameter;
    } else if (g_servo.set_parameter) {
      g_servo.set_parameter = false;
      g_servo.state = kServoSetParameter;
    } else if (g_operator_command.set_parameter) {
      g_operator_command.set_parameter = false;
      g_operator_command.get_parameter = true;
      g_servo.state = kServoSetParameterCommand;
    } else if (g_operator_command.get_parameter) {
      g_operator_command.get_parameter = false;
      g_servo.state = kServoGetParameterCommand;
    } else if (g_operator_command.clear_error_log) {
      g_operator_command.clear_error_log = false;
      g_servo.state = kServoClearErrorLog;
    } else {
      g_servo.state = next;
    }
  }
}

// Incremently cycle through all R22 parameters.
static void ServoNextParameter(void) {
  R22Parameter next_param =
      R22ParamGetNextConfigParam(g_param.param, g_param.mem);
  if ((int32_t)next_param <= (int32_t)g_param.param) {
    if (g_param.mem == kR22MemoryRam) {
      g_param.mem = kR22MemoryFlash;
    } else {
      g_param.mem = kR22MemoryRam;
    }
  }
  g_param.param = next_param;
  g_param.info = R22ParamGetInfo(g_param.param);
  R22ParamGetValuePtr(g_param.param, &g_param.data);
}

// Process ServoCheckParameter state. Periodically check one R22 parameter to
// verify integrity of R22.
static void ServoCheckParameter(ServoProcessState next, int64_t now) {
  if (g_servo.first_entry) {
    ServoNextParameter();
    R22CanGetValue(g_param.param);
    g_servo.timeout = now + R22_TRANSFER_TIMEOUT_US;
  } else {
    ServoTransfer(ServoCheckParameterRead, next, now, &g_bit.get_parameter);
  }
}

// Process ServoSetParameter state.
static void ServoSetParameter(ServoProcessState next, int64_t now) {
  if (g_servo.first_entry) {
    R22CanSetUint8Be(g_param.param, g_param.info->addr_length, g_param.data);
    g_servo.timeout = now + R22_TRANSFER_TIMEOUT_US;
  } else {
    ServoTransfer(NULL, next, now, &g_bit.set_parameter);
  }
}

// Process ServoGetParameterCommand state.
static void ServoGetParameterCommand(ServoProcessState next, int64_t now) {
  if (g_servo.first_entry) {
    R22CanGetValue(g_operator_command.parameter_type);
    g_servo.timeout = now + R22_TRANSFER_TIMEOUT_US;
  } else {
    ServoTransfer(ServoGetParameterRead, next, now, &g_bit.get_parameter);
  }
}

// Process ServoSetParameterCommand state.
static void ServoSetParameterCommand(ServoProcessState next, int64_t now) {
  if (g_servo.first_entry) {
    const R22ParamInfo *info = R22ParamGetInfo(
        g_operator_command.parameter_type);
    uint8_t data[4];
    if (!info) {
      assert(false);
      g_servo.state = next;
      return;
    }
    WriteUint32Le(g_operator_command.set_parameter_value, data);
    R22CanSetUint8Le(g_operator_command.parameter_type, info->addr_length,
                     data);
    g_servo.timeout = now + R22_TRANSFER_TIMEOUT_US;
  } else {
    ServoTransfer(NULL, next, now, &g_bit.set_parameter);
  }
}

// Process ServoClearErrorLog state.
static void ServoClearErrorLog(ServoProcessState next, int64_t now) {
  if (g_servo.first_entry) {
    R22CanClearErrorLog();
    g_servo.timeout = now + R22_TRANSFER_TIMEOUT_US;
  } else {
    ServoTransfer(NULL, next, now, &g_bit.clear_error_log);
  }
}

// Process ServoClearErrors state.
static void ServoClearErrors(ServoProcessState next, int64_t now) {
  if (g_servo.first_entry) {
    R22CanClearErrors();
    ClearErrors(&g_self.control.flags);
    g_servo.timeout = now + R22_TRANSFER_TIMEOUT_US;
  } else {
    ServoTransfer(NULL, next, now, &g_bit.clear_errors);
  }
}

// Process ServoInit state.
static void ServoProcessInit(ServoProcessState next, int64_t now) {
  if (g_servo.first_entry) {
    memset(&g_param, 0, sizeof(g_param));
    memset(&g_bit, 0, sizeof(g_bit));

    PatchConfiguration(AppConfigGetAioNode());
    ServoInit();
    ServoControlInit(&g_self, &g_paired);

    // This flag indicates we are in the first loop iteration.
    g_servo.reset_status = true;

    g_servo.sign = ServoDirectionSign();
    g_param.mem = kR22MemoryRam;
  } else if (ServoInitPoll(now)) {
    g_servo.frame_timeout = now + SERVO_CONTROL_PERIOD_US;
    g_servo.state = next;
  }
}

// Init.
void ServoStateInit(void) {
  memset(&g_servo, 0, sizeof(g_servo));
  g_servo.state = kServoInit;
  g_servo.first_entry = true;
}

// Process state machine.
void ServoStatePoll(int64_t now) {
  ServoProcessState current;
  do {
    current = g_servo.state;
    switch (current) {
      case kServoIdle:
        ServoIdle(kServoR22Read, now);
        break;
      case kServoR22Read:
        ServoR22Read(kServoSendState, now);
        break;
      case kServoSendState:
        ServoSendState(now, kServoGetPairedState);
        break;
      case kServoGetPairedState:
        ServoGetPairedState(kServoRunController, now);
        break;
      case kServoRunController:
        ServoRunController(kServoR22Write);
        break;
      case kServoR22Write:
        ServoR22Write(kServoCompleteTask);
        break;
      case kServoCompleteTask:
        ServoCompleteTask(kServoIdle);
        break;
      case kServoCheckParameter:
        ServoCheckParameter(kServoIdle, now);
        break;
      case kServoSetParameter:
        ServoSetParameter(kServoIdle, now);
        break;
      case kServoSetParameterCommand:
        ServoSetParameterCommand(kServoIdle, now);
        break;
      case kServoGetParameterCommand:
        ServoGetParameterCommand(kServoIdle, now);
        break;
      case kServoClearErrorLog:
        ServoClearErrorLog(kServoIdle, now);
        break;
      case kServoClearErrors:
        ServoClearErrors(kServoIdle, now);
        break;
      case kServoInit:
        ServoProcessInit(kServoIdle, now);
        break;
      default:
        g_servo.state = kServoIdle;
        break;
    }
    // Flag first_entry allows states to perform initialization routines on
    // first execution, then poll until completion. Flag clear_entry allows
    // states to reset and run the initialization routines again as if the
    // state were just entered.
    g_servo.first_entry = current != g_servo.state || g_servo.clear_entry;
    g_servo.clear_entry = false;
  } while (current != g_servo.state);
}

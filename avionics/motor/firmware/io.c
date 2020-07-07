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

#include "avionics/motor/firmware/io.h"

#include <assert.h>
#include <limits.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

#include "avionics/common/actuator_types.h"
#include "avionics/common/avionics_messages.h"
#include "avionics/common/cvt_avionics_messages.h"
#include "avionics/common/fast_math/fast_math.h"
#include "avionics/common/motor_profiler_types.h"
#include "avionics/common/safety_codes.h"
#include "avionics/firmware/cpu/clock.h"
#include "avionics/firmware/cpu/memcpy.h"
#include "avionics/firmware/drivers/ina219_types.h"
#include "avionics/firmware/identity/identity.h"
#include "avionics/firmware/monitors/short_stack_types.h"
#include "avionics/firmware/network/net_mon.h"
#include "avionics/firmware/network/net_send.h"
#include "avionics/firmware/output/slow_status.h"
#include "avionics/network/aio_labels.h"
#include "avionics/network/message_stats.h"
#include "avionics/network/message_type.h"
#include "avionics/motor/firmware/angle.h"
#include "avionics/motor/firmware/angle_meas.h"
#include "avionics/motor/firmware/current_limit.h"
#include "avionics/motor/firmware/errors.h"
#include "avionics/motor/firmware/flags.h"
#include "avionics/motor/firmware/foc.h"
#include "avionics/motor/firmware/io_cvt.h"
#include "avionics/motor/firmware/isr.h"
#include "avionics/motor/firmware/isr_diag.h"
#include "avionics/motor/firmware/main.h"
#include "avionics/motor/firmware/profiler.h"
#include "avionics/motor/firmware/stacking.h"
#include "avionics/motor/firmware/thermal.h"
#include "common/macros.h"

// Loops before discarding a controller message.
//
// TODO: This number should depend on the relative loop
// times of the flight controller and the motor controller.
static const int32_t kNumLoopsUntilStale = 200;

// Allowed MotorCommandFlags in a particular MotorMode.
const MotorCommandFlag kAllowedCommands[kNumMotorModes] = {
  [kMotorModeInit]          = (kMotorCommandClearError
                               | kMotorCommandDisarm
                               | kMotorCommandSendAdcLog
                               | kMotorCommandSendControlLog),
  [kMotorModeArmed]         = (kMotorCommandRun
                               | kMotorCommandDisarm
                               | kMotorCommandSendAdcLog
                               | kMotorCommandSendControlLog),
  [kMotorModeRunning]       = (kMotorCommandRun
                               | kMotorCommandSendAdcLog
                               | kMotorCommandSendControlLog),
  [kMotorModeErrorWindDown] = (kMotorCommandRun
                               | kMotorCommandSendAdcLog
                               | kMotorCommandSendControlLog),
  [kMotorModeErrorDisarmed] = (kMotorCommandClearError
                               | kMotorCommandSendAdcLog
                               | kMotorCommandSendControlLog),
};

// Track the number of loops since a message was updated.
static int32_t g_controller_stale_count;
static int32_t g_stacking_stale_counts[kNumMotors];

// Save the contents of previous messages in case subsequent messages aren't
// updated. The stacking and status messages are also updated and used across
// multiple functions.
static MotorCommand g_motor_command;
static MotorDebugMessage g_debug;
static MotorStackingMessage g_stacking_messages[kNumMotors];
static MotorStatusMessage g_status;

typedef struct {
  float motor_status_message_period;
  float motor_debug_message_period;
  float slow_status_message_period;
} IoMessageTiming;

IoMessageTiming g_io_timing = {
  .motor_debug_message_period = 1.0e-6f * (float)MOTOR_DEBUG_PERIOD_US,
  .motor_status_message_period = 1.0e-6f * (float)MOTOR_STATUS_PERIOD_US,
  .slow_status_message_period = 1.0e-6f * (float)SLOW_STATUS_PERIOD_US,
};

// Pointers to parameters that we can modify through Get/SetParams
// commands.
static float *g_mutable_param_addrs[] = {
  &g_io_timing.motor_debug_message_period,
  &g_io_timing.motor_status_message_period,
  &g_io_timing.slow_status_message_period,

  &g_motor_angle_meas_param.cos_offset,
  &g_motor_angle_meas_param.sin_offset,
  &g_motor_angle_meas_param.sin_scale,
  &g_motor_angle_meas_param.angle_meas_lower_warn_lim,
  &g_motor_angle_meas_param.angle_meas_upper_warn_lim,

  &g_motor_angle_params.theta_offset_e,
  &g_motor_angle_params.omega_transition_deriv,
  &g_motor_angle_params.omega_transition_zc,
  &g_motor_angle_params.zc.omega_filter_pole,
  &g_motor_angle_params.deriv.omega_filter_pole,
  &g_motor_angle_params.deriv.theta_kp,
  &g_motor_angle_params.deriv.theta_ki,
  &g_motor_angle_params.sensor.omega_enable_threshold,
  &g_motor_angle_params.sensor.omega_disable_threshold,
  &g_motor_angle_params.sensor.filter_pole,

  &g_foc_params.i_kp,
  &g_foc_params.i_ki,
  &g_foc_params.omega_to_torque_kp,
  &g_foc_params.omega_to_torque_ki,

  &g_foc_params.omega_anti_windup_kp,
  &g_foc_params.fw_modulation_threshold,
  &g_foc_params.fw_ki,
  &g_foc_params.fw_anti_windup_kp,
  &g_foc_params.foc_phase_current_cmd_limit,

  &g_current_limit_params.ibus_lower_limit,
  &g_current_limit_params.ibus_upper_limit,
  &g_current_limit_params.iq_lower_limit,
  &g_current_limit_params.iq_upper_limit,

  &g_current_limit_params.current_limit_kt_s_pole,
  &g_current_limit_params.current_limit_kt_range,
  &g_current_limit_params.iq_cmd_residual_kp,
  &g_current_limit_params.current_limit_isr_s_pole,

  &g_motor_limits.bus_current_limit,
  &g_motor_limits.phase_current_limit,
  &g_motor_limits.phase_current_sum_limit,
  &g_motor_limits.adc_fault_current_limit,
  &g_motor_limits.v_bus_lower_limit,
  &g_motor_limits.v_bus_upper_limit,
  &g_motor_limits.omega_limit,

  &g_motor_thermal_params.thermal_limits[kMotorThermalChannelBoard],
  &g_motor_thermal_params.thermal_limits[kMotorThermalChannelControllerAir],
  &g_motor_thermal_params.thermal_limits[kMotorThermalChannelStatorCore],
  &g_motor_thermal_params.thermal_limits[kMotorThermalChannelStatorCoil],
  &g_motor_thermal_params.thermal_limits[kMotorThermalChannelNacelleAir],
  &g_motor_thermal_params.thermal_limits[kMotorThermalChannelRotor],
  &g_motor_thermal_params.thermal_limits[kMotorThermalChannelUnused],
  &g_motor_thermal_params.thermal_limits[kMotorThermalChannelHeatPlate1],
  &g_motor_thermal_params.thermal_limits[kMotorThermalChannelHeatPlate2],
  &g_motor_thermal_params.thermal_limits[kMotorThermalChannelCapacitor],
  &g_motor_thermal_params.thermal_limits[kMotorThermalChannelHt3000A],
  &g_motor_thermal_params.thermal_limits[kMotorThermalChannelHt3000B],
  &g_motor_thermal_params.thermal_limits[kMotorThermalChannelHt3000C],

  &g_stacking_params.speed_cmd_gain,
  &g_stacking_params.speed_cmd_zero,
  &g_stacking_params.speed_cmd_pole,
  &g_stacking_params.speed_cmd_lower_limit,
  &g_stacking_params.speed_cmd_upper_limit,
  &g_stacking_params.speed_cmd_lower_rate_limit,
  &g_stacking_params.speed_cmd_upper_rate_limit,
  &g_stacking_params.voltage_pair_bias_gain,
  &g_stacking_params.voltage_pair_bias_zero,
  &g_stacking_params.voltage_pair_bias_pole,
  &g_stacking_params.voltage_pair_bias_limit,
  &g_stacking_params.voltage_control_gain,
  &g_stacking_params.voltage_control_zero,
  &g_stacking_params.voltage_control_pole,
  &g_stacking_params.voltage_control_aw_pole,
  &g_stacking_params.speed_correction_gain,
  &g_stacking_params.speed_correction_zero,
  &g_stacking_params.speed_correction_pole,
  &g_stacking_params.speed_correction_aw_pole,
  &g_stacking_params.iq_cmd_limit_offset,
  &g_stacking_params.iq_cmd_limit_rate,
  &g_stacking_params.iq_cmd_limit_omega_gen,
  &g_stacking_params.gen_limit_dicorr_diqcmd,
  &g_stacking_params.gen_limit_dwcorr_diqcmd,
  &g_stacking_params.gen_limit_wcorr_offset,
  &g_stacking_params.voltage_average_upper_sat,
  &g_stacking_params.shorted_level_run_enable,
  &g_stacking_params.stacking_enable,
  &g_stacking_params.stacking_spin_direction,
  &g_stacking_params.v_delta_averaging_enable,

  &g_stacking_fault_params.comm_timeout_cycles,
  &g_stacking_fault_params.voltage_rel_upper_limit,
  &g_stacking_fault_params.voltage_rel_lower_limit,
  &g_stacking_fault_params.wind_down_current_rate,
  &g_stacking_fault_params.iq_cmd_lower_fault_limit,

  &g_isr_diag_state.isr_diag_enable,
  &g_error_config.shutdown_on_warning_enable,
};

void MotorIoInit(void) {
  // Set all controller and stacking messages to be stale.
  g_controller_stale_count = INT32_MAX;

  for (int32_t i = 0; i < kNumMotors; ++i) {
    g_stacking_stale_counts[i] = (i == AppConfigGetIndex()) ? 0 : INT32_MAX;
    g_stacking_messages[i].motor_status       = kMotorModeInit;
    g_stacking_messages[i].motor_error        = 0x00U;
    g_stacking_messages[i].bus_current        = 0.0f;
    g_stacking_messages[i].bus_voltage        = 1.0f;
    g_stacking_messages[i].current_correction = 0.0f;
    g_stacking_messages[i].iq_cmd_residual    = 0.0f;
  }
  memset(&g_status, 0, sizeof(g_status));
  OutputInitSlowStatusMessage();
}

static bool MotorSelected(uint8_t selected) {
  return (selected & (1 << AppConfigGetIndex())) != 0;
}

// Checks if we have received a valid MotorGetParamMessage.  If we
// have, this sends a MotorAckParamMessage with the parameter data.
static uint32_t HandleGetParam(MotorMode mode, AioNode source) {
  MotorGetParamMessage message;
  if (CvtGetGetParamMessage(source, &message, NULL, NULL)
      && MotorSelected(message.selected_motors)) {
    if (!(mode == kMotorModeInit || mode == kMotorModeErrorDisarmed)) {
      return kMotorErrorBadMode;
    }
    if (message.id >= ARRAYSIZE(g_mutable_param_addrs)) {
      return kMotorErrorBadCommand;
    }

    MotorAckParamMessage ack_param = {message.id,
                                      *g_mutable_param_addrs[message.id]};
    NetSendAioMotorAckParamMessage(&ack_param);
  }
  return kMotorErrorNone;
}

// Checks if we have received a valid MotorSetParamMessage.  If we
// have, this sets the parameter and sends a MotorAckParamMessage with
// the parameter data.
static uint32_t HandleSetParam(MotorMode mode, AioNode source) {
  MotorSetParamMessage message;
  if (CvtGetSetParamMessage(source, &message, NULL, NULL)
      && MotorSelected(message.selected_motors)) {
    if (!(mode == kMotorModeInit || mode == kMotorModeErrorDisarmed)) {
      return kMotorErrorBadMode;
    }
    if (message.id >= ARRAYSIZE(g_mutable_param_addrs)) {
      return kMotorErrorBadCommand;
    }

    *g_mutable_param_addrs[message.id] = message.value;
    MotorAckParamMessage ack_param = {message.id, message.value};
    NetSendAioMotorAckParamMessage(&ack_param);
  }
  return kMotorErrorNone;
}

static uint32_t MotorArmingLogic(uint32_t arming_signal, MotorMode mode,
                                 bool *arm) {
  if (mode != kMotorModeInit) {
    return kMotorErrorBadMode;
  }
  if (arming_signal != MOTOR_ARMING_SIGNAL) {
    return kMotorErrorBadCommand;
  }
  *arm = true;
  return kMotorErrorNone;
}

// Override the controller command message if a SetStateMessage is received.
// TODO(b/25643038): The support for disarming and clearing errors is a hack.
// It should be replaced by an integrated solution which replaces/removes use
// of motor_command.
static uint32_t HandleSetState(MotorMode mode, AioNode source, bool *arm) {
  uint32_t errors = kMotorErrorNone;

  MotorSetStateMessage message;
  if (CvtGetSetStateMessage(source, &message, NULL, NULL)
      && MotorSelected(message.selected_motors)) {
    switch (message.command) {
      case kActuatorStateCommandArm:
        errors |= MotorArmingLogic(message.command_data, mode, arm);
        break;
      case kActuatorStateCommandClearErrors:
        if (kMotorCommandClearError & kAllowedCommands[mode]) {
          g_motor_command.command |= kMotorCommandClearError;
        } else {
          errors |= kMotorErrorBadMode;
        }
        break;
      case kActuatorStateCommandDisarm:
        errors |= kMotorErrorBadMode;  // Indirectly support disarm.
        break;
      case kActuatorStateCommandNone:  // Fall-through intentional.
      case kActuatorStateCommandTest:  // Fall-through intentional.
      default:
        errors |= kMotorErrorBadMode;
        break;
    }
  }
  return errors;
}

// Updates the array of most recent commands from each controller and
// the number of loops that have passed since the last update.
static uint32_t HandleCommands(int64_t now, MotorMode mode) {
  uint32_t errors = kMotorErrorNone;
  ControllerLabel source;
  if ((CvtGetMotorCommand(now, &g_motor_command, &source))) {
    g_status.cmd_arbiter.controllers_used |= 1 << source;
    if (g_motor_command.command & ~(int16_t)kAllowedCommands[mode]) {
      errors |= kMotorErrorBadMode;
    }

    g_controller_stale_count = 0;
  } else if (g_controller_stale_count < INT32_MAX) {
    g_controller_stale_count++;
    // Only allow kMotorCommandRun to be reused between received commands.
    g_motor_command.command &= kMotorCommandRun;
  }
  return errors;
}

// Updates the errors for a motor controller if the short stack has
// initiated a short on that motor's stack level ("block").
//
// The idea is to send all motors into WindDown by way of ErrorBlockShorting
// (for the motors on the shorting level) or ErrorRemoteFaultBlockX (for the
// motors on the nonshorted levels -- these will return to running once the
// short completes if shorted_level_run_enable is set).
// Note that if the short_stack signals that it is shorting the level but then
// is unable to complete the short due to relay failure or other complication,
// all motors will permanently wind down.
static uint32_t HandleShortStackMessage(void) {
  int32_t i_block = AppConfigGetIndex() % (kNumMotors / 2);
  uint32_t errors = kMotorErrorNone;
  ShortStackStackingMessage msg;
  if (CvtGetShortStackStackingMessage(kAioNodeShortStack, &msg, NULL, NULL) &&
      (msg.firing_status & (kShortStackStatusTrippedB0 << i_block))) {
    errors |= kMotorErrorBlockShorting;
  }
  return errors;
}

// Updates the array of most recent stacking messages from each motor and
// the number of loops that have passed since the last update.
static uint32_t HandleStacking(MotorMode mode, int32_t source_index) {
  assert(0 <= source_index && source_index < kNumMotors);

  MotorStackingMessage message;
  if (source_index == AppConfigGetIndex()) {
    g_stacking_stale_counts[source_index] = 0;
  } else if (CvtGetStackingMessage(
      (AioNode)(source_index + kAioNodeMotorSbo), &message,
      &g_debug.sequence[source_index], NULL)) {
    g_stacking_stale_counts[source_index] = 0;
    g_stacking_messages[source_index] = message;
  } else if (g_stacking_stale_counts[source_index] < INT32_MAX
             && (mode == kMotorModeRunning
                 || mode == kMotorModeErrorWindDown)) {
    g_stacking_stale_counts[source_index]++;
  }

  return kMotorErrorNone;
}

// This throws away messages that are stale.
// If there is no up-to-date message, this returns false.
static bool GetCommand(const MotorCommand* motor_command,
                       int32_t stale_count,
                       float *omega_upper_limit,
                       float *omega_lower_limit, float *torque_cmd,
                       int16_t *command) {
  if (stale_count < kNumLoopsUntilStale) {
    *torque_cmd = motor_command->torque;
    *omega_upper_limit = motor_command->speed_upper_limit;
    *omega_lower_limit = motor_command->speed_lower_limit;
    *command = motor_command->command;
    return true;
  } else {
    // If there are no up-to-date messages, set a safe command.
    *torque_cmd = 0.0f;
    *omega_upper_limit = 0.0f;
    *omega_lower_limit = 0.0f;
    *command = kMotorCommandNone;
    return false;
  }
}

// Process the most recently received input messages.
uint32_t MotorIoReceiveInputs(int64_t now, MotorMode mode, bool *arm,
                              int16_t *command, float *omega_upper_limit,
                              float *omega_lower_limit, float *torque_cmd,
                              uint32_t stacking_errors[kNumMotors],
                              int32_t motor_stale_counts[kNumMotors],
                              float bus_currents[kNumMotors],
                              float bus_voltages[kNumMotors],
                              float current_corrections[kNumMotors],
                              float iq_cmd_residuals[kNumMotors]) {
  // Default to no arm command.
  *arm = false;

  uint32_t errors = kMotorErrorNone;

  errors |= HandleSetParam(mode, kAioNodeOperator);
  errors |= HandleGetParam(mode, kAioNodeOperator);

  // Process short-stack message.
  errors |= HandleShortStackMessage();

  // Process controller messages.
  errors |= HandleCommands(now, mode);
  errors |= HandleSetState(mode, kAioNodeOperator, arm);

  bool valid_command = GetCommand(&g_motor_command,
                                  g_controller_stale_count, omega_upper_limit,
                                  omega_lower_limit, torque_cmd, command);
  if ((mode == kMotorModeRunning || mode == kMotorModeErrorWindDown)
      && !valid_command) {
    errors |= kMotorErrorTimeout;
  }

  // Process motor stacking messages.
  for (int32_t i = 0; i < kNumMotors; ++i) {
    // Handle the current stacking message from motor i.
    errors |= HandleStacking(mode, i);

    // Load stacking data into output arrays.
    // TODO: Copy directly into output arrays (g_stacking_messages
    // is presently needed to deal with stale messages).
    motor_stale_counts[i]  = g_stacking_stale_counts[i];
    stacking_errors[i]     = g_stacking_messages[i].motor_error;
    bus_currents[i]        = g_stacking_messages[i].bus_current;
    bus_voltages[i]        = g_stacking_messages[i].bus_voltage;
    current_corrections[i] = g_stacking_messages[i].current_correction;
    iq_cmd_residuals[i]    = g_stacking_messages[i].iq_cmd_residual;
  }

  return errors;
}

// Converts the motor mode to the equivalent status flag.
//
// TODO: Remove redundant means of expressing motor mode.
static MotorStatusFlag MotorModeToStatus(MotorMode mode) {
  switch (mode) {
    case kMotorModeInit: return kMotorStatusInit;
    case kMotorModeArmed: return kMotorStatusArmed;
    case kMotorModeRunning: return kMotorStatusRunning;
    case kMotorModeErrorWindDown: return kMotorStatusWindDown;
    case kMotorModeErrorDisarmed:  // Fall through intentional.
    default:
      return kMotorStatusError;
  }
}

static ActuatorState MotorModeToActuatorState(MotorMode mode) {
  const ActuatorState map[kNumMotorModes] = {
    [kMotorModeInit]          = kActuatorStateInit,
    [kMotorModeArmed]         = kActuatorStateArmed,
    [kMotorModeRunning]       = kActuatorStateRunning,
    [kMotorModeErrorWindDown] = kActuatorStateError,
    [kMotorModeErrorDisarmed] = kActuatorStateError,
  };
  return map[mode];
}

// Sends stacking and motor status messages.
uint32_t MotorIoSendOutputs(MotorMode mode, uint32_t errors, uint32_t warnings,
                            const CurrentLimitNetOutput *current_limit_output,
                            const MotorIsrOutput *isr_output,
                            const MotorThermalData *thermal_data,
                            const ProfilerOutput *profiler_output,
                            bool *motor_status_message_sent) {
  static int32_t motor_debug_counter = 0;
  static int32_t motor_status_counter = 0;
  // Make sure this never lines up with status_counter.
  static int32_t slow_status_counter = (int32_t)(0.5f * 1.0e-6f
                                                 * MOTOR_STATUS_PERIOD_US
                                                 * MOTOR_AIO_FREQUENCY);

  *motor_status_message_sent = false;

  MotorStatusFlag motor_status = MotorModeToStatus(mode);

  // Send a stacking message every call.
  MotorStackingMessage stacking = {
    .motor_status       = motor_status,
    .motor_error        = errors,
    .bus_current        = isr_output->current_limit_data.i_bus,
    .bus_voltage        = isr_output->v_bus,
    .current_correction = isr_output->current_correction,
    .iq_cmd_residual    = current_limit_output->iq_cmd_residual,
  };
  if (!NetSendAioMotorStackingMessage(&stacking)) {
    return kMotorWarningNoTx;
  }

  if (motor_debug_counter >= IRoundf(g_io_timing.motor_debug_message_period
                                     * MOTOR_AIO_FREQUENCY)) {
    // Send a separate debug message with additional information for post-
    // processing.
    g_debug.motor_status       = motor_status;
    g_debug.motor_error        = errors;
    g_debug.motor_warning      = warnings;
    g_debug.bus_current        = isr_output->i_bus;
    g_debug.bus_voltage        = isr_output->v_bus;
    g_debug.chassis_voltage    = isr_output->v_chassis;
    g_debug.cm_voltage         = isr_output->v_cm;
    g_debug.theta              = isr_output->theta;
    g_debug.omega              = isr_output->omega;
    g_debug.omega_upper_limit  = isr_output->omega_upper_limit;
    g_debug.omega_lower_limit  = isr_output->omega_lower_limit;
    g_debug.torque_cmd         = isr_output->torque_cmd;
    g_debug.kt_scale           = current_limit_output->kt_scale;
    g_debug.iq_cmd_residual    = current_limit_output->iq_cmd_residual;
    g_debug.iq_upper_limit     = current_limit_output->iq_upper_limit_bus;
    g_debug.iq_lower_limit     = current_limit_output->iq_lower_limit_bus;
    g_debug.id                 = isr_output->id;
    g_debug.id_cmd             = isr_output->id_cmd;
    g_debug.iq                 = isr_output->iq;
    g_debug.iq_cmd             = isr_output->iq_cmd;
    g_debug.vd                 = isr_output->vd;
    g_debug.vq                 = isr_output->vq;
    g_debug.current_correction = isr_output->current_correction;
    g_debug.speed_correction   = isr_output->speed_correction;
    g_debug.voltage_pair_bias  = isr_output->voltage_pair_bias;
    g_debug.voltage_stack_mean = isr_output->voltage_stack_mean;
    g_debug.angle_sensor       = isr_output->angle_sensor;
    // Sequence is updated in HandleStacking.
    if (!NetSendAioMotorDebugMessage(&g_debug)) {
      return kMotorWarningNoTx;
    }
    motor_debug_counter = 0;
  }
  ++motor_debug_counter;

  int32_t motor_status_message_counts
      = IRoundf(g_io_timing.motor_status_message_period * MOTOR_AIO_FREQUENCY);
  int32_t slow_status_message_counts
      = IRoundf(g_io_timing.slow_status_message_period * MOTOR_AIO_FREQUENCY);
  if (motor_status_counter >= motor_status_message_counts) {
    g_status.motor_status       = motor_status;
    g_status.motor_error        = errors;
    g_status.motor_warning      = warnings;
    g_status.state              = MotorModeToActuatorState(mode);
    g_status.bus_current        = isr_output->i_bus;
    g_status.bus_voltage        = isr_output->v_bus;
    g_status.chassis_voltage    = isr_output->v_chassis;
    g_status.cm_voltage         = isr_output->v_cm;
    g_status.omega              = isr_output->omega;
    g_status.omega_upper_limit  = isr_output->omega_upper_limit;
    g_status.omega_lower_limit  = isr_output->omega_lower_limit;
    g_status.torque_cmd         = isr_output->torque_cmd;
    g_status.id                 = isr_output->id;
    g_status.id_cmd             = isr_output->id_cmd;
    g_status.iq                 = isr_output->iq;
    g_status.iq_cmd             = isr_output->iq_cmd;
    g_status.vd                 = isr_output->vd;
    g_status.vq                 = isr_output->vq;
    g_status.current_correction = isr_output->current_correction;
    g_status.speed_correction   = isr_output->speed_correction;
    g_status.voltage_pair_bias  = isr_output->voltage_pair_bias;
    g_status.v_supply_primary   = isr_output->v_in_mon;
    g_status.v_supply_auxiliary = isr_output->v_aux_mon;
    FastCopy(sizeof(thermal_data->channel_temps),
             thermal_data->channel_temps, g_status.temps);
    g_status.profiler_output    = *profiler_output;

    if (!NetSendAioMotorStatusMessage(&g_status)) {
      return kMotorWarningNoTx;
    } else {
      g_status.cmd_arbiter.controllers_used = 0;
      *motor_status_message_sent = true;
    }
    motor_status_counter -= motor_status_message_counts;
  } else if (slow_status_counter >= slow_status_message_counts) {
    if (!OutputSendSlowStatusMessage(ClockGetUs())) {
      return kMotorWarningNoTx;
    }
    slow_status_counter -= slow_status_message_counts;
  }
  motor_status_counter++;
  slow_status_counter++;

  if (IsrDiagIsEnabled()) {
    MotorIsrDiagMessage *isr_diag = IsrDiagGetMessage();
    if (!NetSendAioMotorIsrDiagMessage(isr_diag)) {
      return kMotorWarningNoTx;
    }
  }

  return kMotorWarningNone;
}

MotorMonitorData *MotorIoGetMotorMonitors(void) {
  return &g_status.motor_mon;
}

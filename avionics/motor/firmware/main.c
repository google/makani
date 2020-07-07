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

#include "avionics/motor/firmware/main.h"

#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "avionics/firmware/cpu/clock.h"
#include "avionics/firmware/cpu/io.h"
#include "avionics/firmware/cpu/i2c.h"
#include "avionics/firmware/cpu/mibspi.h"
#include "avionics/firmware/cpu/spi.h"
#include "avionics/firmware/cpu/vim.h"
#include "avionics/firmware/cpu/watchdog.h"
#include "avionics/firmware/drivers/bcm53101.h"
#include "avionics/firmware/drivers/ext_watchdog.h"
#include "avionics/firmware/drivers/led.h"
#include "avionics/firmware/identity/identity.h"
#include "avionics/firmware/network/net.h"
#include "avionics/firmware/network/net_diag.h"
#include "avionics/firmware/network/net_mon.h"
#include "avionics/firmware/network/switch_config.h"
#include "avionics/firmware/params/param_server.h"
#include "avionics/firmware/serial/board_serial.h"
#include "avionics/motor/firmware/adc.h"
#include "avionics/motor/firmware/angle.h"
#include "avionics/motor/firmware/angle_meas.h"
#include "avionics/motor/firmware/calibrator.h"
#include "avionics/motor/firmware/config_params.h"
#include "avionics/motor/firmware/current_limit.h"
#include "avionics/motor/firmware/dma.h"
#include "avionics/motor/firmware/errors.h"
#include "avionics/motor/firmware/flags.h"
#include "avionics/motor/firmware/foc.h"
#include "avionics/motor/firmware/gio.h"
#include "avionics/motor/firmware/io.h"
#include "avionics/motor/firmware/io_cvt.h"
#include "avionics/motor/firmware/isr.h"
#include "avionics/motor/firmware/logging.h"
#include "avionics/motor/firmware/mon.h"
#include "avionics/motor/firmware/params.h"
#include "avionics/motor/firmware/profiler.h"
#include "avionics/motor/firmware/selftest.h"
#include "avionics/motor/firmware/stacking.h"
#include "avionics/motor/firmware/svpwm.h"
#include "avionics/motor/firmware/thermal.h"
#include "avionics/motor/firmware/util.h"
#include "avionics/network/aio_node.h"

// TODO: Deprecate. We still need to call ClockGetUs() periodically
// to maintain a monotonically increasing clock for NetPoll/CvtPut. If we
// stop receiving messages, our 64-bit timebase will not accumulate
// properly.
#define CLOCKGETUS_PERIOD_CYCLES CLOCK32_MSEC_TO_CYCLES(1000)

static MotorHardware g_motor_controller_type = -1;

// Selects the next state of the motor mode state machine.
static MotorMode SelectMotorMode(MotorMode mode, bool arm, int16_t command,
                                 uint32_t errors, float omega,
                                 uint32_t block_shorted_mask) {
  if (IsCriticalError(errors)) return kMotorModeErrorDisarmed;

  // The bool error_disarm determines whether a motor should be prevented from
  // entering or returning to Running or Armed mode.
  // If shorted_level_running_enabled is set, its value should exclude remote
  // fault errors on a block that the short-stack has successfully shorted out
  // (mask), so that the remaining 6 motors can commence or return to operation.
  // If shutdown on warning enabled, and we have a warning, it will be promoted
  // to a non-critical, latching error by this time, setting error_disarm=true.
  bool error_disarm;
  if (ShortedLevelRunningEnabled()) {
    error_disarm = IsNonCriticalError(errors & ~block_shorted_mask);
  } else {
    error_disarm = IsNonCriticalError(errors);
  }

  switch (mode) {
    case kMotorModeInit:
      if (error_disarm) return kMotorModeErrorDisarmed;
      if (arm) return kMotorModeArmed;
      break;

    case kMotorModeArmed:
      if (error_disarm) return kMotorModeErrorDisarmed;
      if (command & kMotorCommandDisarm) return kMotorModeInit;
      if (command & kMotorCommandRun) return kMotorModeRunning;
      break;

    case kMotorModeRunning:
      if (!(command & kMotorCommandRun)) return kMotorModeInit;
      if (error_disarm) return kMotorModeErrorWindDown;
      break;

    case kMotorModeErrorWindDown:
      if (!error_disarm && ShortedLevelRunningEnabled()) {
        return kMotorModeRunning;  // Bad level shorted --> n-2 motor operation.
      }
      if (!(command & kMotorCommandRun)) return kMotorModeErrorDisarmed;
      // Get rid of magic number 3.0f if testing reveals smarter strategy.
      if (fabsf(omega) < 3.0f) return kMotorModeErrorDisarmed;
      break;

    case kMotorModeErrorDisarmed:
      if (command & kMotorCommandClearError) return kMotorModeInit;
      break;

    default:
      assert(false);
      return kMotorModeErrorDisarmed;
  }

  // Remain in current mode.
  return mode;
}

// The slow (1000 Hz) loop handles sending and receiving messages to
// and from the rest of the avionics, updating the mode of the motor
// controller, stacking corrections, and thermal monitoring.
static void Loop(int64_t now) {
  ProfilerLoopTic();

  static uint32_t errors = kMotorErrorNone;
  static uint32_t warnings = kMotorWarningNone;
  static uint32_t block_shorted_mask = 0x00U;
  static MotorMode mode = kMotorModeInit;

  // Update hardware monitors (including temperature).
  MotorMonPoll();

  // Update thermal data and check temperatures.
  warnings |= MotorThermalTempWarnings();

  bool arm = false;
  int16_t command = kMotorCommandNone;
  uint32_t stacking_errors[kNumMotors];
  int32_t motor_stale_counts[kNumMotors];
  float bus_currents[kNumMotors];
  float bus_voltages[kNumMotors];
  float current_corrections[kNumMotors];
  float iq_cmd_residuals[kNumMotors];
  CurrentLimitNetOutput current_limit_net_output;
  bool motor_status_message_sent;

  // Get a pointer to the unused ISR input buffer.
  MotorIsrInput *isr_input = MotorIsrGetIoInputBuffer();

  // Receive stacking messages from other motors and short-stack
  // and commands from the controllers.
  errors |= MotorIoReceiveInputs(now, mode, &arm, &command,
                                 &isr_input->omega_upper_limit,
                                 &isr_input->omega_lower_limit,
                                 &isr_input->torque_cmd,
                                 stacking_errors, motor_stale_counts,
                                 bus_currents, bus_voltages,
                                 current_corrections, iq_cmd_residuals);

  // Get a pointer to the most recently updated ISR output buffer.
  const MotorIsrOutput *isr_output = MotorIsrFilterRawData();
  errors |= isr_output->errors;      // Update with errors and warnings
  warnings |= isr_output->warnings;  // generated in the ISR.

  if (kMotorConfigParams->calibrator_enable == kMotorCalibratorEnabled) {
    // TODO: Eliminate the redundant double buffering by the
    // calibrator code.
    CalibrationSlowLoop(command);
  } else {
    int32_t index = AppConfigGetIndex();
    stacking_errors[index] = errors;
    bus_voltages[index] = isr_output->v_bus;
    bus_currents[index] = isr_output->current_limit_data.i_bus;
    current_corrections[index] = isr_output->current_correction;
    iq_cmd_residuals[index]
        = MotorCurrentLimitGetResidual(&isr_output->current_limit_data);

    // Precalculate the voltage pair bias and stack averages that only change
    // after receiving new messages from other motors.
    errors |= StackingPreprocess(bus_voltages, current_corrections,
                                 motor_stale_counts, stacking_errors,
                                 &isr_input->stacking, &block_shorted_mask);
  }

  // Calculate the available bus current limits.
  MotorCurrentLimitPreprocess(stacking_errors, motor_stale_counts,
                              bus_currents, iq_cmd_residuals,
                              &isr_output->current_limit_data,
                              &isr_input->current_limit_input,
                              &current_limit_net_output);

  if (mode != kMotorModeRunning && mode != kMotorModeErrorWindDown) {
    warnings |= MotorAdcMagnitudeInterruptReset(g_motor_limits.
                                                adc_fault_current_limit);
    MotorAngleReset(g_svpwm_isr_period, g_motor_params);
  }

  // Retrieve GDB PGOOD warnings.
  warnings |= MotorGdbGetPowerGoodStatus();

  // Retrieve GDB Desat warnings.
  warnings |= MotorGdbDesatStatus();

  // If shutdown on warning enabled, promote warnings to latching, non-critical
  // error so all motors wind down completely even if the warning clears.
  if (ShutDownOnWarning() && IsWarning(warnings)) {
    errors |= kMotorErrorPromoteWarning;
  }

  // Clear errors and warnings if we have received the clear error command.
  // It is possible to be in the Init mode with a warning present if
  // shutdown_on_warning_enable == 0.
  isr_input->clear_errors = false;
  if ((mode == kMotorModeErrorDisarmed || mode == kMotorModeInit)
      && (command & kMotorCommandClearError)) {
    isr_input->clear_errors = true;
    errors = kMotorErrorNone;
    warnings = kMotorWarningNone;
    block_shorted_mask = 0x00U;
    MotorGdbPowerGoodReset();
    MotorGdbDesatReset();
  }

  // Update motor mode.
  mode = SelectMotorMode(mode, arm, command, errors, isr_output->omega,
                         block_shorted_mask);
  isr_input->mode = mode;

  // Pass errors to ISR for logging.
  isr_input->errors = errors;
  isr_input->warnings = warnings;

  warnings |= MotorIoSendOutputs(mode, errors, warnings,
                                 &current_limit_net_output,
                                 isr_output, MotorThermalGetData(),
                                 ProfilerGetOutput(),
                                 &motor_status_message_sent);

  // Un-latch PGOOD and/or DESAT warnings after sending MotorStatusMessage.
  isr_input->clear_warnings_mask =
      warnings & (kMotorWarningPowerGoodHetPinEna | kMotorWarningDesat);
  if (motor_status_message_sent
      && (g_motor_controller_type == kMotorHardwareOzoneA1)
      && isr_input->clear_warnings_mask) {
    if (warnings & kMotorWarningPowerGoodHetPinEna) {
      MotorGdbPowerGoodReset();
    }
    if (warnings & kMotorWarningDesat) {
      MotorGdbDesatReset();
    }
    warnings = warnings & ~isr_input->clear_warnings_mask;
  }

  MotorIsrSwapInputBuffers();  // Expose the input buffer to the ISR.

  // Reset derived stacking filter values if not running.
  if (mode == kMotorModeInit || mode == kMotorModeErrorDisarmed) {
    // TODO: Fix thread safety issue.
    if (StackingIsEnabled()) {
      StackingInit(MOTOR_AIO_PERIOD, g_motor_controller_type);
    }
    MotorCurrentLimitInit(MOTOR_AIO_PERIOD, g_svpwm_isr_period);

    // Handle param request / response messages. Sending the response message
    // takes a very long time (134 us when last benchmarked) and must never be
    // done while running.
    ParamServerPoll(kParamServerCalib
                    | kParamServerConfig
                    | kParamServerSerial);
  }

  // Send ADC log if we have received the command.
  if (command & kMotorCommandSendAdcLog) {
    MotorLogAdcSend();
  }

  // Start sending the control log if we have received the command.
  // Otherwise, continue sending log packets until finished if necessary.
  bool start_control_log_send = command & kMotorCommandSendControlLog;
  MotorLogControlSend(start_control_log_send);

  ProfilerLoopToc();
}

// Called by assert() and VIM exceptions.
void OnFatalException(void) {
  MotorShutdown();
}

int main(void) {
  ExtWatchdogInit();
  MibSPIInit(1, kSpiPinmuxAll);
  SwitchConfigInit();
  Bcm53101Init(true);
  I2cInit(300e3);
  NetInit(AppConfigGetAioNode());
  // Perform self test as soon as possible and immediately after NetInit()
  // such that we validate parameters before calling dependent code.
  SelfTest();

  // Initialize access switch monitor.
  NetMonInit(GetSwitchConfig()->info);
  NetDiagInit(GetSwitchConfig()->info);

  // Initialize motor controller.
  MotorParamsInit(kMotorConfigParams->motor_type);

  ProfilerInit();

  GateDriverInit();
  GateDriverDisable();

  LedInit();

  MotorIoInit();
  MotorMonInit();

  g_motor_controller_type = GetBoardHardwareRevision();

  MotorDmaInit(g_motor_controller_type);

  MotorAdcInit(g_motor_controller_type);
  MotorAdcCalibrateOffsets();
  (void)MotorAdcMagnitudeInterruptInit(g_motor_limits.adc_fault_current_limit);

  MotorLogInit(g_motor_controller_type);

  // Configure the N2HET1 module which governs the PWM and ISR periods. The
  // closest PWM period that the module can achieve is stored in the global
  // variable g_svpwm_isr_period.
  if (g_motor_controller_type == kMotorHardwareOzoneA1) {
    SvpwmInit(15.0e3f, 1);
  } else {
    SvpwmInit(30.0e3f, 2);
  }

  // Setup N2HET2 to count frequency of HT3000 thermal sensors.
  MotorThermalHt3000Init(2.0f);

  MibSPIInit(3, kSpiPinmuxMiso);

  // Setup Gio for GDB power good detection.
  // Must be called after MibSPIInit(3, ...) as it uses pins from MibSPI(3).
  GioInit();
  MotorGdbPowerGoodInit(g_motor_controller_type);

  // Select TMS570 as controller of position sensor.
  MotorPositionControlInit(g_motor_controller_type);

  MotorAngleMeas angle_meas;
  MotorAngleMeasInit(&angle_meas);
  MotorAngleInit(g_svpwm_isr_period, &angle_meas, g_motor_params,
                 kMotorCalibParams);

  FocInit(g_svpwm_isr_period, g_motor_params, kMotorConfigParams->motor_type,
          g_motor_controller_type);
  StackingInit(MOTOR_AIO_PERIOD, g_motor_controller_type);
  MotorCurrentLimitInit(MOTOR_AIO_PERIOD, g_svpwm_isr_period);

  MotorThermalInit(kMotorConfigParams->motor_type, g_motor_controller_type);

  SvpwmSetReference(0.0f, 0.0f, 850.0f);
  SvpwmStart();

  // Wait for the NHET to start-up before starting the watchdog, so
  // the first loop isn't longer than later loops and the watchdog
  // won't trigger.
  Clock32WaitCycles(CLOCK32_USEC_TO_CYCLES(50));
  WatchdogInit((int32_t)(1.5f * 1e6f * g_svpwm_isr_period));

  // Enable IRQs and FIQs to begin running the high speed motor controller ISR.
  VimEnableFiq();
  VimEnableIrq();

  uint32_t last_clock_time_cycles = Clock32GetCycles();
  int64_t last_clock_time_us = ClockGetUs();
  CvtInit(last_clock_time_us, AppConfigGetIndex());

  // Wakeup events are out of phase.
  uint32_t wakeup_loop = Clock32GetCycles() + MOTOR_AIO_PERIOD_CYCLES;
  uint32_t wakeup_net_mon = wakeup_loop + MOTOR_AIO_PERIOD_CYCLES / 2;
  uint32_t wakeup_net_probe = wakeup_loop + MOTOR_AIO_PERIOD_CYCLES / 4;
  uint32_t wakeup_ext_watchdog = wakeup_loop + EXT_WATCHDOG_KICK_CYCLES;
  uint32_t wakeup_clock = wakeup_loop + CLOCKGETUS_PERIOD_CYCLES;
  while (true) {
    // Perform IO, performance polling, and miscellaneous tasks. Determination
    // of PWM times is taken care of in an ISR.
    uint32_t now = Clock32GetCycles();
    if (CLOCK32_GT(now, wakeup_loop)) {
      wakeup_loop += MOTOR_AIO_PERIOD_CYCLES;
      Loop(last_clock_time_us +
           (int64_t)CLOCK32_CYCLES_TO_USEC_F(
               CLOCK32_SUBTRACT(now, last_clock_time_cycles)));
      ProfilerNetPollCountSample();
    } else if (CLOCK32_GT(now, wakeup_net_mon)) {
      wakeup_net_mon += MOTOR_AIO_PERIOD_CYCLES;
      NetMonPoll();
    } else if (CLOCK32_GT(now, wakeup_net_probe)) {
      wakeup_net_probe += MOTOR_AIO_PERIOD_CYCLES;
      NetDiagPoll();
    } else if (CLOCK32_GT(now, wakeup_ext_watchdog)) {
      wakeup_ext_watchdog += EXT_WATCHDOG_KICK_CYCLES;
      ExtWatchdogKick();
    } else if (CLOCK32_GT(now, wakeup_clock)) {
      wakeup_clock += CLOCKGETUS_PERIOD_CYCLES;
      last_clock_time_us = ClockGetUs();
      last_clock_time_cycles = now;
    } else {
      Bcm53101Poll(GetSwitchConfig());
      int32_t data_len = NetPollUdp(0, NULL, NULL, NULL, NULL, NULL);
      // Count extra calls to NetPoll.
      if (data_len == 0)
        ProfilerNetPollCount();
    }
  }
  return 0;
}

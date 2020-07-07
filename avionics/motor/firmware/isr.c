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

#include "avionics/motor/firmware/isr.h"

#include <signal.h>
#include <stdint.h>

#include "avionics/common/fast_math/filter.h"
#include "avionics/firmware/cpu/registers.h"
#include "avionics/firmware/cpu/watchdog.h"
#include "avionics/firmware/drivers/led.h"
#include "avionics/motor/firmware/adc.h"
#include "avionics/motor/firmware/angle.h"
#include "avionics/motor/firmware/angle_meas.h"
#include "avionics/motor/firmware/calibrator.h"
#include "avionics/motor/firmware/config_params.h"
#include "avionics/motor/firmware/current_limit.h"
#include "avionics/motor/firmware/flags.h"
#include "avionics/motor/firmware/errors.h"
#include "avionics/motor/firmware/foc.h"
#include "avionics/motor/firmware/gio.h"
#include "avionics/motor/firmware/io.h"
#include "avionics/motor/firmware/isr_diag.h"
#include "avionics/motor/firmware/logging.h"
#include "avionics/motor/firmware/main.h"
#include "avionics/motor/firmware/profiler.h"
#include "avionics/motor/firmware/stacking.h"
#include "avionics/motor/firmware/svpwm.h"
#include "avionics/motor/firmware/util.h"
#include "common/barrier.h"

MotorLimits g_motor_limits = {
  // Maximum magnitude of the bus current, phase current, and phase current sum
  // [A], checked in the motor control loop.
  .bus_current_limit = 50.0f,
  .phase_current_limit = 75.0f,
  .phase_current_sum_limit = 60.0f,

  // Maximum magnitude of the current [A], checked by the ADC
  // magnitude interrupt.
  .adc_fault_current_limit = 300.0f,

  // Minimum and maximum bus voltage [V].
  .v_bus_lower_limit = 200.0f,
  .v_bus_upper_limit = 960.0f,

  // Maximum magnitude of mechanical angular rate [rad/s].
  .omega_limit = 250.0f,
};

// Double buffering for ADC maginitude interrupt reporting.
sig_atomic_t g_adc_magnitude_errors_buffer_index = 0;
static uint32_t g_adc_magnitude_errors_buffer[2] = {0};

// Double buffering for GDB PGOOD interrupt reporting.
sig_atomic_t g_gdb_power_good_buffer_index = 0;
static uint32_t g_gdb_power_good_buffer[2] = {0};

// Double buffer ISR input and output for thread safety.
static sig_atomic_t g_isr_input_buffer_index = 0;
static MotorIsrInput g_isr_input_buffer[2] = {{0}};

static sig_atomic_t g_isr_raw_data_buffer_index = 0;
static MotorIsrRawData g_isr_raw_data_buffer[2] = {{0}};

static MotorIsrOutput g_isr_output = {0};

static uint32_t MotorIsrReadAdcMagnitudeErrorsBuffer(void) {
  sig_atomic_t old_index = g_adc_magnitude_errors_buffer_index;
  g_adc_magnitude_errors_buffer_index = !old_index;

  // The memory barrier here prevents the compiler from moving the buffer swap
  // after the buffer read and write. Just using volatile indices does not
  // necessarily guarantee order.
  MemoryBarrier();

  uint32_t errors = g_adc_magnitude_errors_buffer[old_index];

  g_adc_magnitude_errors_buffer[old_index] = kMotorErrorNone;

  return errors;
}

static void MotorIsrWriteAdcMagnitudeErrorsBuffer(uint32_t errors) {
  g_adc_magnitude_errors_buffer[g_adc_magnitude_errors_buffer_index] |= errors;
}

static uint32_t MotorIsrReadGdbPowerGoodBuffer(void) {
  sig_atomic_t old_index = g_gdb_power_good_buffer_index;
  g_gdb_power_good_buffer_index = !old_index;

  // The memory barrier here prevents the compiler from moving the buffer swap
  // after the buffer read and write.
  MemoryBarrier();

  uint32_t warnings = g_gdb_power_good_buffer[old_index];

  g_gdb_power_good_buffer[old_index] = kMotorWarningNone;

  return warnings;
}

static void MotorIsrWriteGdbPowerGoodBuffer(uint32_t warnings) {
  g_gdb_power_good_buffer[g_gdb_power_good_buffer_index] |= warnings;
}

// Get a pointer to the ISR's input buffer.
static const MotorIsrInput *MotorIsrGetIsrInputBuffer(void) {
  return &g_isr_input_buffer[g_isr_input_buffer_index];
}

// Get a pointer to the ISR input buffer not presently being used by the ISR.
MotorIsrInput *MotorIsrGetIoInputBuffer(void) {
  return &g_isr_input_buffer[!g_isr_input_buffer_index];
}

// Swap the roles of the ISR input buffers.
void MotorIsrSwapInputBuffers(void) {
  // The barriers here are presently not needed; gcc emits discrete code for
  // this function which guarantees order with surrounding code. This will not
  // necessarily be true if a compiler with interprocedural analysis is used or
  // the calling code is moved into this file.
  MemoryBarrier();
  g_isr_input_buffer_index = !g_isr_input_buffer_index;
  MemoryBarrier();
}

// Get a pointer to the ISR's raw data buffer.
static MotorIsrRawData *MotorIsrGetIsrRawDataBuffer(void) {
  return &g_isr_raw_data_buffer[g_isr_raw_data_buffer_index];
}

// Swap the roles of the ISR output buffers and return a pointer to the buffer
// that was previously being used by the ISR.
static const MotorIsrRawData *MotorIsrSwapRawDataBuffers(void) {
  // The barriers are presently not needed; they are included for compatibility
  // with interprocedural optimization.
  MemoryBarrier();
  sig_atomic_t last_index = g_isr_raw_data_buffer_index;
  g_isr_raw_data_buffer_index = !g_isr_raw_data_buffer_index;
  MemoryBarrier();
  return &g_isr_raw_data_buffer[last_index];
}

// Run filtering on ISR raw data and return pointer to the output buffer.
const MotorIsrOutput *MotorIsrFilterRawData(void) {
  // Get pointer to raw data buffer and perform buffer swap.
  const MotorIsrRawData *isr_raw_data = MotorIsrSwapRawDataBuffers();

  // Pass non-averaged variables through to g_isr_output unmodified.
  g_isr_output.errors = isr_raw_data->errors;
  g_isr_output.warnings = isr_raw_data->warnings;
  g_isr_output.num_samples = isr_raw_data->num_samples;
  g_isr_output.theta = isr_raw_data->theta;
  g_isr_output.angle_sensor = isr_raw_data->angle_sensor;
  g_isr_output.current_limit_data = isr_raw_data->current_limit_data;
  g_isr_output.omega_upper_limit = isr_raw_data->omega_upper_limit;
  g_isr_output.omega_lower_limit = isr_raw_data->omega_lower_limit;
  g_isr_output.torque_cmd = isr_raw_data->torque_cmd;
  g_isr_output.speed_correction = isr_raw_data->speed_correction;
  g_isr_output.voltage_pair_bias = isr_raw_data->voltage_pair_bias;

  // Pre-compute averaging factor based on number of ISR samples.
  float inv_isr_num_samples = 1.0f;  // Guard against a divide by zero.
  if (isr_raw_data->num_samples != 0) {
    inv_isr_num_samples = 1.0f / (float)isr_raw_data->num_samples;
  }

  // Compute average quantities of some ISR variables.
  g_isr_output.i_bus = isr_raw_data->i_bus_sum * inv_isr_num_samples;
  g_isr_output.v_bus = isr_raw_data->v_bus_sum * inv_isr_num_samples;
  g_isr_output.v_chassis = isr_raw_data->v_chassis_sum * inv_isr_num_samples;
  g_isr_output.v_cm = isr_raw_data->v_cm_sum * inv_isr_num_samples;
  g_isr_output.v_in_mon = isr_raw_data->v_in_mon_sum * inv_isr_num_samples;
  g_isr_output.v_aux_mon = isr_raw_data->v_aux_mon_sum * inv_isr_num_samples;
  g_isr_output.omega = isr_raw_data->omega_sum * inv_isr_num_samples;
  g_isr_output.id = isr_raw_data->id_sum * inv_isr_num_samples;
  g_isr_output.id_cmd = isr_raw_data->id_cmd_sum * inv_isr_num_samples;
  g_isr_output.iq = isr_raw_data->iq_sum * inv_isr_num_samples;
  g_isr_output.iq_cmd = isr_raw_data->iq_cmd_sum * inv_isr_num_samples;
  g_isr_output.vd = isr_raw_data->vd_sum * inv_isr_num_samples;
  g_isr_output.vq = isr_raw_data->vq_sum * inv_isr_num_samples;
  g_isr_output.current_correction
      = isr_raw_data->current_correction_sum * inv_isr_num_samples;
  g_isr_output.voltage_stack_mean
      = isr_raw_data->voltage_stack_mean_sum * inv_isr_num_samples;

  return &g_isr_output;
}

static void CheckElectricalLimits(MotorMode mode, const MotorState *state,
                                  const StackingIoInput *stacking,
                                  uint32_t *errors, uint32_t *warnings) {
  // Check for over-current.
  if (state->ia > g_motor_limits.phase_current_limit) {
    GateDriverDisable();
    *errors |= kMotorErrorOverCurrentIaP;
  }
  if (state->ia < -g_motor_limits.phase_current_limit) {
    GateDriverDisable();
    *errors |= kMotorErrorOverCurrentIaN;
  }
  if (state->ib > g_motor_limits.phase_current_limit) {
    GateDriverDisable();
    *errors |= kMotorErrorOverCurrentIbP;
  }
  if (state->ib < -g_motor_limits.phase_current_limit) {
    GateDriverDisable();
    *errors |= kMotorErrorOverCurrentIbN;
  }
  if (state->ic > g_motor_limits.phase_current_limit) {
    GateDriverDisable();
    *errors |= kMotorErrorOverCurrentIcP;
  }
  if (state->ic < -g_motor_limits.phase_current_limit) {
    GateDriverDisable();
    *errors |= kMotorErrorOverCurrentIcN;
  }
  if (state->i_bus > g_motor_limits.bus_current_limit) {
    GateDriverDisable();
    *errors |= kMotorErrorOverCurrentIBusP;
  }
  if (state->i_bus < -g_motor_limits.bus_current_limit) {
    GateDriverDisable();
    *errors |= kMotorErrorOverCurrentIBusN;
  }

  // Check for violation of Kirchhoff's current law.
  if (fabsf(state->ia + state->ib + state->ic) >
      g_motor_limits.phase_current_sum_limit) {
    *warnings |= kMotorWarningPhaseCurrentSum;
  }

  // Check for over-voltage.
  if (state->v_bus > g_motor_limits.v_bus_upper_limit) {
    GateDriverDisable();
    *errors |= kMotorErrorOverVoltage;
  }

  // Check for under-voltage if running.
  if ((mode == kMotorModeRunning || mode == kMotorModeErrorWindDown)
      && state->v_bus < g_motor_limits.v_bus_lower_limit
      && (stacking->voltage_stack_mean >=  // Allow airbrakes.
          g_stacking_params.uv_error_enable_threshold
          || !StackingIsEnabled())) {
    GateDriverDisable();
    *errors |= kMotorErrorUnderVoltage;
  }
}

static uint32_t CheckSpeedLimits(float omega_mech) {
  if (omega_mech > g_motor_limits.omega_limit
      || omega_mech < -g_motor_limits.omega_limit) {
    GateDriverDisable();
    return kMotorErrorOverSpeed;
  }
  return kMotorErrorNone;
}

static uint32_t FocController(const MotorIsrInput *input,
                              MotorState *motor_state,
                              MotorAngleMeas *angle_meas,
                              CurrentLimit *speed_current_limit,
                              FocCurrent *foc_current_cmd,
                              float *current_correction) {
  uint32_t errors = 0;

  MotorAngleMeasGet(angle_meas);
  MotorAngleUpdate(angle_meas, g_motor_params,
                   &motor_state->theta_mech, &motor_state->omega_mech);
  motor_state->theta_elec = ModAngle(motor_state->theta_mech *
                                     g_motor_params->num_pole_pairs_elec);
  errors |= CheckSpeedLimits(motor_state->omega_mech);

  float omega_upper_limit = input->omega_upper_limit;
  float omega_lower_limit = input->omega_lower_limit;

  // Reset the field-oriented controller if we're not running.
  if (input->mode != kMotorModeRunning
      && input->mode != kMotorModeErrorWindDown) {
    FocReset(g_svpwm_isr_period);
    omega_upper_limit = motor_state->omega_mech;
    omega_lower_limit = motor_state->omega_mech;
    StackingReset(motor_state->omega_mech);
  }

  // Run the speed correction loop if stacking is enabled or directly use the
  // omega limits.
  if (StackingIsEnabled()) {
    StackingSpeedCorrection(&omega_upper_limit, &omega_lower_limit,
                            &input->stacking);
  }

  CurrentLimitInput stacking_current_limit;
  MotorCurrentLimitGet(&input->current_limit_input, motor_state,
                       &stacking_current_limit, speed_current_limit);

  // Run field-oriented controller.
  FocSpeedLoop(omega_upper_limit, omega_lower_limit, input->torque_cmd,
               motor_state->omega_mech, speed_current_limit, foc_current_cmd);

  // In WindDown mode, quickly ramp down speed controller's contribution to the
  // current command, so that after a short transition time, the stacking
  // controller alone determines the motor current commands.
  if (input->mode == kMotorModeErrorWindDown) {
    foc_current_cmd->iq = StackingCurrentWindDown(foc_current_cmd->iq);
    foc_current_cmd->id = 0.0f;  // TODO: Ramp down like iq.
    // Enable transition back out of wind-down once fault has been taken
    // care of by the short-stack: don't let omega error wind up omega_int.
    FocSetOmegaInt(foc_current_cmd->iq);
  }

  *current_correction = 0.0f;
  if (StackingIsEnabled()) {
    // TODO: Make sure this code is consistent with flux
    // weakening.
    foc_current_cmd->iq = StackingCurrentCorrection(foc_current_cmd->iq,
                                                    motor_state->v_bus,
                                                    &input->stacking,
                                                    &stacking_current_limit,
                                                    current_correction);
  }

  return errors;
}

// The fast loop (15 kHz), which runs at half the PWM frequency,
// handles running the state estimator and controller and setting the
// PWM output.  Because this is run as an ADC interrupt, communication
// with the slow loop is performed using a double buffer in
// motor_io.c.
void MotorIsrControlLoop(void) {
  // Set profiler and LED pin for debugging.
  ProfilerIsrTic();
  LedOn(true, false);

  static MotorMode last_mode = kMotorModeInit;
  static MotorIsrRawData *last_raw_data = NULL;
  static uint32_t errors = kMotorErrorNone;
  static uint32_t warnings = kMotorWarningNone;

  // Get commands.
  const MotorIsrInput *input = MotorIsrGetIsrInputBuffer();

  // Clear errors. New errors and warnings will be missed in the 1kHz loop
  // during the time from when the clear_errors flag is set to when it is reset.
  // TODO: Eliminate this blind period.
  if ((input->mode == kMotorModeInit || input->mode == kMotorModeErrorDisarmed)
      && input->clear_errors) {
    errors = kMotorErrorNone;
    warnings = kMotorWarningNone;
  } else if (input->clear_warnings_mask) {
    warnings = warnings & ~input->clear_warnings_mask;
  }

  CurrentLimit current_limit;
  FocCurrent foc_current_cmd;
  FocCurrent foc_current;
  FocVoltage foc_voltage_dq;
  FocVoltage foc_voltage_ab;
  MotorAngleMeas angle_meas;
  MotorState motor_state;

  float current_correction;

  // Poll sensor data.
  MotorAdcGetValues(&motor_state.ia, &motor_state.ib, &motor_state.ic,
                    &motor_state.i_bus, &motor_state.v_bus,
                    &motor_state.v_chassis, &motor_state.v_cm,
                    &motor_state.v_in_mon, &motor_state.v_aux_mon);

  // Check electrical limits and run the angle / speed estimator.
  CheckElectricalLimits(input->mode, &motor_state, &input->stacking,
                        &errors, &warnings);
  if (StackingIsEnabled()) {
    errors |= StackingCheckVoltage(motor_state.v_bus, &input->stacking);
  }

  if (kMotorConfigParams->calibrator_enable == kMotorCalibratorEnabled) {
    errors |= CalibrationController(input, &motor_state, &angle_meas,
                                    &current_limit, &foc_current_cmd,
                                    &current_correction);
  } else {
    errors |= FocController(input, &motor_state, &angle_meas, &current_limit,
                            &foc_current_cmd, &current_correction);
  }

  // Command zero torque if we're not running.
  if (input->mode != kMotorModeRunning
      && input->mode != kMotorModeErrorWindDown) {
    foc_current_cmd.id = 0.0f;
    foc_current_cmd.iq = 0.0f;
    foc_current_cmd.i0 = 0.0f;
  }

  FocCurrentLoop(&foc_current_cmd, &motor_state, &foc_current, &foc_voltage_dq);

  // Set PWM output.
  foc_voltage_ab.v_ref = foc_voltage_dq.v_ref;
  foc_voltage_ab.angle = foc_voltage_dq.angle + motor_state.theta_elec;
  SvpwmSetReference(foc_voltage_ab.angle, foc_voltage_ab.v_ref,
                    motor_state.v_bus);

  if (angle_meas.updated && !angle_meas.valid) {
    warnings |= kMotorWarningAngleSensorReadError;
  }
  if ((input->mode == kMotorModeRunning && errors == kMotorErrorNone)
      || (input->mode == kMotorModeErrorWindDown && !IsCriticalError(errors))) {
    // Only enable the gate driver on the first loop that it's
    // running.  It should be hard to enable the motors!
    if (last_mode == kMotorModeArmed) GateDriverEnable();
  } else {
    GateDriverDisable();
  }
  last_mode = input->mode;

  // Report ADC magnitude interrupt errors and reenable interrupts.
  errors |= MotorIsrReadAdcMagnitudeErrorsBuffer();
  MotorAdcMagnitudeInterruptEnable();

  // Report GDB PGOOD interrupt errors.
  warnings |= MotorIsrReadGdbPowerGoodBuffer();

  // Update averaged ISR variables..
  MotorIsrRawData *isr_raw_data = MotorIsrGetIsrRawDataBuffer();
  if (isr_raw_data != last_raw_data) {
    last_raw_data = isr_raw_data;
    isr_raw_data->num_samples = 0;
    isr_raw_data->i_bus_sum = 0.0f;
    isr_raw_data->v_bus_sum = 0.0f;
    isr_raw_data->v_chassis_sum = 0.0f;
    isr_raw_data->v_cm_sum = 0.0f;
    isr_raw_data->v_in_mon_sum = 0.0f;
    isr_raw_data->v_aux_mon_sum = 0.0f;
    isr_raw_data->omega_sum = 0.0f;
    isr_raw_data->id_sum = 0.0f;
    isr_raw_data->id_cmd_sum = 0.0f;
    isr_raw_data->iq_sum = 0.0f;
    isr_raw_data->iq_cmd_sum = 0.0f;
    isr_raw_data->vd_sum = 0.0f;
    isr_raw_data->vq_sum = 0.0f;
    isr_raw_data->current_correction_sum = 0.0f;
    isr_raw_data->voltage_stack_mean_sum = 0.0f;
  }
  ++isr_raw_data->num_samples;
  isr_raw_data->i_bus_sum += motor_state.i_bus;
  isr_raw_data->v_bus_sum += motor_state.v_bus;
  isr_raw_data->v_chassis_sum += motor_state.v_chassis;
  isr_raw_data->v_cm_sum += motor_state.v_cm;
  isr_raw_data->v_in_mon_sum += motor_state.v_in_mon;
  isr_raw_data->v_aux_mon_sum += motor_state.v_aux_mon;
  isr_raw_data->omega_sum += motor_state.omega_mech;
  isr_raw_data->id_sum += foc_current.id;
  isr_raw_data->id_cmd_sum += foc_current_cmd.id;
  isr_raw_data->iq_sum += foc_current.iq;
  isr_raw_data->iq_cmd_sum += foc_current_cmd.iq;
  isr_raw_data->vd_sum += foc_voltage_dq.vd;
  isr_raw_data->vq_sum += foc_voltage_dq.vq;
  isr_raw_data->current_correction_sum += current_correction;
  isr_raw_data->voltage_stack_mean_sum += input->stacking.voltage_stack_mean;

  // Update non-averaged ISR variables.
  isr_raw_data->errors = errors;
  isr_raw_data->warnings = warnings;
  isr_raw_data->theta = motor_state.theta_mech;
  MotorAngleSensorDiag(&isr_raw_data->angle_sensor);
  isr_raw_data->omega_upper_limit = input->omega_upper_limit;
  isr_raw_data->omega_lower_limit = input->omega_lower_limit;
  isr_raw_data->torque_cmd = input->torque_cmd;
  isr_raw_data->speed_correction =
      FirstOrderFilterGet(&g_stacking_isr_state.speed_correction_filter);
  isr_raw_data->voltage_pair_bias =
      FirstOrderFilterGet(&g_stacking_io_state.voltage_pair_bias_filter);

  // Calculate and filter various quantities used by the current limit code.
  MotorCurrentLimitIsrUpdate(&foc_current, &motor_state, &current_limit,
                             &isr_raw_data->current_limit_data);

  // Log ADC and control data if running or recently stopped.
  static int32_t runs_since_stop = 0;
  if ((input->mode == kMotorModeRunning && errors == kMotorErrorNone)
      || runs_since_stop < MOTOR_LOG_LEN / 5) {
    runs_since_stop++;

    MotorLogAdcData();
    MotorLogControlData(
        &motor_state, GetFocState(), &foc_current,
        &foc_current_cmd, &foc_voltage_dq, input->torque_cmd,
        input->omega_upper_limit, input->omega_lower_limit,
        errors | input->errors, warnings | input->warnings);
  }
  if (input->mode == kMotorModeRunning && errors == kMotorErrorNone) {
    runs_since_stop = 0;
  }

  // Log the state for the high speed ISR Diagnostic output (if enabled).
  IsrDiagLog(errors | input->errors, warnings | input->warnings,
             &angle_meas, &motor_state, &foc_voltage_ab);

  // Kick watchdog.
  WatchdogPoll();

  // Clear LED pin and profiler for debugging.
  LedOff(true, false);
  ProfilerIsrToc();

  // Indicate that the ISR is finished.
  DMA.BTCFLAG.BTCI11 = 1;
}

// Shutdown motor if the ADC magnitude interrupt is activated.
void MotorAdcMagnitudeInterruptHandler(void) {
  GateDriverDisable();

  MotorIsrWriteAdcMagnitudeErrorsBuffer(MotorAdcGetMagnitudeErrors());
}

// Shutdown motor if a GDB PGOOD signal goes low.
void MotorGdbPowerGoodInterruptHandler(void) {
  MotorIsrWriteGdbPowerGoodBuffer(MotorGdbProcessPowerGoodInterrupt());
}

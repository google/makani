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

#include "avionics/motor/firmware/logging.h"

#include <signal.h>
#include <stdint.h>

#include "avionics/common/avionics_messages.h"
#include "avionics/common/motor_adc_defines.h"
#include "avionics/common/motor_foc_types.h"
#include "avionics/firmware/cpu/memcpy.h"
#include "avionics/firmware/network/net_send.h"
#include "avionics/firmware/serial/motor_serial_params.h"
#include "avionics/motor/firmware/adc.h"
#include "common/barrier.h"

static Adc1PackedData g_adc_1_log;
static Adc2PackedData g_adc_2_log;

static MotorState g_motor_state_log[MOTOR_LOG_LEN];
static FocState   g_foc_state_log[MOTOR_LOG_LEN];
static FocCurrent g_foc_current_actual_log[MOTOR_LOG_LEN];
static FocCurrent g_foc_current_desired_log[MOTOR_LOG_LEN];
static FocVoltage g_foc_voltage_log[MOTOR_LOG_LEN];
static float      g_torque_cmd_log[MOTOR_LOG_LEN];
static float      g_omega_upper_limit_log[MOTOR_LOG_LEN];
static float      g_omega_lower_limit_log[MOTOR_LOG_LEN];
static uint32_t   g_errors_log[MOTOR_LOG_LEN];
static uint32_t   g_warnings_log[MOTOR_LOG_LEN];

static sig_atomic_t g_sending_adc_data = 0;
static sig_atomic_t g_sending_control_data = 0;

static int32_t g_log_ind = 0;

static MotorHardware g_motor_controller_type = 0;

void MotorLogInit(int32_t motor_controller_type) {
  g_motor_controller_type = (MotorHardware)motor_controller_type;
}

void MotorLogAdcData(void) {
  if (!g_sending_adc_data) {
    // Copy ADC packed structures.
    // FastCopy is used here to drop the execution time from 2.9 us to 1.9 us
    // compared to the default struct copy. The cast-qual warning is disabled so
    // that the volatile attribute on the adc buffers can be discarded.
#pragma GCC diagnostic ignored "-Wcast-qual"
    FastCopy(sizeof(g_adc_1_log), (void *)&g_adc_1_buffer, &g_adc_1_log);
    FastCopy(sizeof(g_adc_2_log), (void *)&g_adc_2_buffer, &g_adc_2_log);
#pragma GCC diagnostic pop
  }
}

void MotorLogAdcSend(void) {
  MotorAdcLogMessage log_data;

  // Lock the ADC buffers while copying. A memory barrier is used to prevent the
  // compiler from moving the lock into the data copy code.
  g_sending_adc_data = 1;
  MemoryBarrier();

  if (g_motor_controller_type == kMotorHardwareOzoneA1) {
    for (int32_t i = 0; i < NUM_ADC_SAMPLES; ++i) {
      log_data.v_in_monitor[i]        = g_adc_1_log.ozone_v_in_monitor[i].DR;
      log_data.chassis_voltage[i]     = g_adc_1_log.ozone_v_chassis[i].DR;
      log_data.phase_a_current[i]     = g_adc_1_log.ozone_i_phase_a[i].DR;
      log_data.phase_b_current[i]     = g_adc_1_log.ozone_i_phase_b[i].DR;
      log_data.bus_current[i]         = g_adc_1_log.ozone_i_bus[i].DR;
      log_data.cm_voltage[i]          = g_adc_2_log.ozone_v_cm[i].DR;
      log_data.phase_c_current[i]     = g_adc_2_log.ozone_i_phase_c[i].DR;
      log_data.phase_b_aux_current[i] = g_adc_2_log.ozone_i_phase_b[i].DR;
      log_data.v_aux_monitor[i]       = g_adc_2_log.ozone_v_aux_monitor[i].DR;
      log_data.bus_voltage[i]         = g_adc_2_log.ozone_v_bus[i].DR;
    }
  } else {
    for (int32_t i = 0; i < NUM_ADC_SAMPLES; ++i) {
      log_data.v_in_monitor[i]        = g_adc_1_log.gin_v_in_monitor[i].DR;
      log_data.phase_a_current[i]     = g_adc_1_log.gin_i_phase_a[i].DR;
      log_data.phase_b_current[i]     = g_adc_1_log.gin_i_phase_b[i].DR;
      log_data.bus_current[i]         = g_adc_1_log.gin_i_bus[i].DR;
      log_data.chassis_voltage[i]     = 0;
      log_data.bus_voltage[i]         = g_adc_2_log.gin_v_bus[i].DR;
      log_data.phase_c_current[i]     = g_adc_2_log.gin_i_phase_c[i].DR;
      log_data.phase_b_aux_current[i] = g_adc_2_log.gin_i_phase_b[i].DR;
      log_data.v_aux_monitor[i]       = g_adc_2_log.gin_v_aux_monitor[i].DR;
      log_data.cm_voltage[i]          = 0;
    }
  }

  MemoryBarrier();
  g_sending_adc_data = 0;

  NetSendAioMotorAdcLogMessage(&log_data);
}

void MotorLogControlData(const MotorState *motor_state,
                         const FocState *foc_state,
                         const FocCurrent *foc_current_actual,
                         const FocCurrent *foc_current_desired,
                         const FocVoltage *foc_voltage,
                         float torque_cmd,
                         float omega_upper_limit,
                         float omega_lower_limit,
                         uint32_t errors,
                         uint32_t warnings) {
  if (!g_sending_control_data) {
    // Copy controller structures.
    g_motor_state_log[g_log_ind]         = *motor_state;
    g_foc_state_log[g_log_ind]           = *foc_state;
    g_foc_current_actual_log[g_log_ind]  = *foc_current_actual;
    g_foc_current_desired_log[g_log_ind] = *foc_current_desired;
    g_foc_voltage_log[g_log_ind]         = *foc_voltage;
    g_torque_cmd_log[g_log_ind]          = torque_cmd;
    g_omega_upper_limit_log[g_log_ind]   = omega_upper_limit;
    g_omega_lower_limit_log[g_log_ind]   = omega_lower_limit;
    g_errors_log[g_log_ind]              = errors;
    g_warnings_log[g_log_ind]            = warnings;

    g_log_ind = (g_log_ind + 1) % MOTOR_LOG_LEN;
  }
}

void MotorLogControlSend(bool start_send) {
  static int32_t send_ind = 0;

  if (start_send || send_ind > 0) {
    MotorIsrLogMessage log_data;

    // Lock the control data buffer while copying. A memory barrier is used to
    // prevent the compiler from moving the lock into the data copy code.
    g_sending_control_data = 1;
    MemoryBarrier();

    int32_t i = (send_ind + g_log_ind) % MOTOR_LOG_LEN;

    // Copy controller structures.
    log_data.timestep = send_ind;

    // MotorState
    log_data.motor_state_ia         = g_motor_state_log[i].ia;
    log_data.motor_state_ib         = g_motor_state_log[i].ib;
    log_data.motor_state_ic         = g_motor_state_log[i].ic;
    log_data.motor_state_v_bus      = g_motor_state_log[i].v_bus;
    log_data.motor_state_i_bus      = g_motor_state_log[i].i_bus;
    log_data.motor_state_theta_elec = g_motor_state_log[i].theta_elec;
    log_data.motor_state_omega_mech = g_motor_state_log[i].omega_mech;

    // FocState
    log_data.foc_state_id_int    = g_foc_state_log[i].id_int;
    log_data.foc_state_iq_int    = g_foc_state_log[i].iq_int;
    log_data.foc_state_id_error  = g_foc_state_log[i].id_error;
    log_data.foc_state_iq_error  = g_foc_state_log[i].iq_error;
    log_data.foc_state_omega_int = g_foc_state_log[i].omega_int;

    // FocCurrent
    log_data.foc_current_actual_id = g_foc_current_actual_log[i].id;
    log_data.foc_current_actual_iq = g_foc_current_actual_log[i].iq;
    log_data.foc_current_actual_i0 = g_foc_current_actual_log[i].i0;

    // FocCurrent
    log_data.foc_current_desired_id = g_foc_current_desired_log[i].id;
    log_data.foc_current_desired_iq = g_foc_current_desired_log[i].iq;
    log_data.foc_current_desired_i0 = g_foc_current_desired_log[i].i0;

    // FocVoltage
    log_data.foc_voltage_vd    = g_foc_voltage_log[i].vd;
    log_data.foc_voltage_vq    = g_foc_voltage_log[i].vq;
    log_data.foc_voltage_v_ref = g_foc_voltage_log[i].v_ref;
    log_data.foc_voltage_angle = g_foc_voltage_log[i].angle;

    // Motor commands
    log_data.torque_cmd        = g_torque_cmd_log[i];
    log_data.omega_upper_limit = g_omega_upper_limit_log[i];
    log_data.omega_lower_limit = g_omega_lower_limit_log[i];

    // Motor errors and warnings
    log_data.errors = g_errors_log[i];
    log_data.warnings = g_warnings_log[i];

    NetSendAioMotorIsrLogMessage(&log_data);

    send_ind++;
    if (send_ind >= MOTOR_LOG_LEN) {
      send_ind = 0;

      // Unlock the buffer after all other operations have finished.
      MemoryBarrier();
      g_sending_control_data = 0;
    }
  }
}

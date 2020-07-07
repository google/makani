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

// The motor stacking code implements a feedback control system designed to
// stabilize voltages when running pairs of motors in series. The high level
// block diagram is shown below.
//
// i_corr_mean ----------.
//                       |
//                       V -   _______
// i_corr_local_mean --> o -->| G2(z) |---.       Speed
//                 +           -------    |     Controller
//                 _______                V +    _______    +
//  omega_cmd --->| G1(z) | ------------> o --->| G0(z) |---> o -----> i_q
//                 -------           +           -------      ^  +
//                                                            |
//                               V_ave -----.                 |
//                                          |                 |
//                                 +        V -    _______    |
//  V_local -----------------------> o ---> o --->| G4(z) |---' i_corr_local
//                 |                 ^ -   +       -------
//                 V +    _______    |
//  V_pair ------> o --->| G3(z) |---'
//               -        -------      Voltage pair bias
//
// Most of the work is done by the voltage control filter (G4) which is used as
// an integrator. Positive perturbations in local voltage end up producing
// increased commanded current which pulls the voltage back down. Filter G3 is
// present to estimate the measurement bias between the local motor and its
// pair which prevents the voltage controller introducing a steady state power
// difference.
//
// The speed controller (G0) is implemented in motor_foc.c. The commanded
// rotational velocity, omega_cmd, is filtered by the speed command filter (G1)
// to prevent high frequency changes from appearing on iq which could induce
// moderate voltage imbalances when coupled with communication delays. The
// speed correction filter (G2) is present to keep the integrators in G0 and G4
// from fighting each other.

#ifndef AVIONICS_MOTOR_FIRMWARE_STACKING_H_
#define AVIONICS_MOTOR_FIRMWARE_STACKING_H_

#include <stdint.h>
#include <stdbool.h>

#include "avionics/common/fast_math/filter.h"
#include "avionics/firmware/serial/motor_serial_params.h"
#include "avionics/network/aio_labels.h"
#include "avionics/motor/firmware/current_limit.h"

typedef struct {
  float speed_cmd_gain;
  float speed_cmd_zero;
  float speed_cmd_pole;
  float speed_cmd_lower_limit;
  float speed_cmd_upper_limit;
  float speed_cmd_lower_rate_limit;
  float speed_cmd_upper_rate_limit;

  float voltage_pair_bias_gain;
  float voltage_pair_bias_zero;
  float voltage_pair_bias_pole;
  float voltage_pair_bias_limit;

  float voltage_control_gain;
  float voltage_control_zero;
  float voltage_control_pole;
  float voltage_control_aw_pole;

  float speed_correction_gain;
  float speed_correction_zero;
  float speed_correction_pole;
  float speed_correction_aw_pole;

  float iq_cmd_limit_offset;
  float iq_cmd_limit_rate;
  float iq_cmd_limit_omega_gen;
  float gen_limit_dicorr_diqcmd;
  float gen_limit_dwcorr_diqcmd;
  float gen_limit_wcorr_offset;

  float voltage_average_upper_sat;
  float uv_error_enable_threshold;

  // shorted_level_run_enable and stacking_enable need to be floats in
  // order to set them in motor_io.c. Otherwise, they really should be booleans.
  float shorted_level_run_enable;  // Run with n-2 motors when level shorts.
  float stacking_enable;
  float stacking_spin_direction;
  float v_delta_averaging_enable;
} StackingParams;

typedef struct {
  FirstOrderFilterState upper_speed_cmd_filter;
  FirstOrderFilterState lower_speed_cmd_filter;
  FirstOrderFilterState speed_correction_filter;
  FirstOrderFilterState voltage_control_filter;
  float upper_speed_cmd_rate_limited;
  float lower_speed_cmd_rate_limited;
  float current_cmd_last;
  float voltage_delta_last;
  bool winding_down;
  float wind_down_current;
} StackingIsrState;

typedef struct {
  FirstOrderFilterState voltage_pair_bias_filter;
  float current_correction_pair_sum[kNumMotors / 2];
  float v_pair_sum[kNumMotors / 2];
} StackingIoState;

typedef struct {
  float voltage_stack_mean;
  float voltage_pair_bias;
  float current_correction_stack_mean;
  float current_correction_pair_mean;
} StackingIoInput;

typedef struct {
  float comm_timeout_cycles;  // Needs to be a float to be mutable.
  float voltage_rel_upper_limit;
  float voltage_rel_lower_limit;
  float wind_down_current_rate;
  float iq_cmd_lower_fault_limit;
} StackingFaultParams;

extern StackingParams g_stacking_params;
extern StackingIsrState g_stacking_isr_state;
extern StackingIoState g_stacking_io_state;
extern StackingFaultParams g_stacking_fault_params;

// Initialize the stacking controller.
//
// Args:
//   dt_io:            The period at which stacking messages are processed and
//                     sent to the ISR, e.g. dt = 1.0e-3 s for a 1 kHz loop.
//   controller_type:  Gin or Ozone.
void StackingInit(float dt_io, MotorHardware controller_type);

// Reset the stacking controller.
// Args:
//   omega_mech:  The mechanical angular velocity [rad/s] to reset the command
//                filter with. Note that this should be the actual angular
//                velocity instead of commanded velocity to minimize jumps in
//                torque.
void StackingReset(float omega_mech);

// Returns the stacking controller torque limits.
//
// Args:
//   omega_mech:     The mechancial angular velocity [rad/s] of the motor.
//   current_limit:  An output structure containing the upper and lower
//                   quadrature current limits.
void StackingCurrentLimit(float omega_mech, CurrentLimitInput *current_limit);

// Returns the corrected mechanical angular rate [rad/s] used for speed control.
//
// Args:
//   omega_upper_cmd: The commanded upper limit of mechanical rotational
//                    velocity of the motor (rad/s).
//   omega_lower_cmd: The commanded lower limit of mechanical rotational
//                    velocity of the motor (rad/s).
//   io_input:        A StackingIoInput structure filled out by
//                    StackingPreprocess.
// Returns:
//   The corrected mechanical rotational velocity command (rad/s).
void StackingSpeedCorrection(float *omega_upper_cmd, float *omega_lower_cmd,
                             const StackingIoInput *io_input);

// Returns the quadrature current [A] used by the FOC current loop with a
// perturbation to keep the stack stable.
//
// Args:
//   current_cmd:         Commanded mechanical angular velocity of local motor.
//   voltage_local:       Present bus voltage of local motor.
//   io_input:            A StackingIoInput structure filled out by
//                        StackingPreprocess.
//   iq_limit:            Upper and lower quadrature current limits.
//   current_correction:  Contains the local current correction upon return.
// Returns:
//   The quadrature current with a voltage correction.
float StackingCurrentCorrection(float current_cmd,
                                float voltage_local,
                                const StackingIoInput *io_input,
                                const CurrentLimitInput *iq_limit,
                                float *current_correction);

// Precompute voltage and current_correction means along with the voltage bias
// from received stacking messages and store the results in io_input. This
// function should be run prior to sending data to the ISR.
//
// Args:
//   voltages:             Array of stack voltages including the local voltage.
//   current_corrections:  Array of stack current corrections including the
//                         local current correction).
//   stale_counts:         Array of counts since last receiving a message from
//                         a motor.
//   stacking_errors:      Array of bitmasked errors.
//   io_input:             A StackingIoInput structure containing preprocessed
//                         values for the ISR.
uint32_t StackingPreprocess(const float voltages[kNumMotors],
                            const float current_corrections[kNumMotors],
                            const int32_t stale_counts[kNumMotors],
                            uint32_t stacking_errors[kNumMotors],
                            StackingIoInput *io_input,
                            uint32_t *block_shorted_mask);

// Returns whether we've enabled running with n-2 motors after level shorts.
static inline bool ShortedLevelRunningEnabled(void) {
  return g_stacking_params.shorted_level_run_enable > 0.5f;
}

// Returns whether stacking is enabled.
static inline bool StackingIsEnabled(void) {
  return g_stacking_params.stacking_enable > 0.5f;
}

// Check for relative voltage errors and set kMotorErrorVoltageRel
// if the local voltage is voltage_rel_upper_limit /
// voltage_rel_lower_limit above / below the stack mean.
//
// Args:
//   local_voltage:  The local measured voltage.
//   voltages:       A kNumMotors length array with other voltages in the stack.
//                   Note: the local element also needs to be updated.
//   errors          A kNumMotors length array of errors in the stack.
// Returns:
//   A bitmask of relative voltage errors.
uint32_t StackingCheckVoltage(float local_voltage,
                              const StackingIoInput *io_input);

// Monotonically decrease current command during wind down.
//
// Args:
//   iq:  Commanded iq when first entering wind down. This argument is ignored
//        in subsequent calls until StackingReset is called.
// Returns:
//   The wind down iq command (without the current correction applied).
float StackingCurrentWindDown(float iq);

#endif  // AVIONICS_MOTOR_FIRMWARE_STACKING_H_

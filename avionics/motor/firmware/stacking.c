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

#include "avionics/motor/firmware/stacking.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "avionics/common/fast_math/fast_math.h"
#include "avionics/common/fast_math/filter.h"
#include "avionics/firmware/identity/identity.h"
#include "avionics/firmware/serial/motor_serial_params.h"
#include "avionics/network/aio_labels.h"
#include "avionics/motor/firmware/config_params.h"
#include "avionics/motor/firmware/current_limit.h"
#include "avionics/motor/firmware/flags.h"
#include "avionics/motor/firmware/errors.h"
#include "avionics/motor/firmware/foc.h"
#include "avionics/motor/firmware/isr.h"
#include "avionics/motor/firmware/svpwm.h"

typedef enum {
  kSpinDirectionNegative = -1,
  kSpinDirectionPositive = 1,
} SpinDirection;

StackingParams g_stacking_params = {
  // Speed command filter and limits.
  .speed_cmd_gain             = 1.0f,
  .speed_cmd_zero             = -INFINITY,             // [rad/s] - Averager.
  .speed_cmd_pole             = -2.0f * PI_F * 16.0f,  // [rad/s]
  .speed_cmd_lower_limit      = -240.0f,               // [rad/s]
  .speed_cmd_upper_limit      = 240.0f,                // [rad/s]
  .speed_cmd_lower_rate_limit = -500.0f,               // [rad/s^2]
  .speed_cmd_upper_rate_limit = 500.0f,                // [rad/s^2]

  // Half of the maximum difference in bus voltage [V] between motors in a
  // block.
  .voltage_pair_bias_gain     = 1.0f,                  // [V/V]
  .voltage_pair_bias_zero     = -INFINITY,             // [rad/s] - Averager.
  .voltage_pair_bias_pole     = -2.0f * PI_F,          // [rad/s]
  .voltage_pair_bias_limit    = 90.0f,                 // [V]

  // Voltage control.
  .voltage_control_gain       = 23.8f,                 // [A/V]
  .voltage_control_zero       = -2.0f * PI_F * 20.0f,  // [rad/s]
  .voltage_control_pole       = -2.0f * PI_F,          // [rad/s]
  .voltage_control_aw_pole    = -1000.0f,              // [rad/s]

  // Speed correction.
  // TODO: Re-examine the speed correction gain: 1/kp?
  .speed_correction_gain      = 1.0f,                  // [(rad/s)/A]
  .speed_correction_zero      = -INFINITY,             // [rad/s] - Averager.
  .speed_correction_pole      = -2.0f * PI_F * 3.0f,   // [rad/s]
  .speed_correction_aw_pole   = -1000.0f,              // [rad/s]

  // Torque limits.
  .iq_cmd_limit_offset        = 25.0f,                 // [A]
  .iq_cmd_limit_rate          = 4.0f,                  // [A/(rad/s)]
  .iq_cmd_limit_omega_gen     = 30.0f,                 // [rad/s]
  .gen_limit_dicorr_diqcmd    = -1.0f,                 // [A/A]
  .gen_limit_dwcorr_diqcmd    = -1.0f,                 // [(rad/s)/A]
  .gen_limit_wcorr_offset     = -3.0f,                 // [rad/s]

  // Upper saturation threshold of the average stack voltage. Past this point,
  // the voltage controller causes motors to go to full power to try and save
  // themselves from an over voltage.
  .voltage_average_upper_sat  = 885.0f,                // [V]

  // If average voltage of levels included in stacking is below this threshold,
  // the undervoltage error will not be set even when voltage falls below
  // v_bus_lower_limit (assuming stacking is enabled). Allows airbrakes.
  .uv_error_enable_threshold  = 400.0f,                // [V]

  // Config param overrides. Separate instances of these parameters are kept
  // around in order to allow run-time modification until flash overlay is
  // ready.
  .shorted_level_run_enable   = 0.0f,
  .stacking_enable            = 0.0f,
  .stacking_spin_direction    = (float)kSpinDirectionPositive,
  .v_delta_averaging_enable   = 1.0f,
};

StackingIsrState g_stacking_isr_state = {
  .upper_speed_cmd_rate_limited = 0.0f,
  .lower_speed_cmd_rate_limited = 0.0f,
  .current_cmd_last = 0.0f,
  .voltage_delta_last = 0.0f,
  .winding_down = false,
  .wind_down_current = 0.0f,  // Is actually always set before being used.
  // ISR filters are initialized in StackingInit().
};

StackingIoState g_stacking_io_state = {
  .current_correction_pair_sum = {0.0f},
  .v_pair_sum = {0.0f},
  // voltage_pair_bias_filter is initialized in StackingInit().
};

// Initialize stacking fault parameters.
StackingFaultParams g_stacking_fault_params = {
  // Communication timeout.
  .comm_timeout_cycles = 20.0f,         // [# of 1 kHz loops]

  // Voltages.
  .voltage_rel_lower_limit = -1000.0f,  // [V]
  .voltage_rel_upper_limit = 200.0f,    // [V]

  // Wind down parameters.
  .wind_down_current_rate = 50000.0f,   // [A / s]
  .iq_cmd_lower_fault_limit = -20.0f    // [A] - Min iq_cmd during wind down.
};

static FirstOrderFilterParams g_speed_cmd_params;
static FirstOrderFilterParams g_speed_correction_params;
static FirstOrderFilterParams g_voltage_control_params;
static FirstOrderFilterParams g_voltage_pair_bias_params;

// Keep track of the current motor's index and its pair's index.
static int32_t g_motor_index;
static int32_t g_pair_index;
static int32_t g_block_index;

// Initialize the stacking filters.
void StackingInit(float dt_io, MotorHardware controller_type) {
  assert(kNumMotors == 8);
  g_motor_index = AppConfigGetIndex();
  assert(0 <= g_motor_index && g_motor_index < kNumMotors);
  g_pair_index = ((uint32_t)(g_motor_index + kNumMotors / 2)) % kNumMotors;
  assert(0 <= g_pair_index && g_pair_index < kNumMotors);
  g_block_index = g_motor_index % (kNumMotors / 2);
  assert(0 <= g_block_index && g_block_index < kNumMotors / 2);

  g_stacking_isr_state.winding_down = false;

  // Initialize stacking filter state.
  FirstOrderFilterInit(g_stacking_params.speed_cmd_gain,
                       g_stacking_params.speed_cmd_zero,
                       g_stacking_params.speed_cmd_pole,
                       g_svpwm_isr_period,
                       0.0f,
                       &g_speed_cmd_params,
                       &g_stacking_isr_state.upper_speed_cmd_filter);
  g_stacking_isr_state.lower_speed_cmd_filter =
      g_stacking_isr_state.upper_speed_cmd_filter;

  FirstOrderFilterInit(g_stacking_params.voltage_pair_bias_gain,
                       g_stacking_params.voltage_pair_bias_zero,
                       g_stacking_params.voltage_pair_bias_pole,
                       dt_io,
                       0.0f,
                       &g_voltage_pair_bias_params,
                       &g_stacking_io_state.voltage_pair_bias_filter);

  FirstOrderFilterInit(g_stacking_params.voltage_control_gain,
                       g_stacking_params.voltage_control_zero,
                       g_stacking_params.voltage_control_pole,
                       g_svpwm_isr_period,
                       0.0f,
                       &g_voltage_control_params,
                       &g_stacking_isr_state.voltage_control_filter);
  FirstOrderWindupInit(g_stacking_params.voltage_control_aw_pole,
                       g_svpwm_isr_period,
                       &g_voltage_control_params);

  FirstOrderFilterInit(g_stacking_params.speed_correction_gain,
                       g_stacking_params.speed_correction_zero,
                       g_stacking_params.speed_correction_pole,
                       g_svpwm_isr_period,
                       0.0f,
                       &g_speed_correction_params,
                       &g_stacking_isr_state.speed_correction_filter);
  FirstOrderWindupInit(g_stacking_params.speed_correction_aw_pole,
                       g_svpwm_isr_period,
                       &g_speed_correction_params);

  // Enable or disable stacking depending on the motor config params. This is
  // only done once to allow stacking to be manually set after startup.
  // TODO: Remove when parameter overlay becomes functional.
  static bool first_pass = true;
  if (first_pass) {
    g_stacking_params.stacking_enable
        = kMotorConfigParams->topology == kMotorBusTopologyStacked;

    // For Ozones, increase saturation voltage threshold above the Gin default.
    if (controller_type == kMotorHardwareOzoneA1) {
      g_stacking_params.voltage_average_upper_sat = 1550.0f;
    }

    switch (kMotorConfigParams->load_type) {
      // Note: the sign conventions of the propeller and motor are presently
      // reversed.
      case kMotorLoadTypePropRev2PositiveX:
        g_stacking_params.stacking_spin_direction
            = (float)kSpinDirectionNegative;
        break;
      case kMotorLoadTypePropRev1NegativeX:  // Fall-through intentional.
      case kMotorLoadTypeDyno:  // Fall-through intentional.
      case kMotorLoadTypeNone:
        g_stacking_params.stacking_spin_direction
            = (float)kSpinDirectionPositive;
        break;
      default:
        assert(false);
        break;
    }
    // Reminder to update the switch statement when additional loads get added.
    assert(kNumMotorLoadTypes == 4);

    first_pass = false;
  }
}

// Reset the stacking filters.
void StackingReset(float omega_mech) {
  // The speed command filter uses present angular velocity to prevent jumps in
  // the torque if omega != omega_cmd.
  FirstOrderFilterReset(omega_mech, &g_speed_cmd_params,
                        &g_stacking_isr_state.upper_speed_cmd_filter);
  FirstOrderFilterReset(omega_mech, &g_speed_cmd_params,
                        &g_stacking_isr_state.lower_speed_cmd_filter);

  // Reset the rate limited speed command state for similar reasons as the
  // speed command filter.
  g_stacking_isr_state.upper_speed_cmd_rate_limited = omega_mech;
  g_stacking_isr_state.lower_speed_cmd_rate_limited = omega_mech;

  // The voltage pair bias filter nominally converges prior to any of the motors
  // turning on; do not reset.

  // The voltage controller is nominally zero and varies quickly; reset to 0.0f.
  FirstOrderFilterReset(0.0f, &g_voltage_control_params,
                        &g_stacking_isr_state.voltage_control_filter);
  g_stacking_isr_state.voltage_delta_last = 0.0f;

  // Because the voltage controller is nominally zero, the speed correction is
  // also nominally zero.
  FirstOrderFilterReset(0.0f, &g_speed_correction_params,
                        &g_stacking_isr_state.speed_correction_filter);

  g_stacking_isr_state.winding_down = false;
}

// Calculate the admissible upper and lower torques based off of rotational
// velocity.
// TODO: This function prevents the stacking controller from
// entering a region of instability found at very low speeds and high torque. It
// would be preferable to make the stacking controller stable under all
// operating conditions.
void StackingCurrentLimit(float omega, CurrentLimitInput *current_limit) {
  float iq_cmd_limit_motor = g_stacking_params.iq_cmd_limit_offset
      + g_stacking_params.iq_cmd_limit_rate * fabsf(omega);
  float iq_cmd_limit_gen
      = Minf(-(fabsf(omega) - fabsf(g_stacking_params.iq_cmd_limit_omega_gen))
             * g_stacking_params.iq_cmd_limit_rate, 0.0f);

  if (StackingIsEnabled()) {
    if (g_stacking_params.stacking_spin_direction == kSpinDirectionPositive) {
      current_limit->iq_lower_limit = iq_cmd_limit_gen;
      current_limit->iq_upper_limit = iq_cmd_limit_motor;
    } else {
      current_limit->iq_lower_limit = -iq_cmd_limit_motor;
      current_limit->iq_upper_limit = -iq_cmd_limit_gen;
    }
  } else {
    current_limit->iq_lower_limit = -INFINITY;
    current_limit->iq_upper_limit = INFINITY;
  }
}

// Adjusts the commanded speed to remove high frequency components and keep the
// integrators in the speed controller and voltage controller from fighting each
// other. See diagram / code:
//
// i_corr_mean -----------.
//                        |
//                        V -    _______
// i_corr_local_mean ---> o --->| G2(z) |-----.
//                      +        -------      |
//                   .---_-.     _______      V +
//  omega_cmd -------| _/  |--->| G1(z) | --> o ------> omega_cmd_filtered
//                   `-----`    `-------`   +
void StackingSpeedCorrection(float *omega_upper_cmd, float *omega_lower_cmd,
                             const StackingIoInput *io_input) {
  // The speed correction is a perturbation to the speed command to satisfy the
  // equal power stacking constraint. Because the averaged current correction
  // is positive when needing to dissipate power instead of being a direct
  // modification to the motor current, the speed correction is dependent on the
  // sign of the rotational velocity; excess voltage requires increasing the
  // speed command when omega > 0 and decreasing the command when omega < 0.
  float spin_dir = g_stacking_params.stacking_spin_direction;
  float current_correction_diff = spin_dir
      * (io_input->current_correction_pair_mean
         - io_input->current_correction_stack_mean);
  float omega_correction
      = FirstOrderFilter(current_correction_diff, &g_speed_correction_params,
                         &g_stacking_isr_state.speed_correction_filter);

  // Apply anti-windup to prevent the speed controller from running away during
  // generation at lower speed. Even though the local current correction limits
  // near zero, positive current corrections on other motors will still cause
  // the local speed correction to decrease the speed command magnitude.
  float omega_gen_limit
      = Minf(g_stacking_params.gen_limit_dwcorr_diqcmd
             * g_stacking_isr_state.current_cmd_last * spin_dir, 0.0f)
      + g_stacking_params.gen_limit_wcorr_offset;
  omega_correction = (spin_dir == kSpinDirectionPositive)
      ? FirstOrderWindup(omega_gen_limit, INFINITY,
                         &g_speed_correction_params,
                         &g_stacking_isr_state.speed_correction_filter)
      : FirstOrderWindup(-INFINITY, -omega_gen_limit,
                         &g_speed_correction_params,
                         &g_stacking_isr_state.speed_correction_filter);
  // TODO: The sum of speed command and correction is not
  // saturated to the command limit. This is potentially dangerous and should be
  // looked at in more detail.

  // Upper omega command.
  *omega_upper_cmd = Saturatef(*omega_upper_cmd,
                               g_stacking_params.speed_cmd_lower_limit,
                               g_stacking_params.speed_cmd_upper_limit);
  *omega_upper_cmd = RateLimitf(
      *omega_upper_cmd,
      g_stacking_params.speed_cmd_lower_rate_limit,
      g_stacking_params.speed_cmd_upper_rate_limit,
      g_svpwm_isr_period, &g_stacking_isr_state.upper_speed_cmd_rate_limited);
  *omega_upper_cmd = FirstOrderFilter(
      *omega_upper_cmd, &g_speed_cmd_params,
      &g_stacking_isr_state.upper_speed_cmd_filter);
  *omega_upper_cmd += omega_correction;

  // Lower omega command.
  *omega_lower_cmd = Saturatef(*omega_lower_cmd,
                               g_stacking_params.speed_cmd_lower_limit,
                               g_stacking_params.speed_cmd_upper_limit);
  *omega_lower_cmd = RateLimitf(
      *omega_lower_cmd,
      g_stacking_params.speed_cmd_lower_rate_limit,
      g_stacking_params.speed_cmd_upper_rate_limit,
      g_svpwm_isr_period, &g_stacking_isr_state.lower_speed_cmd_rate_limited);
  *omega_lower_cmd = FirstOrderFilter(
      *omega_lower_cmd, &g_speed_cmd_params,
      &g_stacking_isr_state.lower_speed_cmd_filter);
  *omega_lower_cmd += omega_correction;
}

// Implements the high speed stacking voltage controller.
float StackingCurrentCorrection(float current_cmd, float voltage_local,
                                const StackingIoInput *io_input,
                                const CurrentLimitInput *iq_limit,
                                float *current_correction) {
  float spin_dir = g_stacking_params.stacking_spin_direction;
  float voltage_delta = voltage_local
      - io_input->voltage_pair_bias - io_input->voltage_stack_mean;

  // Compute a two-sample average for voltage_delta to prevent stacking
  // instability at 1/2 ISR frequency. For a given current correction during
  // generation, the FOC controller must apply an instantaneous power delta
  // opposite to the direction indicated by the current correction. In motoring,
  // this instability is not present because the instantaneous power delta is in
  // the same direction as the current correction.
  float voltage_error = voltage_delta;
  if (g_stacking_params.v_delta_averaging_enable > 0.5f) {
    voltage_error
        = 0.5f * (voltage_delta + g_stacking_isr_state.voltage_delta_last);
  }
  g_stacking_isr_state.voltage_delta_last = voltage_delta;

  (void)FirstOrderFilter(voltage_error * spin_dir,
                         &g_voltage_control_params,
                         &g_stacking_isr_state.voltage_control_filter);

  // Fault dependent current limits. During normal operation, we require that
  // 0.0f be an admissible command. During faults, it is useful to be able to
  // both motor or generate, at least slightly.
  float fault_current = g_stacking_isr_state.winding_down
      ? fabsf(g_stacking_fault_params.iq_cmd_lower_fault_limit) : 0.0f;
  float lower_current_limit = Minf(iq_limit->iq_lower_limit, -fault_current);
  float upper_current_limit = Maxf(iq_limit->iq_upper_limit, fault_current);

  // Pre-saturate the command to have limits consistent with what is finally
  // returned. This prevents large commands from artificially winding up the
  // filter. Note that this step is not necessarily consistent when flux
  // weakening.
  current_cmd
      = Saturatef(current_cmd, lower_current_limit, upper_current_limit);

  // Apply anti-windup to the voltage control filter state.
  // The offset introduced by current_cmd is subtracted from the current limits
  // so that anti-windup is effectively applied to the sum of current_cmd and
  // current_correction; i.e. the value that is ultimately saturated.
  float upper_correction_limit = upper_current_limit - current_cmd;
  float lower_correction_limit = lower_current_limit - current_cmd;

  // A non-linearity is introduced to prevent the current correction from
  // causing the mechanical speed to diverge as it tries to balance power during
  // generation.
  float iq_gen_limit = Minf(g_stacking_params.gen_limit_dicorr_diqcmd
                            * current_cmd * spin_dir, 0.0f);
  if (spin_dir == kSpinDirectionPositive) {
    lower_correction_limit = Maxf(lower_correction_limit, iq_gen_limit);
  } else {  // spin_dir == kSpinDirectionNegative
    upper_correction_limit = Minf(upper_correction_limit, -iq_gen_limit);
  }

  *current_correction
      = FirstOrderWindup(lower_correction_limit, upper_correction_limit,
                         &g_voltage_control_params,
                         &g_stacking_isr_state.voltage_control_filter);

  // The current correction contains the correct sign to adjust iq by, but the
  // speed correction loop cares about power which necessitates multiplying by
  // sgn(omega). Before doing so, we apply the correction to the command.
  g_stacking_isr_state.current_cmd_last = current_cmd;
  current_cmd += *current_correction;
  *current_correction *= spin_dir;

  // TODO: Open these limits up slightly beyond nominal.
  return Saturatef(current_cmd, lower_current_limit, upper_current_limit);
}

// Measure the communication delay and return a bitmask with the following bits
// set:
// - kMotorErrorTimeoutRemoteMotor: set if the delay on any motor exceeds
//                                  thresholds in g_stacking_fault_params.
// - kMotorErrorTimeoutPairMotor:   set if the delay on the pair motor exceeds
//                                  thresholds in g_stacking_fault_params.
// Args:
//   stale_counts:  Array with the number of times through the 1 kHz loop that
//                  the message from a particular motor hasn't been updated.
static uint32_t StackingCheckCommTimeouts(
    const int32_t stale_counts[kNumMotors]) {
  if (stale_counts[g_pair_index]
      >= (int32_t)g_stacking_fault_params.comm_timeout_cycles) {
    return kMotorErrorTimeoutRemoteMotor | kMotorErrorTimeoutPairMotor;
  }

  // Loop through each motor and test if it's stale.
  for (int32_t i = 0; i < kNumMotors; ++i) {
    // The local motor is inherently not stale.
    if (i == g_motor_index) continue;

    if (stale_counts[i]
        >= (int32_t)g_stacking_fault_params.comm_timeout_cycles) {
      return kMotorErrorTimeoutRemoteMotor;
    }
  }
  return kMotorErrorNone;
}

// Set the remote fault error flags if a motor in the stack at i_block == X has
// faulted and has not set kMotorErrorRemoteFaultBlockX. This is done to allow
// errors in the stack to be cleared.
// Args:
//   errors:  kNumMotors array of errors from each motor in the stack.
static uint32_t StackingCheckRemoteFaults(const uint32_t errors[kNumMotors]) {
  uint32_t local_errors = 0x00U;

  for (int32_t i = 0; i < kNumMotors; ++i) {
    if (i == g_motor_index) continue;

    if (errors[i] & ~MOTOR_FLAGS_REMOTE_FAULT_MASK) {
      // Motor i has faulted, but not because of another motor; there is an
      // actual issue on motor i.
      int32_t i_block = i % (kNumMotors / 2);
      local_errors |= (kMotorErrorRemoteFaultBlock0 << i_block);  // BlockX.
    }
  }
  return local_errors;
}

// Precompute voltage and current_correction means along with the voltage bias.
// This prevents the speed correction and current correction functions from
// having to recompute these values with each pass through the ISR; the averages
// remain constant between communication cycles.
uint32_t StackingPreprocess(const float voltages[kNumMotors],
                            const float current_corrections[kNumMotors],
                            const int32_t stale_counts[kNumMotors],
                            uint32_t stacking_errors[kNumMotors],
                            StackingIoInput *io_input,
                            uint32_t *block_shorted_mask) {
  if (!StackingIsEnabled()) {
    return kMotorErrorNone;
  }

  uint32_t errors = kMotorErrorNone;
  errors |= StackingCheckCommTimeouts(stale_counts);
  errors |= StackingCheckRemoteFaults(stacking_errors);
  stacking_errors[g_motor_index] |= errors;

  // Calculate the voltage and current correction stack means.
  float voltage_stack_sum = 0.0f;
  int32_t num_active_blocks = 0;  // Number of active (non-errored) blocks.
  float current_correction_stack_mean = 0.0f;
  int32_t timeout_counts = (int32_t)g_stacking_fault_params.comm_timeout_cycles;

  for (int32_t i_block = 0; i_block < kNumMotors / 2; ++i_block) {
    // TODO: Use helper functions in "avionics/common/motor_util.h".
    // TODO: Get rid of dependence on the ordering of motors.
    int32_t i_a = i_block;
    int32_t i_b = i_block + kNumMotors / 2;

    // Take faulted blocks out of the stack once they no longer have a
    // meaningful contribution to the system dynamics.
    if (((IsCriticalError(stacking_errors[i_a]) ||
          (stacking_errors[i_a] & kMotorErrorBlockShorting))
         && voltages[i_a] < g_motor_limits.v_bus_lower_limit) ||
        ((IsCriticalError(stacking_errors[i_b]) ||
          (stacking_errors[i_b] & kMotorErrorBlockShorting))
         && voltages[i_b] < g_motor_limits.v_bus_lower_limit)) {
      // TODO: Also set mask if short-stack has reported that it's
      // fired a level and is measuring less than threshold for that level.
      *block_shorted_mask |= (kMotorErrorRemoteFaultBlock0 << i_block);
      continue;
    }

    // TODO: Revisit whether we need a second limit here.
    bool updated_a = stale_counts[i_a] < timeout_counts;
    bool updated_b = stale_counts[i_b] < timeout_counts;

    if (updated_a && updated_b) {  // Both motors are current.
      g_stacking_io_state.v_pair_sum[i_block] = voltages[i_a] + voltages[i_b];
      g_stacking_io_state.current_correction_pair_sum[i_block]
          = current_corrections[i_a] + current_corrections[i_b];
    } else if (updated_a) {  // Motor b is stale so replace with motor a.
      g_stacking_io_state.v_pair_sum[i_block] = 2.0f * voltages[i_a];
      g_stacking_io_state.current_correction_pair_sum[i_block]
          = current_corrections[i_a];
    } else if (updated_b) {  // Motor a is stale so replace with motor b.
      g_stacking_io_state.v_pair_sum[i_block] = 2.0f * voltages[i_b];
      g_stacking_io_state.current_correction_pair_sum[i_block]
          = current_corrections[i_b];
    } else {  // Both motors in the block are stale.
      // Use stale values.
    }

    ++num_active_blocks;
    voltage_stack_sum += g_stacking_io_state.v_pair_sum[i_block];
    current_correction_stack_mean
        += g_stacking_io_state.current_correction_pair_sum[i_block];
  }

  const float half_inv[] = {  // half_inv[i] = 1/(2*i), i > 0.
    1.0f / 1.0f,  // Divide by zero.
    1.0f / 2.0f,
    1.0f / 4.0f,
    1.0f / 6.0f,
    1.0f / 8.0f
  };
  if (num_active_blocks > 0) {
    io_input->voltage_stack_mean
        = half_inv[num_active_blocks] * voltage_stack_sum;
    io_input->current_correction_stack_mean
        = half_inv[num_active_blocks] * current_correction_stack_mean;
  } else {
    // When num_active_blocks == 0, it's still useful to return a representative
    // value.
    io_input->voltage_stack_mean = voltages[g_motor_index];
    io_input->current_correction_stack_mean
        = current_corrections[g_motor_index];
  }

  io_input->voltage_stack_mean
      = Saturatef(io_input->voltage_stack_mean, 0.0f,
                  g_stacking_params.voltage_average_upper_sat);

  // Assume the pair motor is running and average the two values.
  io_input->current_correction_pair_mean
      = 0.5f * g_stacking_io_state.current_correction_pair_sum[g_block_index];

  // By computing the voltage pair bias in the preprocess function, the current
  // correction will appear to jump every communication cycle. As long as the
  // pair bias pole is slow enough, this shouldn't be an issue.
  io_input->voltage_pair_bias
      = FirstOrderFilter(Saturatef(0.5f * (voltages[g_motor_index]
                                           - voltages[g_pair_index]),
                                   -g_stacking_params.voltage_pair_bias_limit,
                                   g_stacking_params.voltage_pair_bias_limit),
                         &g_voltage_pair_bias_params,
                         &g_stacking_io_state.voltage_pair_bias_filter);
  return errors;
}

// Check for over / under voltage errors relative to the stack mean.
// TODO Reexamine whether relative voltage checking is needed.
uint32_t StackingCheckVoltage(float local_voltage,
                              const StackingIoInput *input) {
  float delta_voltage = local_voltage - input->voltage_stack_mean;

  if (delta_voltage < g_stacking_fault_params.voltage_rel_lower_limit ||
      delta_voltage > g_stacking_fault_params.voltage_rel_upper_limit) {
    return kMotorErrorVoltageRel;
  }
  return kMotorErrorNone;
}

// Wind down the iq command over a finite length of time.
float StackingCurrentWindDown(float iq) {
  if (g_stacking_isr_state.winding_down) {
    RateLimitf(0.0f, -g_stacking_fault_params.wind_down_current_rate,
               g_stacking_fault_params.wind_down_current_rate,
               g_svpwm_isr_period,
               &g_stacking_isr_state.wind_down_current);
  } else {
    g_stacking_isr_state.winding_down = true;
    g_stacking_isr_state.wind_down_current = iq;
  }
  return g_stacking_isr_state.wind_down_current;
}

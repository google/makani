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

#include "avionics/motor/firmware/current_limit.h"

#include <assert.h>
#include <stdbool.h>
#include <math.h>

#include "avionics/common/fast_math/fast_math.h"
#include "avionics/firmware/identity/identity.h"
#include "avionics/motor/firmware/current_limit_types.h"
#include "avionics/motor/firmware/errors.h"
#include "avionics/motor/firmware/foc.h"
#include "avionics/motor/firmware/params.h"
#include "avionics/motor/firmware/stacking.h"

#define LOW_POWER_THRESHOLD (850.0f * 6.0f)

typedef struct {
  // Power conservation arguments.
  FirstOrderFilterState p_i2r;
  FirstOrderFilterState p_bus;
  FirstOrderFilterState p_mech;
  FirstOrderFilterState v_bus;

  // Current redistribution states.
  FirstOrderFilterState i_bus;
  FirstOrderFilterState iq_cmd;
} IsrState;

typedef struct {
  float iq_upper_limit_bus;
  float iq_lower_limit_bus;
  float kt_scale;
} PreprocessState;

CurrentLimitParams g_current_limit_params = {
  // Motor / Stack limits.
  .ibus_upper_limit = 50.0f,             // [A].
  .ibus_lower_limit = -50.0f,            // [A].
  .iq_upper_limit = 50.0f,               // [A].
  .iq_lower_limit = -50.0f,              // [A].

  // ISR estimation and control parameters.
  .current_limit_kt_s_pole = -20.0f,     // [rad/s].
  .current_limit_kt_range = 0.2f,        // [none].

  .iq_cmd_residual_kp = 0.1f,            // [A_bus/A_iq].
  .current_limit_isr_s_pole = -2000.0f,  // [rad/s].
};

static int32_t g_index_local = 0;
static int32_t g_index_pair = 0;

// Kt scale filter pole and filter state.
static float g_kt_z_pole = 0.0f;
static PreprocessState g_preproc_state = {
  .iq_upper_limit_bus = 0.0f,
  .iq_lower_limit_bus = 0.0f,
  .kt_scale = 1.0f,
};

// ISR filter and filter states.
static FirstOrderFilterParams g_isr_filter;
static IsrState g_isr_state;

// Reinitialize the motor current limit static variables to correct values and
// calculate derived parameters.
void MotorCurrentLimitInit(float dt_io, float dt_isr) {
  g_kt_z_pole
      = TustinSToZ(g_current_limit_params.current_limit_kt_s_pole, dt_io);
  g_preproc_state.kt_scale = 1.0f;

  FirstOrderFilterState filter;
  FirstOrderFilterInit(1.0f, -INFINITY,
                       g_current_limit_params.current_limit_isr_s_pole, dt_isr,
                       0.0f, &g_isr_filter, &filter);
  g_isr_state.i_bus  = filter;
  g_isr_state.p_i2r  = filter;
  g_isr_state.p_bus  = filter;
  g_isr_state.p_mech = filter;
  g_isr_state.v_bus  = filter;
  g_isr_state.iq_cmd = filter;

  g_index_local = AppConfigGetIndex();
  assert(0 <= g_index_local && g_index_local < kNumMotors);
  g_index_pair = ((uint32_t)(g_index_local + kNumMotors / 2)) % kNumMotors;
  assert(0 <= g_index_pair && g_index_pair < kNumMotors);

  assert(g_current_limit_params.iq_lower_limit <= 0.0f);
  assert(g_current_limit_params.iq_upper_limit >= 0.0f);
  assert(g_current_limit_params.ibus_lower_limit <= 0.0f);
  assert(g_current_limit_params.ibus_upper_limit >= 0.0f);
}

// Fill out the current limit.
void MotorCurrentLimitGet(const CurrentLimitInput *input,
                          const MotorState *motor_state,
                          CurrentLimitInput *stacking_limit,
                          CurrentLimit *speed_limit) {
  // Calculate limits for the stacking current correction.
  CurrentLimitInput limit;
  FocCurrentLimit(&limit);
  stacking_limit->iq_upper_limit = Minf(g_current_limit_params.iq_upper_limit,
                                        limit.iq_upper_limit);
  stacking_limit->iq_lower_limit = Maxf(g_current_limit_params.iq_lower_limit,
                                        limit.iq_lower_limit);

  // Add limits for the speed controller.
  speed_limit->iq_upper_limit = Minf(input->iq_upper_limit,
                                     stacking_limit->iq_upper_limit);
  speed_limit->iq_lower_limit = Maxf(input->iq_lower_limit,
                                     stacking_limit->iq_lower_limit);
  speed_limit->iq_cmd_raw = 0.0f;

  StackingCurrentLimit(motor_state->omega_mech, &limit);
  speed_limit->iq_upper_limit = Minf(limit.iq_upper_limit,
                                     speed_limit->iq_upper_limit);
  speed_limit->iq_lower_limit = Maxf(limit.iq_lower_limit,
                                     speed_limit->iq_lower_limit);
}

// Calculate and filter or copy all quantities needed by the low speed loop.
void MotorCurrentLimitIsrUpdate(const FocCurrent *idq, const MotorState *state,
                                const CurrentLimit *limit,
                                CurrentLimitData *data) {
  data->p_i2r = FirstOrderFilter(
      g_motor_params->Rs * (idq->id * idq->id + idq->iq * idq->iq),
      &g_isr_filter, &g_isr_state.p_i2r);
  data->p_mech = FirstOrderFilter(
      g_foc_params.iq_to_torque * idq->iq * state->omega_mech,
      &g_isr_filter, &g_isr_state.p_mech);
  data->p_bus = FirstOrderFilter(
      state->v_bus * state->i_bus,
      &g_isr_filter, &g_isr_state.p_bus);

  data->v_bus = FirstOrderFilter(
      state->v_bus, &g_isr_filter, &g_isr_state.v_bus);
  data->i_bus = FirstOrderFilter(
      state->i_bus, &g_isr_filter, &g_isr_state.i_bus);
  data->iq_cmd_raw = FirstOrderFilter(
      limit->iq_cmd_raw, &g_isr_filter, &g_isr_state.iq_cmd);

  data->omega = state->omega_mech;
}

// Returns true if the stale count is greater than the stacking timeout
// threshold.
static inline bool IsStale(int32_t stale_count) {
  return stale_count > g_stacking_fault_params.comm_timeout_cycles;
}

// Calculate the upper and lower iq commands using the power argument:
//
//   P_bus = P_mech + P_therm
//
//   v_bus i_bus = 3/2 Kt * iq * omega + Rs * (id^2 + iq^2)
//
//   iq_{upper,lower} = (v_bus i_bus - Rs * (id^2 + iq^2)) / (3/2 Kt * omega)
//
// To keep the math simple and avoid needing a square root, the thermal loss
// term is treated as a constant and calculated from measured phase currents.
// Using i_bus = i_bus_{lower, upper}_limit then leads to the desired limits on
// iq.
static inline void CalculateIqLimits(float ibus_lower_limit,
                                     float ibus_upper_limit,
                                     const CurrentLimitData *data,
                                     CurrentLimitInput *input) {
  if (fabsf(data->omega) > 10.0f) {
    // Calculate the conversion from power to iq.
    float power_to_iq
        = g_foc_params.torque_to_iq * g_preproc_state.kt_scale / data->omega;

    g_preproc_state.iq_upper_limit_bus
        = (ibus_upper_limit * data->v_bus - data->p_i2r) * power_to_iq;
    g_preproc_state.iq_lower_limit_bus
        = (ibus_lower_limit * data->v_bus - data->p_i2r) * power_to_iq;

    // The limit associated with motoring or generating will swap sign with
    // omega. We only want to stay between the two limits, regardless of sign.
    if (g_preproc_state.iq_upper_limit_bus
        < g_preproc_state.iq_lower_limit_bus) {
      Swapf(&g_preproc_state.iq_upper_limit_bus,
            &g_preproc_state.iq_lower_limit_bus);
    }

    // Add in the hard upper and lower iq limits. Typically, only a Minf or Maxf
    // would be needed, respectively. However, pathological cases such as
    // iq_upper_limit (bus) < iq_lower_limit (param) can create issues.
    input->iq_upper_limit = Saturatef(g_preproc_state.iq_upper_limit_bus,
                                      g_current_limit_params.iq_lower_limit,
                                      g_current_limit_params.iq_upper_limit);
    input->iq_lower_limit = Saturatef(g_preproc_state.iq_lower_limit_bus,
                                      g_current_limit_params.iq_lower_limit,
                                      g_current_limit_params.iq_upper_limit);

    // Like at end of StackingIsEnabled section of MotorCurrentLimitPreprocess,
    // make sure doing nothing is an option.
    // Having upper limit < 0 can create problems when bus current limits
    // get merged back in with stacking current limits, if upper < lower,
    // causing Saturatef to fail an assert. Since MotorCurrentLimitPreprocess
    // already applies this constraint to the g_preproc_state current limits,
    // the only way we could go back to upper_limit < 0 is for high loss at
    // very low v_bus, like when a level is shorted out by the short stack.
    // Trying to offset bus current with iq < 0 under these conditions is
    // not a great option anyway, then, since low v_bus means that very little
    // torque is available and we probably have no control over iq and id.
    input->iq_lower_limit = Minf(input->iq_lower_limit, 0.0f);
    input->iq_upper_limit = Maxf(input->iq_upper_limit, 0.0f);

  } else {
    // At low enough speed, just use the hard limits.
    g_preproc_state.iq_upper_limit_bus = g_current_limit_params.iq_upper_limit;
    g_preproc_state.iq_lower_limit_bus = g_current_limit_params.iq_lower_limit;

    input->iq_upper_limit = g_current_limit_params.iq_upper_limit;
    input->iq_lower_limit = g_current_limit_params.iq_lower_limit;
  }
}

// Calculate a correction factor for the motor constant. Again, a power
// argument is used except that Kt is replaced by Kt / Kt_scale.
//
//   v_bus i_bus = 3/2 Kt * iq * omega / Kt_scale + Rs * (id^2 + iq^2)
//
//   v_bus i_bus - Rs * (id^2 + iq^2) = 3/2 Kt * iq * omega / Kt_scale
//
//   Kt_scale = 3/2 Kt * iq * omega / (v_bus i_bus - Rs * (id^2 + iq^2))
//
// The measured values of id and iq are used in all terms.
static inline void EstimateKtScale(const CurrentLimitData *data) {
  // Calculate the available electrical power.
  float power_elec = data->p_bus - data->p_i2r;

  // Calculate the ratio between mechanical power (measured using iq and the
  // speed estimate) and electrical power.
  float kt_scale = (fabsf(power_elec) > 0.0f)
      ? data->p_mech / power_elec : 1.0f;

  // At low power, unmodeled losses and noise become more significant, so a
  // nominal factor of 1.0f is cross faded in.
  kt_scale = Crossfadef(1.0f, kt_scale, fabsf(power_elec),
                        LOW_POWER_THRESHOLD, 2.0f * LOW_POWER_THRESHOLD);

  // Filter the scale factor.
  g_preproc_state.kt_scale = (1.0f - g_kt_z_pole) * kt_scale
      + g_kt_z_pole * g_preproc_state.kt_scale;

  // Saturate kt_scale.
  g_preproc_state.kt_scale
      = Saturatef(g_preproc_state.kt_scale,
                  1.0f - g_current_limit_params.current_limit_kt_range,
                  1.0f + g_current_limit_params.current_limit_kt_range);
}

void MotorCurrentLimitPreprocess(const uint32_t *errors,
                                 const int32_t *stale_counts,
                                 const float *bus_currents,
                                 const float *iq_cmd_residuals,
                                 const CurrentLimitData *data,
                                 CurrentLimitInput *input,
                                 CurrentLimitNetOutput *net_output) {
  // Get a pointer to g_current_limit_params to make the code more readable.
  const CurrentLimitParams *params = &g_current_limit_params;
  float ibus_upper_limit, ibus_lower_limit;

  static float cmd_residual_correction_last = 0;
  static float ibus_last = 0, ibus_stack_last = 0;

  // Calculate the available bus current.
  if (StackingIsEnabled()) {
    if (IsStale(stale_counts[g_index_pair])) {
      // Replace the current from the stale motor with our own, thereby limiting
      // us to half the stack bus current. It's still possible to over-draw the
      // bus; the pair is not necessarily doing the same thing, but this would
      // require it to still be running and hearing messages from us. In this
      // case, it should still obey the stack current limit.
      ibus_upper_limit = 0.5f * params->ibus_upper_limit;
      ibus_lower_limit = 0.5f * params->ibus_lower_limit;
    } else if (IsCriticalError(errors[g_index_pair])) {
      // Ignore measurements from the other motor. The local motor should also
      // error out and be removed from the stack, but if that doesn't happen,
      // it has the full stack current available to it.
      ibus_upper_limit = params->ibus_upper_limit;
      ibus_lower_limit = params->ibus_lower_limit;
    } else {
      // The stack current limit constrains motors such that their bus current
      // sum is equal to the limit. Another correction term is needed to
      // constrain the motors to share their torque deficit equally. Note that
      // residual is multiplied by sgn(omega) prior to transmission so that
      // positive residual always indicates wanting more bus current.
      float cmd_residual_correction = params->iq_cmd_residual_kp
          * (iq_cmd_residuals[g_index_local] - iq_cmd_residuals[g_index_pair]);

      // Implement two-sample averaging of data used for calculating the bus
      // current limits. This prevents the bus current controller from trying to
      // react to 1kHz noise.
      // TODO: Investigate using a true filter instead of averager.
      float cmd_residual_correction_avg =
          0.5 * (cmd_residual_correction + cmd_residual_correction_last);
      cmd_residual_correction_last = cmd_residual_correction;

      float ibus_avg = 0.5 * (data->i_bus + ibus_last);
      ibus_last = data->i_bus;

      float ibus_stack = data->i_bus + bus_currents[g_index_pair];
      float ibus_stack_avg = 0.5 * (ibus_stack + ibus_stack_last);
      ibus_stack_last = ibus_stack;

      // Calculate the locally available bus current. Half the bus current
      // margin is used to account for the possibility of common mode steps on
      // both motors.
      ibus_upper_limit = ibus_avg
          + 0.5f * (params->ibus_upper_limit - ibus_stack_avg)
          + cmd_residual_correction_avg;
      ibus_lower_limit = ibus_avg
          + 0.5f * (params->ibus_lower_limit - ibus_stack_avg)
          + cmd_residual_correction_avg;

      // On top of the stack limit constraints, we require that doing nothing be
      // a valid option. This is especially useful at low speed when generating
      // is disabled; otherwise, noise on the lower bus limit will force the
      // system to continue spinning regardless of the input.
      ibus_upper_limit = Saturatef(ibus_upper_limit,
                                   0.0f, params->ibus_upper_limit);
      ibus_lower_limit = Saturatef(ibus_lower_limit,
                                   params->ibus_lower_limit, 0.0f);
    }
  } else {
    // Directly use the bus current limits.
    ibus_upper_limit = params->ibus_upper_limit;
    ibus_lower_limit = params->ibus_lower_limit;
  }

  // Calculate a correction factor on the motor constant.
  EstimateKtScale(data);

  // Calculate the upper and lower iq limits.
  CalculateIqLimits(ibus_lower_limit, ibus_upper_limit, data, input);

  net_output->kt_scale = g_preproc_state.kt_scale;
  net_output->iq_cmd_residual = iq_cmd_residuals[g_index_local];
  net_output->iq_lower_limit_bus = ibus_lower_limit;
  net_output->iq_upper_limit_bus = ibus_upper_limit;
}

// Calculate the iq command residual with respect to the bus current based iq
// limit. The true residual is not used because it's counterproductive to ask
// for more bus current if the motor is torque limited.
float MotorCurrentLimitGetResidual(const CurrentLimitData *data) {
  float iq_cmd_res = 0.0f;
  if (data->iq_cmd_raw > g_preproc_state.iq_upper_limit_bus) {
    // Current will be redistributed based off of the deficit between the
    // command and the iq bus current limit.
    iq_cmd_res = data->iq_cmd_raw - g_preproc_state.iq_upper_limit_bus;
  } else if (data->iq_cmd_raw < g_preproc_state.iq_lower_limit_bus) {
    // We are limiting against one of the lower iq limits.
    iq_cmd_res = data->iq_cmd_raw - g_preproc_state.iq_lower_limit_bus;
  }

  // The preprocessor uses positive command residual as an indication that a
  // motor should get a larger portion of the bus current when motoring.
  // However, the pole is dependent on ki * v_bus / omega; the residual needs to
  // change sign with omega.
  return iq_cmd_res * copysignf(1.0f, data->omega);
}

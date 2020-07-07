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

#include "avionics/motor/firmware/foc.h"

#include <assert.h>
#include <math.h>

#include "avionics/common/fast_math/fast_math.h"
#include "avionics/firmware/serial/motor_serial_params.h"
#include "avionics/motor/firmware/config_params.h"
#include "avionics/motor/firmware/current_limit_types.h"
#include "avionics/motor/firmware/params.h"
#include "avionics/motor/firmware/svpwm.h"

FocParams g_foc_params = {
  // Proportional gain [N-m/(rad/s)] and integral gain [N-m/rad] on
  // the motor's mechanical angular rate.
  .omega_to_torque_kp = 48.0f,
  .omega_to_torque_ki = 287.0f,

  // Gain [A/A] on the error between the raw and saturated id command.
  .omega_anti_windup_kp = 0.1f,

  // Proportional and integral gains [V/A] on the direct and quadrature
  // currents. Default values are set during FocInit.
  .i_kp = 0.0f,
  .i_ki = 0.0f,

  // Threshold for the modulation index [#] above which the field
  // weakening angle rotation begins.
  .fw_modulation_threshold = 0.95f,

  // Integral gain [rad/#] for converting the error in the modulation
  // index to a phase current angle.
  .fw_ki = 0.01f,

  // Proportional gain [#/rad] on the error between the raw and
  // saturated field weakening angle.
  .fw_anti_windup_kp = 1.0f,

  // Phase current amplitude command limit [A_phpk]. Due to the constant
  // amplitude Park transform, this is also the maximum id-iq vector norm.
  .foc_phase_current_cmd_limit = 250.0f,
};

static FocState g_foc_state;

void FocInit(float dt_isr, const MotorParams *motor_params,
             MotorType motor_type, MotorHardware motor_controller_type) {
  // Set default PI gains for the current controller.
  switch (motor_type) {
    case kMotorTypeYasa:
      if (motor_controller_type == kMotorHardwareOzoneA1) {
        g_foc_params.i_kp = 2.3f;
        g_foc_params.i_ki = 0.07f;
      } else {
        g_foc_params.i_kp = 4.0f;
        g_foc_params.i_ki = 0.09f;
      }
      break;
    case kMotorTypeProtean:
      g_foc_params.i_kp = 0.78f;
      g_foc_params.i_ki = 0.03f;
      break;
    default:
      assert(false);
      break;
  }

  g_foc_params.iq_to_torque
      = (3.0f / 2.0f) * motor_params->Ke * motor_params->num_pole_pairs_elec;
  g_foc_params.torque_to_iq = 1.0f /  g_foc_params.iq_to_torque;

  // Approximate short circuit id value [A], which is accurate if Ld is
  // approximately equal to Lq and omega * Ld >> Rs.
  g_foc_params.short_circuit_id = -g_motor_params->Ke / g_motor_params->Ld;

  FocReset(dt_isr);
}

void FocReset(float dt_isr) {
  g_foc_state.id_int = 0.0f;
  g_foc_state.iq_int = 0.0f;
  g_foc_state.id_error = 0.0f;
  g_foc_state.iq_error = 0.0f;
  g_foc_state.modulation_int = 0.0f;
  g_foc_state.fw_angle = 0.0f;
  g_foc_state.fw_cos_angle = 1.0f;
  g_foc_state.fw_sin_angle = 0.0f;
  g_foc_state.omega_int = 0.0f;
  g_foc_state.speed_limit = kMotorSpeedLimitNone;

  g_foc_params.omega_to_iq_kp
      = g_foc_params.omega_to_torque_kp * g_foc_params.torque_to_iq;
  g_foc_params.omega_to_iq_ki
      = g_foc_params.omega_to_torque_ki * g_foc_params.torque_to_iq * dt_isr;
}

// Calculates the sine and cosine values of the three phase angles.
static inline void PhaseSinCos(float theta,
                               float *sin_theta, float *cos_theta,
                               float *sin_m_2pi3, float *cos_m_2pi3,
                               float *sin_p_2pi3, float *cos_p_2pi3) {
  SinCosLookup(theta, sin_theta, cos_theta);

  // Calculate sin(theta +/- 2 pi / 3) and cos(theta +/- 2 pi / 3)
  // using trigonometry identities.
  float sin_cos_2pi3 = *sin_theta * cosf(2.0f * PI_F / 3.0f);
  float sin_sin_2pi3 = *sin_theta * sinf(2.0f * PI_F / 3.0f);
  float cos_sin_2pi3 = *cos_theta * sinf(2.0f * PI_F / 3.0f);
  float cos_cos_2pi3 = *cos_theta * cosf(2.0f * PI_F / 3.0f);
  *cos_m_2pi3 = cos_cos_2pi3 + sin_sin_2pi3;
  *cos_p_2pi3 = cos_cos_2pi3 - sin_sin_2pi3;
  *sin_m_2pi3 = sin_cos_2pi3 - cos_sin_2pi3;
  *sin_p_2pi3 = sin_cos_2pi3 + cos_sin_2pi3;
}

// Park transformation.  This is similar to the direct-quadrature-zero
// transformation, except that it is not power invariant.  It projects
// the three phase currents to a coordinate system that rotates with
// the motor.
static void ParkTransform(const MotorState *motor_state,
                          FocCurrent *foc_current) {
  float sin_theta, cos_theta, sin_m_2pi3, cos_m_2pi3, sin_p_2pi3, cos_p_2pi3;
  PhaseSinCos(motor_state->theta_elec,
              &sin_theta, &cos_theta,
              &sin_m_2pi3, &cos_m_2pi3,
              &sin_p_2pi3, &cos_p_2pi3);

  foc_current->id = (2.0f / 3.0f) * (cos_theta * motor_state->ia
                                     + cos_m_2pi3 * motor_state->ib
                                     + cos_p_2pi3 * motor_state->ic);

  foc_current->iq = -(2.0f / 3.0f) * (sin_theta * motor_state->ia
                                      + sin_m_2pi3 * motor_state->ib
                                      + sin_p_2pi3 * motor_state->ic);

  foc_current->i0 = (1.0f / 3.0f) * (motor_state->ia
                                     + motor_state->ib
                                     + motor_state->ic);
}

void FocSpeedLoop(float omega_upper_limit, float omega_lower_limit,
                  float torque_request, float omega,
                  CurrentLimit *cmd_limits, FocCurrent *foc_current_cmd) {
  float iq_request = torque_request * g_foc_params.torque_to_iq;

  // If in torque mode, update integrator with requested Iq.
  if (g_foc_state.speed_limit == kMotorSpeedLimitNone) {
    g_foc_state.omega_int = Saturatef(iq_request,
                                      cmd_limits->iq_lower_limit,
                                      cmd_limits->iq_upper_limit);
    g_foc_state.omega_int = Saturatef(g_foc_state.omega_int,
                                      -g_foc_params.foc_phase_current_cmd_limit,
                                      g_foc_params.foc_phase_current_cmd_limit);
  }

  float omega_upper_error = omega_upper_limit - omega;
  float omega_lower_error = omega_lower_limit - omega;

  // Take care of inverted omega limits. Default to omega_upper_limit.
  if (omega_lower_limit > omega_upper_limit) {
    omega_lower_error = omega_upper_error;
  }

  float iq_upper_cmd = g_foc_params.omega_to_iq_kp * omega_upper_error
      + g_foc_state.omega_int;
  float iq_lower_cmd = g_foc_params.omega_to_iq_kp * omega_lower_error
      + g_foc_state.omega_int;

  // Pick actual Iq based on suggested Iq from the three competing sources:
  // upper omega loop, lower omega loop, and torque request.
  float raw_iq_cmd;
  if (iq_request >= iq_upper_cmd) {
    raw_iq_cmd = iq_upper_cmd;
    g_foc_state.speed_limit = kMotorSpeedLimitUpper;
  } else if (iq_request <= iq_lower_cmd) {
    raw_iq_cmd = iq_lower_cmd;
    g_foc_state.speed_limit = kMotorSpeedLimitLower;
  } else {
    raw_iq_cmd = iq_request;
    g_foc_state.speed_limit = kMotorSpeedLimitNone;
  }

  // Saturate the phase current magnitude.
  bool phase_current_saturated
      = fabsf(raw_iq_cmd) > g_foc_params.foc_phase_current_cmd_limit;
  raw_iq_cmd = Saturatef(raw_iq_cmd, -g_foc_params.foc_phase_current_cmd_limit,
                         g_foc_params.foc_phase_current_cmd_limit);

  // "Rotate" raw iq command by flux weakening angle.  This roughly
  // rotates the command around a circle when |iq_cmd| is greater than
  // |short_circuit_id|; otherwise it rotates the command around an
  // ellipse with major and minor radii, short_circuit_id and iq_cmd.
  // Note that the phase current command limit is not necessarily
  // respected if |short_circuit_id| > foc_phase_current_cmd_limit.
  float id_cmd = Minf(g_foc_params.short_circuit_id, -fabsf(raw_iq_cmd))
      * g_foc_state.fw_sin_angle;
  float iq_cmd = raw_iq_cmd * g_foc_state.fw_cos_angle;

  // Limit Iq command and prevent integrator windup.
  cmd_limits->iq_cmd_raw = iq_cmd;
  if (iq_cmd > cmd_limits->iq_upper_limit) {
    iq_cmd = cmd_limits->iq_upper_limit;
  } else if (iq_cmd < cmd_limits->iq_lower_limit) {
    iq_cmd = cmd_limits->iq_lower_limit;
  } else if (!phase_current_saturated) {
    if (g_foc_state.speed_limit == kMotorSpeedLimitUpper) {
      g_foc_state.omega_int += g_foc_params.omega_to_iq_ki * omega_upper_error;
    } else if (g_foc_state.speed_limit == kMotorSpeedLimitLower) {
      g_foc_state.omega_int += g_foc_params.omega_to_iq_ki * omega_lower_error;
    }
  }

  foc_current_cmd->id = Maxf(id_cmd, g_foc_params.short_circuit_id);
  foc_current_cmd->iq = iq_cmd;
  foc_current_cmd->i0 = 0.0f;

  // Apply anti-windup to omega integrator when the id command is
  // saturated.
  g_foc_state.omega_int += g_foc_params.omega_anti_windup_kp * Signf(iq_cmd)
      * (id_cmd - foc_current_cmd->id);
}

void FocSetOmegaInt(float new_omega_int) {
  g_foc_state.omega_int = new_omega_int;
}

// Ideally, this function would return the iq values associated with the
// intersection of the flux weakening circle with the phase current limit
// circle. However, this value is somewhat expensive to compute and sensitive to
// motor parameters so we instead return the q component of
//
//   [i_lim * cos(fw_angle + pi/2), +/-i_lim * sin(fw_angle + pi/2)]^T
//
// where i_lim is the phase current limit. As a consequence, the returned values
// will generally overestimate the true limits until the phase current limit is
// actually reached.
void FocCurrentLimit(CurrentLimitInput *current_limit) {
  float iq_limit
      = g_foc_params.foc_phase_current_cmd_limit * g_foc_state.fw_cos_angle;

  current_limit->iq_lower_limit = -iq_limit;
  current_limit->iq_upper_limit = iq_limit;
}

void FocCurrentLoop(const FocCurrent *foc_current_cmd,
                    const MotorState *motor_state,
                    FocCurrent *foc_current,
                    FocVoltage *foc_voltage) {
  // Convert phase currents to direct-quadrature currents.
  ParkTransform(motor_state, foc_current);

  // PID controller.
  float id_error = foc_current_cmd->id - foc_current->id;
  float iq_error = foc_current_cmd->iq - foc_current->iq;

  float vd_cmd = g_foc_params.i_kp * id_error
      + g_foc_params.i_ki * g_foc_state.id_int;

  float vq_cmd = g_foc_params.i_kp * iq_error
      + g_foc_params.i_ki * g_foc_state.iq_int;

  // Direct-quadrature decoupling.
  float omega_elec
      = motor_state->omega_mech * g_motor_params->num_pole_pairs_elec;
  foc_voltage->vd = vd_cmd - omega_elec
      * g_motor_params->Lq * foc_current_cmd->iq;
  foc_voltage->vq = vq_cmd + omega_elec
      * (g_motor_params->Ld * foc_current_cmd->id + g_motor_params->Ke);

  foc_voltage->v_ref = sqrtf(foc_voltage->vd * foc_voltage->vd
                             + foc_voltage->vq * foc_voltage->vq);
  foc_voltage->angle = Atan2Lookup(foc_voltage->vq, foc_voltage->vd);

  // Field weakening.  This is based on the approach described in the
  // article: J. Wai, T. Jahns, "A New Control Technique for Achieving
  // Wide Constant Power Speed Operation with an Interior PM
  // Alternator Machine", Industry Applications Conference, 2001.
  const float v_ref_max = g_svpwm_vref_vbus_limit * motor_state->v_bus;
  float modulation_error = foc_voltage->v_ref / Maxf(v_ref_max, 1.0f)
      - g_foc_params.fw_modulation_threshold;
  float fw_anti_windup_error = g_foc_params.fw_anti_windup_kp
      * (g_foc_state.modulation_int - g_foc_state.fw_angle);
  g_foc_state.modulation_int += g_foc_params.fw_ki
      * (modulation_error - fw_anti_windup_error);
  g_foc_state.fw_angle = Saturatef(g_foc_state.modulation_int,
                                   0.0f, PI_F / 2.0f);
  // Update the sin and cos states for use on the next cycle.
  SinCosLookup(g_foc_state.fw_angle,
               &g_foc_state.fw_sin_angle, &g_foc_state.fw_cos_angle);

  // Limit voltage and prevent windup.
  if (foc_voltage->v_ref > v_ref_max) {
    foc_voltage->v_ref = v_ref_max;
  } else {
    g_foc_state.id_int += id_error;
    g_foc_state.iq_int += iq_error;
  }

  g_foc_state.id_error = id_error;
  g_foc_state.iq_error = iq_error;
}

const FocState *GetFocState(void) {
  return &g_foc_state;
}

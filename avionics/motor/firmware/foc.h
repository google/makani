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

// Field-oriented controller.

#ifndef AVIONICS_MOTOR_FIRMWARE_FOC_H_
#define AVIONICS_MOTOR_FIRMWARE_FOC_H_

#include "avionics/common/motor_foc_types.h"
#include "avionics/firmware/serial/motor_serial_params.h"
#include "avionics/motor/firmware/config_params.h"
#include "avionics/motor/firmware/current_limit_types.h"
#include "avionics/motor/firmware/params.h"

typedef struct {
  float omega_to_torque_kp;
  float omega_to_torque_ki;
  float omega_anti_windup_kp;
  float omega_to_iq_kp;  // Derived quantity.
  float omega_to_iq_ki;  // Derived quantity.
  float short_circuit_id;  // Derived quantity.
  float torque_to_iq;
  float iq_to_torque;
  float i_kp;
  float i_ki;
  float fw_modulation_threshold;
  float fw_ki;
  float fw_anti_windup_kp;
  float foc_phase_current_cmd_limit;
} FocParams;

// Initialize the field-oriented controller parameters and state.
void FocInit(float dt_isr, const MotorParams *motor_params,
             MotorType motor_type, MotorHardware motor_controller_type);

// Reset the field-oriented controller state and update derived parameters.
void FocReset(float dt_isr);

// Controls speed between upper and lower limits using a PI loop that outputs a
// current (represented in the rotating d-q coordinate system).  Otherwise,
// provides the requested torque.  Note that omega_cmd and omega are the
// electrical angular rates.
void FocSpeedLoop(float omega_upper_limit, float omega_lower_limit,
                  float torque_request, float omega,
                  CurrentLimit *cmd_limits,
                  FocCurrent *foc_current_cmd);

// We want to prevent omega error from winding up g_foc_state.omega_int when
// in WindDown mode, so motors can gracefully exit WindDown mode and return to
// normal speed loop operation when short-stack has finished closing relay.
void FocSetOmegaInt(float new_omega_int);

// Calculates the minimum and maximum quadrature current available at the
// present flux weakening angle. Instantaneously, this value is not necessarily
// achievable but will approach the true value as the phase current increases.
void FocCurrentLimit(CurrentLimitInput *current_limit);

// Controls current using a PID loop that outputs a reference voltage
// and phase angle.  Note that the FOC current, calculated from the
// motor's state, is returned only for logging and debugging purposes.
void FocCurrentLoop(const FocCurrent *foc_current_cmd,
                    const MotorState *motor_state,
                    FocCurrent *foc_current, FocVoltage *foc_voltage);

const FocState *GetFocState(void);

extern FocParams g_foc_params;

#endif  // AVIONICS_MOTOR_FIRMWARE_FOC_H_

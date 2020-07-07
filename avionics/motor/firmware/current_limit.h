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

#ifndef AVIONICS_MOTOR_FIRMWARE_CURRENT_LIMIT_H_
#define AVIONICS_MOTOR_FIRMWARE_CURRENT_LIMIT_H_

#include <stdint.h>

#include "avionics/common/motor_foc_types.h"
#include "avionics/motor/firmware/current_limit_types.h"

typedef struct {
  // Single motor:  Maximum allowable bus current.
  // Stacked motor: Total available stack current.
  float ibus_upper_limit;
  float ibus_lower_limit;

  // Maximum quadrature current command.
  float iq_upper_limit;
  float iq_lower_limit;

  // Kt_scale filter pole and saturation parameters.
  float current_limit_kt_s_pole;  // [rad/s]
  float current_limit_kt_range;   // [none]

  // Gain for redistributing bus current based off of command residuals.
  float iq_cmd_residual_kp;

  // Pole to use when filtering data in the ISR.
  float current_limit_isr_s_pole;
} CurrentLimitParams;

void MotorCurrentLimitInit(float dt_io, float dt_isr);

// Fill out the current limit structure.
void MotorCurrentLimitGet(const CurrentLimitInput *input,
                          const MotorState *motor_state,
                          CurrentLimitInput *stacking_limit,
                          CurrentLimit *speed_limit);

void MotorCurrentLimitIsrUpdate(const FocCurrent *idq, const MotorState *state,
                                const CurrentLimit *limit,
                                CurrentLimitData *data);

// Preprocess stacking data to generate local bus current limits.
void MotorCurrentLimitPreprocess(const uint32_t *errors,
                                 const int32_t *stale_counts,
                                 const float *bus_currents,
                                 const float *iq_cmd_residuals,
                                 const CurrentLimitData *data,
                                 CurrentLimitInput *input,
                                 CurrentLimitNetOutput *net_output);

// Calculate and return the speed controller command residual for bus current
// redistribution.
float MotorCurrentLimitGetResidual(const CurrentLimitData *data);

extern CurrentLimitParams g_current_limit_params;

#endif  // AVIONICS_MOTOR_FIRMWARE_CURRENT_LIMIT_H_

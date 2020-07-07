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

#include "avionics/motor/firmware/params.h"

#include <assert.h>
#include <stdlib.h>

#include "avionics/motor/firmware/config_params.h"
#include "avionics/motor/firmware/param_types.h"
#include "common/macros.h"

const MotorParams kMotorParamSets[kNumMotorTypes] = {
  [kMotorTypeProtean] = {
    .Ld = 165e-6f,  // [H].
    .Lq = 165e-6f,  // [H].
    .Ke = 0.06203f,  // [Vphpk/(rad/s)].
    .Rs = 0.041f,  // [Ohm].
    .num_pole_pairs_elec = 32.0f,
    .num_pole_pairs_sens = 32.0f,
  },
  [kMotorTypeYasa] = {
    .Ld = 1000e-6f,  // [H].
    .Lq = 1000e-6f,  // [H].
    .Ke = 0.16667f,  // [Vphpk/(rad/s)].
    .Rs = 0.103f,  // [Ohm].
    .num_pole_pairs_elec = 15.0f,
    .num_pole_pairs_sens = 5.0f,
  }
};

const MotorParams *g_motor_params = NULL;

const MotorParams *GetMotorParamsByType(MotorType motor_type) {
  assert(0 <= motor_type && motor_type < ARRAYSIZE(kMotorParamSets));
  return &kMotorParamSets[motor_type];
}

void MotorParamsInit(MotorType motor_type) {
  g_motor_params = GetMotorParamsByType(motor_type);
}

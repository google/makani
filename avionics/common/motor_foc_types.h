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

#ifndef AVIONICS_COMMON_MOTOR_FOC_TYPES_H_
#define AVIONICS_COMMON_MOTOR_FOC_TYPES_H_

typedef enum {
  kMotorSpeedLimitForceSigned = -1,
  kMotorSpeedLimitNone,
  kMotorSpeedLimitUpper,
  kMotorSpeedLimitLower
} MotorSpeedLimit;

typedef struct {
  float ia;
  float ib;
  float ic;
  float v_bus;
  float v_chassis;
  float v_cm;
  float v_in_mon;
  float v_aux_mon;
  float i_bus;
  float theta_elec;
  float theta_mech;
  float omega_mech;
} MotorState;

typedef struct {
  float id_int;
  float iq_int;
  float id_error;
  float iq_error;
  float modulation_int;
  float fw_angle;
  float fw_cos_angle;
  float fw_sin_angle;
  float omega_int;
  MotorSpeedLimit speed_limit;
} FocState;

typedef struct {
  float id;
  float iq;
  float i0;
} FocCurrent;

typedef struct {
  float vd;
  float vq;
  float v_ref;
  float angle;
} FocVoltage;

#endif  // AVIONICS_COMMON_MOTOR_FOC_TYPES_H_

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

#ifndef AVIONICS_MOTOR_FIRMWARE_ANGLE_H_
#define AVIONICS_MOTOR_FIRMWARE_ANGLE_H_

#include "avionics/common/fast_math/filter.h"
#include "avionics/common/motor_angle_types.h"
#include "avionics/motor/firmware/angle_meas.h"
#include "avionics/motor/firmware/calib_params.h"
#include "avionics/motor/firmware/params.h"

typedef struct {
  float theta_kp;
  float theta_ki;
  float omega_filter_pole;
  FirstOrderFilterParams omega_filter;  // Derived quantity.
} DerivParams;

typedef struct {
  float omega_filter_pole;
  FirstOrderFilterParams omega_filter;  // Derived quantity.
} ZeroCrossingParams;

typedef struct {
  float omega_enable_threshold;
  float omega_disable_threshold;

  float filter_pole;
} SensorProfileParams;

typedef struct {
  DerivParams deriv;
  SensorProfileParams sensor;
  ZeroCrossingParams zc;

  float theta_offset_e;
  float theta_offset_m;

  float omega_transition_deriv;
  float omega_transition_zc;
} MotorAngleParams;

void MotorAngleInit(float period, const MotorAngleMeas *meas,
                    const MotorParams *motor_params,
                    const MotorCalibParams *motor_calib);
void MotorAngleReset(float period, const MotorParams *motor_params);
void MotorAngleUpdate(const MotorAngleMeas *meas,
                      const MotorParams *motor_params,
                      float *theta_mech, float *omega_mech);
void MotorAngleSensorDiag(SensorProfileDiag *diag);

extern MotorAngleParams g_motor_angle_params;

#endif  // AVIONICS_MOTOR_FIRMWARE_ANGLE_H_

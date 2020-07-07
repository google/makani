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

#ifndef AVIONICS_MOTOR_FIRMWARE_ISR_H_
#define AVIONICS_MOTOR_FIRMWARE_ISR_H_

#include <signal.h>

#include "avionics/common/motor_angle_types.h"
#include "avionics/motor/firmware/current_limit_types.h"
#include "avionics/motor/firmware/main.h"
#include "avionics/motor/firmware/stacking.h"

typedef struct {
  float bus_current_limit;
  float phase_current_limit;
  float phase_current_sum_limit;
  float adc_fault_current_limit;
  float v_bus_lower_limit;
  float v_bus_upper_limit;
  float omega_limit;
} MotorLimits;

typedef struct {
  MotorMode mode;
  uint32_t errors;
  uint32_t warnings;
  bool clear_errors;
  uint32_t clear_warnings_mask;
  float torque_cmd;
  float omega_upper_limit;
  float omega_lower_limit;
  StackingIoInput stacking;
  CurrentLimitInput current_limit_input;
} MotorIsrInput;

typedef struct {
  uint32_t errors;
  uint32_t warnings;
  uint32_t num_samples;
  float i_bus_sum;
  float v_bus_sum;
  float v_chassis_sum;
  float v_cm_sum;
  float v_in_mon_sum;
  float v_aux_mon_sum;
  float theta;
  float omega_sum;
  SensorProfileDiag angle_sensor;
  float omega_upper_limit;
  float omega_lower_limit;
  float torque_cmd;
  CurrentLimitData current_limit_data;
  float id_sum;
  float id_cmd_sum;
  float iq_sum;
  float iq_cmd_sum;
  float vd_sum;
  float vq_sum;
  float current_correction_sum;
  float speed_correction;
  float voltage_pair_bias;
  float voltage_stack_mean_sum;
} MotorIsrRawData;

typedef struct {
  uint32_t errors;
  uint32_t warnings;
  uint32_t num_samples;
  float i_bus;
  float v_bus;
  float v_chassis;
  float v_cm;
  float v_in_mon;
  float v_aux_mon;
  float theta;
  float omega;
  SensorProfileDiag angle_sensor;
  float omega_upper_limit;
  float omega_lower_limit;
  float torque_cmd;
  CurrentLimitData current_limit_data;
  float id;
  float id_cmd;
  float iq;
  float iq_cmd;
  float vd;
  float vq;
  float current_correction;
  float speed_correction;
  float voltage_pair_bias;
  float voltage_stack_mean;
} MotorIsrOutput;

// Sends commands to and receives data from the motor controller ISR.
MotorIsrInput *MotorIsrGetIoInputBuffer(void);
void MotorIsrSwapInputBuffers(void);
const MotorIsrOutput *MotorIsrFilterRawData(void);

void MotorIsrControlLoop(void);
void MotorAdcMagnitudeInterruptHandler(void);
void MotorGdbPowerGoodInterruptHandler(void);

extern MotorLimits g_motor_limits;

#endif  // AVIONICS_MOTOR_FIRMWARE_ISR_H_

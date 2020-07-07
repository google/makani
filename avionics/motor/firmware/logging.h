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

#ifndef AVIONICS_MOTOR_FIRMWARE_LOGGING_H_
#define AVIONICS_MOTOR_FIRMWARE_LOGGING_H_

#include <stdbool.h>
#include <stdint.h>

#include "avionics/common/motor_foc_types.h"

#define MOTOR_LOG_LEN 512

void MotorLogInit(int32_t motor_controller_type);
void MotorLogAdcData(void);
void MotorLogAdcSend(void);
void MotorLogControlData(const MotorState *motor_state,
                         const FocState *foc_state,
                         const FocCurrent *foc_current_actual,
                         const FocCurrent *foc_current_desired,
                         const FocVoltage *foc_voltage,
                         float torque_cmd,
                         float omega_upper_limit,
                         float omega_lower_limit,
                         uint32_t errors,
                         uint32_t warnings);
void MotorLogControlSend(bool start_send);

#endif  // AVIONICS_MOTOR_FIRMWARE_LOGGING_H_

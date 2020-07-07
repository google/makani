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

// Motor input/output handles sending and receiving messages between
// the motors and the rest of the avionics and also between the main
// motor loop and the fast motor control loop ISR.

#ifndef AVIONICS_MOTOR_FIRMWARE_IO_H_
#define AVIONICS_MOTOR_FIRMWARE_IO_H_

#include <stdbool.h>
#include <stdint.h>

#include "avionics/common/motor_profiler_types.h"
#include "avionics/motor/firmware/current_limit_types.h"
#include "avionics/motor/firmware/isr.h"
#include "avionics/motor/firmware/main.h"
#include "avionics/motor/firmware/thermal.h"
#include "avionics/motor/monitors/types.h"

// Sets all controller and stacking messages to be stale.
void MotorIoInit(void);

// Receives controller commands and stacking messages from the avionics network.
uint32_t MotorIoReceiveInputs(int64_t now, MotorMode mode, bool *arm,
                              int16_t *command, float *omega_upper_limit,
                              float *omega_lower_limit, float *torque_cmd,
                              uint32_t stacking_errors[kNumMotors],
                              int32_t motor_stale_counts[kNumMotors],
                              float bus_currents[kNumMotors],
                              float bus_voltages[kNumMotors],
                              float current_corrections[kNumMotors],
                              float iq_cmd_residuals[kNumMotors]);

// Sends stacking messages and motor controller status messages.
uint32_t MotorIoSendOutputs(MotorMode mode, uint32_t errors, uint32_t warnings,
                            const CurrentLimitNetOutput *current_limit_output,
                            const MotorIsrOutput *isr_output,
                            const MotorThermalData *thermal_data,
                            const ProfilerOutput *profiler_output,
                            bool *motor_status_message_sent);

// Called by hardware monitoring infrastructure to update the INA219 bus
// voltage and current measurements.
MotorMonitorData *MotorIoGetMotorMonitors(void);

#endif  // AVIONICS_MOTOR_FIRMWARE_IO_H_

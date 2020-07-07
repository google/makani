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

#ifndef AVIONICS_MOTOR_FIRMWARE_GIO_H_
#define AVIONICS_MOTOR_FIRMWARE_GIO_H_

#include <stdint.h>

#include "avionics/firmware/serial/motor_serial_params.h"

void MotorPositionControlInit(MotorHardware motor_controller_type);
void MotorGdbPowerGoodInit(MotorHardware motor_controller_type);
uint32_t MotorGdbProcessPowerGoodInterrupt(void);
uint32_t MotorGdbGetPowerGoodStatus(void);
void MotorGdbPowerGoodReset(void);
uint32_t MotorGdbDesatStatus(void);
void MotorGdbDesatReset(void);

#endif  // AVIONICS_MOTOR_FIRMWARE_GIO_H_

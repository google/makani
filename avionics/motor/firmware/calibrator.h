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

#ifndef AVIONICS_MOTOR_FIRMWARE_CALIBRATOR_H_
#define AVIONICS_MOTOR_FIRMWARE_CALIBRATOR_H_

#include <stdint.h>

#include "avionics/motor/firmware/angle_meas.h"
#include "avionics/motor/firmware/current_limit_types.h"
#include "avionics/motor/firmware/foc.h"
#include "avionics/motor/firmware/isr.h"

uint32_t CalibrationController(const MotorIsrInput *input,
                               MotorState *motor_state,
                               MotorAngleMeas *angle_meas,
                               CurrentLimit *current_lmiit,
                               FocCurrent *foc_current_cmd,
                               float *current_correction);
void CalibrationSlowLoop(int16_t command);

#endif  // AVIONICS_MOTOR_FIRMWARE_CALIBRATOR_H_

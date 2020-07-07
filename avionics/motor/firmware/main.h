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

#ifndef AVIONICS_MOTOR_FIRMWARE_MAIN_H_
#define AVIONICS_MOTOR_FIRMWARE_MAIN_H_

#include "avionics/motor/firmware/util.h"

#define MOTOR_AIO_PERIOD_US 1000
#define MOTOR_AIO_PERIOD_CYCLES CLOCK32_USEC_TO_CYCLES(MOTOR_AIO_PERIOD_US)
#define MOTOR_AIO_PERIOD (MOTOR_AIO_PERIOD_US * 1.0e-6f)
#define MOTOR_AIO_FREQUENCY (1.0f / MOTOR_AIO_PERIOD)

typedef enum {
  kMotorModeInit,
  kMotorModeArmed,
  kMotorModeRunning,
  kMotorModeErrorWindDown,
  kMotorModeErrorDisarmed,
  kNumMotorModes,
} MotorMode;

static inline void MotorShutdown(void) {
  GateDriverDisable();
}

#endif  // AVIONICS_MOTOR_FIRMWARE_MAIN_H_

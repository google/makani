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

#ifndef AVIONICS_MOTOR_FIRMWARE_ERRORS_H_
#define AVIONICS_MOTOR_FIRMWARE_ERRORS_H_

#include <stdbool.h>

#include "avionics/motor/firmware/flags.h"

#define MOTOR_ERRORS_CRITICAL (kMotorErrorTimeout               \
                               | kMotorErrorBadMode             \
                               | kMotorErrorBadCommand          \
                               | kMotorErrorOverCurrentIaP      \
                               | kMotorErrorOverCurrentIaN      \
                               | kMotorErrorOverCurrentIbP      \
                               | kMotorErrorOverCurrentIbN      \
                               | kMotorErrorOverCurrentIcP      \
                               | kMotorErrorOverCurrentIcN      \
                               | kMotorErrorOverCurrentIBusP    \
                               | kMotorErrorOverCurrentIBusN    \
                               | kMotorErrorOverVoltage         \
                               | kMotorErrorUnderVoltage        \
                               | kMotorErrorFaultCurrentIaP     \
                               | kMotorErrorFaultCurrentIaN     \
                               | kMotorErrorFaultCurrentIbP     \
                               | kMotorErrorFaultCurrentIbN     \
                               | kMotorErrorFaultCurrentIcP     \
                               | kMotorErrorFaultCurrentIcN)
#define MOTOR_ERRORS_NON_CRITICAL               \
  (kMotorErrorAll ^ MOTOR_ERRORS_CRITICAL)

typedef struct {
  // This should really be a bool, but needs to be compatible with the address
  // list in io.c.
  float shutdown_on_warning_enable;
} ErrorConfig;

static inline bool IsNonCriticalError(uint32_t errors) {
  return (bool)(errors & MOTOR_ERRORS_NON_CRITICAL);
}

static inline bool IsCriticalError(uint32_t errors) {
  return (bool)(errors & MOTOR_ERRORS_CRITICAL);
}

static inline bool IsWarning(uint32_t warning) {
  return (bool)(warning & kMotorWarningAll);
}

extern ErrorConfig g_error_config;
static inline bool ShutDownOnWarning(void) {
  return g_error_config.shutdown_on_warning_enable > 0.5f;
}

#endif  // AVIONICS_MOTOR_FIRMWARE_ERRORS_H_

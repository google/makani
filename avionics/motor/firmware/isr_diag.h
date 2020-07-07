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

// High rate diagnostic data from the motor controller's main control ISR.
// Samples from each pass through the ISR are accumulated in a buffer to be
// periodically sent at 1 kHz. The number of samples in a buffer along with the
// total number of recorded samples neglecting the current buffer are recorded
// for later reconstruction of the data.

#ifndef AVIONICS_MOTOR_FIRMWARE_ISR_DIAG_H_
#define AVIONICS_MOTOR_FIRMWARE_ISR_DIAG_H_

#include <signal.h>
#include <stdbool.h>
#include <stdint.h>

#include "avionics/common/avionics_messages.h"
#include "avionics/common/motor_foc_types.h"
#include "avionics/motor/firmware/angle_meas.h"

typedef struct {
  // The motor_client expects io.c to list pointers to floats in structs;
  // otherwise, isr_diag_enable would be a simple boolean. To make this behave
  // in an intuitive manner, let isr_diag > 0.5f correspond to true.
  float isr_diag_enable;
} IsrDiagState;

extern IsrDiagState g_isr_diag_state;

// Test if the ISR Diagnostic code is enabled.
static inline bool IsrDiagIsEnabled(void) {
  return g_isr_diag_state.isr_diag_enable > 0.5f;
}

// Log data from the control ISR into a buffer at high rate. The buffer length
// is limited to MOTOR_ISR_DIAG_MESSAGE_LENGTH, but an additional reserve buffer
// is available. If both buffers fill up without being serviced by
// IsrDiagGetMessage, additional data is discarded.
void IsrDiagLog(uint32_t errors, uint32_t warnings,
                const MotorAngleMeas *angle_meas,
                const MotorState *motor_state,
                const FocVoltage *voltage_ab);

// Return the most recently filled MotorIsrDiagMessage.
MotorIsrDiagMessage *IsrDiagGetMessage(void);

#endif  // AVIONICS_MOTOR_FIRMWARE_ISR_DIAG_H_

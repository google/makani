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

#include "avionics/motor/firmware/isr_diag.h"

#include <signal.h>
#include <stdint.h>

#include "avionics/common/avionics_messages.h"
#include "avionics/common/motor_foc_types.h"
#include "avionics/motor/firmware/angle_meas.h"
#include "common/barrier.h"
#include "common/macros.h"

typedef enum {
  kBufferIsrPrimary,
  kBufferIsrReserve,
  kBufferIo,
  kNumBuffers,
} IsrDiagBufferOffsets;

IsrDiagState g_isr_diag_state = {
  // Default to false so that motors don't automatically saturate the network.
  .isr_diag_enable = 0.0f,
};

// Pointer to the buffer being filled by the ISR.
static sig_atomic_t g_isr_buffer_ind = 0;

// Keep track of the total number of samples from the ISR. This is fairly
// convenient when reconstructing the data in addition to making the process
// robust to out of order messages and clearly indicating dropped messages or
// missing samples.
static uint32_t g_total_samples = 0;

static MotorIsrDiagMessage g_buffer[] = {
  {
    .total = 0,
    .num_samples = 0,
    .errors = {0},
    .vbus = {0.0},
    .ibus = {0.0},
    .ia = {0.0},
    .ib = {0.0},
    .ic = {0.0},
    .sin = {0},
    .cos = {0},
    .vab_ref = {0.0},
    .vab_angle = {0.0},
  }, {
    .total = 0,
    .num_samples = 0,
    .errors = {0},
    .vbus = {0.0},
    .ibus = {0.0},
    .ia = {0.0},
    .ib = {0.0},
    .ic = {0.0},
    .sin = {0},
    .cos = {0},
    .vab_ref = {0.0},
    .vab_angle = {0.0},
  }, {
    .total = 0,
    .num_samples = 0,
    .errors = {0},
    .vbus = {0.0},
    .ibus = {0.0},
    .ia = {0.0},
    .ib = {0.0},
    .ic = {0.0},
    .sin = {0},
    .cos = {0},
    .vab_ref = {0.0},
    .vab_angle = {0.0},
  }
};
COMPILE_ASSERT(ARRAYSIZE(g_buffer) == kNumBuffers,
               g_buffer_should_be_length_kNumBuffers);

// Log data to one of the buffers in g_buffer. If space is available, IsrDiagLog
// will update the primary buffer with index g_isr_buffer_ind. If that buffer is
// full, (g_isr_buffer_ind + kBufferIsrReserve) % kNumBuffers is used. This is
// the next primary buffer such that it will continue to be filled after
// IsrDiagGetMessage is called resulting in a continuous set of samples.
void IsrDiagLog(uint32_t errors, uint32_t warnings,
                const MotorAngleMeas *angle_meas,
                const MotorState *motor_state,
                const FocVoltage *voltage_ab) {
  if (IsrDiagIsEnabled()) {
    uint32_t i_buffer = g_isr_buffer_ind;
    if (g_buffer[i_buffer].num_samples >= MOTOR_ISR_DIAG_MESSAGE_LENGTH) {
      // Overflow to the next primary ISR buffer.
      i_buffer = (i_buffer + kBufferIsrReserve) % kNumBuffers;
    }

    // Buffer data if space is available.
    uint32_t i_sample = g_buffer[i_buffer].num_samples;
    if (i_sample < MOTOR_ISR_DIAG_MESSAGE_LENGTH) {
      g_buffer[i_buffer].errors[i_sample] = errors;
      g_buffer[i_buffer].warnings[i_sample] = warnings;
      g_buffer[i_buffer].vbus[i_sample] = motor_state->v_bus;
      g_buffer[i_buffer].ibus[i_sample] = motor_state->i_bus;
      g_buffer[i_buffer].ia[i_sample] = motor_state->ia;
      g_buffer[i_buffer].ib[i_sample] = motor_state->ib;
      g_buffer[i_buffer].ic[i_sample] = motor_state->ic;
      g_buffer[i_buffer].sin[i_sample] = angle_meas->sin_m;
      g_buffer[i_buffer].cos[i_sample] = angle_meas->cos_m;
      g_buffer[i_buffer].vab_ref[i_sample] = voltage_ab->v_ref;
      g_buffer[i_buffer].vab_angle[i_sample] = voltage_ab->angle;

      // Update the number of samples in the buffer and total number of samples
      // neglecting the contents of the buffer.
      g_buffer[i_buffer].num_samples++;
      g_buffer[i_buffer].total = g_total_samples - i_sample;
    }

    ++g_total_samples;  // Always increment the total number of samples.
  }
}

// Advance g_isr_buffer_ind and return a pointer to the buffer it was previously
// indexing.
MotorIsrDiagMessage *IsrDiagGetMessage() {
  // Reset the sample counter in the most recently transmitted buffer before
  // exposing it to the ISR.
  g_buffer[(g_isr_buffer_ind + kBufferIo) % kNumBuffers].num_samples = 0;

  // Advance buffers and return the last buffer to be filled by the ISR. A
  // memory barrier is used to enforce ordering instead of volatile because of
  // auto-generation issues.
  uint32_t last_isr_buffer_ind = g_isr_buffer_ind;
  MemoryBarrier();
  g_isr_buffer_ind = (g_isr_buffer_ind + kBufferIsrReserve) % kNumBuffers;
  MemoryBarrier();
  return &g_buffer[last_isr_buffer_ind];
}

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

#include "avionics/motor/firmware/calibrator.h"

#include <stdint.h>
#include <signal.h>

#include "avionics/common/fast_math/fast_math.h"
#include "avionics/firmware/drivers/ad7265.h"
#include "avionics/firmware/network/net_send.h"
#include "avionics/motor/firmware/current_limit.h"
#include "avionics/motor/firmware/flags.h"
#include "avionics/motor/firmware/foc.h"
#include "avionics/motor/firmware/isr.h"
#include "avionics/motor/firmware/params.h"
#include "avionics/motor/firmware/svpwm.h"
#include "common/barrier.h"

#define START_ANGLE 0.0f
#define PULL_CURRENT 50.0f
#define CYCLE_MS 16

typedef enum {
  kSettlePhase,
  kNoisePhase,
  kRotationPhase,
  kIdlePhase,
} kCalibrationPhase;

// Used to pass angle information.
static volatile float g_pos_angle[2] = {START_ANGLE, START_ANGLE};
static volatile sig_atomic_t g_pos_angle_index = 0;

static void SetNextAngle(float angle) {
  sig_atomic_t next_index = !g_pos_angle_index;
  g_pos_angle[next_index] = angle;

  // Force the compiler to write g_pos_angle[next_index] before swapping the
  // buffer.
  MemoryBarrier();
  g_pos_angle_index = next_index;
  MemoryBarrier();
}

static float GetAngle(void) {
  return g_pos_angle[g_pos_angle_index];
}

typedef struct {
  int32_t a1;
  int32_t b1;
  int32_t a2;
  int32_t b2;
} PosMeasurementData;

static volatile sig_atomic_t g_pos_buf_ind = 0;
PosMeasurementData g_pos_buf[] = {
  {
    .a1 = 0,
    .b1 = 0,
    .a2 = 0,
    .b2 = 0
  }, {
    .a1 = 0,
    .b1 = 0,
    .a2 = 0,
    .b2 = 0
  }
};

static PosMeasurementData *GetIsrAngleBuffer(void) {
  return &g_pos_buf[g_pos_buf_ind];
}

static PosMeasurementData *GetIoAngleBuffer(void) {
  return &g_pos_buf[!g_pos_buf_ind];
}

static void SwapAngleBuffer(void) {
  MemoryBarrier();
  g_pos_buf_ind = !g_pos_buf_ind;
  MemoryBarrier();
}

uint32_t CalibrationController(const MotorIsrInput *input,
                               MotorState *motor_state,
                               MotorAngleMeas *angle_meas,
                               CurrentLimit *current_limit,
                               FocCurrent *foc_current_cmd,
                               float *current_correction) {
  (void)angle_meas;

  if (input->mode != kMotorModeRunning
      && input->mode != kMotorModeErrorWindDown) {
    // This call to FocReset should be consolidated into MotorIsrControlLoop().
    FocReset(g_svpwm_isr_period);
  }

  motor_state->theta_elec = GetAngle();
  motor_state->omega_mech = 0;

  CurrentLimitInput unused_limit;
  MotorCurrentLimitGet(&input->current_limit_input, motor_state,
                       &unused_limit, current_limit);
  foc_current_cmd->id = Saturatef(PULL_CURRENT,
                                  current_limit->iq_lower_limit,
                                  current_limit->iq_upper_limit);
  foc_current_cmd->iq = 0.0f;
  foc_current_cmd->i0 = 0.0f;

  *current_correction = 0;

  // Read raw position values.
  PosMeasurementData *pos = GetIsrAngleBuffer();
  Ad7265ReadAsyncGet(&pos->a1, &pos->b1, &pos->a2, &pos->b2);
  Ad7265ReadAsyncStart();

  return kMotorErrorNone;
}

// Execute angle calibration.  This function assumes it's called at 1KHz.
void CalibrationSlowLoop(int16_t command) {
  static uint32_t seq_number = 0;
  static uint32_t ms_counter = 0;
  static uint32_t phase_counter = 0;
  static kCalibrationPhase phase = kSettlePhase;

  ++ms_counter;
  ++phase_counter;
  // TODO: Investigate if we can eliminate this cycle division and
  //                run the calibrator at 1KHz.
  if (ms_counter < CYCLE_MS) {
    return;
  }

  ms_counter = 0;

  if (!(command & kMotorCommandRun)) {
    // If we're not running reset the calibration state machine and return.
    phase = kSettlePhase;
    seq_number = 0;
    phase_counter = 0;
    SetNextAngle(START_ANGLE);
    SetNextAngle(START_ANGLE);
    return;
  }

  MotorCalibrationMessage data;
  bool send_packet = false;
  kCalibrationPhase next_phase = phase;

  // Record angle before it's updated by the rotation phase.
  data.angle = GetAngle();

  switch (phase) {
    case kSettlePhase:
      // The settle phase lets the motor move to START_ANGLE and stop
      // oscillating.
      if (phase_counter >= 1000) {
        next_phase = kNoisePhase;
      }
      break;

    case kNoisePhase:
      // The noise phase holds the motor still and provides data to
      // measure electrical noise.
      data.mode = kMotorAngleCalModeNoise;
      send_packet = true;
      if (phase_counter >= 1000) {
        next_phase = kRotationPhase;
      }
      break;

    case kRotationPhase: {
      // The electrical phase rotates the motor and provides data to
      // calibrate the angular offset.
      const int32_t pole_period = 1600;  // milliseconds
      const float angle_increment =
          2.0f * PI_F / (float)(pole_period / CYCLE_MS);

      data.mode = kMotorAngleCalModeAngle;
      send_packet = true;

      float next_angle = WrapAngle(GetAngle() + angle_increment);
      SetNextAngle(next_angle);

      // Give the motor enough cycles to make a full rotation.
      if (phase_counter >= pole_period * g_motor_params->num_pole_pairs_elec) {
        next_phase = kIdlePhase;
      }
      break;
    }
    case kIdlePhase:
      break;

    default:
      break;
  }

  if (next_phase != phase) {
    phase = next_phase;
    phase_counter = 0;
  }

  if (send_packet) {
    data.index = seq_number;

    SwapAngleBuffer();
    PosMeasurementData *pos = GetIoAngleBuffer();
    data.a1 = pos->a1;
    data.b1 = pos->b1;
    data.a2 = pos->a2;
    data.b2 = pos->b2;

    NetSendAioMotorCalibrationMessage(&data);

    ++seq_number;
  }
}

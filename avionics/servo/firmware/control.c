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

#include "avionics/servo/firmware/control.h"

#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "avionics/common/fast_math/fast_math.h"
#include "avionics/servo/firmware/config_params.h"
#include "avionics/servo/firmware/def.h"
#include "avionics/servo/firmware/input.h"
#include "avionics/servo/firmware/r22_types.h"
#include "common/c_math/filter.h"
#include "common/c_math/util.h"

typedef struct {
  float ts;
  float fc_resolver;
  float fc_variance;
  float gear_ratio;
  float current_max;
  float variance_min;
  float variance_max;

  struct {
    float angle_bias_tc;
    float angle_feedback_max;
    float angle_feedback_tc;
    float current_ki;
    float self_trust_zero;
    float self_trust_one;
  } paired;
} ControlParams;

static const ControlParams kControlParam = {
  .ts = (float)SERVO_CONTROL_PERIOD_US / 1e6f,  // [s]
  .fc_resolver = 0.10f,   // [Hz]
  .fc_variance = 0.32f,   // [Hz]
  .gear_ratio = 160.0f,   // [#]
  .variance_min = 1e-9f,  // [rad^2]
  .variance_max = 0.08f,  // [rad^2]

  .paired = {
    .angle_bias_tc = 1000.0f,      // [s]
    .angle_feedback_max = 0.785f,  // [rad]
    .angle_feedback_tc = 1.0f,     // [s]
    .current_ki = 1e-4f,           // [(rad/s)/amp]
    .self_trust_zero = 0.1f,       // [#]
    .self_trust_one = 0.4f         // [#]
  }
};

// Estimate servo angle and variance. Initialize angle_z1 and var_z1, then
// call this function for each control loop iteration with T=kControlParam.ts.
static void EstimateAngle(const ServoState *self, float *velocity_prev,
                          float *angle_z1, float *var_z1) {
  assert(self != NULL && angle_z1 != NULL && var_z1 != NULL);
  float k = 2.0f * (float)PI * kControlParam.ts * kControlParam.fc_resolver;
  assert(0.0f <= k && k <= 1.0f);

  // Propagate.
  *angle_z1 += (self->input.r22.velocity.value + *velocity_prev)
      * kControlParam.ts * 0.5f;
  *velocity_prev = self->input.r22.velocity.value;
  float dz = WrapAngle(self->input.r22.angle - *angle_z1);  // Find innovation.
  *angle_z1 += k*dz;                                        // Update.

  // Saturate variance. Prevent huge innovation (e.g. sensor unplugged) from
  // taking forever to recover.
  double angle_var = Saturate(*var_z1,
                              kControlParam.variance_min,
                              kControlParam.variance_max);

  // Use innovation as an estimate of uncertainty.
  PeakDetector(dz * dz, kControlParam.fc_variance, kControlParam.ts,
               &angle_var);
  *var_z1 = (float)angle_var;
}

// Adjust output as a function of the relative difference in effort between
// paired servos.
static void PairedCurrentControl(const ServoState *self,
                                 const ServoState *paired,
                                 float *angle_feedback) {
  // Compute current error and error integral.
  if (paired->updated
      && !CheckWarning(&self->control.flags, kServoWarningPairFailed)) {
    float error = paired->input.r22.current.value
        - self->input.r22.current.value;
    *angle_feedback +=
        kControlParam.paired.current_ki * kControlParam.ts * error;
  } else {
    assert(kControlParam.ts < kControlParam.paired.angle_feedback_tc);
    *angle_feedback -= (*angle_feedback) *
        (kControlParam.ts / kControlParam.paired.angle_feedback_tc);
  }

  // Bound error.
  *angle_feedback = (float)Saturate(*angle_feedback,
                                    -kControlParam.paired.angle_feedback_max,
                                    kControlParam.paired.angle_feedback_max);
}

static float CurrentLimit(const ServoState *self) {
  float velocity = fabs(self->input.r22.velocity.value);
  float current_limit = kServoConfigParams->current_limit;
  if (velocity > 0) {
    float current_velocity_limit =
        kServoConfigParams->current_velocity_limit / velocity;
    if (current_velocity_limit < current_limit) {
      return current_velocity_limit;
    }
  }
  return current_limit;
}

// Perform common control tasks between single and paired servos.
static void ControlAngle(ServoState *self, float *desired_angle) {
  // Initial state estimate.
  if (!self->control.init_estimate) {
    self->control.init_estimate = true;
    self->control.angle_estimate = self->input.r22.angle;
    self->control.angle_variance = kControlParam.variance_max;
  }

  // Estimate angle given sensor measurements to detect potential failure.
  EstimateAngle(self, &self->control.velocity_prev,
                &self->control.angle_estimate, &self->control.angle_variance);

  float angle_command = self->input.cmd.desired_angle;
  bool scuttle = self->input.tether_released &&
      (self->input.scuttle_command || !self->input.cmd.valid);
  SignalWarning(kServoWarningScuttle, scuttle, &self->control.flags);
  if (scuttle) {
    angle_command = kServoConfigParams->scuttle_angle;
  }

  self->control.valid = scuttle || self->input.cmd.valid;

  if (self->state == kActuatorStateRunning) {
    // Limit command error to prevent Copley's trajectory controller from
    // chosing the shortest path around the circle instead of the correct
    // direction.
    float error = angle_command + self->control.angle_bias +
        self->control.angle_feedback - self->control.angle_estimate;
    error = (float)Saturate(error, -PI/4.0, PI/4.0);
    *desired_angle = self->control.angle_estimate + error;
  } else {
    *desired_angle = 0.0f;
  }
}

static void ServoUpdateStatus(ServoState *self, const ServoState *paired) {
  StatusFlags *flags = &self->control.flags;
  if (ServoIsPaired()) {
    SignalWarning(kServoWarningPairFailed, paired->state == kActuatorStateError,
                  &self->control.flags);
    if (self->state == kActuatorStateArmed
        || self->state == kActuatorStateRunning) {
      if ((paired->state == kActuatorStateArmed ||
           paired->state == kActuatorStateRunning) &&
          paired->updated) {
        self->control.pair_timeout =
            self->sync_timestamp + SERVO_PAIR_TIMEOUT_US;
      }
      SignalWarning(kServoWarningPairTimeout,
                    self->sync_timestamp >= self->control.pair_timeout, flags);
    } else {
      SignalWarning(kServoWarningPairTimeout, false, flags);
    }
    SetStatus(kServoStatusPairSynced,
              abs(self->sync_timestamp - paired->sync_timestamp)
              < SERVO_NET_TIMEOUT_US, flags);
  }
  SetStatus(kServoStatusPaired, ServoIsPaired() &&
            !CheckWarning(flags, kServoWarningPairTimeout) &&
            !CheckWarning(flags, kServoWarningPairFailed), flags);
}

// Init.
void ServoControlInit(ServoState *self, ServoState *paired) {
  memset(self, 0x0, sizeof(*self));
  memset(paired, 0x0, sizeof(*paired));
}

static void ServoControlPairedSync(ServoState *self,
                                   const ServoState *paired) {
  // Wait for paired servo to ensure proper bias initialization. We assume
  // the servos are mechanically connected and any error between the two
  // measurements corresponds to mechanical misalignment in their zero
  // positions.
  // TODO: Make control scheme more robust to a flaky pair.
  if (!ServoIsPaired()) {
    self->control.init_alignment = true;
    self->control.angle_bias = 0.0f;
    self->control.angle_feedback = 0.0f;
  }
  if ((paired->sync_timestamp - self->sync_timestamp)
      <= SERVO_CONTROL_PERIOD_US
      && (self->sync_timestamp - paired->sync_timestamp)
      <= SERVO_CONTROL_PERIOD_US) {
    if (!self->control.init_alignment) {
      self->control.init_alignment = true;
      self->control.angle_bias = WrapAngle(self->input.r22.angle
                                           - paired->input.r22.angle) / 2.0f;
      self->control.angle_feedback = 0.0f;
    } else {
      float jitter = (float)self->control.jitter / 1e6f;
      float k = (kControlParam.ts + jitter)
          / kControlParam.paired.angle_bias_tc;
      assert(0.0f <= k && k < 1.0f);
      float angle_bias_new = WrapAngle(self->input.r22.angle
                                       - paired->input.r22.angle) / 2.0f;
      float error = WrapAngle(self->control.angle_bias - angle_bias_new);
      self->control.angle_bias = WrapAngle(self->control.angle_bias
                                           - k * error);
    }
  }

  if (ServoIsPaired()) {
    PairedCurrentControl(self, paired, &self->control.angle_feedback);
  }
  ControlAngle(self, &self->control.desired_angle);
  if (self->state == kActuatorStateRunning
      && self->control.init_alignment && self->control.init_estimate) {
    self->control.current_limit = CurrentLimit(self);
    assert(self->control.current_limit >= 0);
  } else {
    self->control.current_limit = 0.0f;
  }
}

// Compute R22 output for paired servos.
void ServoControlPaired(ServoState *self, ServoState *paired) {
  ServoState self_copy = *self;
  ServoUpdateStatus(&self_copy, paired);
  ServoUpdateStatus(paired, self);
  self->control = self_copy.control;
  ServoControlPairedSync(&self_copy, paired);
  ServoControlPairedSync(paired, self);
  self->control = self_copy.control;
}

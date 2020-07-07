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

#include "avionics/motor/firmware/angle_meas.h"

#include <stdbool.h>
#include <stdint.h>

#include "avionics/firmware/drivers/ad7265.h"
#include "avionics/motor/firmware/calib_params.h"

// TODO: Add to flash calibration parameters.
MotorAngleMeasParam g_motor_angle_meas_param = {
  .sin_offset = 0.0f,
  .cos_offset = 0.0f,
  .sin_scale = 1.0f,
  .angle_meas_lower_warn_lim = 100.0f,
  .angle_meas_upper_warn_lim = 2047.0f,
};

static inline void CorrectSensor(int32_t sin_raw, int32_t cos_raw,
                                 MotorAngleMeas *meas) {
  meas->cos_m = (float)cos_raw - g_motor_angle_meas_param.cos_offset;
  meas->sin_m = ((float)sin_raw - g_motor_angle_meas_param.sin_offset)
      * g_motor_angle_meas_param.sin_scale;
}

static inline bool IsSensorValid(int32_t sin_m, int32_t cos_m) {
  int32_t lower_lim
      = (int32_t)g_motor_angle_meas_param.angle_meas_lower_warn_lim;
  int32_t upper_lim
      = (int32_t)g_motor_angle_meas_param.angle_meas_upper_warn_lim;
  int32_t sin2_cos2 = sin_m * sin_m + cos_m * cos_m;
  return lower_lim * lower_lim <= sin2_cos2
      && sin2_cos2 <= upper_lim * upper_lim;
}

void MotorAngleMeasInit(MotorAngleMeas *meas) {
  g_motor_angle_meas_param.sin_offset = kMotorCalibParams->sin_offset;
  g_motor_angle_meas_param.cos_offset = kMotorCalibParams->cos_offset;
  g_motor_angle_meas_param.sin_scale = kMotorCalibParams->sin_scale;

  int32_t unused;
  int32_t sin_raw = 0;
  int32_t cos_raw = 0;
  Ad7265Init();
  Ad7265Read(&sin_raw, &cos_raw, &unused, &unused);
  CorrectSensor(sin_raw, cos_raw, meas);
  meas->updated = true;
  meas->valid = IsSensorValid(sin_raw, cos_raw);

  // Trigger first transfer for MotorAngleMeasGet().
  Ad7265ReadAsyncStart();
}

void MotorAngleMeasGet(MotorAngleMeas *meas) {
  int32_t unused;
  int32_t sin_raw = 0;
  int32_t cos_raw = 0;
  if (Ad7265ReadAsyncGet(&sin_raw, &cos_raw, &unused, &unused)) {
    // Retrigger an asynchronous read of the angle sensor for the next pass.
    Ad7265ReadAsyncStart();
    CorrectSensor(sin_raw, cos_raw, meas);
    meas->updated = true;
    meas->valid = IsSensorValid(sin_raw, cos_raw);
  } else {
    meas->sin_m = 0;
    meas->cos_m = 0;
    meas->updated = false;
    meas->valid = false;
  }
}

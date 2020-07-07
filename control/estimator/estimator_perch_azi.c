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

#include "control/estimator/estimator_perch_azi.h"

#include <assert.h>
#include <string.h>

#include "common/c_math/filter.h"
#include "control/estimator/estimator_types.h"
#include "control/fault_detection/fault_detection_types.h"
#include "control/sensor_util.h"
#include "control/system_params.h"

void EstimatorPerchAziInit(EstimatorPerchAziState *state) {
  assert(state != NULL);
  memset(state, 0, sizeof(*state));
  state->last_valid_perch_azi_angle = 0.0;
  state->angle_vel_filter_state[0] = 0.0;
  state->angle_vel_filter_state[1] = 0.0;
}

// Populate the perch azimuth and azimuth angular rate estimate.
//
// If a fault is declared, the last valid angle is held.
void EstimatorPerchAziStep(double perch_azi_encoder,
                           bool perch_azi_encoder_valid,
                           const EstimatorPerchAziParams *params,
                           EstimatorPerchAziState *state,
                           PerchAziEstimate *perch_azi) {
  assert(params != NULL && state != NULL && perch_azi != NULL);

  // If there are any faults, hold last_valid_perch_azi_angle.
  if (perch_azi_encoder_valid) {
    perch_azi->valid = true;
    perch_azi->angle = perch_azi_encoder;
  } else {
    perch_azi->valid = false;
    perch_azi->angle = state->last_valid_perch_azi_angle;
  }

  // This call updates last_valid_perch_azi_angle to be equal to
  // perch_azi->angle.
  double angle_vel = DiffCircular(perch_azi->angle, 2.0 * PI, *g_sys.ts,
                                  &state->last_valid_perch_azi_angle);

  // This saturation handles the case where a perch azimuth fault
  // clears.  When this happens there can be a very large difference
  // between perch_azi->angle and last_valid_perch_azi_angle leading to large
  // angle_vel values.
  angle_vel =
      Saturate(angle_vel, -params->max_angle_vel, params->max_angle_vel);

  perch_azi->angle_vel_f =
      Lpf2(angle_vel, params->fc_angle_vel, params->damping_ratio_angle_vel,
           *g_sys.ts, state->angle_vel_filter_state);
}

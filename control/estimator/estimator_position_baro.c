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

#include "control/estimator/estimator_position_baro.h"

#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <string.h>

#include "control/estimator/estimator_types.h"
#include "control/fault_detection/fault_detection_types.h"
#include "control/sensor_types.h"
#include "control/sensor_util.h"
#include "control/system_params.h"

bool EstimatorPositionBaroValidateParams(
    const EstimatorPositionBaroParams *params) {
  assert(params != NULL);

  if (params->kalman_est.fc_min < 0.0 ||
      params->kalman_est.fc_min > params->kalman_est.fc_max ||
      params->kalman_est.Q <= 0.0) {
    assert(!(bool)"kalman_est fields out of range.");
    return false;
  }

  if (params->sigma_Xg_z_bias_0 <= 0.0) {
    assert(!(bool)"sigma_Xg_z_bias_0 must be positive.");
    return false;
  }

  if (params->sigma_Xg_z <= 0.0) {
    assert(!(bool)"sigma_Xg_z must be positive.");
    return false;
  }

  return true;
}

void EstimatorPositionBaroInit(const EstimatorPositionBaroParams *params,
                               EstimatorPositionBaroState *state) {
  assert(params != NULL && state != NULL);
  memset(state, 0, sizeof(*state));

  state->last_valid_Xg_z = 0.0;
  state->Xg_z_bias = 0.0;
  state->cov_Xg_z_bias = params->sigma_Xg_z_bias_0 * params->sigma_Xg_z_bias_0;
}

void EstimatorPositionBaroUpdateBias(double Xg_z, double sigma_Xg_z,
                                     const EstimatorPositionBaroEstimate *baro,
                                     const EstimatorPositionBaroParams *params,
                                     EstimatorPositionBaroState *state) {
  if (baro->valid) {
    double meas_cov =
        sigma_Xg_z * sigma_Xg_z + params->sigma_Xg_z * params->sigma_Xg_z;
    BoundedKalman1dEstimator(baro->Xg_z + state->Xg_z_bias - Xg_z, meas_cov,
                             *g_sys.ts, &params->kalman_est, NULL, NULL,
                             &state->Xg_z_bias, &state->cov_Xg_z_bias);
  }
}

void EstimatorPositionBaroStep(const Mat3 *dcm_g2b, const PitotData pitots[],
                               const FaultMask *pitot_high_speed_static_fault,
                               const FaultMask *pitot_low_speed_static_fault,
                               const PitotParams *pitot_params,
                               const EstimatorPositionBaroParams *params,
                               EstimatorPositionBaroState *state,
                               EstimatorPositionBaroEstimate *baro) {
  assert(pitots != NULL && pitot_high_speed_static_fault != NULL &&
         pitot_low_speed_static_fault != NULL && params != NULL &&
         state != NULL && baro != NULL);
  (void)pitot_low_speed_static_fault;

  baro->valid = !HasAnyFault(pitot_high_speed_static_fault) &&
                fabs(state->Xg_z_bias) < 2.0 * params->sigma_Xg_z_bias_0;

  if (baro->valid) {
    Vec3 pos_b2pitot_g;
    Mat3TransVec3Mult(dcm_g2b, &pitot_params->pos, &pos_b2pitot_g);
    state->last_valid_Xg_z =
        -PressureToAltitude(pitots[kPitotSensorHighSpeed].stat_press,
                            g_sys.phys) -
        pos_b2pitot_g.z;
  }

  baro->Xg_z = state->last_valid_Xg_z - state->Xg_z_bias;
  baro->sigma_Xg_z =
      Sqrt(params->sigma_Xg_z * params->sigma_Xg_z + state->cov_Xg_z_bias);
}

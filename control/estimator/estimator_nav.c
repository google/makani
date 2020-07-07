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

#include "control/estimator/estimator_nav.h"

#include <assert.h>
#include <stdbool.h>
#include <string.h>

#include "common/c_math/filter.h"
#include "common/macros.h"
#include "control/ground_frame.h"
#include "control/system_params.h"

void EstimatorNavInit(const EstimatorNavParams *params, double g_heading,
                      EstimatorNavState *state) {
  assert(params != NULL && state != NULL);
  memset(state, 0, sizeof(*state));

  state->vel_type_z1 = kEstimatorVelocitySolutionTypeDeadReckoned;
  for (int32_t i = 0; i < ARRAYSIZE(state->p_filter); ++i) {
    state->p_filter[i] = 0.0;
    state->q_filter[i] = 0.0;
    state->r_filter[i] = 0.0;
  }
  for (int32_t i = 0; i < ARRAYSIZE(state->acc_b_x_filter); ++i) {
    state->acc_b_x_filter[i] = 0.0;
    state->acc_b_y_filter[i] = 0.0;
    state->acc_b_z_filter[i] = 0.0;
  }
  state->acc_norm_f_z1 = kVec3Zero;
  for (int32_t i = 0; i < ARRAYSIZE(state->Vb_x_filter_state); ++i) {
    state->Vb_x_filter_state[i] = 0.0;
    state->Vb_y_filter_state[i] = 0.0;
    state->Vb_z_filter_state[i] = 0.0;
  }

  // Compute mag_g upon initialization and cache the value.
  RotNedToG(&g_sys.phys->mag_ned, g_heading, &state->mag_g);
}

// Determine if an IMU is giving valid data based on a fault array.
bool IsImuValid(const FaultMask imu_faults[]) {
  for (int32_t i = 0; i < kNumFaultDetectionImuSignals; ++i) {
    if (HasAnyFault(&imu_faults[i])) return false;
  }
  return true;
}

void FilterInertialMeasurements(const Vec3 *pqr, const Vec3 *acc_b,
                                const EstimatorNavParams *params,
                                EstimatorNavState *state, Vec3 *pqr_f,
                                Vec3 *acc_b_f, double *acc_norm_f) {
  pqr_f->x = Filter(pqr->x, ARRAYSIZE(params->vibration_filter_a),
                    params->vibration_filter_a, params->vibration_filter_b,
                    state->p_filter);
  pqr_f->y = Filter(pqr->y, ARRAYSIZE(params->vibration_filter_a),
                    params->vibration_filter_a, params->vibration_filter_b,
                    state->q_filter);
  pqr_f->z = Filter(pqr->z, ARRAYSIZE(params->vibration_filter_a),
                    params->vibration_filter_a, params->vibration_filter_b,
                    state->r_filter);

  acc_b_f->x = Filter(acc_b->x, ARRAYSIZE(params->vibration_filter_a),
                      params->vibration_filter_a, params->vibration_filter_b,
                      state->acc_b_x_filter);
  acc_b_f->y = Filter(acc_b->y, ARRAYSIZE(params->vibration_filter_a),
                      params->vibration_filter_a, params->vibration_filter_b,
                      state->acc_b_y_filter);
  acc_b_f->z = Filter(acc_b->z, ARRAYSIZE(params->vibration_filter_a),
                      params->vibration_filter_a, params->vibration_filter_b,
                      state->acc_b_z_filter);

  *acc_norm_f = Vec3Norm(
      LpfVec3(acc_b, params->fc_acc_norm, *g_sys.ts, &state->acc_norm_f_z1));
}

void FilterVelocity(const Mat3 *dcm_g2b, const Vec3 *Vb,
                    const EstimatorNavParams *params, EstimatorNavState *state,
                    Vec3 *Vb_f, Vec3 *Vg_f) {
  // Apply a filter to remove vibrations from the velocity estimate.
  Vb_f->x = Filter(Vb->x, ARRAYSIZE(params->Vb_filter_a), params->Vb_filter_a,
                   params->Vb_filter_b, state->Vb_x_filter_state);
  Vb_f->y = Filter(Vb->y, ARRAYSIZE(params->Vb_filter_a), params->Vb_filter_a,
                   params->Vb_filter_b, state->Vb_y_filter_state);
  Vb_f->z = Filter(Vb->z, ARRAYSIZE(params->Vb_filter_a), params->Vb_filter_a,
                   params->Vb_filter_b, state->Vb_z_filter_state);
  Mat3TransVec3Mult(dcm_g2b, Vb_f, Vg_f);
}

// Calculate representation of the inertial state in alternative coordinates.
void CalcAlternativeCoordinates(const Mat3 *dcm_g2b, const Vec3 *acc_b,
                                const Vec3 *acc_b_f, const Vec3 *Vg,
                                const SystemParams *system_params, Vec3 *Vb,
                                Vec3 *Ag, Vec3 *Ab_f) {
  Mat3Vec3Mult(dcm_g2b, Vg, Vb);
  Mat3Vec3Axpby(dcm_g2b, kTrans, acc_b, 1.0, &system_params->phys.g_g, Ag);
  Mat3Vec3Axpby(dcm_g2b, kNoTrans, &system_params->phys.g_g, 1.0, acc_b_f,
                Ab_f);
}

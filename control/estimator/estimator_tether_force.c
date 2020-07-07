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

#include "control/estimator/estimator_tether_force.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "common/c_math/filter.h"
#include "common/c_math/vec3.h"
#include "control/estimator/estimator_types.h"
#include "control/fault_detection/fault_detection_types.h"
#include "control/sensor_util.h"
#include "control/system_params.h"
#include "control/system_types.h"
#include "system/labels.h"

void EstimatorTetherForceInit(EstimatorTetherForceState *state) {
  assert(state != NULL);
  memset(state, 0, sizeof(*state));
  for (int32_t i = 0; i < kNumLoadcellSensors; ++i) {
    state->last_valid_loadcells[i] = 0.0;
  }
  state->vector_f_z1 = kVec3Zero;
}

void EstimatorTetherForceStep(const double loadcells[],
                              const FaultMask loadcell_faults[],
                              const WingParams *wing_params,
                              const LoadcellParams loadcell_params[],
                              const EstimatorTetherForceParams *params,
                              EstimatorTetherForceState *state,
                              TetherForceEstimate *tether_force_b) {
  assert(loadcells != NULL && loadcell_faults != NULL && wing_params != NULL &&
         loadcell_params != NULL && params != NULL && state != NULL &&
         tether_force_b != NULL);

  tether_force_b->valid = true;
  for (int32_t i = 0; i < kNumLoadcellSensors; ++i) {
    // Only update a loadcell value if it is fault free.
    if (HasAnyFault(&loadcell_faults[i])) {
      tether_force_b->valid = false;
    } else {
      state->last_valid_loadcells[i] = loadcells[i];
    }
  }

  LoadcellsToTetherForce(
      state->last_valid_loadcells, wing_params, loadcell_params,
      &tether_force_b->vector, &tether_force_b->sph,
      &tether_force_b->bridle_port_vector, &tether_force_b->bridle_star_vector);

  LpfVec3(&tether_force_b->vector, params->fc_tension, *g_sys.ts,
          &state->vector_f_z1);
  tether_force_b->vector_f = state->vector_f_z1;
  tether_force_b->tension_f = Vec3Norm(&state->vector_f_z1);
}

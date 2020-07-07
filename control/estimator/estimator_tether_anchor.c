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

#include "control/estimator/estimator_tether_anchor.h"

#include "common/c_math/filter.h"
#include "common/macros.h"
#include "control/sensor_util.h"
#include "control/system_params.h"

void EstimatorTetherAnchorPointInit(EstimatorTetherAnchorState *state) {
  state->z_z1 = 0.0;
}

void EstimatorTetherAnchorPointStep(bool initializing,
                                    const WinchEstimate *winch,
                                    const VesselEstimate *vessel, double payout,
                                    const EstimatorTetherAnchorParams *params,
                                    EstimatorTetherAnchorState *state,
                                    TetherAnchorEstimate *tether_anchor) {
  if (winch->valid && vessel->position_valid && vessel->attitude_valid) {
    tether_anchor->valid = true;
    CalcTetherAnchorPoint(winch->position, &vessel->dcm_g2p, &vessel->pos_g,
                          &tether_anchor->pos_g);
    state->last_valid = tether_anchor->pos_g;
  } else {
    tether_anchor->valid = false;
    tether_anchor->pos_g = state->last_valid;
  }

  if (initializing && tether_anchor->valid) {
    state->z_z1 = tether_anchor->pos_g.z;
    Lpf2Vec3Init(&tether_anchor->pos_g, params->fc_lateral,
                 params->zeta_lateral, *g_sys.ts, state->pos_zs);
  }

  Lpf2Vec3(&tether_anchor->pos_g, params->fc_lateral, params->zeta_lateral,
           *g_sys.ts, &tether_anchor->pos_g_f, state->pos_zs);

  // We would prefer a second-order filter here, but it turns out that the
  // second-order filter suffers from stability issues when the cutoff frequency
  // is linearly scaled. See b/137727704.
  double fc =
      Crossfade(params->fc_near_perch, params->fc_far_from_perch, payout,
                params->payout_near_perch, params->payout_far_from_perch);

  tether_anchor->pos_g_f.z =
      Lpf(tether_anchor->pos_g.z, fc, *g_sys.ts, &state->z_z1);
}

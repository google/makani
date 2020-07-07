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

#include "control/estimator/estimator_wind.h"

#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <string.h>

#include "common/c_math/filter.h"
#include "common/c_math/vec3.h"
#include "common/macros.h"
#include "control/control_params.h"
#include "control/control_types.h"
#include "control/estimator/estimator_types.h"
#include "control/fault_detection/fault_detection_types.h"
#include "control/ground_frame.h"
#include "control/sensor_util.h"
#include "control/system_params.h"
#include "control/system_types.h"

void EstimatorWindInit(EstimatorWindState *state) {
  assert(state != NULL);
  memset(state, 0, sizeof(*state));
  state->last_valid_wind_g = kVec3Zero;
  state->vector_f_z1 = kVec3Zero;
  state->vector_f_slow_z1 = kVec3Zero;
  state->speed_f_z1 = 0.0;
  for (int32_t i = 0; i < ARRAYSIZE(state->speed_f_pb_zs); i++) {
    state->speed_f_pb_zs[i] = 0.0;
  }
  for (int32_t i = 0; i < ARRAYSIZE(state->wind_direction_vector_f_zs); ++i) {
    state->wind_direction_vector_f_zs[i] = kVec3Zero;
  }
}

// Update WindEstimate using a new measurement.
static void UpdateWindEstimate(bool initializing, Vec3 *update,
                               WindSolutionType solution_type,
                               const EstimatorWindParams *params,
                               EstimatorWindState *state,
                               WindEstimate *wind_g) {
  if (solution_type == kWindSolutionTypeNone) {
    wind_g->valid = false;
  } else {
    state->last_valid_wind_g = *update;
    wind_g->valid = true;
  }

  wind_g->solution_type = solution_type;
  wind_g->vector = state->last_valid_wind_g;

  const double ts = *g_sys.ts;
  wind_g->vector_f = *LpfVec3(
      &wind_g->vector, initializing ? params->fc_initialize : params->fc_vector,
      ts, &state->vector_f_z1);
  wind_g->vector_f_slow =
      *LpfVec3(&wind_g->vector,
               initializing ? params->fc_initialize : params->fc_vector_slow,
               ts, &state->vector_f_slow_z1);

  double speed = Vec3Norm(&wind_g->vector);
  // TODO: Low-pass filtering the norm of the wind vector
  // likely rectifies variation in direction and over estimates the wind
  // speed.  Consider removing this estimate.
  wind_g->speed_f =
      Lpf(speed, initializing ? params->fc_initialize : params->fc_speed, ts,
          &state->speed_f_z1);
  wind_g->speed_f_playbook = Lpf2(
      speed, initializing ? params->fc_initialize : params->fc_speed_playbook,
      params->zeta_speed_playbook, ts, state->speed_f_pb_zs);

  // Low-pass filtered angle [rad] indicating the direction the wind is
  // pointing to in ground frame coordinate system.
  // Note that this definition is different than typical wind direction
  // notation that specifies the direction the wind is coming from, which is
  // used in setting wind direction in the sim configuration.
  // Advantages to using this algorithm for wind direction:
  // 1) Strong wind is weighted greater than weak wind.
  // 2) There is no -pi to pi averaging issue.
  // 3) In low wind, the direction will not jump around.
  // 4) If the wind sensor fails to zero, wind direction is held.
  double fc_dir = params->fc_dir * (1.0 - Exp10(-speed));
  Vec3 wind_direction_vector_f;
  Lpf2Vec3(&wind_g->vector, initializing ? params->fc_initialize : fc_dir,
           params->zeta_dir, *g_sys.ts, &wind_direction_vector_f,
           state->wind_direction_vector_f_zs);
  wind_g->dir_f = VecGToAzimuth(&wind_direction_vector_f);

  // A separate value is stored to enable a more aggressive filter for
  // playbook usage, or allow for additional manipulation (saturation, etc).
  double fc_dir_playbook = params->fc_dir_playbook * (1.0 - Exp10(-speed));
  Vec3 wind_direction_vector_f_playbook;
  Lpf2Vec3(
      &wind_g->vector, initializing ? params->fc_initialize : fc_dir_playbook,
      params->zeta_dir_playbook, *g_sys.ts, &wind_direction_vector_f_playbook,
      state->wind_direction_vector_f_zs_playbook);
  wind_g->dir_f_playbook = VecGToAzimuth(&wind_direction_vector_f_playbook);
}

void EstimatorWindStep(bool initializing, const Vec3 *wind_ws,
                       const FaultMask *wind_sensor_fault,
                       const VesselEstimate *vessel,
                       const WindSensorParams *wind_sensor_params,
                       const EstimatorWindParams *params,
                       EstimatorWindState *state, WindEstimate *wind_g) {
  assert(wind_ws != NULL && wind_sensor_fault != NULL && vessel != NULL &&
         wind_sensor_params != NULL && params != NULL && state != NULL &&
         wind_g != NULL);

  Vec3 vector;
  WindSolutionType solution_type;

  // Compute the new value and store it in wind_g->vector.
  if (*g_cont.control_opt & kControlOptHardCodeWind) {
    // Overwrite wind_g->vector if in kControlOptHardCodeWind mode.
    vector = params->hard_coded_wind_g;
    solution_type = kWindSolutionTypeHardcoded;
  } else if (!HasAnyFault(wind_sensor_fault) &&
             !(wind_sensor_params->on_perch &&
               !(vessel->position_valid && vessel->attitude_valid))) {
    // Use the ground station wind sensor if it is available.

    // TODO: There is a large difference in the update rates
    // of the wind sensor (updated at 20 Hz) and the perch azimuth
    // encoder (updated at 100 Hz).  To address this currently, we
    // should only update wind_g when a new wind sensor measurement
    // arrives.

    WindWsToWindG(wind_ws, &vessel->dcm_g2p, &vessel->pqr, &vessel->vel_g,
                  wind_sensor_params, &vector);
    solution_type = kWindSolutionTypeGroundStationSensor;
  } else {
    solution_type = kWindSolutionTypeNone;
  }

  UpdateWindEstimate(initializing, &vector, solution_type, params, state,
                     wind_g);
}

// Estimate winds aloft.  We could simply add apparent_wind_b and Vg,
// except that the estimator's intermediate solution types (i.e. other
// than from the Pitot and from the ground station sensor) are
// insufficiently accurate to produce a useful estimate of winds
// aloft.  Therefore we explicitly choose between using the
// Pitot-based apparent wind measurement or reporting the wind at the
// ground if it is not available.
void EstimatorWindAloftStep(bool initializing,
                            const ApparentWindEstimate *apparent_wind,
                            const ApparentWindSph *apparent_wind_pitot,
                            const Vec3 *Vg, const Mat3 *dcm_g2b,
                            const WindEstimate *wind_g,
                            const EstimatorWindParams *params,
                            EstimatorWindState *state,
                            WindEstimate *wind_aloft_g) {
  Vec3 wind_aloft_vector_g;
  WindSolutionType solution_type;

  if (apparent_wind->solution_type == kApparentWindSolutionTypePitot ||
      apparent_wind->solution_type == kApparentWindSolutionTypeComplementary) {
    // If we have a good measurement of apparent wind at the kite, we
    // can combine it with our inertial velocity to estimate the wind
    // at the kite.
    Vec3 apparent_wind_g;
    Vec3 apparent_wind_pitot_b;
    ApparentWindSphToCart(apparent_wind_pitot, &apparent_wind_pitot_b);
    Mat3TransVec3Mult(dcm_g2b, &apparent_wind_pitot_b, &apparent_wind_g);
    Vec3Add(&apparent_wind_g, Vg, &wind_aloft_vector_g);
    solution_type = kWindSolutionTypePitotAndInertial;
  } else if (wind_g->valid) {
    // When the Pitot is not available, the best we can do is to use
    // the ground station wind estimates; the other solution_types of
    // apparent_wind_b are not good enough.
    wind_aloft_vector_g = wind_g->vector;
    // Pass through the solution type, which should be either
    // kWindSolutionTypeGroundStationSensor or
    // kWindSolutionTypeHardcoded.
    solution_type = wind_g->solution_type;
  } else {
    solution_type = kWindSolutionTypeNone;
  }

  UpdateWindEstimate(initializing, &wind_aloft_vector_g, solution_type, params,
                     state, wind_aloft_g);
}

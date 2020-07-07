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

#include "control/estimator/estimator_attitude_ground.h"

#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "common/c_math/filter.h"
#include "common/c_math/mat3.h"
#include "common/c_math/quaternion.h"
#include "common/c_math/vec2.h"
#include "common/c_math/vec3.h"
#include "common/c_math/voting.h"
#include "common/macros.h"
#include "control/common.h"
#include "control/control_telemetry.h"
#include "control/control_types.h"
#include "control/estimator/estimator_filter.h"
#include "control/fault_detection/fault_detection_types.h"
#include "control/sensor_types.h"
#include "control/sensor_util.h"
#include "control/system_params.h"
#include "system/labels.h"

COMPILE_ASSERT(ARRAYSIZE(((EstimatorAttitudeFilterState *)NULL)->ud) ==
                   (kNumAttitudeStates + 1) * kNumAttitudeStates / 2,
               ud_vector_must_be_correct_size);

// Calculate the measured (ned-frame) and expected (platform frame) GPS compass
// vectors using parameters and measurements.
static void CalcCompassVecs(const GpsCompassData *gps_compass,
                            const GsGpsParams *gps_compass_params,
                            Vec3 *compass_p, Vec3 *compass_ned) {
  Vec3Sub(&gps_compass_params->secondary_antenna_p.pos,
          &gps_compass_params->primary_antenna_p.pos, compass_p);

  // Negate gps_compass->pitch because positive pitch corresponds to the
  // -Z direction.
  SphToCart(gps_compass->heading, -gps_compass->pitch, Vec3Norm(compass_p),
            compass_ned);
}

// Run a single update step of the attitude filter.
//
// Args:
//   gyro: Gyro measurement [rad/s].
//   acc: Accelerometer measurement [m/s^2].
//   f_sf_g: Expected specific force measurement [m/s^2] in g-coordinates.
//   gps_compass: GPS compass data
//   gps_compass_params: GPS compass parameters
//   gps_compass_faults: GPS compass faults
//   params: Parameters.
//   state: State.
//   correct: The filter corrections.
static void RunFilter(const Vec3 *gyro, const Vec3 *acc, const Vec3 *f_sf_g,
                      const GpsCompassData *gps_compass,
                      const GsGpsParams *gps_compass_params,
                      const FaultMask *gps_compass_faults,
                      const EstimatorAttitudeParams *params,
                      EstimatorAttitudeFilterState *state,
                      EstimatorAttitudeCorrections *correct) {
  VEC_INIT(kNumAttitudeStates, x_hat, {0});

  if (!HasAnyFault(
          &gps_compass_faults[kFaultDetectionGpsCompassSignalAngles]) &&
      gps_compass->new_data) {
    Vec3 compass_p, compass_ned;
    CalcCompassVecs(gps_compass, gps_compass_params, &compass_p, &compass_ned);

    double sigma_compass =
        hypot(gps_compass->heading_sigma, gps_compass->pitch_sigma);

    if (sigma_compass < params->max_gps_compass_sigma) {
      AttitudeFilterCorrectVector(&compass_ned, &compass_p,
                                  sigma_compass * Vec3Norm(&compass_p), &x_hat,
                                  state, &correct->gps_compass);
    }
  }

  double sigma_acc =
      fmax(params->plumb_bob_relative_err * g_sys.phys->g,
           params->plumb_bob_g_err_scale * fabs(g_sys.phys->g - Vec3Norm(acc)));
  AttitudeFilterCorrectVector(f_sf_g, acc, sigma_acc, &x_hat, state,
                              &correct->gravity_vector);

  ApplyAttitudeErrorState(&x_hat, params, state);
  AttitudeFilterPropagate(gyro, params, state);
}

void EstimatorAttitudeGroundStep(
    bool initialize, const ImuData *imu, const GpsCompassData *gps_compass,
    const ImuParams *imu_params, const EstimatorAttitudeParams *params,
    const FaultMask *imu_faults, const GsGpsParams *gps_compass_params,
    const FaultMask *gps_compass_faults, EstimatorAttitudeState *state,
    Vec3 *pqr, Quat *q_g2p, Vec3 *acc_p,
    EstimatorAttitudeCorrections *correct) {
  assert(imu != NULL && gps_compass != NULL && gps_compass_params != NULL &&
         gps_compass_faults != NULL && imu_params != NULL && params != NULL &&
         imu_faults != NULL && state != NULL && pqr != NULL && q_g2p != NULL &&
         acc_p != NULL && correct != NULL);

  // TODO(b/136982444): Check IMU faults.
  UNUSED(imu_faults);

  Vec3 f_sf_g;
  Vec3Scale(&g_sys.phys->g_g, -1.0, &f_sf_g);
  memset(correct, 0, sizeof(*correct));

  if (initialize) {
    Vec3 compass_p, compass_ned;
    CalcCompassVecs(gps_compass, gps_compass_params, &compass_p, &compass_ned);
    state->acc_f_z1 = imu->acc;

    // Here we pass the GPS compass vectors in lieu of the magnetometer vectors.
    RunCoarseInitialization(&imu->gyro, &state->acc_f_z1, &f_sf_g, &compass_p,
                            &compass_ned, params, &state->filter);
  } else {
    LpfVec3(&imu->acc, params->fc_acc, *g_sys.ts, &state->acc_f_z1);
    RunFilter(&imu->gyro, &state->acc_f_z1, &f_sf_g, gps_compass,
              gps_compass_params, gps_compass_faults, params, &state->filter,
              correct);
  }

  *q_g2p = state->filter.q_g2b;
  Vec3Sub(&imu->gyro, &state->filter.gyro_bias, pqr);

  // Correct accelerometers for centrifugal accelerations (there are no
  // Coriolis accelerations as the IMUs are fixed).  Note that
  // tangential acceleration is intentionally omitted, as it is
  // difficult to estimate.
  Vec3 omega_cross_pos;
  Vec3Cross(pqr, &imu_params->pos, &omega_cross_pos);
  Vec3Cross(pqr, &omega_cross_pos, acc_p);
  Vec3Sub(&imu->acc, acc_p, acc_p);
}

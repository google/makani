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

#include "control/estimator/estimator_nav_kite.h"

#include <assert.h>
#include <stdbool.h>
#include <string.h>

#include "common/c_math/mat3.h"
#include "common/c_math/vec3.h"
#include "common/c_math/voting.h"
#include "common/macros.h"
#include "control/control_telemetry.h"
#include "control/control_types.h"
#include "control/estimator/estimator_attitude_kite.h"
#include "control/estimator/estimator_position_kite.h"
#include "control/estimator/estimator_types.h"
#include "control/fault_detection/fault_detection_types.h"
#include "control/ground_frame.h"
#include "control/perch_frame.h"
#include "control/sensor_types.h"
#include "control/sensor_util.h"
#include "control/system_params.h"
#include "control/system_types.h"

void EstimatorNavKiteInit(const EstimatorNavParams *params, double g_heading,
                          EstimatorNavKiteState *state) {
  assert(params != NULL && state != NULL);
  memset(state, 0, sizeof(*state));

  EstimatorNavInit(params, g_heading, &state->estimator_nav_state);
  state->last_used_imu = kWingImuA;
  for (int32_t i = 0; i < kNumWingImus; ++i) {
    EstimatorAttitudeInit(&params->attitude, &state->attitude[i]);
  }
  EstimatorPositionKiteInit(&params->position, &state->position);
  state->Vg_f_z1 = kVec3Zero;
}

// Combines triple redundant data and indicated faults into a single estimated
// attitude, angular rates, and specific force measurements in the body frame.
static void SelectEstimates(const bool imu_valids[], const Vec3 pqr_estimates[],
                            const Quat q_g2b_estimates[],
                            const Vec3 acc_b_estimates[],
                            EstimatorNavKiteState *state, Vec3 *pqr,
                            Mat3 *dcm_g2b, Vec3 *acc_b) {
  int32_t num_valid = 0;
  int32_t valid_index = 0;
  for (int32_t i = kNumWingImus - 1; i >= 0; --i) {
    if (imu_valids[i]) {
      num_valid++;
      valid_index = i;
    }
  }

  // If only one IMU is faulted, continue to apply median voting.
  //
  // This is motivated by the concern that we may have a single IMU
  // rapidly being declared faulted / non-faulted.  If there is a
  // non-neglible difference in the three attitude estimates, this
  // could result in chattering between the median solution and the
  // and a single IMU solution.
  //
  // Chatter can still occur if:
  //   - A single IMU behaves wildly but the fault is not detected.
  //   - Faults are rapidly declared and cleared on two IMUs.
  Quat q;
  if (num_valid >= 2) {
    // Median voting at the output of the estimator can potentially
    // hurt dead reckoning performance, however selecting one of the
    // three IMUs in a stateless manner can lead to chatter.
    //
    // TODO: Evaluate using:
    //   1) A "hot spare" configuration where on IMU is for monitoring only.
    //   2) Voting after the position estimator.
    Median3Quat(&q_g2b_estimates[0], &q_g2b_estimates[1], &q_g2b_estimates[2],
                &q);
    Median3Vec3(&pqr_estimates[0], &pqr_estimates[1], &pqr_estimates[2], pqr);
    Median3Vec3(&acc_b_estimates[0], &acc_b_estimates[1], &acc_b_estimates[2],
                acc_b);

    // If all three IMUs are simultaneously declared faulted next
    // iteration, default to using the lowest index valid IMU.
    state->last_used_imu = valid_index;
  } else {
    // Make use of the only currently valid IMU's output, or if all
    // IMUs are declared faulted use the last one you had available.
    int32_t index = state->last_used_imu;
    if (!imu_valids[state->last_used_imu] && num_valid > 0) index = valid_index;

    q = q_g2b_estimates[index];
    *pqr = pqr_estimates[index];
    *acc_b = acc_b_estimates[index];
    state->last_used_imu = index;
  }

  QuatToDcm(&q, dcm_g2b);
}

// Run the position estimator.
void EstimatorNavKiteStep(
    bool initializing, const ImuData imus[], const GpsData wing_gps[],
    const GsGpsData *gs_gps, const PitotData pitots[],
    const GroundStationPoseEstimate *ground_station, const WindEstimate *wind_g,
    const EncodersEstimate *encoders, const PerchAziEstimate *perch_azi,
    const TetherForceEstimate *tether_force_b, const WinchEstimate *winch,
    FlightMode flight_mode, const FaultMask faults[],
    const SystemParams *system_params, const EstimatorNavParams *params,
    EstimatorNavKiteState *state, Vec3 *pqr, Mat3 *dcm_g2b, Vec3 *Ag,
    bool *gps_active, Vec3 *Vg, Vec3 *Xg, Vec3 *pqr_f, Vec3 *Ab_f, Vec3 *acc_b,
    double *acc_norm_f, Vec3 *Vb, Vec3 *Vb_f, Vec3 *Vg_f) {
  assert(imus != NULL && wing_gps != NULL && gs_gps != NULL && pitots != NULL &&
         ground_station != NULL && wind_g != NULL && encoders != NULL &&
         perch_azi != NULL && tether_force_b != NULL && winch != NULL &&
         faults != NULL && system_params != NULL && params != NULL &&
         state != NULL && pqr != NULL && dcm_g2b != NULL && Ag != NULL &&
         gps_active != NULL && Vg != NULL && Xg != NULL && pqr_f != NULL &&
         Ab_f != NULL && acc_b != NULL && acc_norm_f != NULL && Vb != NULL &&
         Vb_f != NULL && Vg_f != NULL);
  // Handle attitude estimation.
  bool imu_valids[kNumWingImus];
  Vec3 pqr_estimates[kNumWingImus];
  Quat q_g2b_estimates[kNumWingImus];
  Vec3 acc_b_estimates[kNumWingImus];
  EstimatorAttitudeCorrections corrections[kNumWingImus];
  for (int32_t i = 0; i < kNumWingImus; ++i) {
    const FaultMask *imu_faults = GetImuFaults(faults, i);
    imu_valids[i] = IsImuValid(imu_faults);
    EstimatorAttitudeKiteStep(
        initializing, &imus[i], state->estimator_nav_state.vel_type_z1,
        &state->Vg_f_z1, wind_g, ground_station,
        &state->estimator_nav_state.mag_g, &wing_gps[kWingGpsReceiverCrosswind],
        &system_params->wing_gps[kWingGpsReceiverCrosswind],
        GetWingGpsPosFault(faults, kWingGpsReceiverCrosswind),
        &wing_gps[kWingGpsReceiverPort],
        &system_params->wing_gps[kWingGpsReceiverPort],
        GetWingGpsPosFault(faults, kWingGpsReceiverPort),
        &wing_gps[kWingGpsReceiverStar],
        &system_params->wing_gps[kWingGpsReceiverStar],
        GetWingGpsPosFault(faults, kWingGpsReceiverStar), flight_mode,
        &system_params->wing_imus[i], &params->attitude, imu_faults,
        &state->attitude[i], &pqr_estimates[i], &q_g2b_estimates[i],
        &acc_b_estimates[i], &corrections[i]);
  }
  SelectEstimates(imu_valids, pqr_estimates, q_g2b_estimates, acc_b_estimates,
                  state, pqr, dcm_g2b, acc_b);

  // Handle positon estimation.
  EstimatorPositionKiteStep(acc_b, pqr, dcm_g2b, wing_gps, gs_gps,
                            ground_station, encoders, perch_azi, tether_force_b,
                            winch, pitots, flight_mode, faults, system_params,
                            &params->position, &state->position, Xg, Vg,
                            &state->estimator_nav_state.vel_type_z1);
  // Update filtered inertial velocity used in crosswind attitude estimation.
  LpfVec3(Vg, params->fc_Vg, *g_sys.ts, &state->Vg_f_z1);
  *gps_active = state->estimator_nav_state.vel_type_z1 ==
                kEstimatorVelocitySolutionTypeGps;

  // Produce post-processed outputs for the controller.
  Vec3 acc_b_f;
  FilterInertialMeasurements(pqr, acc_b, params, &state->estimator_nav_state,
                             pqr_f, &acc_b_f, acc_norm_f);

  CalcAlternativeCoordinates(dcm_g2b, acc_b, &acc_b_f, Vg, system_params, Vb,
                             Ag, Ab_f);

  FilterVelocity(dcm_g2b, Vb, params, &state->estimator_nav_state, Vb_f, Vg_f);

  // Update telemetry.
  EstimatorTelemetry *est = GetEstimatorTelemetry();
  for (int32_t i = 0; i < kNumWingImus; ++i) {
    est->q_g2b[i] = state->attitude[i].filter.q_g2b;
    est->gyro_biases[i] = state->attitude[i].filter.gyro_bias;
    EstimatorAttitudeGetCovariances(
        &state->attitude[i], &est->cov_attitude_err[i], &est->cov_gyro_bias[i]);
    est->acc_b_estimates[i] = acc_b_estimates[i];
    est->attitude_corrections[i] = corrections[i];
  }
}

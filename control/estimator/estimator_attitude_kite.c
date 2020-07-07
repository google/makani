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

#include "control/estimator/estimator_attitude_kite.h"

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

// Require that the two GPS updates are from the same measurement epoch. We
// also rate limit GPS corrections to address concerns with potential control
// coupling with the structural modes.
static bool CheckGpsMeasurementEpochs(const GpsData *a, const GpsData *b) {
  return (a->new_data || b->new_data) &&
         a->time_of_week_ms == b->time_of_week_ms &&
         a->time_of_week_ms % 500 == 0;  // Rate limit to 2 Hz.
}

// Apply an attitude correction given two wing GPS position measurements. We
// compute the measured vector from first GPS position to the second GPS
// position in the g coordinate frame, then relate this vector to the known
// antenna positions within body frame.
static bool ApplyGpsVectorCorrection(
    const GroundStationPoseEstimate *ground_station, const GpsData *gps_a_data,
    const GpsParams *gps_a_params, const FaultMask *gps_a_faults,
    const GpsData *gps_b_data, const GpsParams *gps_b_params,
    const FaultMask *gps_b_faults, double wing_a_to_b_sigma,
    double wing_a_to_b_sigma_ratio, const EstimatorAttitudeParams *params,
    Vec *x_hat, EstimatorAttitudeFilterState *state,
    EstimatorAttitudeCorrection3 *correct) {
  if (ground_station->valid && !HasAnyFault(gps_a_faults) &&
      gps_a_data->pos_sol_type != kGpsSolutionTypeNone &&
      !HasAnyFault(gps_b_faults) &&
      gps_b_data->pos_sol_type != kGpsSolutionTypeNone &&
      CheckGpsMeasurementEpochs(gps_a_data, gps_b_data)) {
    // Compute the position of the a and b GPS antennas in the
    // ground frame.
    Vec3 Xg_a_gps, Xg_b_gps;
    PoseTransform(&ground_station->dcm_ecef2g, &ground_station->pos_ecef,
                  &gps_a_data->pos, &Xg_a_gps);
    PoseTransform(&ground_station->dcm_ecef2g, &ground_station->pos_ecef,
                  &gps_b_data->pos, &Xg_b_gps);

    // Compute the relative position vector pointing from a to b
    // in the ground frame using GPS data.
    Vec3 X_a_to_b_g;
    Vec3Sub(&Xg_b_gps, &Xg_a_gps, &X_a_to_b_g);
    const double measured_distance = Vec3Norm(&X_a_to_b_g);

    // Compute the relative position vector pointing from a to b
    // in the body frame using the physical positions.
    Vec3 X_a_to_b_b;
    Vec3Sub(&gps_b_params->pos, &gps_a_params->pos, &X_a_to_b_b);
    const double physical_distance = Vec3Norm(&X_a_to_b_b);

    // We need a reasonable baseline for this correction to work.
    assert(physical_distance > wing_a_to_b_sigma);

    // Compute the relative position vector standard deviation.
    const double meas_error = measured_distance - physical_distance;
    const double meas_sigma =
        fabs(meas_error) * params->gps_vector_relative_err;
    const double a_to_b_sigma =
        sqrt(wing_a_to_b_sigma * wing_a_to_b_sigma + meas_sigma * meas_sigma +
             Vec3NormSquared(&gps_a_data->pos_sigma) +
             Vec3NormSquared(&gps_b_data->pos_sigma));

    // Reject measurements with a large position standard deviation.
    if (Vec3Norm(&gps_a_data->pos_sigma) > params->max_gps_position_sigma) {
      return false;
    }
    if (Vec3Norm(&gps_b_data->pos_sigma) > params->max_gps_position_sigma) {
      return false;
    }

    // Reject measurements that do not agree with the known antenna locations.
    if (fabs(meas_error) > params->max_gps_vector_error) {
      return false;
    }

    // Reject measurements with a large standard deviation relative to the
    // baseline.
    if (physical_distance < wing_a_to_b_sigma_ratio * a_to_b_sigma) {
      return false;
    }

    // Apply correction.
    AttitudeFilterCorrectVector(&X_a_to_b_g, &X_a_to_b_b, a_to_b_sigma, x_hat,
                                state, correct);
    return true;
  }
  return false;
}

// Run a single update step of the attitude filter.
//
// Args:
//   flight_mode: Current flight mode.
//   gyro: Gyro measurement [rad/s].
//   acc: Accelerometer measurement [m/s^2].
//   f_sf_g: Expected specific force measurement [m/s^2] in g-coordinates.
//   mag: Magnetometer measurements [Gauss].
//   mag_g: Expected magnetometer measurement [Gauss] in g-coordinates.
//   vel_type: Current velocity solution type.
//   Vg_z1: Kite velocity [m/s] from previous control cycle in g frame.
//   wind_g: Wind velocity [m/s] in g frame.
//   ground_station: Ground station frame estimate.
//   gps_center_data: GPS data from the center position.
//   gps_center_params: GPS parameters for the center position.
//   gps_center_faults: GPS position faults for the center position.
//   gps_port_data: GPS data from the port wingtip.
//   gps_port_params: GPS parameters for the port wingtip.
//   gps_port_faults: GPS position faults for the port wingtip.
//   gps_star_data: GPS data from the starboard wingtip.
//   gps_star_params: GPS parameters for the starboard wingtip.
//   gps_star_faults: GPS position faults for the starboard wingtip.
//   params: Parameters.
//   state: State.
//   correct: The filter corrections.
static void RunFilter(
    FlightMode flight_mode, const Vec3 *gyro, const Vec3 *acc,
    const Vec3 *f_sf_g, const Vec3 *mag, const Vec3 *mag_g,
    EstimatorVelocitySolutionType vel_type, const Vec3 *Vg_z1,
    const WindEstimate *wind_g, const GroundStationPoseEstimate *ground_station,
    const GpsData *gps_center_data, const GpsParams *gps_center_params,
    const FaultMask *gps_center_faults, const GpsData *gps_port_data,
    const GpsParams *gps_port_params, const FaultMask *gps_port_faults,
    const GpsData *gps_star_data, const GpsParams *gps_star_params,
    const FaultMask *gps_star_faults, const EstimatorAttitudeParams *params,
    const FaultMask *imu_faults, EstimatorAttitudeFilterState *state,
    EstimatorAttitudeCorrections *correct) {
  VEC_INIT(kNumAttitudeStates, x_hat, {0});

  if (!HasAnyFault(&imu_faults[kFaultDetectionImuSignalMag])) {
    AttitudeFilterCorrectVector(mag_g, mag,
                                params->mag_relative_err * Vec3Norm(mag_g),
                                &x_hat, state, &correct->magnetometer);
  }

  bool applied_gps_vector_correction = false;
  if (params->enable_gps_vector_correction) {
    applied_gps_vector_correction |= ApplyGpsVectorCorrection(
        ground_station, gps_port_data, gps_port_params, gps_port_faults,
        gps_star_data, gps_star_params, gps_star_faults,
        params->wing_port_to_star_sigma, params->wing_vector_sigma_ratio,
        params, &x_hat, state, &correct->gps_port_to_star);

    applied_gps_vector_correction |= ApplyGpsVectorCorrection(
        ground_station, gps_port_data, gps_port_params, gps_port_faults,
        gps_center_data, gps_center_params, gps_center_faults,
        params->wing_wingtip_to_center_sigma, params->wing_vector_sigma_ratio,
        params, &x_hat, state, &correct->gps_port_to_center);

    applied_gps_vector_correction |= ApplyGpsVectorCorrection(
        ground_station, gps_star_data, gps_star_params, gps_star_faults,
        gps_center_data, gps_center_params, gps_center_faults,
        params->wing_wingtip_to_center_sigma, params->wing_vector_sigma_ratio,
        params, &x_hat, state, &correct->gps_star_to_center);

    // Set applied_gps_vector_correction true while the gps_vector_timer is
    // less than the gps_vector_timeout value to prevent apparent wind
    // corrections between GPS updates (20 Hz).
    if (applied_gps_vector_correction) {
      state->gps_vector_timer_cycles = 0;
    } else if (state->gps_vector_timer_cycles < INT32_MAX) {
      ++state->gps_vector_timer_cycles;
    }
    applied_gps_vector_correction |=
        state->gps_vector_timer_cycles < params->gps_vector_timeout_cycles;
  }

  if (!AnyCrosswindFlightMode(flight_mode)) {
    // When not flying crosswind, plumb bob gravity is used as a vector
    // measurement.
    double sigma_acc = fmax(
        params->plumb_bob_relative_err * g_sys.phys->g,
        params->plumb_bob_g_err_scale * fabs(g_sys.phys->g - Vec3Norm(acc)));
    AttitudeFilterCorrectVector(f_sf_g, acc, sigma_acc, &x_hat, state,
                                &correct->gravity_vector);

  } else if (vel_type != kEstimatorVelocitySolutionTypeDeadReckoned) {
    // When flying crosswind we assume that the wind vector
    // measurement and inertial speed can be combined to approximate
    // the apparent wind in g-coordinates, and that the kite is flying
    // near its nominal AOA and AOS.
    //
    // This correction is turned off if we are not receiving velocity
    // updates.
    if (params->enable_apparent_wind_correction &&
        !applied_gps_vector_correction && wind_g->valid) {
      Vec3 V_app_g;
      Vec3Sub(Vg_z1, &wind_g->vector, &V_app_g);
      Vec3 V_app_b;
      ApparentWindSph tmp = {.alpha = params->nominal_angle_of_attack,
                             .beta = params->nominal_angle_of_sideslip,
                             .v = -Vec3Norm(&V_app_g)};
      ApparentWindSphToCart(&tmp, &V_app_b);

      AttitudeFilterCorrectVector(
          &V_app_g, &V_app_b,
          params->v_app_relative_err *
              fmax(Vec3Norm(&V_app_g), params->v_app_relative_err_min_airspeed),
          &x_hat, state, &correct->apparent_wind);
    }
  }

  ApplyAttitudeErrorState(&x_hat, params, state);
  AttitudeFilterPropagate(gyro, params, state);
}

void EstimatorAttitudeKiteStep(
    bool initialize, const ImuData *imu, EstimatorVelocitySolutionType vel_type,
    const Vec3 *Vg_z1, const WindEstimate *wind_g,
    const GroundStationPoseEstimate *ground_station, const Vec3 *mag_g,
    const GpsData *gps_center_data, const GpsParams *gps_center_params,
    const FaultMask *gps_center_faults, const GpsData *gps_port_data,
    const GpsParams *gps_port_params, const FaultMask *gps_port_faults,
    const GpsData *gps_star_data, const GpsParams *gps_star_params,
    const FaultMask *gps_star_faults, FlightMode flight_mode,
    const ImuParams *imu_params, const EstimatorAttitudeParams *params,
    const FaultMask *imu_faults, EstimatorAttitudeState *state, Vec3 *pqr,
    Quat *q_g2b, Vec3 *acc_b, EstimatorAttitudeCorrections *correct) {
  assert(imu != NULL && Vg_z1 != NULL && wind_g != NULL &&
         ground_station != NULL && mag_g != NULL && gps_center_data != NULL &&
         gps_center_params != NULL && gps_center_faults != NULL &&
         gps_port_data != NULL && gps_port_params != NULL &&
         gps_port_faults != NULL && gps_star_data != NULL &&
         gps_star_params != NULL && gps_star_faults != NULL &&
         imu_params != NULL && params != NULL && state != NULL && pqr != NULL &&
         q_g2b != NULL && acc_b != NULL && correct != NULL);

  Vec3 f_sf_g;
  Vec3Scale(&g_sys.phys->g_g, -1.0, &f_sf_g);
  memset(correct, 0, sizeof(*correct));

  if (initialize) {
    state->acc_f_z1 = imu->acc;
    RunCoarseInitialization(&imu->gyro, &state->acc_f_z1, &f_sf_g, &imu->mag,
                            mag_g, params, &state->filter);
  } else {
    LpfVec3(&imu->acc, params->fc_acc, *g_sys.ts, &state->acc_f_z1);
    RunFilter(flight_mode, &imu->gyro, &state->acc_f_z1, &f_sf_g, &imu->mag,
              mag_g, vel_type, Vg_z1, wind_g, ground_station, gps_center_data,
              gps_center_params, gps_center_faults, gps_port_data,
              gps_port_params, gps_port_faults, gps_star_data, gps_star_params,
              gps_star_faults, params, imu_faults, &state->filter, correct);
  }

  *q_g2b = state->filter.q_g2b;
  Vec3Sub(&imu->gyro, &state->filter.gyro_bias, pqr);

  // Correct accelerometers for centrifugal accelerations (there are no
  // Coriolis accelerations as the IMUs are fixed).  Note that
  // tangential acceleration is intentionally omitted, as it is
  // difficult to estimate.
  Vec3 omega_cross_pos;
  Vec3Cross(pqr, &imu_params->pos, &omega_cross_pos);
  Vec3Cross(pqr, &omega_cross_pos, acc_b);
  Vec3Sub(&imu->acc, acc_b, acc_b);
}

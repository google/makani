// Copyright 2020 Makani Technologies LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "control/estimator/estimator_attitude_kite.h"

#include <gtest/gtest.h>

#include <math.h>
#include <stdlib.h>

#include "common/c_math/util.h"
#include "common/macros.h"
#include "control/control_params.h"
#include "control/control_types.h"
#include "control/ground_frame.h"
#include "control/sensor_types.h"
#include "control/system_params.h"
#include "lib/util/test_util.h"

using ::test_util::LpfIterationTolerance;
using ::test_util::Rand;
using ::test_util::RandNormal;

extern "C" {

void MahonyFilter(const Vec3 *pqr, double Kp, double Ki, const Vec3 *vi,
                  const Vec3 *vb, const double *k, const double *fc,
                  int32_t num_vec, double ts, double bias_bound_low,
                  double bias_bound_high, Quat *q, Vec3 *bias, Vec3 *ef);

void RunFilter(
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
    EstimatorAttitudeCorrections *correct);

}  // extern "C"

namespace {

static const int32_t kOneSecond = static_cast<int32_t>(1.0 / (*g_sys.ts));

}  // namespace

TEST(MahonyFilter, Zero) {
  const Quat q_0 = {sqrt(2.0) / 2.0, 0.0, sqrt(2.0) / 2.0, 0.0};
  const Vec3 pqr = kVec3Zero;
  const Vec3 Ag = {0.0, 0.0, -9.81};
  const Vec3 acc = {9.81, 0.0, 0.0};
  const Vec3 mag_ned = {0.228, 0.0573, 0.4288};
  Vec3 mag;
  QuatRotate(&q_0, &mag_ned, &mag);
  Mat3 dcm_i2b;
  QuatToDcm(&q_0, &dcm_i2b);

  // These are a typical set of parameters taken from the controller
  // in commit: 29b2e01453.  There's nothing special about them for
  // the purposes of this test.
  const int32_t num_mahony_vec = 3;
  const double Kp = 3.7;
  const double Ki = 0.09;
  const double kp_crosswind[num_mahony_vec] = {0.667, 0.333, 0.0};
  const double fc[num_mahony_vec] = {0.0, 0.1, 0.1};
  const double ts = 0.01;
  const Vec2 pqr_bias_bounds = {-0.5, 0.5};

  const Vec3 vi[num_mahony_vec] = {kVec3Zero, Ag, mag_ned};
  const Vec3 vb[num_mahony_vec] = {kVec3Zero, acc, mag};

  Quat q = q_0;
  Vec3 bias = kVec3Zero;
  Vec3 ef_z1[num_mahony_vec] = {kVec3Zero, kVec3Zero, kVec3Zero};
  Mat3 dcm_i2b_out;
  MahonyFilter(&pqr, Kp, Ki, vi, vb, kp_crosswind, fc, num_mahony_vec, ts,
               pqr_bias_bounds.x, pqr_bias_bounds.y, &q, &bias, ef_z1);
  QuatToDcm(&q, &dcm_i2b_out);

  EXPECT_NEAR_MAT3(dcm_i2b_out, dcm_i2b, 1e-9);
  EXPECT_NEAR_VEC3(bias, kVec3Zero, 1e-9);
}

TEST(RunFilter, Convergence) {
  const Quat qs[] = {{1.0, 0.0, 0.0, 0.0},
                     {0.7071, 0.7071, 0.0, 0.0},
                     {0.7071, -0.7071, 0.0, 0.0},
                     {0.7071, 0.0, 0.7071, 0.0},
                     {0.7071, 0.0, -0.7071, 0.0}};
  const Vec3 bias = {0.02, -0.01, 0.01};

  const EstimatorAttitudeParams &params =
      GetControlParams()->estimator.nav.attitude;

  for (int32_t q_i = 0; q_i < ARRAYSIZE(qs); ++q_i) {
    EstimatorAttitudeState state;
    EstimatorAttitudeInit(&params, &state);

    const Vec3 mag_g = g_sys.phys->mag_ned;
    Vec3 mag;
    QuatRotate(&qs[q_i], &mag_g, &mag);
    Vec3 f_sf_g;
    Vec3Scale(&g_sys.phys->g_g, -1.0, &f_sf_g);
    Vec3 acc;
    QuatRotate(&qs[q_i], &f_sf_g, &acc);

    // Clear all faults.
    FaultMask faults[kNumSubsystems];
    memset(faults, 0, sizeof(faults));

    // This test does not use the wind sensor.
    WindEstimate wind_g;
    wind_g.valid = false;

    // This test does not use the GPS sensors.
    GpsData gps_center_data, gps_port_data, gps_star_data;
    memset(&gps_center_data, 0, sizeof(gps_center_data));
    memset(&gps_port_data, 0, sizeof(gps_port_data));
    memset(&gps_star_data, 0, sizeof(gps_star_data));
    GpsParams gps_center_params, gps_port_params, gps_star_params;
    memset(&gps_center_params, 0, sizeof(gps_center_params));
    memset(&gps_port_params, 0, sizeof(gps_port_params));
    memset(&gps_star_params, 0, sizeof(gps_star_params));
    GroundStationPoseEstimate ground_station;
    memset(&ground_station, 0, sizeof(ground_station));
    const double g_heading = 0.0;
    ground_station.pos_ecef = kVec3Zero;
    CalcDcmEcefToG(&ground_station.pos_ecef, g_heading,
                   &ground_station.dcm_ecef2g);

    EstimatorAttitudeCorrections correct;
    memset(&correct, 0, sizeof(correct));

    // Test that the attitude settles quickly with the initial gains
    // and no bias.
    for (int32_t i = 0; i < 10 * kOneSecond; ++i) {
      RunFilter(kFlightModePerched, &kVec3Zero, &acc, &f_sf_g, &mag, &mag_g,
                kEstimatorVelocitySolutionTypeGps, &kVec3Zero, &wind_g,
                &ground_station, &gps_center_data, &gps_center_params,
                GetWingGpsPosFault(faults, kWingGpsReceiverCrosswind),
                &gps_port_data, &gps_port_params,
                GetWingGpsPosFault(faults, kWingGpsReceiverPort),
                &gps_star_data, &gps_star_params,
                GetWingGpsPosFault(faults, kWingGpsReceiverStar), &params,
                &faults[SUBSYS_IMU_A], &state.filter, &correct);
    }
    EXPECT_NEAR_QUAT(state.filter.q_g2b, qs[q_i], 1e-2);

    // Test that the introduction of a step bias on pqr is eventually
    // rejected.
    for (int32_t i = 0; i < 500 * kOneSecond; ++i) {
      RunFilter(kFlightModePerched, &bias, &acc, &f_sf_g, &mag, &mag_g,
                kEstimatorVelocitySolutionTypeGps, &kVec3Zero, &wind_g,
                &ground_station, &gps_center_data, &gps_center_params,
                GetWingGpsPosFault(faults, kWingGpsReceiverCrosswind),
                &gps_port_data, &gps_port_params,
                GetWingGpsPosFault(faults, kWingGpsReceiverPort),
                &gps_star_data, &gps_star_params,
                GetWingGpsPosFault(faults, kWingGpsReceiverStar), &params,
                &faults[SUBSYS_IMU_A], &state.filter, &correct);
    }
    EXPECT_NEAR_QUAT(state.filter.q_g2b, qs[q_i], 1e-1);
    EXPECT_NEAR_VEC3(state.filter.gyro_bias, bias, 2e-3);
  }
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

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

#include "control/estimator/estimator_nav_kite.h"

#include <gtest/gtest.h>

#include <math.h>
#include <stdlib.h>

#include "common/c_math/mat3.h"
#include "common/c_math/util.h"
#include "common/c_math/vec3.h"
#include "common/macros.h"
#include "control/control_params.h"
#include "control/control_types.h"
#include "control/fault_detection/fault_detection_types.h"
#include "control/fault_detection/fault_detection_util.h"
#include "control/ground_frame.h"
#include "control/sensor_types.h"
#include "control/system_params.h"
#include "lib/util/test_util.h"

using ::test_util::LpfIterationTolerance;
using ::test_util::Rand;

extern "C" {

void SelectEstimates(const bool imu_valids[], const Vec3 pqr_estimates[],
                     const Quat q_g2b_estimates[], const Vec3 acc_b_estimates[],
                     EstimatorNavKiteState *state, Vec3 *pqr, Mat3 *dcm_g2b,
                     Vec3 *acc_b);

}  // extern "C"

namespace {

static const int32_t kOneSecond = static_cast<int32_t>(1.0 / (*g_sys.ts));

}  // namespace

namespace {

void TestSelectEstimatesLastUsed(const bool first_valid[],
                                 const bool second_valid[],
                                 WingImuLabel correct_imu) {
  const Quat q_estimates[] = {
      {0.0, 1.0, 0.0, 0.0}, {0.0, 0.0, 1.0, 0.0}, {0.0, 0.0, 0.0, 1.0}};
  const Vec3 pqr_estimates[] = {
      {4.0, 4.0, 4.0}, {5.0, 5.0, 5.0}, {6.0, 6.0, 6.0}};
  const Vec3 acc_b_estimates[] = {
      {7.0, 7.0, 7.0}, {8.0, 8.0, 8.0}, {9.0, 9.0, 9.0}};

  EstimatorNavKiteState state = EstimatorNavKiteState();
  Vec3 pqr;
  Mat3 dcm_g2b;
  Vec3 acc_b;
  SelectEstimates(first_valid, pqr_estimates, q_estimates, acc_b_estimates,
                  &state, &pqr, &dcm_g2b, &acc_b);
  SelectEstimates(second_valid, pqr_estimates, q_estimates, acc_b_estimates,
                  &state, &pqr, &dcm_g2b, &acc_b);

  Quat q;
  DcmToQuat(&dcm_g2b, &q);
  EXPECT_NEAR_VEC3(pqr_estimates[correct_imu], pqr, 0.0);
  EXPECT_NEAR_QUAT(q_estimates[correct_imu], q, 0.0);
  EXPECT_NEAR_VEC3(acc_b_estimates[correct_imu], acc_b, 0.0);
}

}  // namespace

TEST(SelectEstimates, ThreeValidToNoValid) {
  const bool all_valid[] = {true, true, true};
  const bool none_valid[] = {false, false, false};
  TestSelectEstimatesLastUsed(all_valid, none_valid, kWingImuA);
}

TEST(SelectEstimates, ThreeToOneValid) {
  const bool all_valid[] = {true, true, true};
  for (int32_t i = 0; i < kNumWingImus; ++i) {
    bool one_valid[kNumWingImus] = {false, false, false};
    one_valid[i] = true;
    TestSelectEstimatesLastUsed(all_valid, one_valid,
                                static_cast<WingImuLabel>(i));
  }
}

TEST(SelectEstimates, TwoValidToOneValid) {
  for (int32_t i = 0; i < kNumWingImus; ++i) {
    for (int32_t j = i + 1; j < kNumWingImus; ++j) {
      for (int32_t k = 0; k < kNumWingImus; ++k) {
        bool two_valid[kNumWingImus] = {false, false, false};
        two_valid[i] = true;
        two_valid[j] = true;

        bool one_valid[kNumWingImus] = {false, false, false};
        one_valid[k] = true;

        TestSelectEstimatesLastUsed(two_valid, one_valid,
                                    static_cast<WingImuLabel>(k));
      }
    }
  }
}

TEST(SelectEstimates, TwoValidToNoValid) {
  const bool none_valid[] = {false, false, false};
  for (int32_t i = 0; i < kNumWingImus; ++i) {
    for (int32_t j = i + 1; j < kNumWingImus; ++j) {
      bool two_valid[kNumWingImus] = {false, false, false};
      two_valid[i] = true;
      two_valid[j] = true;
      TestSelectEstimatesLastUsed(two_valid, none_valid,
                                  static_cast<WingImuLabel>(i));
    }
  }
}

TEST(SelectEstimates, OneValidToNoValid) {
  const bool none_valid[] = {false, false, false};
  for (int32_t i = 0; i < kNumWingImus; ++i) {
    bool one_valid[kNumWingImus] = {false, false, false};
    one_valid[i] = true;
    TestSelectEstimatesLastUsed(one_valid, none_valid,
                                static_cast<WingImuLabel>(i));
  }
}

namespace {

// Generates a sequence of observations corresponding to an attitude
// starting at q_0 and rotating with the body rates pqr.  Calls
// EstimatorInertialStep to track that attitude and checks the
// tracking performance.  Optionally simulates a serious fault with
// one gyroscope.
void TestFixedPqr(const Quat &q_0, const Vec3 &pqr, double gyro_noise_level,
                  int32_t imu_failed) {
  const double g_heading = GetSystemParams()->ground_frame.heading;
  double rate = Vec3Norm(&pqr);
  Vec3 axis = pqr;
  Vec3Normalize(&axis, &axis);

  Vec3 mag_g;
  RotNedToG(&g_sys.phys->mag_ned, g_heading, &mag_g);

  Vec3 acc_g;
  Vec3Scale(&g_sys.phys->g_g, -1.0, &acc_g);

  // The rate of rotation will be fixed, with a different bias for each IMU.
  const EstimatorNavParams &params = GetControlParams()->estimator.nav;
  EstimatorNavKiteState state;
  EstimatorNavKiteInit(&params, g_heading, &state);
  ImuData imus[kNumWingImus];
  Vec3 biases[kNumWingImus];
  for (int32_t i = 0; i < kNumWingImus; ++i) {
    biases[i] = {Rand(-0.05, 0.05), Rand(-0.05, 0.05), Rand(-0.05, 0.05)};

    // Each estimator is initialized with exact initial conditions.
    state.attitude[i].filter.q_g2b = q_0;
    state.attitude[i].filter.gyro_bias = biases[i];
  }

  Quat q, q_hat;
  Vec3 pqr_hat;
  FaultMask faults[kNumSubsystems];
  for (int32_t i = 0; i < kNumSubsystems; ++i) {
    ClearAllFaults(&faults[i]);
  }

  for (int32_t i = 0; i < 200 * kOneSecond; ++i) {
    double t = (*g_sys.ts) * i;
    AxisAngleToQuat(&axis, rate * t, &q);
    QuatMultiply(&q_0, &q, &q);
    for (int32_t j = 0; j < kNumWingImus; ++j) {
      Vec3 noise = {Rand(-1.0, 1.0), Rand(-1.0, 1.0), Rand(-1.0, 1.0)};
      Vec3Add(&biases[j], &pqr, &imus[j].gyro);
      Vec3Axpy(gyro_noise_level, &noise, &imus[j].gyro);
      QuatRotate(&q, &mag_g, &imus[j].mag);
      QuatRotate(&q, &acc_g, &imus[j].acc);
    }
    // Gyroscope zero is subject to additive noise.
    if (imu_failed >= 0) {
      imus[imu_failed].gyro.x += 4.0 * cos(2.0 * M_PI * 2.0 * t);
    }
    GpsData wing_gps[kNumWingGpsReceivers] = {GpsData(), GpsData()};
    GsGpsData gs_gps = GsGpsData();
    PitotData pitots[kNumPitotSensors] = {PitotData(), PitotData()};
    bool gps_active;
    double acc_norm_f;
    GroundStationPoseEstimate ground_station = GroundStationPoseEstimate();
    CalcDcmEcefToG(&ground_station.pos_ecef,
                   GetSystemParams()->ground_frame.heading,
                   &ground_station.dcm_ecef2g);
    ground_station.valid = true;

    const WindEstimate wind_g = WindEstimate();
    const EncodersEstimate encoders = EncodersEstimate();
    const PerchAziEstimate perch_azi = PerchAziEstimate();
    const TetherForceEstimate tether_force_b = TetherForceEstimate();
    const WinchEstimate winch = WinchEstimate();
    Mat3 dcm_g2b;
    Vec3 not_used;
    EstimatorNavKiteStep(
        false, imus, wing_gps, &gs_gps, pitots, &ground_station, &wind_g,
        &encoders, &perch_azi, &tether_force_b, &winch, kFlightModeHoverAscend,
        faults, GetSystemParams(), &params, &state, &pqr_hat, &dcm_g2b,
        &not_used, &gps_active, &not_used, &not_used, &not_used, &not_used,
        &not_used, &acc_norm_f, &not_used, &not_used, &not_used);
    DcmToQuat(&dcm_g2b, &q_hat);
    double quat_dot = QuatDot(&q, &q_hat);
    EXPECT_NEAR(1.0 - quat_dot * quat_dot, 0.0, 5e-4);
    EXPECT_NEAR_VEC3(pqr, pqr_hat, 2.0 * gyro_noise_level + 1e-2);
  }
}

}  // namespace

TEST(EstimateAttitude, SingleFailure) {
  const Quat q_0 = {0.7071, 0.7071, 0.0, 0.0};  // Initial orientation.
  const Vec3 pqr = {0.0, 0.0, -0.1};            // Axis of rotation.

  // Test the filter once without any fault.
  TestFixedPqr(q_0, pqr, 0.0, -1);
  TestFixedPqr(q_0, pqr, 0.01, -1);

  if (kNumWingImus > 1) {
    for (int32_t imu_failed = 0; imu_failed < kNumWingImus; ++imu_failed) {
      TestFixedPqr(q_0, pqr, 0.01, imu_failed);
      TestFixedPqr(q_0, pqr, 0.0, imu_failed);
    }
  }
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

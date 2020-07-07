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

#include "control/estimator/estimator_attitude_ground.h"

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

void RunFilter(const Vec3 *gyro, const Vec3 *acc, const Vec3 *f_sf_g,
               const GpsCompassData *gps_compass,
               const GsGpsParams *gps_compass_params,
               const FaultMask *gps_compass_faults,
               const EstimatorAttitudeParams *params,
               EstimatorAttitudeFilterState *state,
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

  const GsGpsParams *gs_gps_params = &GetSystemParams()->gs_gps;

  for (int32_t q_i = 0; q_i < ARRAYSIZE(qs); ++q_i) {
    EstimatorAttitudeState state;
    EstimatorAttitudeInit(&params, &state);

    Vec3 compass_p;
    Vec3Sub(&gs_gps_params->secondary_antenna_p.pos,
            &gs_gps_params->primary_antenna_p.pos, &compass_p);

    Quat q_inv;
    QuatInv(&qs[q_i], &q_inv);

    Vec3 compass_ned;
    QuatRotate(&q_inv, &compass_p, &compass_ned);
    GpsCompassData compass_data;

    double azi, ele, r;
    CartToSph(&compass_ned, &azi, &ele, &r);
    compass_data.new_data = true;
    compass_data.heading = Wrap(azi, 0.0, 2.0 * PI);
    compass_data.pitch = Wrap(-ele, -PI, PI);

    // NOTE: If the sigmas are too small, the convergence rate from the
    // initial state may be degraded.  A typical value for heading and pitch
    // sigmas in practice is 3e-3 to 4e-3 rad.  See b/136181998.
    compass_data.heading_sigma = 1e-2;
    compass_data.pitch_sigma = 1e-2;

    Vec3 f_sf_g;
    Vec3Scale(&g_sys.phys->g_g, -1.0, &f_sf_g);
    Vec3 acc;
    QuatRotate(&qs[q_i], &f_sf_g, &acc);

    // Clear all faults.
    FaultMask faults[kNumSubsystems];
    memset(faults, 0, sizeof(faults));

    EstimatorAttitudeCorrections correct;
    memset(&correct, 0, sizeof(correct));

    // Test that the attitude settles quickly with the initial gains
    // and no bias.
    for (int32_t i = 0; i < 10 * kOneSecond; ++i) {
      RunFilter(&kVec3Zero, &acc, &f_sf_g, &compass_data, gs_gps_params,
                &faults[kSubsysGsCompassAngles], &params, &state.filter,
                &correct);
    }
    EXPECT_NEAR_QUAT(state.filter.q_g2b, qs[q_i], 1e-2);

    // Test that the introduction of a step bias on pqr is eventually
    // rejected.
    for (int32_t i = 0; i < 500 * kOneSecond; ++i) {
      RunFilter(&bias, &acc, &f_sf_g, &compass_data, gs_gps_params,
                &faults[kSubsysGsCompassAngles], &params, &state.filter,
                &correct);
    }
    EXPECT_NEAR_QUAT(state.filter.q_g2b, qs[q_i], 1e-1);
    EXPECT_NEAR_VEC3(state.filter.gyro_bias, bias, 2e-3);
  }
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

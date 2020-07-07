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

#include <gtest/gtest.h>

#include <math.h>

#include "common/c_math/filter.h"
#include "common/c_math/util.h"
#include "common/c_math/vec3.h"
#include "control/crosswind/crosswind_frame.h"
#include "control/crosswind/crosswind_util.h"
#include "control/sensor_util.h"
#include "lib/util/test_util.h"

TEST(CalcCurvature, PerpendicularUnitVectors) {
  // The acc_b vector must have a non-zero y-component because we use
  // the y-component to determine the sign of the curvature.
  EXPECT_NEAR(CalcCurvature(&kVec3Y, &kVec3Z), 1.0, 1e-9);
  EXPECT_NEAR(CalcCurvature(&kVec3Y, &kVec3X), 1.0, 1e-9);
  Vec3 negative_y;
  Vec3Scale(&kVec3Y, -1.0, &negative_y);
  EXPECT_NEAR(CalcCurvature(&negative_y, &kVec3Z), -1.0, 1e-9);
  EXPECT_NEAR(CalcCurvature(&negative_y, &kVec3X), -1.0, 1e-9);
}

TEST(CalcCurvature, ParallelVectors) {
  EXPECT_NEAR(CalcCurvature(&kVec3X, &kVec3X), 0.0, 1e-9);
  EXPECT_NEAR(CalcCurvature(&kVec3Y, &kVec3Y), 0.0, 1e-9);
  EXPECT_NEAR(CalcCurvature(&kVec3Z, &kVec3Z), 0.0, 1e-9);
}

TEST(CurvatureToTetherRoll, ZeroCurvatureZeroCY) {
  for (double lift_coeff = 0.1; lift_coeff < 2.0; lift_coeff += 0.1) {
    EXPECT_NEAR(CurvatureToTetherRoll(0.0, lift_coeff, 0.0), 0.0, 1e-9);
  }
}

TEST(CalcLoopAngle, Normal) {
  Vec3 negative_z;
  Vec3Scale(&kVec3Z, -1.0, &negative_z);
  Vec3 negative_y;
  Vec3Scale(&kVec3Y, -1.0, &negative_y);
  EXPECT_NEAR(CalcLoopAngle(&kVec3X, &negative_y), 0.0, 1e-9);
  EXPECT_NEAR(CalcLoopAngle(&kVec3X, &kVec3Z), M_PI / 2.0, 1e-9);
  EXPECT_NEAR(CalcLoopAngle(&kVec3X, &kVec3Y), M_PI, 1e-9);
  EXPECT_NEAR(CalcLoopAngle(&kVec3X, &negative_z), 3.0 * M_PI / 2.0, 1e-9);
}

TEST(BetweenLoopAngles, Normal) {
  EXPECT_TRUE(BetweenLoopAngles(M_PI, 0.0, M_PI / 2.0));
  EXPECT_FALSE(BetweenLoopAngles(M_PI, 0.0, M_PI * 1.5));

  EXPECT_TRUE(BetweenLoopAngles(M_PI / 2.0, M_PI * 2.0, M_PI / 4.0));
  EXPECT_FALSE(BetweenLoopAngles(M_PI / 2.0, M_PI * 2.0, M_PI));

  EXPECT_TRUE(BetweenLoopAngles(M_PI / 2.0, M_PI * 1.5, 0.0));
  EXPECT_FALSE(BetweenLoopAngles(M_PI / 2.0, M_PI * 1.5, M_PI));

  EXPECT_TRUE(BetweenLoopAngles(M_PI, 0.0, 0.0));
  EXPECT_TRUE(BetweenLoopAngles(M_PI, 0.0, M_PI));
}

TEST(CalcCircularLoopGeometry, LoopAngleRecovery) {
  Vec3 path_center_g;
  path_center_g.x = -400.0;
  path_center_g.y = 10.0;
  path_center_g.z = -200.0;
  for (double loop_angle = 0.0; loop_angle < 2.0 * PI; loop_angle += 0.1) {
    Vec3 pos_g, vel_hat_g, acc_perp_hat_g;
    const double radius = 123.45;
    CalcCircularLoopGeometry(&path_center_g, loop_angle, radius, &pos_g,
                             &vel_hat_g, &acc_perp_hat_g);

    // Verify that we recover the loop angle that we used to compute
    // this position.
    EXPECT_NEAR(CalcLoopAngle(&path_center_g, &pos_g), loop_angle, 1e-6);

    // We assume that acceleration is entirely centripetal.
    EXPECT_NEAR(Vec3Dot(&vel_hat_g, &acc_perp_hat_g), 0.0, 1e-6);

    // We expect unit vectors.
    EXPECT_NEAR(Vec3Norm(&vel_hat_g), 1.0, 1e-6);
    EXPECT_NEAR(Vec3Norm(&acc_perp_hat_g), 1.0, 1e-6);
  }
}

TEST(CalcCircularLoopKinematics, Integration) {
  Vec3 wind_g = {-10.0, 2.0, 1.0};
  const Vec3 path_center_g = {-400.0, -20.0, -200.0};
  const double loop_radius = 155.0;

  // Get the starting position of the kite.
  Vec3 pos_g;
  CalcCircularLoopGeometry(&path_center_g, 0.0, loop_radius, &pos_g, NULL,
                           NULL);

  Vec3 pos_g_z1, vel_g_z1, acc_g_z1;
  double kitespeed_z1;

  const double dt = 0.01;  // Integration timestep [s].

  // Integrate forward in time.
  for (double t = 0.0; t < 100.0; t += dt) {
    const double loop_angle = CalcLoopAngle(&path_center_g, &pos_g);

    // Make up an airspeed schedule.
    const double airspeed_cmd = 40.0 + 10.0 * sin(loop_angle + 0.1);
    const double d_airspeed_d_loopangle = 10.0 * cos(loop_angle + 0.1);

    // Compute the kite velocity and acceleration.
    Vec3 vel_g, acc_g;
    const double kiteaccel = CalcCircularLoopKinematics(
        &wind_g, &path_center_g, loop_angle, loop_radius, airspeed_cmd,
        d_airspeed_d_loopangle, &vel_g, &acc_g);

    // Check that the wind triangle works out.
    Vec3 apparent_wind_g;
    Vec3Sub(&wind_g, &vel_g, &apparent_wind_g);
    EXPECT_NEAR(Vec3Norm(&apparent_wind_g), airspeed_cmd, 1e-3);

    // Check speed (scalar).
    Vec3 dx;
    Vec3Sub(&pos_g, &pos_g_z1, &dx);
    const double kitespeed_meas = Vec3Norm(&dx) / dt;
    const double kitespeed = Vec3Norm(&vel_g);
    if (t > 0.0) {
      EXPECT_NEAR(Vec3Norm(&vel_g), kitespeed_meas, 1e-1);
    }

    // Check acceleration (scalar).
    const double kiteaccel_meas = Diff(kitespeed, dt, &kitespeed_z1);
    if (t > 0.0) {
      EXPECT_NEAR(kiteaccel, kiteaccel_meas, 1e-1);
    }

    // Check vectors.
    Vec3 vel_g_meas, acc_g_meas;
    DiffVec3(&pos_g, dt, &vel_g_meas, &pos_g_z1);
    DiffVec3(&vel_g, dt, &acc_g_meas, &vel_g_z1);

    if (t > 0.0) {
      Vec3 vel_g_mean, acc_g_mean;
      Vec3LinComb(0.5, &acc_g, 0.5, &acc_g_z1, &acc_g_mean);
      Vec3LinComb(0.5, &vel_g, 0.5, &vel_g_z1, &vel_g_mean);

      EXPECT_NEAR_VEC3(vel_g_mean, vel_g_meas, 1e-1);
      EXPECT_NEAR_VEC3(acc_g_mean, acc_g_meas, 1e-3);
    }

    // Integrate forwards.
    Vec3LinComb3(1.0, &pos_g, dt, &vel_g, Square(dt) / 2.0, &acc_g, &pos_g);
    acc_g_z1 = acc_g;
  }
}

TEST(CalcCrosswindAttitude, AeroAngleRecovery) {
  const Vec3 wind_g = {-10.0, 2.0, 1.0};
  const Vec3 path_center_g = {-400.0, -20.0, -200.0};
  const double loop_radius = 155.0;

  double tether_roll_cmd = 0.1;
  double alpha_cmd = -0.5;
  double beta_cmd = 0.2;
  double airspeed_cmd = 40.0;
  double d_airspeed_d_loopangle = 0.0;  // Doesn't matter.

  for (double loop_angle = 0.0; loop_angle < 2 * PI; loop_angle += 0.1) {
    // Construct the apparent wind vector in the ground frame.
    Vec3 wing_vel_g, apparent_wind_g;
    CalcCircularLoopKinematics(&wind_g, &path_center_g, loop_angle, loop_radius,
                               airspeed_cmd, d_airspeed_d_loopangle,
                               &wing_vel_g, NULL);
    Vec3Sub(&wind_g, &wing_vel_g, &apparent_wind_g);

    // Check that the kite velocity is in the x-direction in the tangent frame.
    Mat3 dcm_g2t;
    CalcDcmGToT(&path_center_g, loop_angle, &dcm_g2t);

    Vec3 wing_vel_t;
    Mat3Vec3Mult(&dcm_g2t, &wing_vel_g, &wing_vel_t);
    EXPECT_NEAR(wing_vel_t.x, Vec3Norm(&wing_vel_g), 1e-3);
    EXPECT_NEAR(wing_vel_t.y, 0.0, 1e-3);
    EXPECT_NEAR(wing_vel_t.z, 0.0, 1e-3);

    Vec3 path_center_t;
    Mat3Vec3Mult(&dcm_g2t, &path_center_g, &path_center_t);
    EXPECT_NEAR(path_center_t.x, 0.0, 1e-6);
    EXPECT_NEAR(path_center_t.y, 0.0, 1e-6);

    // Construct the dcm_g2b that we expect.
    Mat3 dcm_g2b;
    CalcCrosswindAttitude(&wind_g, &path_center_g, loop_angle, loop_radius,
                          tether_roll_cmd, alpha_cmd, beta_cmd, airspeed_cmd,
                          d_airspeed_d_loopangle, &dcm_g2b);

    // Transform apparent_wind_g to the body frame and verify that the
    // aero angles are what we wanted.  TODO: Also check the
    // tether roll angle.
    Vec3 apparent_wind_b;
    Mat3Vec3Mult(&dcm_g2b, &apparent_wind_g, &apparent_wind_b);

    ApparentWindSph apparent_wind_sph;
    ApparentWindCartToSph(&apparent_wind_b, &apparent_wind_sph);

    EXPECT_NEAR(apparent_wind_sph.alpha, alpha_cmd, 1e-6);
    EXPECT_NEAR(apparent_wind_sph.beta, beta_cmd, 1e-6);
  }
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

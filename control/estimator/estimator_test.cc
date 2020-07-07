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
#include <stdint.h>

#include "common/c_math/coord_trans.h"
#include "common/c_math/util.h"
#include "common/macros.h"
#include "control/control_params.h"
#include "control/control_types.h"
#include "control/estimator/estimator.h"
#include "control/estimator/estimator_perch_azi.h"
#include "control/estimator/estimator_wind.h"
#include "control/fault_detection/fault_detection_types.h"
#include "control/fault_detection/fault_detection_util.h"
#include "control/perch_frame.h"
#include "control/platform_frame.h"
#include "control/sensor_util.h"
#include "control/system_params.h"
#include "lib/util/test_util.h"
#include "system/labels.h"

using ::test_util::LpfIterationTolerance;
using ::test_util::Rand;
using ::test_util::RandNormal;

TEST(EstimateWind, OnAxisWindSensorRecovery) {
  WindSensorParams wind_sensor_params;
  wind_sensor_params.pos_parent = kVec3Zero;
  wind_sensor_params.dcm_parent2ws = kMat3Identity;
  wind_sensor_params.on_perch = true;

  FaultMask fault;
  ClearAllFaults(&fault);
  EstimatorWindState wind_state;
  EstimatorWindInit(&wind_state);
  Vec3 wind_g = {1.0, 0.0, 0.0};
  double ts = 0.01;
  for (double t = 0.0; t < 10.0; t += ts) {
    double perch_azi_angle = sin(2.0 * M_PI * ts);
    Vec3 wind_ws;
    RotGToP(&wind_g, perch_azi_angle, &wind_ws);

    VesselEstimate vessel = VesselEstimate();
    vessel.pos_g = kVec3Zero;
    vessel.vel_g = kVec3Zero;
    vessel.dcm_g2v = kMat3Identity;

    Mat3 dcm_v2p;
    CalcDcmVesselToPlatform(perch_azi_angle, &dcm_v2p);
    Mat3Mat3Mult(&dcm_v2p, &vessel.dcm_g2v, &vessel.dcm_g2p);

    vessel.pqr = kVec3Zero;
    vessel.position_valid = true;
    vessel.attitude_valid = true;

    WindEstimate wind_g_est;
    EstimatorWindStep(false, &wind_ws, &fault, &vessel, &wind_sensor_params,
                      &GetControlParams()->estimator.wind, &wind_state,
                      &wind_g_est);
    EXPECT_NEAR_VEC3(wind_g, wind_g_est.vector, 1e-9);
  }
}

TEST(EstimateWind, WindSensorNotOnPerch) {
  WindSensorParams wind_sensor_params;
  wind_sensor_params.pos_parent = kVec3Ones;
  AngleToDcm(PI / 4.0, 0.0, 0.0, kRotationOrderZyx,
             &wind_sensor_params.dcm_parent2ws);
  wind_sensor_params.on_perch = false;

  FaultMask fault;
  ClearAllFaults(&fault);
  EstimatorWindState wind_state;
  EstimatorWindInit(&wind_state);
  Vec3 wind_g = {1.0, 0.0, 0.0};
  Vec3 wind_ws;
  Mat3Vec3Mult(&wind_sensor_params.dcm_parent2ws, &wind_g, &wind_ws);

  double perch_azi_angle = PI / 7.0;

  VesselEstimate vessel = VesselEstimate();
  vessel.pos_g = kVec3Zero;
  vessel.vel_g = kVec3Zero;
  vessel.dcm_g2v = kMat3Identity;

  Mat3 dcm_v2p;
  CalcDcmVesselToPlatform(perch_azi_angle, &dcm_v2p);
  Mat3Mat3Mult(&dcm_v2p, &vessel.dcm_g2v, &vessel.dcm_g2p);

  vessel.pqr = kVec3Zero;
  vessel.position_valid = true;
  vessel.attitude_valid = true;

  WindEstimate wind_g_est;
  EstimatorWindStep(false, &wind_ws, &fault, &vessel, &wind_sensor_params,
                    &GetControlParams()->estimator.wind, &wind_state,
                    &wind_g_est);
  EXPECT_NEAR_VEC3(wind_g, wind_g_est.vector, 1e-9);
}

TEST(EstimateWind, BadPosBoundedError) {
  WindSensorParams wind_sensor_params;
  wind_sensor_params.pos_parent = {3.0, 0.0, 0.0};
  wind_sensor_params.dcm_parent2ws = kMat3Identity;
  wind_sensor_params.on_perch = true;

  FaultMask fault;
  ClearAllFaults(&fault);
  EstimatorWindState wind_state;
  EstimatorWindInit(&wind_state);
  Vec3 wind_g = {1.0, 0.0, 0.0};
  double ts = 0.01;
  for (double omega = -2.0; omega < 2.0; omega += 0.15) {
    for (double t = 0.0; t < 10.0; t += ts) {
      double perch_azi_angle = sin(omega * ts);
      Vec3 wind_sensor;
      RotGToP(&wind_g, perch_azi_angle, &wind_sensor);

      VesselEstimate vessel = VesselEstimate();
      vessel.pos_g = kVec3Zero;
      vessel.vel_g = kVec3Zero;
      vessel.dcm_g2v = kMat3Identity;

      Mat3 dcm_v2p;
      CalcDcmVesselToPlatform(perch_azi_angle, &dcm_v2p);
      Mat3Mat3Mult(&dcm_v2p, &vessel.dcm_g2v, &vessel.dcm_g2p);

      vessel.pqr = kVec3Zero;
      vessel.position_valid = true;
      vessel.attitude_valid = true;

      WindEstimate wind_g_est;
      EstimatorWindStep(
          false, &wind_sensor, &fault, &vessel, &wind_sensor_params,
          &GetControlParams()->estimator.wind, &wind_state, &wind_g_est);
      EXPECT_NEAR_VEC3(wind_g, wind_g_est.vector,
                       fabs(omega) * Vec3Norm(&wind_sensor_params.pos_parent));
    }
  }
}

TEST(EstimateWind, LowFreqPass) {
  WindSensorParams wind_sensor_params;
  wind_sensor_params.pos_parent = {1.0, 0.0, 0.0};
  wind_sensor_params.dcm_parent2ws = kMat3Identity;
  wind_sensor_params.on_perch = true;

  FaultMask fault;
  ClearAllFaults(&fault);
  EstimatorWindState wind_state;
  EstimatorWindInit(&wind_state);
  Vec3 wind_g = {5.0, 0.0, 0.0};
  double ts = *g_sys.ts;
  double A_pass = 0.1;
  double f_pass = 1.0 / 10.0;
  double f_stop = 10.0;
  for (double A_stop = 0.01; A_stop < 1.0; A_stop *= 1.2) {
    for (double t = 0.0; t < 10.0; t += ts) {
      double perch_azi_angle = A_pass * sin(2.0 * M_PI * f_pass * t);
      double omega_perch_azi =
          A_pass * 2.0 * M_PI * f_pass * cos(2.0 * M_PI * f_pass * t);
      Vec3 wind_sensor;
      RotGToP(&wind_g, perch_azi_angle, &wind_sensor);
      wind_sensor.y -= omega_perch_azi;
      perch_azi_angle += A_stop * sin(2.0 * M_PI * f_stop * t);

      VesselEstimate vessel = VesselEstimate();
      vessel.pos_g = kVec3Zero;
      vessel.vel_g = kVec3Zero;
      vessel.dcm_g2v = kMat3Identity;

      Mat3 dcm_v2p;
      CalcDcmVesselToPlatform(perch_azi_angle, &dcm_v2p);
      Mat3Mat3Mult(&dcm_v2p, &vessel.dcm_g2v, &vessel.dcm_g2p);

      vessel.pqr = {0.0, 0.0, omega_perch_azi};
      vessel.position_valid = true;
      vessel.attitude_valid = true;

      WindEstimate wind_g_est;
      EstimatorWindStep(
          false, &wind_sensor, &fault, &vessel, &wind_sensor_params,
          &GetControlParams()->estimator.wind, &wind_state, &wind_g_est);
      if (t > 1.0) {
        EXPECT_NEAR_VEC3(wind_g, wind_g_est.vector,
                         0.05 * A_pass + 2.0 * M_PI * f_stop * A_stop / 6.0);
      }
    }
  }
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

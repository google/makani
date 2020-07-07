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

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <algorithm>
#include <functional>
#include <string>

#include "avionics/common/avionics_messages.h"
#include "avionics/common/novatel_types.h"
#include "control/avionics/avionics_interface_types.h"
#include "control/avionics/avionics_saturation.h"
#include "control/control_params.h"
#include "control/control_types.h"
#include "control/fault_detection/fault_detection_types.h"
#include "control/fault_detection/fault_detection_util.h"
#include "lib/util/test_util.h"

TEST(AvionicsSaturateSensorsTest, Max0) {
  const Vec3 plus_one = {1.0, 1.0, 1.0};
  ControlInput sensors_raw;
  ControlInput sensors_out;
  const SensorLimitsParams *lim = &GetControlParams()->sensor_limits;

  // Set gs_sensors value to sane values. These do not get saturated, but are
  // asserted for correctness.
  sensors_raw.gs_sensors.mode = lim->min.gs_sensors.mode;
  sensors_raw.gs_sensors.transform_stage = lim->min.gs_sensors.transform_stage;
  sensors_raw.gs_sensors.winch_pos = lim->min.gs_sensors.winch_pos;
  sensors_raw.gs_sensors.detwist_pos = lim->min.gs_sensors.detwist_pos;
  sensors_raw.gs_sensors.proximity = lim->min.gs_sensors.proximity;

  for (int32_t i = 0; i < kNumDrums; ++i) {
    // Violate position limits.
    sensors_raw.gsg[i].ele = lim->max.gsg[i].ele + 1.0;
    sensors_raw.gsg[i].azi = lim->max.gsg[i].azi + 1.0;
  }

  for (int32_t i = 0; i < kNumWingGpsReceivers; ++i) {
    Vec3Add(&lim->max.wing_gps[i].pos, &plus_one, &sensors_raw.wing_gps[i].pos);
    Vec3Add(&lim->max.wing_gps[i].vel, &plus_one, &sensors_raw.wing_gps[i].vel);
    Vec3Add(&lim->max.wing_gps[i].pos_sigma, &plus_one,
            &sensors_raw.wing_gps[i].pos_sigma);
    Vec3Add(&lim->max.wing_gps[i].vel_sigma, &plus_one,
            &sensors_raw.wing_gps[i].vel_sigma);
  }

  sensors_raw.joystick.throttle = lim->max.joystick.throttle + 1.0;
  sensors_raw.joystick.roll = lim->max.joystick.roll + 1.0;
  sensors_raw.joystick.pitch = lim->max.joystick.pitch + 1.0;
  sensors_raw.joystick.yaw = lim->max.joystick.yaw + 1.0;

  for (int32_t i = 0; i < kNumLoadcellSensors; ++i) {
    sensors_raw.loadcells[i] = lim->max.loadcells[i] + 1.0;
  }

  for (int32_t i = 0; i < kNumWingImus; ++i) {
    Vec3Add(&lim->max.imus[i].acc, &plus_one, &sensors_raw.imus[i].acc);
    Vec3Add(&lim->max.imus[i].gyro, &plus_one, &sensors_raw.imus[i].gyro);
    Vec3Add(&lim->max.imus[i].mag, &plus_one, &sensors_raw.imus[i].mag);
  }

  for (int32_t i = 0; i < kNumPitotSensors; ++i) {
    sensors_raw.pitots[i].stat_press = lim->max.pitots[i].stat_press + 1.0;
    sensors_raw.pitots[i].diff.alpha_press =
        lim->max.pitots[i].diff.alpha_press + 1.0;
    sensors_raw.pitots[i].diff.dyn_press =
        lim->max.pitots[i].diff.dyn_press + 1.0;
    sensors_raw.pitots[i].diff.beta_press =
        lim->max.pitots[i].diff.beta_press + 1.0;
  }

  for (int32_t i = 0; i < kNumFlaps; ++i) {
    sensors_raw.flaps[i] = lim->max.flaps[i] + 1.0;
  }

  for (int32_t i = 0; i < kNumMotors; ++i) {
    sensors_raw.rotors[i] = lim->max.rotors[i] + 1.0;
  }

  Vec3Add(&lim->max.wind_ws, &plus_one, &sensors_raw.wind_ws);

  sensors_raw.perch.winch_pos = lim->max.perch.winch_pos + 1.0;
  sensors_raw.perch.perch_heading = lim->max.perch.perch_heading + 1.0;
  for (int32_t i = 0; i < kNumPlatforms; ++i) {
    sensors_raw.perch.perch_azi[i] = lim->max.perch.perch_azi[i] + 1.0;
    sensors_raw.perch.levelwind_ele[i] = lim->max.perch.levelwind_ele[i] + 1.0;
  }

  FaultMask faults[kNumFaultTypes] = {0};
  AvionicsSaturateSensors(&sensors_raw, lim, faults, &sensors_out);

  for (int32_t i = 0; i < kNumDrums; ++i) {
    EXPECT_NEAR(sensors_out.gsg[i].ele, lim->max.gsg[i].ele, DBL_EPSILON);
    EXPECT_NEAR(sensors_out.gsg[i].azi, lim->max.gsg[i].azi, DBL_EPSILON);
  }

  for (int32_t i = 0; i < kNumWingGpsReceivers; ++i) {
    WingGpsReceiverLabel gps_label = static_cast<WingGpsReceiverLabel>(i);
    const FaultMask *gps_faults = GetWingGpsSubsysFaults(faults, gps_label);
    EXPECT_TRUE(HasFault(kFaultTypeOutOfRange,
                         &gps_faults[kFaultDetectionGpsSignalPos]));
    EXPECT_TRUE(HasFault(kFaultTypeOutOfRange,
                         &gps_faults[kFaultDetectionGpsSignalVel]));
  }

  EXPECT_NEAR(sensors_out.joystick.throttle, lim->max.joystick.throttle,
              DBL_EPSILON);
  EXPECT_NEAR(sensors_out.joystick.roll, lim->max.joystick.roll, DBL_EPSILON);
  EXPECT_NEAR(sensors_out.joystick.pitch, lim->max.joystick.pitch, DBL_EPSILON);
  EXPECT_NEAR(sensors_out.joystick.yaw, lim->max.joystick.yaw, DBL_EPSILON);

  for (int32_t i = 0; i < kNumLoadcellSensors; ++i) {
    EXPECT_NEAR(sensors_out.loadcells[i], lim->max.loadcells[i], DBL_EPSILON);
  }

  for (int32_t i = 0; i < kNumWingImus; ++i) {
    EXPECT_NEAR_VEC3(sensors_out.imus[i].acc, lim->max.imus[i].acc,
                     DBL_EPSILON);
    EXPECT_NEAR_VEC3(sensors_out.imus[i].gyro, lim->max.imus[i].gyro,
                     DBL_EPSILON);
    EXPECT_NEAR_VEC3(sensors_out.imus[i].mag, lim->max.imus[i].mag,
                     DBL_EPSILON);
  }

  for (int32_t i = 0; i < kNumPitotSensors; ++i) {
    EXPECT_NEAR(sensors_out.pitots[i].stat_press, lim->max.pitots[i].stat_press,
                DBL_EPSILON);
    EXPECT_NEAR(sensors_out.pitots[i].diff.alpha_press,
                lim->max.pitots[i].diff.alpha_press, DBL_EPSILON);
    EXPECT_NEAR(sensors_out.pitots[i].diff.dyn_press,
                lim->max.pitots[i].diff.dyn_press, DBL_EPSILON);
    EXPECT_NEAR(sensors_out.pitots[i].diff.beta_press,
                lim->max.pitots[i].diff.beta_press, DBL_EPSILON);
  }

  for (int32_t i = 0; i < kNumFlaps; ++i) {
    EXPECT_NEAR(sensors_out.flaps[i], lim->max.flaps[i], DBL_EPSILON);
  }

  for (int32_t i = 0; i < kNumMotors; ++i) {
    EXPECT_NEAR(sensors_out.rotors[i], lim->max.rotors[i], DBL_EPSILON);
  }

  EXPECT_NEAR_VEC3(sensors_out.wind_ws, lim->max.wind_ws, DBL_EPSILON);
  EXPECT_NEAR(sensors_out.perch.winch_pos, lim->max.perch.winch_pos,
              DBL_EPSILON);
  EXPECT_NEAR(sensors_out.perch.perch_heading, lim->max.perch.perch_heading,
              DBL_EPSILON);
  for (int32_t i = 0; i < kNumPlatforms; ++i) {
    EXPECT_NEAR(sensors_out.perch.perch_azi[i], lim->max.perch.perch_azi[i],
                DBL_EPSILON);
    EXPECT_NEAR(sensors_out.perch.levelwind_ele[i],
                lim->max.perch.levelwind_ele[i], DBL_EPSILON);
  }
}

TEST(AvionicsSaturateSensorsTest, Min0) {
  const Vec3 minus_one = {-1.0, -1.0, -1.0};
  ControlInput sensors_raw;
  ControlInput sensors_out;
  const SensorLimitsParams *lim = &GetControlParams()->sensor_limits;

  // Set gs_sensors value to sane values. These do not get saturated, but are
  // asserted for correctness.
  sensors_raw.gs_sensors.mode = lim->min.gs_sensors.mode;
  sensors_raw.gs_sensors.transform_stage = lim->min.gs_sensors.transform_stage;
  sensors_raw.gs_sensors.winch_pos = lim->min.gs_sensors.winch_pos;
  sensors_raw.gs_sensors.detwist_pos = lim->min.gs_sensors.detwist_pos;
  sensors_raw.gs_sensors.proximity = lim->min.gs_sensors.proximity;

  for (int32_t i = 0; i < kNumDrums; ++i) {
    // Violate position limits.
    sensors_raw.gsg[i].ele = lim->min.gsg[i].ele - 1.0;
    sensors_raw.gsg[i].azi = lim->min.gsg[i].azi - 1.0;
  }

  for (int32_t i = 0; i < kNumWingGpsReceivers; ++i) {
    Vec3Add(&lim->min.wing_gps[i].pos, &minus_one,
            &sensors_raw.wing_gps[i].pos);
    Vec3Add(&lim->min.wing_gps[i].vel, &minus_one,
            &sensors_raw.wing_gps[i].vel);
    Vec3Add(&lim->min.wing_gps[i].pos_sigma, &minus_one,
            &sensors_raw.wing_gps[i].pos_sigma);
    Vec3Add(&lim->min.wing_gps[i].vel_sigma, &minus_one,
            &sensors_raw.wing_gps[i].vel_sigma);
  }

  sensors_raw.joystick.throttle = lim->min.joystick.throttle - 1.0;
  sensors_raw.joystick.roll = lim->min.joystick.roll - 1.0;
  sensors_raw.joystick.pitch = lim->min.joystick.pitch - 1.0;
  sensors_raw.joystick.yaw = lim->min.joystick.yaw - 1.0;

  for (int32_t i = 0; i < kNumLoadcellSensors; ++i) {
    sensors_raw.loadcells[i] = lim->min.loadcells[i] - 1.0;
  }

  for (int32_t i = 0; i < kNumWingImus; ++i) {
    Vec3Add(&lim->min.imus[i].acc, &minus_one, &sensors_raw.imus[i].acc);
    Vec3Add(&lim->min.imus[i].gyro, &minus_one, &sensors_raw.imus[i].gyro);
    Vec3Add(&lim->min.imus[i].mag, &minus_one, &sensors_raw.imus[i].mag);
  }

  for (int32_t i = 0; i < kNumPitotSensors; ++i) {
    sensors_raw.pitots[i].stat_press = lim->min.pitots[i].stat_press - 1.0;
    sensors_raw.pitots[i].diff.alpha_press =
        lim->min.pitots[i].diff.alpha_press - 1.0;
    sensors_raw.pitots[i].diff.dyn_press =
        lim->min.pitots[i].diff.dyn_press - 1.0;
    sensors_raw.pitots[i].diff.beta_press =
        lim->min.pitots[i].diff.beta_press - 1.0;
  }

  for (int32_t i = 0; i < kNumFlaps; ++i) {
    sensors_raw.flaps[i] = lim->min.flaps[i] - 1.0;
  }

  for (int32_t i = 0; i < kNumMotors; ++i) {
    sensors_raw.rotors[i] = lim->min.rotors[i] - 1.0;
  }

  Vec3Add(&lim->min.wind_ws, &minus_one, &sensors_raw.wind_ws);

  sensors_raw.perch.winch_pos = lim->min.perch.winch_pos - 1.0;
  sensors_raw.perch.perch_heading = lim->min.perch.perch_heading - 1.0;
  for (int32_t i = 0; i < kNumPlatforms; ++i) {
    sensors_raw.perch.perch_azi[i] = lim->min.perch.perch_azi[i] - 1.0;
    sensors_raw.perch.levelwind_ele[i] = lim->min.perch.levelwind_ele[i] - 1.0;
  }

  FaultMask faults[kNumFaultTypes] = {0};
  AvionicsSaturateSensors(&sensors_raw, lim, faults, &sensors_out);

  for (int32_t i = 0; i < kNumDrums; ++i) {
    EXPECT_NEAR(sensors_out.gsg[i].ele, lim->min.gsg[i].ele, DBL_EPSILON);
    EXPECT_NEAR(sensors_out.gsg[i].azi, lim->min.gsg[i].azi, DBL_EPSILON);
  }

  for (int32_t i = 0; i < kNumWingGpsReceivers; ++i) {
    WingGpsReceiverLabel gps_label = static_cast<WingGpsReceiverLabel>(i);
    const FaultMask *gps_faults = GetWingGpsSubsysFaults(faults, gps_label);
    EXPECT_TRUE(HasFault(kFaultTypeOutOfRange,
                         &gps_faults[kFaultDetectionGpsSignalPos]));
    EXPECT_TRUE(HasFault(kFaultTypeOutOfRange,
                         &gps_faults[kFaultDetectionGpsSignalVel]));
  }

  EXPECT_NEAR(sensors_out.joystick.throttle, lim->min.joystick.throttle,
              DBL_EPSILON);
  EXPECT_NEAR(sensors_out.joystick.roll, lim->min.joystick.roll, DBL_EPSILON);
  EXPECT_NEAR(sensors_out.joystick.pitch, lim->min.joystick.pitch, DBL_EPSILON);
  EXPECT_NEAR(sensors_out.joystick.yaw, lim->min.joystick.yaw, DBL_EPSILON);

  for (int32_t i = 0; i < kNumLoadcellSensors; ++i) {
    EXPECT_NEAR(sensors_out.loadcells[i], lim->min.loadcells[i], DBL_EPSILON);
  }

  for (int32_t i = 0; i < kNumWingImus; ++i) {
    EXPECT_NEAR_VEC3(sensors_out.imus[i].acc, lim->min.imus[i].acc,
                     DBL_EPSILON);
    EXPECT_NEAR_VEC3(sensors_out.imus[i].gyro, lim->min.imus[i].gyro,
                     DBL_EPSILON);
    EXPECT_NEAR_VEC3(sensors_out.imus[i].mag, lim->min.imus[i].mag,
                     DBL_EPSILON);
  }

  for (int32_t i = 0; i < kNumPitotSensors; ++i) {
    EXPECT_NEAR(sensors_out.pitots[i].stat_press, lim->min.pitots[i].stat_press,
                DBL_EPSILON);
    EXPECT_NEAR(sensors_out.pitots[i].diff.alpha_press,
                lim->min.pitots[i].diff.alpha_press, DBL_EPSILON);
    EXPECT_NEAR(sensors_out.pitots[i].diff.dyn_press,
                lim->min.pitots[i].diff.dyn_press, DBL_EPSILON);
    EXPECT_NEAR(sensors_out.pitots[i].diff.beta_press,
                lim->min.pitots[i].diff.beta_press, DBL_EPSILON);
  }

  for (int32_t i = 0; i < kNumFlaps; ++i) {
    EXPECT_NEAR(sensors_out.flaps[i], lim->min.flaps[i], DBL_EPSILON);
  }

  for (int32_t i = 0; i < kNumMotors; ++i) {
    EXPECT_NEAR(sensors_out.rotors[i], lim->min.rotors[i], DBL_EPSILON);
  }

  EXPECT_NEAR_VEC3(sensors_out.wind_ws, lim->min.wind_ws, DBL_EPSILON);

  EXPECT_NEAR(sensors_out.perch.winch_pos, lim->min.perch.winch_pos,
              DBL_EPSILON);
  EXPECT_NEAR(sensors_out.perch.perch_heading, lim->min.perch.perch_heading,
              DBL_EPSILON);
  for (int32_t i = 0; i < kNumPlatforms; ++i) {
    EXPECT_NEAR(sensors_out.perch.perch_azi[i], lim->min.perch.perch_azi[i],
                DBL_EPSILON);
    EXPECT_NEAR(sensors_out.perch.levelwind_ele[i],
                lim->min.perch.levelwind_ele[i], DBL_EPSILON);
  }
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

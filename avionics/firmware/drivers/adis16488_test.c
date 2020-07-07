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

#include "avionics/firmware/drivers/adis16488_test.h"

#include <float.h>
#include <math.h>
#include <stdint.h>

#include "avionics/common/adis16488_types.h"
#include "avionics/firmware/cpu/clock.h"
#include "avionics/firmware/cpu/mibspi.h"
#include "avionics/firmware/drivers/adis16488.h"
#include "avionics/firmware/test/test.h"
#include "common/c_math/geometry.h"
#include "common/c_math/util.h"
#include "common/c_math/vec3.h"
#include "common/macros.h"

static void TestSetup(void) {
  MibSPIInit(3, kSpiPinmuxAll);
  Adis16488Init();
}

static void TestTeardown(void) {
}

static void TestSyncPulse(void) {
  // Sync to first pulse.
  Adis16488WaitForSyncPulse();

  // Count one second's worth of ePWM sync periods.
  uint32_t start = Clock32GetCycles();
  for (int32_t i = 0; i < ADIS16488_SYNC_FREQ_HZ; ++i) {
    Adis16488WaitForSyncPulse();
  }
  uint32_t cycles = Clock32GetCycles() - start;
  float duration = CLOCK32_CYCLES_TO_USEC_F(cycles) / 1e6;

  // Expect no more than 0.25 period jitter.
  EXPECT_NEAR(duration, 1.0f, 0.25f / ADIS16488_SYNC_FREQ_HZ);
}

static void TestOutputData(void) {
  // Wait for IMU to initialize.
  Adis16488OutputData out;
  do {
    Adis16488WaitForSyncPulse();
  } while (!Adis16488PollOutputData(ClockGetUs(), &out));

  // Buffer inertial data to compute statistics later. Buffers are static to
  // avoid placing this data on the stack.
  static double accel_x[ADIS16488_SYNC_FREQ_HZ];
  static double accel_y[ADIS16488_SYNC_FREQ_HZ];
  static double accel_z[ADIS16488_SYNC_FREQ_HZ];
  static double gyro_x[ADIS16488_SYNC_FREQ_HZ];
  static double gyro_y[ADIS16488_SYNC_FREQ_HZ];
  static double gyro_z[ADIS16488_SYNC_FREQ_HZ];
  int32_t mag_updates = 0;

  // Acquire data for 1 second.
  for (int32_t i = 0; i < ADIS16488_SYNC_FREQ_HZ; ++i) {
    Adis16488WaitForSyncPulse();
    EXPECT_TRUE(Adis16488PollOutputData(ClockGetUs(), &out));

    // Expect all self-tests to pass.
    EXPECT_EQ(out.diag_sts, 0x0);

    // Expect no internal sensor errors.
    EXPECT_EQ(out.sys_e_flag & ~(kAdis16488StatusNewMagnetometer
                                 | kAdis16488StatusNewBarometer
                                 | kAdis16488StatusAlarm), 0x0);

    // Count magnetometer updates.
    if (out.sys_e_flag & kAdis16488StatusNewMagnetometer) {
      ++mag_updates;
    }

    // Verify sample period.
    EXPECT_NEAR(out.dt, 1.0f / ADIS16488_SYNC_FREQ_HZ, FLT_EPSILON);

    // Verify temperature.
    EXPECT_GT(out.temp, 10.0f);
    EXPECT_LT(out.temp, 65.0f);

    // Store inertial data to compute statistics.
    accel_x[i] = out.accel[0];
    accel_y[i] = out.accel[1];
    accel_z[i] = out.accel[2];
    gyro_x[i] = out.gyro[0];
    gyro_y[i] = out.gyro[1];
    gyro_z[i] = out.gyro[2];
  }

  // Magnetometer updates at 100 Hz.
  EXPECT_GE(mag_updates, 99);
  EXPECT_LE(mag_updates, 101);

  // Check accel mean (bias and orientation).
  // Body z-axis points downward in default orientation (Flight Computer RevBb).
  double expected_elev = -PI / 2.0;
  Vec3 accel = {MeanArray(accel_x, ARRAYSIZE(accel_x)),
                MeanArray(accel_y, ARRAYSIZE(accel_y)),
                MeanArray(accel_z, ARRAYSIZE(accel_z))};
  double azi, elev, r;
  CartToSph(&accel, &azi, &elev, &r);
  EXPECT_NEAR(elev, expected_elev, 15.0 * PI / 180.0);
  EXPECT_NEAR(r, 9.8, 0.5);

  // Check accel variance.
  double accel_std_threshold = 0.1;  // [m/s^2]
  double accel_x_std = Sqrt(VarArray(accel_x, ARRAYSIZE(accel_x)));
  EXPECT_GT(accel_x_std, 0.0);
  EXPECT_LT(accel_x_std, accel_std_threshold);
  double accel_y_std = Sqrt(VarArray(accel_y, ARRAYSIZE(accel_y)));
  EXPECT_GT(accel_y_std, 0.0);
  EXPECT_LT(accel_y_std, accel_std_threshold);
  double accel_z_std = Sqrt(VarArray(accel_z, ARRAYSIZE(accel_z)));
  EXPECT_GT(accel_z_std, 0.0);
  EXPECT_LT(accel_z_std, accel_std_threshold);

  // Check gyro mean (bias).
  double gyro_mean_threshold = 0.1;  // [rad/s]
  double gyro_x_mean = MeanArray(gyro_x, ARRAYSIZE(gyro_x));
  EXPECT_LT(fabs(gyro_x_mean), gyro_mean_threshold);
  double gyro_y_mean = MeanArray(gyro_y, ARRAYSIZE(gyro_y));
  EXPECT_LT(fabs(gyro_y_mean), gyro_mean_threshold);
  double gyro_z_mean = MeanArray(gyro_z, ARRAYSIZE(gyro_z));
  EXPECT_LT(fabs(gyro_z_mean), gyro_mean_threshold);

  // Check gyro variance.
  double gyro_std_threshold = 0.1;  // [rad/s]
  double gyro_x_std = Sqrt(VarArray(gyro_x, ARRAYSIZE(gyro_x)));
  EXPECT_GT(gyro_x_std, 0.0);
  EXPECT_LT(gyro_x_std, gyro_std_threshold);
  double gyro_y_std = Sqrt(VarArray(gyro_y, ARRAYSIZE(gyro_y)));
  EXPECT_GT(gyro_y_std, 0.0);
  EXPECT_LT(gyro_y_std, gyro_std_threshold);
  double gyro_z_std = Sqrt(VarArray(gyro_z, ARRAYSIZE(gyro_z)));
  EXPECT_GT(gyro_z_std, 0.0);
  EXPECT_LT(gyro_z_std, gyro_std_threshold);
}

static const TestConfig kAdis16488Tests[] = {
  TEST_CONFIG_INIT(TestSyncPulse, 1500000),
  TEST_CONFIG_INIT(TestOutputData, 5000000),
};

const TestSuite kAdis16488Test =
    TEST_SUITE_INIT(kAdis16488Tests, TestSetup, TestTeardown);

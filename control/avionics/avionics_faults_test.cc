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

#include "control/avionics/avionics_faults.h"

#include <glog/logging.h>
#include <gtest/gtest.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <algorithm>
#include <cstdlib>
#include <functional>
#include <string>
#include <vector>

#include "avionics/common/avionics_messages.h"
#include "avionics/common/encoder_types.h"
#include "avionics/common/novatel_types.h"
#include "common/macros.h"
#include "control/avionics/avionics_interface_types.h"
#include "control/control_params.h"
#include "control/control_types.h"
#include "control/fault_detection/fault_detection_types.h"
#include "control/fault_detection/fault_detection_util.h"
#include "control/system_params.h"
#include "lib/util/test_util.h"
#include "system/labels.h"

extern "C" {

void CountWithReset(bool reset, int32_t *count);

}  // extern "C"

using ::test_util::RandNormalVec3;

class AvionicsFaultsCheckGsgTest : public ::testing::Test {
 public:
  AvionicsFaultsCheckGsgTest()
      : params_(GetControlParams()->fault_detection.gsg), state_(), faults_() {
    for (int32_t i = 0; i < kNumFaultDetectionGsgSignals; ++i) {
      ClearAllFaults(&faults_[i]);
    }
  }

 protected:
  const FaultDetectionGsgParams &params_;
  AvionicsFaultsGsgState state_;
  FaultMask faults_[kNumFaultDetectionGsgSignals];
};

TEST(CountWithReset, Trivial) {
  int32_t num_no_updates = INT32_MAX;
  CountWithReset(false, &num_no_updates);
  EXPECT_EQ(INT32_MAX, num_no_updates);

  num_no_updates = 0;
  for (int32_t i = 0; i < 22; ++i) {
    EXPECT_EQ(i, num_no_updates);
    CountWithReset(false, &num_no_updates);
  }
  for (int32_t i = 0; i < 22; ++i) {
    CountWithReset(true, &num_no_updates);
    EXPECT_EQ(0, num_no_updates);
  }
}

TEST_F(AvionicsFaultsCheckGsgTest, NoUpdate) {
  const int32_t *no_update_counts_limits =
      &params_.no_update_counts_limit[kFaultDetectionGsgSignalAzi];

  // Determine the max no-update limit, and initialize no update counters to
  // zero.
  int32_t max_limit = 0;
  for (int32_t i = 0; i < kNumFaultDetectionGsgSignals; ++i) {
    state_.num_no_updates[i] = 0;
    if (no_update_counts_limits[i] > max_limit) {
      max_limit = no_update_counts_limits[i];
    }
  }

  TetherDrum drum = TetherDrum();
  GsgData gsg_data = GsgData();
  for (int32_t i = 0; i < max_limit + 10; ++i) {
    AvionicsFaultsCheckGsg(&drum, false, &gsg_data, &params_, &state_, faults_);
    for (int32_t j = 0; j < kNumFaultDetectionGsgSignals; ++j) {
      EXPECT_EQ(i >= no_update_counts_limits[j],
                HasFault(kFaultTypeNoUpdate, &faults_[j]));
    }
  }
}

TEST_F(AvionicsFaultsCheckGsgTest, OutOfRange) {
  GsgData gsg_data;
  TetherDrum drum = TetherDrum();

  gsg_data.azi = params_.signal_min[0];
  gsg_data.ele = params_.signal_min[1];
  AvionicsFaultsCheckGsg(&drum, true, &gsg_data, &params_, &state_, faults_);
  for (int32_t i = 0; i < kNumFaultDetectionGsgSignals; ++i) {
    EXPECT_FALSE(HasFault(kFaultTypeOutOfRange, &faults_[i]));
  }

  gsg_data.azi = params_.signal_min[0] - 1.0;
  gsg_data.ele = params_.signal_min[1] - 1.0;
  AvionicsFaultsCheckGsg(&drum, true, &gsg_data, &params_, &state_, faults_);
  for (int32_t i = 0; i < kNumFaultDetectionGsgSignals; ++i) {
    EXPECT_TRUE(HasFault(kFaultTypeOutOfRange, &faults_[i]));
  }

  gsg_data.azi = params_.signal_max[0];
  gsg_data.ele = params_.signal_max[1];
  AvionicsFaultsCheckGsg(&drum, true, &gsg_data, &params_, &state_, faults_);
  for (int32_t i = 0; i < kNumFaultDetectionGsgSignals; ++i) {
    EXPECT_FALSE(HasFault(kFaultTypeOutOfRange, &faults_[i]));
  }

  gsg_data.azi = params_.signal_max[0] + 1.0;
  gsg_data.ele = params_.signal_max[1] + 1.0;
  AvionicsFaultsCheckGsg(&drum, true, &gsg_data, &params_, &state_, faults_);
  for (int32_t i = 0; i < kNumFaultDetectionGsgSignals; ++i) {
    EXPECT_TRUE(HasFault(kFaultTypeOutOfRange, &faults_[i]));
  }
}

TEST_F(AvionicsFaultsCheckGsgTest, ThrownError) {
  GsgData gsg_data;

  // Choose some sensible mid-range values.
  gsg_data.azi = (params_.signal_min[0] + params_.signal_max[0]) / 2.0;
  gsg_data.ele = (params_.signal_min[1] + params_.signal_max[1]) / 2.0;

  // Try all combinations of flags.
  for (bool azi_fault : {false, true}) {
    for (bool ele_fault : {false, true}) {
      TetherDrum drum = TetherDrum();
      if (azi_fault) {
        drum.flags |= kTetherDrumFlagGsgAxis1Fault;
      }
      if (ele_fault) {
        drum.flags |= kTetherDrumFlagGsgAxis2Fault;
      }

      AvionicsFaultsCheckGsg(&drum, true, &gsg_data, &params_, &state_,
                             faults_);

      EXPECT_EQ(azi_fault, HasFault(kFaultTypeThrownError,
                                    &faults_[kFaultDetectionGsgSignalAzi]));
      EXPECT_EQ(ele_fault, HasFault(kFaultTypeThrownError,
                                    &faults_[kFaultDetectionGsgSignalEle]));
    }
  }
}

TEST(GsCompass, NoUpdate) {
  const FaultDetectionGsCompassParams *params =
      &GetControlParams()->fault_detection.gs_compass;
  AvionicsFaultsGsCompassState state = AvionicsFaultsGsCompassState();
  auto gs_compass_message = TetherGsGpsCompass();
  FaultMask fault;

  gs_compass_message.flags = 0x0;

  int32_t limit = params->no_update_counts_limit;
  for (int32_t i = 0; i < limit + 10; ++i) {
    AvionicsFaultsCheckGsCompass(&gs_compass_message, false, params, &state,
                                 &fault);
    EXPECT_EQ(i >= limit, HasFault(kFaultTypeNoUpdate, &fault));
  }

  state.num_no_updates = 0;
  AvionicsFaultsCheckGsCompass(&gs_compass_message, true, params, &state,
                               &fault);
  EXPECT_FALSE(HasFault(kFaultTypeNoUpdate, &fault));
  EXPECT_FALSE(HasFault(kFaultTypeThrownError, &fault));

  // Check for thrown errors.
  state.num_no_updates = 0;
  gs_compass_message.flags = kTetherGsGpsCompassFlagFault;
  AvionicsFaultsCheckGsCompass(&gs_compass_message, true, params, &state,
                               &fault);
  EXPECT_FALSE(HasFault(kFaultTypeNoUpdate, &fault));
  EXPECT_TRUE(HasFault(kFaultTypeThrownError, &fault));
}

TEST(AvionicsFaultsCheckJoystick, NoUpdate) {
  const FaultDetectionJoystickParams *params =
      &GetControlParams()->fault_detection.joystick;
  AvionicsFaultsJoystickState state = AvionicsFaultsJoystickState();
  FaultMask fault;
  TetherJoystick message = TetherJoystick();

  int32_t limit = params->no_update_counts_limit;
  for (int32_t i = 0; i < limit + 10; ++i) {
    AvionicsFaultsCheckJoystick(&message, false, params, &state, &fault);
    EXPECT_EQ(i >= limit, HasFault(kFaultTypeNoUpdate, &fault));
  }

  message.flags = kTetherJoystickFlagFault;
  AvionicsFaultsCheckJoystick(&message, true, params, &state, &fault);
  EXPECT_TRUE(HasFault(kFaultTypeNoUpdate, &fault));

  message.flags = 0x0;
  AvionicsFaultsCheckJoystick(&message, true, params, &state, &fault);
  EXPECT_FALSE(HasFault(kFaultTypeNoUpdate, &fault));
}

TEST(LevelwindEle, NoUpdate) {
  const FaultDetectionLevelwindEleParams *params =
      &GetControlParams()->fault_detection.levelwind_ele;
  AvionicsFaultsLevelwindEleState state = AvionicsFaultsLevelwindEleState();
  TetherPlatform message = TetherPlatform();
  FaultMask fault;

  int32_t limit = params->no_update_counts_limit;
  for (int32_t i = 0; i < limit + 10; ++i) {
    AvionicsFaultsCheckLevelwindEle(&message, false, params, &state, &fault);
    EXPECT_EQ(i >= limit, HasFault(kFaultTypeNoUpdate, &fault));
  }
}

TEST(LevelwindEle, ThrownError) {
  const FaultDetectionLevelwindEleParams *params =
      &GetControlParams()->fault_detection.levelwind_ele;
  AvionicsFaultsLevelwindEleState state = AvionicsFaultsLevelwindEleState();
  TetherPlatform message = TetherPlatform();
  FaultMask fault;

  message.flags = 0x0;
  AvionicsFaultsCheckLevelwindEle(&message, true, params, &state, &fault);
  EXPECT_FALSE(HasFault(kFaultTypeThrownError, &fault));
  EXPECT_FALSE(HasFault(kFaultTypeNoUpdate, &fault));

  message.flags = kTetherPlatformFlagLevelwindElevationFault;
  AvionicsFaultsCheckLevelwindEle(&message, true, params, &state, &fault);
  EXPECT_TRUE(HasFault(kFaultTypeThrownError, &fault));
  EXPECT_FALSE(HasFault(kFaultTypeNoUpdate, &fault));
}

TEST(PerchAzi, NoUpdate) {
  const FaultDetectionPerchAziParams *params =
      &GetControlParams()->fault_detection.perch_azi;
  AvionicsFaultsPerchAziState state = AvionicsFaultsPerchAziState();
  TetherPlatform message = TetherPlatform();
  FaultMask fault;

  int32_t limit = params->no_update_counts_limit;
  for (int32_t i = 0; i < limit + 10; ++i) {
    AvionicsFaultsCheckPerchAzi(&message, false, params, &state, &fault);
    EXPECT_EQ(i >= limit, HasFault(kFaultTypeNoUpdate, &fault));
    EXPECT_FALSE(HasFault(kFaultTypeThrownError, &fault));
  }
}

TEST(PerchAzi, ThrownError) {
  const FaultDetectionPerchAziParams *params =
      &GetControlParams()->fault_detection.perch_azi;
  AvionicsFaultsPerchAziState state = AvionicsFaultsPerchAziState();
  TetherPlatform message = TetherPlatform();
  FaultMask fault;

  message.flags = 0x0;
  AvionicsFaultsCheckPerchAzi(&message, true, params, &state, &fault);
  EXPECT_FALSE(HasFault(kFaultTypeThrownError, &fault));
  EXPECT_FALSE(HasFault(kFaultTypeNoUpdate, &fault));

  message.flags = kTetherPlatformFlagPerchAzimuthFault;
  AvionicsFaultsCheckPerchAzi(&message, true, params, &state, &fault);
  EXPECT_TRUE(HasFault(kFaultTypeThrownError, &fault));
  EXPECT_FALSE(HasFault(kFaultTypeNoUpdate, &fault));
}

TEST(WinchSensor, NoUpdate) {
  const FaultDetectionWinchParams *params =
      &GetControlParams()->fault_detection.winch;
  AvionicsFaultsWinchSensorState state = AvionicsFaultsWinchSensorState();
  TetherPlc message = TetherPlc();
  FaultMask fault;

  int32_t limit = params->no_update_counts_limit;
  for (int32_t i = 0; i < limit + 10; ++i) {
    AvionicsFaultsCheckWinchSensor(&message, false, params, &state, &fault);
    EXPECT_EQ(i >= limit, HasFault(kFaultTypeNoUpdate, &fault));
  }
}

TEST(WinchSensor, ThrownError) {
  const FaultDetectionWinchParams *params =
      &GetControlParams()->fault_detection.winch;
  AvionicsFaultsWinchSensorState state = AvionicsFaultsWinchSensorState();
  TetherPlc message = TetherPlc();
  FaultMask fault;

  message.flags = 0x0;
  AvionicsFaultsCheckWinchSensor(&message, true, params, &state, &fault);
  EXPECT_FALSE(HasFault(kFaultTypeThrownError, &fault));
  EXPECT_FALSE(HasFault(kFaultTypeNoUpdate, &fault));

  message.flags = kTetherPlcFlagPlcWarning;
  AvionicsFaultsCheckWinchSensor(&message, true, params, &state, &fault);
  EXPECT_TRUE(HasFault(kFaultTypeThrownError, &fault));
  EXPECT_FALSE(HasFault(kFaultTypeNoUpdate, &fault));

  message.flags = kTetherPlcFlagPlcError;
  AvionicsFaultsCheckWinchSensor(&message, true, params, &state, &fault);
  EXPECT_TRUE(HasFault(kFaultTypeThrownError, &fault));
  EXPECT_FALSE(HasFault(kFaultTypeNoUpdate, &fault));

  message.flags = kTetherPlcFlagDrumFault;
  AvionicsFaultsCheckWinchSensor(&message, true, params, &state, &fault);
  EXPECT_TRUE(HasFault(kFaultTypeThrownError, &fault));
  EXPECT_FALSE(HasFault(kFaultTypeNoUpdate, &fault));
}

TEST(WindSensor, NoUpdate) {
  const FaultDetectionWindSensorParams *params =
      &GetControlParams()->fault_detection.wind;
  AvionicsFaultsWindSensorState state = AvionicsFaultsWindSensorState();
  TetherWind message = TetherWind();
  FaultMask fault;

  int32_t limit = params->no_update_counts_limit;
  for (int32_t i = 0; i < limit + 10; ++i) {
    AvionicsFaultsCheckWindSensor(&message, false, params, &state, &fault);
    EXPECT_EQ(i >= limit, HasFault(kFaultTypeNoUpdate, &fault));
  }
}

TEST(WindSensor, ThrownError) {
  const FaultDetectionWindSensorParams *params =
      &GetControlParams()->fault_detection.wind;
  AvionicsFaultsWindSensorState state = AvionicsFaultsWindSensorState();
  TetherWind message = TetherWind();
  FaultMask fault;

  message.status = kTetherWindStatusGood;
  AvionicsFaultsCheckWindSensor(&message, true, params, &state, &fault);
  EXPECT_FALSE(HasFault(kFaultTypeThrownError, &fault));
  EXPECT_FALSE(HasFault(kFaultTypeNoUpdate, &fault));

  message.status = kTetherWindStatusWarning;
  AvionicsFaultsCheckWindSensor(&message, true, params, &state, &fault);
  EXPECT_TRUE(HasFault(kFaultTypeThrownError, &fault));
  EXPECT_FALSE(HasFault(kFaultTypeNoUpdate, &fault));

  message.status = kTetherWindStatusFault;
  AvionicsFaultsCheckWindSensor(&message, true, params, &state, &fault);
  EXPECT_TRUE(HasFault(kFaultTypeThrownError, &fault));
  EXPECT_FALSE(HasFault(kFaultTypeNoUpdate, &fault));
}

template <int n>
void ClearFaultArray(FaultMask (*faults)[n]) {
  for (int32_t i = 0; i < n; i++) {
    ClearAllFaults(&(*faults)[i]);
  }
}

class AvionicsFaultsCheckImuTest : public ::testing::Test {
 public:
  AvionicsFaultsCheckImuTest()
      : imu_sensor_(),
        mag_sensor_(),
        imu_data_(),
        params_(GetControlParams()->fault_detection.imu),
        state_(),
        faults_(),
        num_signals_(kNumFaultDetectionImuSignals) {
    ClearFaultArray(&faults_);

    AvionicsFaultsState full_state;
    AvionicsFaultsInit(&full_state);
    state_ = full_state.imus[0];
  }

  void RunFaultCheck(bool cvt_updated) {
    ClearFaultArray(&faults_);
    AvionicsFaultsCheckImuAccGyro(&imu_sensor_, &imu_data_, cvt_updated,
                                  &params_, &state_, faults_);
    AvionicsFaultsCheckImuMag(&mag_sensor_, &imu_data_, cvt_updated, &params_,
                              &state_, faults_);
  }

 protected:
  FlightComputerImuMessage imu_sensor_;
  FlightComputerSensorMessage mag_sensor_;
  ImuData imu_data_;
  const FaultDetectionImuParams &params_;
  AvionicsFaultsImuState state_;
  FaultMask faults_[kNumFaultDetectionImuSignals];
  const int32_t num_signals_;
};

TEST_F(AvionicsFaultsCheckImuTest, NoUpdateInitialization) {
  RunFaultCheck(false);
  for (int32_t i = 0; i < num_signals_; ++i) {
    EXPECT_TRUE(HasFault(kFaultTypeNoUpdate, &faults_[i]));
    EXPECT_FALSE(state_.initialized[i]);
  }

  RunFaultCheck(true);
  for (int32_t i = 0; i < num_signals_; ++i) {
    EXPECT_FALSE(HasFault(kFaultTypeNoUpdate, &faults_[i]));
    EXPECT_TRUE(state_.initialized[i]);
  }
}

TEST_F(AvionicsFaultsCheckImuTest, NoUpdateByLatency_All) {
  RunFaultCheck(true);  // Initial update.

  imu_sensor_.raw.latency = static_cast<int32_t>(1e6 * params_.max_latency) + 1;
  RunFaultCheck(true);
  for (int32_t i = 0; i < num_signals_; ++i) {
    EXPECT_EQ(i != static_cast<int32_t>(kFaultDetectionImuSignalMag),
              HasFault(kFaultTypeNoUpdate, &faults_[i]));
  }
}

TEST_F(AvionicsFaultsCheckImuTest, NoUpdateByLatency_Mag) {
  RunFaultCheck(true);  // Initial update.

  mag_sensor_.aux.mag_latency =
      static_cast<int32_t>(1e6 * params_.mag_max_latency) + 1;
  RunFaultCheck(true);
  for (int32_t i = 0; i < num_signals_; ++i) {
    EXPECT_EQ(i == static_cast<int32_t>(kFaultDetectionImuSignalMag),
              HasFault(kFaultTypeNoUpdate, &faults_[i]));
  }
}

TEST_F(AvionicsFaultsCheckImuTest, NoUpdateByValue) {
  RunFaultCheck(true);  // Initial update.

  // Make sure latency checks pass.
  imu_sensor_.raw.latency = static_cast<int32_t>(0.5e6 * params_.max_latency);
  mag_sensor_.aux.mag_latency =
      static_cast<int32_t>(0.5e6 * params_.max_latency);

  // Determine the max no-update count limit.
  int32_t max_limit = 0;
  for (int32_t i = 0; i < num_signals_; ++i) {
    max_limit = std::max(max_limit, params_.no_update_counts_limits[i]);
  }

  for (int32_t i = 0; i < max_limit + 10; ++i) {
    RunFaultCheck(true);
    for (int32_t j = 0; j < num_signals_; ++j) {
      EXPECT_EQ(i >= params_.no_update_counts_limits[j],
                HasFault(kFaultTypeNoUpdate, &faults_[j]));
    }
  }
}

// TODO(b/27502284): Test Septentrio GPS.  Generalize tests to cover both GPS's.
void TestGpsNoUpdate(
    const std::function<void(const GpsData &, GpsData *)> &hold,
    FaultDetectionGpsSignalType faulted_signal) {
  const FaultDetectionGpsParams *params =
      &GetControlParams()->fault_detection.wing_gps;
  int32_t no_update_counts_limit =
      params->no_update_counts_limit[kFaultDetectionGpsSignalPos];

  // Clear no updated.
  NovAtelSolutionMessage gps_message = NovAtelSolutionMessage();

  GpsData gps = GpsData();
  GpsData gps_z1 = GpsData();

  FaultMask faults[kNumFaultDetectionGpsSignals];
  for (int32_t i = 0; i < kNumFaultDetectionGpsSignals; ++i) {
    ClearAllFaults(&faults[i]);
  }

  AvionicsFaultsGpsState state = AvionicsFaultsGpsState();
  state.pos_z1 = RandNormalVec3();
  state.vel_z1 = RandNormalVec3();

  for (int32_t i = 0; i < no_update_counts_limit + 10; ++i) {
    // Randomize the GpsData, and then apply the hold function to reset part of
    // it to the z1 values in `state`.
    gps.pos = RandNormalVec3();
    gps.vel = RandNormalVec3();
    gps_z1.pos = state.pos_z1;
    gps_z1.vel = state.vel_z1;
    hold(gps_z1, &gps);

    AvionicsFaultsCheckWingGpsNovAtel(&gps_message, false, params, &state,
                                      faults, &gps);

    EXPECT_EQ(i < no_update_counts_limit,
              !HasFault(kFaultTypeNoUpdate, &faults[faulted_signal]));
  }
}

TEST(AvionicsFaultsCheckWingGpsNovAtel, NoUpdate) {
  std::srand(314U);

  TestGpsNoUpdate(
      [](const GpsData &gps_z1, GpsData *gps) { gps->pos = gps_z1.pos; },
      kFaultDetectionGpsSignalPos);
  TestGpsNoUpdate(
      [](const GpsData &gps_z1, GpsData *gps) { gps->pos.x = gps_z1.pos.x; },
      kFaultDetectionGpsSignalPos);
  TestGpsNoUpdate(
      [](const GpsData &gps_z1, GpsData *gps) { gps->pos.y = gps_z1.pos.y; },
      kFaultDetectionGpsSignalPos);
  TestGpsNoUpdate(
      [](const GpsData &gps_z1, GpsData *gps) { gps->pos.z = gps_z1.pos.z; },
      kFaultDetectionGpsSignalPos);

  TestGpsNoUpdate(
      [](const GpsData &gps_z1, GpsData *gps) { gps->vel = gps_z1.vel; },
      kFaultDetectionGpsSignalVel);
  TestGpsNoUpdate(
      [](const GpsData &gps_z1, GpsData *gps) { gps->vel.x = gps_z1.vel.x; },
      kFaultDetectionGpsSignalVel);
  TestGpsNoUpdate(
      [](const GpsData &gps_z1, GpsData *gps) { gps->vel.y = gps_z1.vel.y; },
      kFaultDetectionGpsSignalVel);
  TestGpsNoUpdate(
      [](const GpsData &gps_z1, GpsData *gps) { gps->vel.z = gps_z1.vel.z; },
      kFaultDetectionGpsSignalVel);
}

TEST(AvionicsFaultsCheckWingGps, ThrownError) {
  GpsData gps = GpsData();

  NovAtelSolutionMessage gps_message = NovAtelSolutionMessage();
  gps_message.best_xyz.pos_sol_status = kNovAtelSolutionStatusSolComputed;
  gps_message.best_xyz.vel_sol_status = kNovAtelSolutionStatusSolComputed;

  const FaultDetectionGpsParams *params =
      &GetControlParams()->fault_detection.wing_gps;

  AvionicsFaultsGpsState state = AvionicsFaultsGpsState();
  state.pos_z1 = gps.pos;
  state.vel_z1 = gps.vel;

  FaultMask faults[kNumFaultDetectionGpsSignals];
  for (int32_t i = 0; i < kNumFaultDetectionGpsSignals; ++i) {
    ClearAllFaults(&faults[i]);
  }
  AvionicsFaultsCheckWingGpsNovAtel(&gps_message, false, params, &state, faults,
                                    &gps);
  // All zero message should result in thrown error.
  EXPECT_TRUE(
      HasFault(kFaultTypeThrownError, &faults[kFaultDetectionGpsSignalPos]));
  EXPECT_TRUE(
      HasFault(kFaultTypeThrownError, &faults[kFaultDetectionGpsSignalVel]));

  // Once satellites are reported, trust the solution.
  gps_message.best_xyz.num_sol = 5;

  memset(faults, 0, sizeof(faults));
  AvionicsFaultsCheckWingGpsNovAtel(&gps_message, false, params, &state, faults,
                                    &gps);
  EXPECT_FALSE(
      HasFault(kFaultTypeThrownError, &faults[kFaultDetectionGpsSignalPos]));
  EXPECT_FALSE(
      HasFault(kFaultTypeThrownError, &faults[kFaultDetectionGpsSignalVel]));

  // If the solution status is bad, don't trust the message.
  gps_message.best_xyz.vel_sol_status = kNovAtelSolutionStatusInsufficientObs;
  state.pos_z1 = gps.pos;
  state.vel_z1 = gps.vel;
  AvionicsFaultsCheckWingGpsNovAtel(&gps_message, false, params, &state, faults,
                                    &gps);
  EXPECT_FALSE(
      HasFault(kFaultTypeThrownError, &faults[kFaultDetectionGpsSignalPos]));
  EXPECT_TRUE(
      HasFault(kFaultTypeThrownError, &faults[kFaultDetectionGpsSignalVel]));

  gps_message.best_xyz.pos_sol_status = kNovAtelSolutionStatusInsufficientObs;
  AvionicsFaultsCheckWingGpsNovAtel(&gps_message, false, params, &state, faults,
                                    &gps);
  EXPECT_TRUE(
      HasFault(kFaultTypeThrownError, &faults[kFaultDetectionGpsSignalPos]));
  EXPECT_TRUE(
      HasFault(kFaultTypeThrownError, &faults[kFaultDetectionGpsSignalVel]));

  gps_message.best_xyz.vel_sol_status = kNovAtelSolutionStatusSolComputed;
  AvionicsFaultsCheckWingGpsNovAtel(&gps_message, false, params, &state, faults,
                                    &gps);
  EXPECT_TRUE(
      HasFault(kFaultTypeThrownError, &faults[kFaultDetectionGpsSignalPos]));
  EXPECT_FALSE(
      HasFault(kFaultTypeThrownError, &faults[kFaultDetectionGpsSignalVel]));
}

TEST(AvionicsFaultsCheckWingGps, LatencyNoUpdate) {
  GpsData gps = GpsData();
  NovAtelSolutionMessage gps_message = NovAtelSolutionMessage();
  gps_message.best_xyz.pos_sol_status = kNovAtelSolutionStatusSolComputed;
  gps_message.best_xyz.vel_sol_status = kNovAtelSolutionStatusSolComputed;
  gps_message.best_xyz.num_sol = 5;

  const FaultDetectionGpsParams *params =
      &GetControlParams()->fault_detection.wing_gps;

  AvionicsFaultsGpsState state = AvionicsFaultsGpsState();
  state.pos_z1 = gps.pos;
  state.vel_z1 = gps.vel;

  FaultMask faults[kNumFaultDetectionGpsSignals];
  for (int32_t i = 0; i < kNumFaultDetectionGpsSignals; ++i) {
    ClearAllFaults(&faults[i]);
  }

  AvionicsFaultsCheckWingGpsNovAtel(&gps_message, false, params, &state, faults,
                                    &gps);
  EXPECT_FALSE(
      HasFault(kFaultTypeNoUpdate, &faults[kFaultDetectionGpsSignalPos]));
  EXPECT_FALSE(
      HasFault(kFaultTypeNoUpdate, &faults[kFaultDetectionGpsSignalVel]));

  // If the latency is large, stop trusting the solution.
  gps_message.best_xyz_latency = INT32_MAX;

  memset(faults, 0, sizeof(faults));
  AvionicsFaultsCheckWingGpsNovAtel(&gps_message, false, params, &state, faults,
                                    &gps);
  EXPECT_TRUE(
      HasFault(kFaultTypeNoUpdate, &faults[kFaultDetectionGpsSignalPos]));
  EXPECT_TRUE(
      HasFault(kFaultTypeNoUpdate, &faults[kFaultDetectionGpsSignalVel]));
}

TEST(AvionicsFaultsCheckGpsCompass, CvtNoUpdate) {
  NovAtelCompassMessage novatel_compass = NovAtelCompassMessage();
  bool cvt_updated = false;
  const FaultDetectionGsCompassParams *params =
      &GetControlParams()->fault_detection.gs_compass;
  FaultMask faults[kNumFaultDetectionGpsCompassSignals];
  GpsCompassData compass;

  memset(faults, 0, sizeof(faults));
  novatel_compass.heading_latency = 1;
  novatel_compass.heading.pos_type = kNovAtelSolutionTypeSingle;
  novatel_compass.heading.pos_sol_status = kNovAtelSolutionStatusSolComputed;
  novatel_compass.heading.num_sol = 5;
  novatel_compass.heading_rate_latency = 1;
  novatel_compass.heading_rate.pos_type = kNovAtelSolutionTypeSingle;
  novatel_compass.heading_rate.pos_sol_status =
      kNovAtelSolutionStatusSolComputed;
  compass.new_data = true;
  ASSERT_TRUE(compass.new_data);
  AvionicsFaultsCheckNovAtelCompass(&novatel_compass, cvt_updated, params,
                                    faults, &compass);
  EXPECT_FALSE(compass.new_data);
  EXPECT_FALSE(HasAnyFault(&faults[kFaultDetectionGpsCompassSignalAngles]));
  EXPECT_FALSE(
      HasAnyFault(&faults[kFaultDetectionGpsCompassSignalAngularRates]));
}

TEST(AvionicsFaultsCheckGpsCompass, AngleFaults) {
  NovAtelCompassMessage novatel_compass = NovAtelCompassMessage();
  bool cvt_updated = false;
  const FaultDetectionGsCompassParams *params =
      &GetControlParams()->fault_detection.gs_compass;
  FaultMask faults[kNumFaultDetectionGpsCompassSignals];
  GpsCompassData compass;

  memset(faults, 0, sizeof(faults));
  novatel_compass.heading_latency =
      int32_t(params->max_latency * 1000000.0 + 1.0);
  novatel_compass.heading.pos_type = kNovAtelSolutionTypeSingle;
  novatel_compass.heading.pos_sol_status = kNovAtelSolutionStatusSolComputed;
  novatel_compass.heading.num_sol = 5;
  novatel_compass.heading_rate_latency = 1;
  novatel_compass.heading_rate.pos_type = kNovAtelSolutionTypeSingle;
  novatel_compass.heading_rate.pos_sol_status =
      kNovAtelSolutionStatusSolComputed;
  AvionicsFaultsCheckNovAtelCompass(&novatel_compass, cvt_updated, params,
                                    faults, &compass);
  EXPECT_TRUE(HasFault(kFaultTypeNoUpdate,
                       &faults[kFaultDetectionGpsCompassSignalAngles]));
  EXPECT_FALSE(
      HasAnyFault(&faults[kFaultDetectionGpsCompassSignalAngularRates]));

  memset(faults, 0, sizeof(faults));
  novatel_compass.heading_latency = 1;
  novatel_compass.heading.pos_sol_status =
      kNovAtelSolutionStatusInsufficientObs;
  AvionicsFaultsCheckNovAtelCompass(&novatel_compass, cvt_updated, params,
                                    faults, &compass);
  EXPECT_TRUE(HasFault(kFaultTypeThrownError,
                       &faults[kFaultDetectionGpsCompassSignalAngles]));
  EXPECT_FALSE(
      HasAnyFault(&faults[kFaultDetectionGpsCompassSignalAngularRates]));

  memset(faults, 0, sizeof(faults));
  novatel_compass.heading_latency = 1;
  novatel_compass.heading.pos_sol_status = kNovAtelSolutionStatusSolComputed;
  novatel_compass.heading.num_sol = 0;
  AvionicsFaultsCheckNovAtelCompass(&novatel_compass, cvt_updated, params,
                                    faults, &compass);
  EXPECT_TRUE(HasFault(kFaultTypeThrownError,
                       &faults[kFaultDetectionGpsCompassSignalAngles]));
  EXPECT_FALSE(
      HasAnyFault(&faults[kFaultDetectionGpsCompassSignalAngularRates]));
}

TEST(AvionicsFaultsCheckGpsCompass, AngularRateFaults) {
  NovAtelCompassMessage novatel_compass = NovAtelCompassMessage();
  bool cvt_updated = false;
  const FaultDetectionGsCompassParams *params =
      &GetControlParams()->fault_detection.gs_compass;
  FaultMask faults[kNumFaultDetectionGpsCompassSignals];
  GpsCompassData compass;

  memset(faults, 0, sizeof(faults));
  novatel_compass.heading_latency = 1;
  novatel_compass.heading.pos_type = kNovAtelSolutionTypeSingle;
  novatel_compass.heading.pos_sol_status = kNovAtelSolutionStatusSolComputed;
  novatel_compass.heading.num_sol = 5;
  novatel_compass.heading_rate_latency =
      int32_t(params->max_latency * 1000000.0 + 1.0);
  novatel_compass.heading_rate.pos_type = kNovAtelSolutionTypeSingle;
  novatel_compass.heading_rate.pos_sol_status =
      kNovAtelSolutionStatusSolComputed;
  AvionicsFaultsCheckNovAtelCompass(&novatel_compass, cvt_updated, params,
                                    faults, &compass);
  EXPECT_FALSE(HasAnyFault(&faults[kFaultDetectionGpsCompassSignalAngles]));

  EXPECT_TRUE(HasFault(kFaultTypeNoUpdate,
                       &faults[kFaultDetectionGpsCompassSignalAngularRates]));

  memset(faults, 0, sizeof(faults));
  novatel_compass.heading_rate_latency = 1;
  novatel_compass.heading_rate.pos_sol_status =
      kNovAtelSolutionStatusInsufficientObs;
  AvionicsFaultsCheckNovAtelCompass(&novatel_compass, cvt_updated, params,
                                    faults, &compass);
  EXPECT_FALSE(HasAnyFault(&faults[kFaultDetectionGpsCompassSignalAngles]));
  EXPECT_TRUE(HasFault(kFaultTypeThrownError,
                       &faults[kFaultDetectionGpsCompassSignalAngularRates]));
}

void TestGpsBadSigma(const std::function<void(GpsData *)> &adjust,
                     FaultDetectionGpsSignalType faulted_signal) {
  GpsData gps = GpsData();
  NovAtelSolutionMessage gps_message = NovAtelSolutionMessage();
  gps_message.best_xyz.pos_sol_status = kNovAtelSolutionStatusSolComputed;
  gps_message.best_xyz.vel_sol_status = kNovAtelSolutionStatusSolComputed;
  gps_message.best_xyz.num_sol = 5;

  const FaultDetectionGpsParams *params =
      &GetControlParams()->fault_detection.wing_gps;
  AvionicsFaultsGpsState state = AvionicsFaultsGpsState();

  FaultMask faults[kNumFaultDetectionGpsSignals];
  for (int32_t i = 0; i < kNumFaultDetectionGpsSignals; ++i) {
    ClearAllFaults(&faults[i]);
  }
  AvionicsFaultsCheckWingGpsNovAtel(&gps_message, false, params, &state, faults,
                                    &gps);
  EXPECT_FALSE(HasFault(kFaultTypeOutOfRange, &faults[faulted_signal]));

  adjust(&gps);
  AvionicsFaultsCheckWingGpsNovAtel(&gps_message, false, params, &state, faults,
                                    &gps);
  EXPECT_TRUE(HasFault(kFaultTypeOutOfRange, &faults[faulted_signal]));
}

class AvionicsFaultsCheckLoadcellsTest : public ::testing::Test {
 protected:
  AvionicsFaultsCheckLoadcellsTest()
      : state_(),
        sensor_faults_(),
        tether_release_fault_(),
        updated_{false, false, false, false},
        nodes_faulted_{false, false, false, false} {
    // Utility methods depend on the order of node labels.
    LoadcellNodeLabel ordered_labels[] = {
        kLoadcellNodePortA, kLoadcellNodePortB, kLoadcellNodeStarboardA,
        kLoadcellNodeStarboardB};
    for (int32_t i = 0; i < ARRAYSIZE(ordered_labels); ++i) {
      CHECK_EQ(i, static_cast<int32_t>(ordered_labels[i]));
    }

    AvionicsFaultsInit(&state_);

    for (FaultMask &f : sensor_faults_) {
      ClearAllFaults(&f);
    }
    ClearAllFaults(&tether_release_fault_);
  }

  void CheckLoadcells() {
    AvionicsFaultsCheckLoadcells(updated_, GetSystemParams()->loadcells,
                                 &GetControlParams()->fault_detection.loadcell,
                                 &state_.loadcells, sensor_faults_,
                                 nodes_faulted_, &tether_release_fault_);
  }

  void ExpectNodesFaulted(std::vector<bool> expected, bool actual[]) {
    CHECK_EQ(kNumLoadcellNodes, expected.size());
    for (int32_t i = 0; static_cast<size_t>(i) < expected.size(); ++i) {
      EXPECT_EQ(expected[i], actual[i]);
    }
  }

  void SetUpdated(const std::vector<bool> &values) {
    CHECK_EQ(ARRAYSIZE(updated_), values.size());
    for (int32_t i = 0; i < ARRAYSIZE(updated_); ++i) {
      updated_[i] = values[i];
    }
  }

  AvionicsFaultsState state_;
  FaultMask sensor_faults_[kNumLoadcellSensors];
  FaultMask tether_release_fault_;

  bool updated_[kNumLoadcellNodes];
  bool nodes_faulted_[kNumLoadcellNodes];
};

TEST_F(AvionicsFaultsCheckLoadcellsTest, PortSingleFault) {
  SetUpdated({true, true, true, true});
  updated_[kLoadcellNodePortB] = false;
  CheckLoadcells();
  ExpectNodesFaulted({false, true, false, false}, nodes_faulted_);
  EXPECT_FALSE(HasAnyFault(&tether_release_fault_));
}

TEST_F(AvionicsFaultsCheckLoadcellsTest, PortDoubleFault) {
  SetUpdated({false, false, true, true});
  CheckLoadcells();
  ExpectNodesFaulted({true, true, false, false}, nodes_faulted_);
  EXPECT_TRUE(HasFault(kFaultTypeNoUpdate, &tether_release_fault_));
}

TEST_F(AvionicsFaultsCheckLoadcellsTest, StarboardSingleFault) {
  SetUpdated({true, true, true, true});
  updated_[kLoadcellNodeStarboardB] = false;
  CheckLoadcells();
  ExpectNodesFaulted({false, false, false, true}, nodes_faulted_);
  EXPECT_FALSE(HasAnyFault(&tether_release_fault_));
}

TEST_F(AvionicsFaultsCheckLoadcellsTest, StarboardDoubleFault) {
  SetUpdated({true, true, false, false});
  CheckLoadcells();
  ExpectNodesFaulted({false, false, true, true}, nodes_faulted_);
  EXPECT_TRUE(HasFault(kFaultTypeNoUpdate, &tether_release_fault_));
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

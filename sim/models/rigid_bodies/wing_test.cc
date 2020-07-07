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

#include <stdint.h>

#include <limits>

#include "common/c_math/force_moment.h"
#include "common/c_math/vec3.h"
#include "control/system_params.h"
#include "control/system_types.h"
#include "lib/util/test_util.h"
#include "sim/faults/faults.h"
#include "sim/models/environment.h"
#include "sim/models/rigid_bodies/wing.h"
#include "sim/sim_params.h"
#include "sim/sim_types.h"

using ::test_util::RandNormal;

class WingTest : public ::testing::Test {
 protected:
  const SystemParams &system_params_;
  const SimParams &sim_params_;

  std::unique_ptr<FaultSchedule> faults_;
  std::unique_ptr<Environment> environment_;
  std::unique_ptr<Wing> wing_;

  Vec3 accel_no_disturb;
  Vec3 angular_accel_no_disturb;

  WingTest();
  void CheckForceAndMoment(double t, const Vec3 &force, const Vec3 &moment);
};

WingTest::WingTest()
    : system_params_(*GetSystemParams()),
      sim_params_(*GetSimParams()),
      faults_(nullptr),
      environment_(nullptr),
      wing_(nullptr),
      accel_no_disturb(kVec3Zero),
      angular_accel_no_disturb(kVec3Zero) {
  faults_.reset(new FaultSchedule(nullptr));
  environment_.reset(new Environment(
      sim_params_.iec_sim, sim_params_.phys_sim, system_params_.phys,
      system_params_.wind_sensor, system_params_.ground_frame));
  wing_.reset(new Wing(*environment_, environment_->ned_frame(),
                       system_params_.wing, sim_params_.aero_sim,
                       sim_params_.wing_sim, faults_.get()));
}

void WingTest::CheckForceAndMoment(double t, const Vec3 &force,
                                   const Vec3 &moment) {
  ForceMomentPos fmx_disturb;
  wing_->CalcDisturbForceMomentPos(t, &fmx_disturb);
  EXPECT_NEAR_VEC3(force, fmx_disturb.force, 0.0);
  EXPECT_NEAR_VEC3(moment, fmx_disturb.moment, 0.0);
}

TEST_F(WingTest, DisturbanceBodyForceSineTest) {
  Vec3 force = {RandNormal(), RandNormal(), RandNormal()};
  double freq = 1.0;
  SimFaultEvent fault = {22.0,   25.0,
                         "Wing", kSimFaultDisturbanceBodyForceSine,
                         4,      {force.x, force.y, force.z, freq}};
  faults_->AddFault(fault);

  CheckForceAndMoment(fault.t_start * (1.0 - DBL_EPSILON), kVec3Zero,
                      kVec3Zero);
  CheckForceAndMoment(fault.t_end, kVec3Zero, kVec3Zero);

  const int32_t kNumSteps = 7;
  for (int32_t i = 0; i < kNumSteps; ++i) {
    double t = fault.t_start + i * (fault.t_end - fault.t_start) / kNumSteps;
    Vec3 force_compare;
    Vec3Scale(&force, sin(2.0 * M_PI * freq * t), &force_compare);
    CheckForceAndMoment(t, force_compare, kVec3Zero);
  }
}

TEST_F(WingTest, DisturbanceBodyForceStepTest) {
  Vec3 force = {RandNormal(), RandNormal(), RandNormal()};
  SimFaultEvent fault = {22.0,   std::numeric_limits<double>::infinity(),
                         "Wing", kSimFaultDisturbanceBodyForceStep,
                         3,      {force.x, force.y, force.z}};
  faults_->AddFault(fault);

  CheckForceAndMoment(fault.t_start * (1.0 - DBL_EPSILON), kVec3Zero,
                      kVec3Zero);

  const int32_t kNumSteps = 7;
  for (int32_t i = 0; i < kNumSteps; ++i) {
    double t = fault.t_start + i;
    CheckForceAndMoment(t, force, kVec3Zero);
  }
}

TEST_F(WingTest, DisturbanceBodyTorqueSineTest) {
  Vec3 torque = {RandNormal(), RandNormal(), RandNormal()};
  double freq = 1.0;
  SimFaultEvent fault = {22.0,   25.0,
                         "Wing", kSimFaultDisturbanceBodyTorqueSine,
                         4,      {torque.x, torque.y, torque.z, freq}};
  faults_->AddFault(fault);

  CheckForceAndMoment(fault.t_start * (1.0 - DBL_EPSILON), kVec3Zero,
                      kVec3Zero);
  CheckForceAndMoment(fault.t_end, kVec3Zero, kVec3Zero);

  const int32_t kNumSteps = 7;
  for (int32_t i = 0; i < kNumSteps; ++i) {
    double t = fault.t_start + i * (fault.t_end - fault.t_start) / kNumSteps;
    Vec3 torque_compare;
    Vec3Scale(&torque, sin(2.0 * M_PI * freq * t), &torque_compare);
    CheckForceAndMoment(t, kVec3Zero, torque_compare);
  }
}

TEST_F(WingTest, DisturbanceBodyTorqueStepTest) {
  Vec3 torque = {RandNormal(), RandNormal(), RandNormal()};
  SimFaultEvent fault = {22.0,   std::numeric_limits<double>::infinity(),
                         "Wing", kSimFaultDisturbanceBodyTorqueStep,
                         3,      {torque.x, torque.y, torque.z}};
  faults_->AddFault(fault);

  CheckForceAndMoment(fault.t_start * (1.0 - DBL_EPSILON), kVec3Zero,
                      kVec3Zero);

  const int32_t kNumSteps = 7;
  for (int32_t i = 0; i < kNumSteps; ++i) {
    double t = fault.t_start + i;
    CheckForceAndMoment(t, kVec3Zero, torque);
  }
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  test_util::SetRunfilesDir();
  return RUN_ALL_TESTS();
}

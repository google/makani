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

#include <functional>
#include <map>
#include <vector>

#include "lib/util/test_util.h"
#include "sim/faults/faults.h"

class FaultScheduleTest : public ::testing::Test {
 protected:
  FaultSchedule faults_;
  SimFaultEvent event_a_, event_b_, event_c_;

  FaultScheduleTest() : faults_(nullptr), event_a_(), event_b_(), event_c_() {
    // Lint has trouble with these values in the initialization list.
    event_a_ = SimFaultEvent{
        0.0, 10.0, "A", kSimFaultMeasurementRescale, 3, {1.0, 2.0, 3.0}};
    event_b_ = SimFaultEvent{
        -10.0, 20.0, "B", kSimFaultMeasurementNoiseRescale, 2, {1.0, 2.0}};
    event_c_ = SimFaultEvent{-10.0, 20.0, "C", kSimFaultActuatorZero, 0, {0.0}};
    faults_.AddFault(event_a_);
    faults_.AddFault(event_b_);
  }

  void ClaimFaultFuncTest() {
    FaultSchedule::FaultFunc funcA = faults_.ClaimFaultFunc(
        event_a_.component, {{event_a_.type, event_a_.num_parameters}});
    FaultSchedule::FaultFunc funcC = faults_.ClaimFaultFunc(
        event_c_.component, {{event_c_.type, event_c_.num_parameters}});

    std::vector<double> params = {22.0};
    EXPECT_TRUE(funcA((event_a_.t_start + event_a_.t_end) / 2.0, event_a_.type,
                      &params));
    for (int32_t i = 0; i < event_a_.num_parameters; ++i) {
      EXPECT_EQ(event_a_.parameters[i], params[i]);
    }
    EXPECT_TRUE(funcA(event_a_.t_start, event_a_.type, &params));
    for (int32_t i = 0; i < event_a_.num_parameters; ++i) {
      EXPECT_EQ(event_a_.parameters[i], params[i]);
    }
    EXPECT_FALSE(funcA(event_a_.t_end, event_a_.type, &params));
    EXPECT_FALSE(funcA(event_a_.t_end + 1.0, event_a_.type, &params));
    EXPECT_FALSE(funcA(event_a_.t_start - 1.0, event_a_.type, &params));

    // Check that testing faults not in the schedule does not cause
    // an error.
    EXPECT_FALSE(funcA(event_a_.t_end, kSimFaultNoFault, &params));
    EXPECT_FALSE(funcA(event_a_.t_end, event_b_.type, &params));
    EXPECT_FALSE(funcC(event_a_.t_start, event_a_.type, &params));
    EXPECT_FALSE(funcC(event_a_.t_start, event_c_.type, &params));
  }

  void ClaimFaultFuncDeathTest() {
    faults_.ClaimFaultFunc(event_a_.component,
                           {{event_a_.type, event_a_.num_parameters}});
    EXPECT_DEATH(
        faults_.ClaimFaultFunc(event_a_.component,
                               {{event_a_.type, event_a_.num_parameters}}),
        "");
  }

  void AddFaultTest() {
    // Faults are defined on the interval [t_start, t_end), so this should
    // succeed.
    SimFaultEvent event = event_b_;
    event.t_start = -20.0;
    event.t_end = -10.0;
    faults_.AddFault(event);
    std::vector<double> params;
    EXPECT_TRUE(
        faults_.HasFault(-15.0, event_b_.component, event_b_.type, &params));
  }

  void AddFaultDeathTest() {
    SimFaultEvent event = event_b_;
    event.t_start = 10.0;
    event.t_end = 30.0;
    EXPECT_DEATH(faults_.AddFault(event), "");

    event.t_start = -20.0;
    event.t_end = 10.0;
    EXPECT_DEATH(faults_.AddFault(event), "");
  }

  void AllFaultsAreClaimedTest() {
    FaultKey key_a(event_a_.component, event_a_.type);
    FaultKey key_b(event_b_.component, event_b_.type);
    FaultKey key_c(event_c_.component, event_c_.type);

    // Test detection of missing FaultKey.
    // No simulated faults, so this should fail.
    EXPECT_FALSE(faults_.AllFaultsAreClaimed());

    // The "B" fault hasn't been claimed yet, so this should still fail.
    faults_.ClaimFaultFunc(event_a_.component,
                           {{event_a_.type, event_a_.num_parameters}});
    EXPECT_FALSE(faults_.AllFaultsAreClaimed());

    // Now A and B are claimed , so this should pass.
    faults_.ClaimFaultFunc(event_b_.component,
                           {{event_b_.type, event_b_.num_parameters}});
    EXPECT_TRUE(faults_.AllFaultsAreClaimed());

    // Claiming non-simluated faults shouldn't cause a problem.
    faults_.ClaimFaultFunc(event_c_.component,
                           {{event_c_.type, event_c_.num_parameters}});
    EXPECT_TRUE(faults_.AllFaultsAreClaimed());
  }
};

TEST_F(FaultScheduleTest, AllFaultsAreClaimedTest) {
  AllFaultsAreClaimedTest();
}

TEST_F(FaultScheduleTest, AddFaultTest) { AddFaultTest(); }

#if !defined(NDEBUG)

TEST_F(FaultScheduleTest, AddFaultDeathTest) { AddFaultDeathTest(); }

#endif

TEST_F(FaultScheduleTest, ClaimFaultFuncTest) { ClaimFaultFuncTest(); }

TEST_F(FaultScheduleTest, ClaimFaultFuncDeathTest) {
  ClaimFaultFuncDeathTest();
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

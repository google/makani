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

extern "C" {

#include "avionics/motor/firmware/flags.h"
#include "avionics/common/motor_thermal_types.h"

}

TEST(MotorWarningTempOffset, Limits) {
  EXPECT_EQ(kMotorWarningOverTempBoard, (1 << MOTOR_WARNING_TEMP_OFFSET));
  EXPECT_EQ(kMotorWarningOverTempHt3000C,
            (1 << (MOTOR_WARNING_TEMP_OFFSET + kNumMotorThermalChannels - 1)));
}

TEST(MotorWarningTempOffset, Ordering) {
  int32_t offset = MOTOR_WARNING_TEMP_OFFSET;
  EXPECT_EQ(kMotorWarningOverTempBoard,
            1 << (offset + kMotorThermalChannelBoard));
  EXPECT_EQ(kMotorWarningOverTempControllerAir,
            1 << (offset + kMotorThermalChannelControllerAir));
  EXPECT_EQ(kMotorWarningOverTempStatorCore,
            1 << (offset + kMotorThermalChannelStatorCore));
  EXPECT_EQ(kMotorWarningOverTempStatorCoil,
            1 << (offset + kMotorThermalChannelStatorCoil));
  EXPECT_EQ(kMotorWarningOverTempNacelleAir,
            1 << (offset + kMotorThermalChannelNacelleAir));
  EXPECT_EQ(kMotorWarningOverTempRotor,
            1 << (offset + kMotorThermalChannelRotor));
  EXPECT_EQ(kMotorWarningOverTempHeatPlate1,
            1 << (offset + kMotorThermalChannelHeatPlate1));
  EXPECT_EQ(kMotorWarningOverTempHeatPlate2,
            1 << (offset + kMotorThermalChannelHeatPlate2));
  EXPECT_EQ(kMotorWarningOverTempCapacitor,
            1 << (offset + kMotorThermalChannelCapacitor));
  EXPECT_EQ(kMotorWarningOverTempHt3000A,
            1 << (offset + kMotorThermalChannelHt3000A));
  EXPECT_EQ(kMotorWarningOverTempHt3000B,
            1 << (offset + kMotorThermalChannelHt3000B));
  EXPECT_EQ(kMotorWarningOverTempHt3000C,
            1 << (offset + kMotorThermalChannelHt3000C));
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

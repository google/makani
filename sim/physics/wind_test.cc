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

#include "lib/util/test_util.h"
#include "sim/physics/wind.h"

namespace sim {

namespace physics {

namespace wind {

class FrozenTurbulenceWindDatabaseTest : public ::testing::Test {
 protected:
  FrozenTurbulenceWindDatabaseTest() {}

  void TestExampleFile() const;
};

void FrozenTurbulenceWindDatabaseTest::TestExampleFile() const {
  const std::string example_wind(test_util::TestRunfilesDir() +
                                 "/sim/physics/test_data/example_wind.h5");
  FrozenTurbulenceWindDatabase database(0.0, 0.0, example_wind);

  EXPECT_EQ(1.0, database.mean_wind_speed_);

  EXPECT_EQ(10, database.num_t_);
  EXPECT_EQ(4, database.num_y_);
  EXPECT_EQ(2, database.num_z_);

  EXPECT_EQ(100.0, database.duration_);
  EXPECT_EQ(15.0, database.width_);
  EXPECT_EQ(5.0, database.height_);

  int32_t size = database.num_t_ * database.num_y_ * database.num_z_;
  EXPECT_EQ(size, database.u_->size);
  EXPECT_EQ(size, database.v_->size);
  EXPECT_EQ(size, database.w_->size);
  int32_t index = 0;
  // The last indices in the t and y axis are skipped
  // due to the wrapping.
  for (int32_t i = 0; i < database.num_t_ - 1; ++i) {
    for (int32_t j = 0; j < database.num_y_ - 1; ++j) {
      for (int32_t k = 0; k < database.num_z_; ++k) {
        Vec3 pos_w = {
            -database.duration_ * static_cast<float>(i) /
                static_cast<float>(database.num_t_ - 1),
            database.width_ * (static_cast<float>(j) /
                                   static_cast<float>(database.num_y_ - 1) -
                               0.5),
            database.height_ * (static_cast<float>(k) /
                                static_cast<float>(database.num_z_ - 1))};
        Vec3 wind_w;
        database.CalcWind(0.0, pos_w, &wind_w);
        EXPECT_NEAR(index, wind_w.x, 1e-5);
        EXPECT_NEAR(size + index, wind_w.y, 1e-5);
        EXPECT_NEAR(2 * size + index, wind_w.z, 1e-5);
        double t = -pos_w.x;
        pos_w.x = 0;
        database.CalcWind(t, pos_w, &wind_w);
        EXPECT_NEAR(index, wind_w.x, 1e-5);
        EXPECT_NEAR(size + index, wind_w.y, 1e-5);
        EXPECT_NEAR(2 * size + index, wind_w.z, 1e-5);
        index++;
      }
    }
    index += database.num_z_;
  }
}

TEST_F(FrozenTurbulenceWindDatabaseTest, ExampleFile) { TestExampleFile(); }

}  // namespace wind

}  // namespace physics

}  // namespace sim

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

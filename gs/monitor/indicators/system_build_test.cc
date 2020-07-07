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

#include <string.h>

#include "gs/monitor/indicators/system_build.h"

TEST(SystemBuildUpdate, Normal) {
  SystemBuild sys;
  SystemBuildInit(&sys);

  BuildInfo build;
  memset(&build, 0, sizeof(build));
  build.checksum[0] = 0xAE;
  build.flags = 0x01;

  // Test repeated insertions.
  AioNode node = kAioNodeFcA;
  for (int32_t i = 0; i < 4; ++i) {
    EXPECT_TRUE(SystemBuildUpdate(node, &build, &sys));
    EXPECT_TRUE(sys.prev_valid[node]);
    EXPECT_EQ(0, memcmp(&sys.prev[node], &build, sizeof(build)));
    EXPECT_EQ(1, sys.count);
    EXPECT_EQ(1, sys.list[0].ref_count);
    EXPECT_EQ(0, memcmp(build.checksum, sys.list[0].checksum,
                        sizeof(build.checksum)));
    EXPECT_EQ(build.flags, sys.list[0].flags);
  }

  // Test all nodes on the same version.
  for (int32_t i = 0; i < kNumAioNodes; ++i) {
    node = static_cast<AioNode>(i);
    EXPECT_TRUE(SystemBuildUpdate(node, &build, &sys));
    EXPECT_TRUE(sys.prev_valid[node]);
    EXPECT_EQ(0, memcmp(&sys.prev[node], &build, sizeof(build)));
  }
  EXPECT_EQ(1, sys.count);
  EXPECT_EQ(kNumAioNodes, sys.list[0].ref_count);

  // Test all nodes on a different version.
  for (int32_t i = 0; i < kNumAioNodes; ++i) {
    node = static_cast<AioNode>(i);
    build.checksum[1] = static_cast<uint8_t>(i);
    SystemBuildUpdate(node, &build, &sys);
  }
  for (int32_t i = 0; i < sys.count; ++i) {
    EXPECT_EQ(1, sys.list[i].ref_count);
  }
  EXPECT_EQ(kNumAioNodes, sys.count);

  // Test collapse back to the same version.
  build.checksum[2] = 0x20;
  for (int32_t i = 0; i < kNumAioNodes; ++i) {
    node = static_cast<AioNode>(i);
    EXPECT_TRUE(SystemBuildUpdate(node, &build, &sys) || i == 0);
  }
  EXPECT_EQ(1, sys.count);
  EXPECT_EQ(kNumAioNodes, sys.list[0].ref_count);

  // Test all nodes on same version, but different flags.
  for (int32_t i = 0; i < kNumAioNodes; ++i) {
    node = static_cast<AioNode>(i);
    build.flags = static_cast<uint8_t>(i);
    SystemBuildUpdate(node, &build, &sys);
  }
  for (int32_t i = 0; i < sys.count; ++i) {
    EXPECT_EQ(1, sys.list[i].ref_count);
  }
  EXPECT_EQ(kNumAioNodes, sys.count);
}

TEST(SystemBuildTimeout, Normal) {
  SystemBuild sys;
  SystemBuildInit(&sys);

  BuildInfo build;
  memset(&build, 0, sizeof(build));
  build.checksum[0] = 0xAE;
  build.flags = 0x01;

  // Test timeout before valid.
  for (int32_t i = 0; i < kNumAioNodes; ++i) {
    EXPECT_FALSE(sys.prev_valid[i]);
    SystemBuildTimeout(static_cast<AioNode>(i), &sys);
    EXPECT_FALSE(sys.prev_valid[i]);
  }

  // Test all nodes on the same version.
  for (int32_t i = 0; i < kNumAioNodes; ++i) {
    SystemBuildUpdate(static_cast<AioNode>(i), &build, &sys);
  }
  EXPECT_EQ(1, sys.count);
  EXPECT_EQ(kNumAioNodes, sys.list[0].ref_count);
  for (int32_t i = 0; i < kNumAioNodes; ++i) {
    EXPECT_TRUE(sys.prev_valid[i]);
    SystemBuildTimeout(static_cast<AioNode>(i), &sys);
    EXPECT_FALSE(sys.prev_valid[i]);
  }
  EXPECT_EQ(0, sys.count);

  // Test all nodes on a different version.
  for (int32_t i = 0; i < kNumAioNodes; ++i) {
    build.checksum[1] = static_cast<uint8_t>(i);
    SystemBuildUpdate(static_cast<AioNode>(i), &build, &sys);
  }
  EXPECT_EQ(kNumAioNodes, sys.count);
  EXPECT_EQ(1, sys.list[0].ref_count);
  for (int32_t i = 0; i < kNumAioNodes; ++i) {
    EXPECT_TRUE(sys.prev_valid[i]);
    SystemBuildTimeout(static_cast<AioNode>(i), &sys);
    EXPECT_FALSE(sys.prev_valid[i]);
  }
  EXPECT_EQ(0, sys.count);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

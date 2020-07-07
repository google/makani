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

#include "nav/ins/messages/seq_num.h"

TEST(InsSequenceUpdateSnapshot, Normal) {
  InsSequenceSnapshot snapshot;
  uint16_t seq_num;

  // Test typical use.
  snapshot.seq_num = 0U;
  snapshot.seq_bitmask = 0x121;
  seq_num = 1U;
  InsSequenceUpdateSnapshot(seq_num, &snapshot);
  EXPECT_EQ(seq_num, snapshot.seq_num);
  EXPECT_EQ(0x243, snapshot.seq_bitmask);

  // Test repeated sequence numbers.
  snapshot.seq_num = 23U;
  snapshot.seq_bitmask = 0x121;
  seq_num = 23U;
  InsSequenceUpdateSnapshot(seq_num, &snapshot);
  EXPECT_EQ(seq_num, snapshot.seq_num);
  EXPECT_EQ(0x121, snapshot.seq_bitmask);

  // Test significant gap in sequence numbers.
  snapshot.seq_num = 23U;
  snapshot.seq_bitmask = 0x121;
  seq_num = static_cast<uint16_t>(
      snapshot.seq_num + 8 * sizeof(snapshot.seq_bitmask));
  InsSequenceUpdateSnapshot(seq_num, &snapshot);
  EXPECT_EQ(seq_num, snapshot.seq_num);
  EXPECT_EQ(0x1, snapshot.seq_bitmask);

  // Test previous sequence numbers (invalid).
  snapshot.seq_num = 23U;
  snapshot.seq_bitmask = 0x121;
  seq_num = static_cast<uint16_t>(snapshot.seq_num - 1U);
  InsSequenceUpdateSnapshot(seq_num, &snapshot);
  EXPECT_EQ(seq_num, snapshot.seq_num);
  EXPECT_EQ(0x1, snapshot.seq_bitmask);

  // Test invalid snapshot.seq_num.
  snapshot.seq_num = 23U;
  snapshot.seq_bitmask = 0x328E;  // LSB not set.
  seq_num = static_cast<uint16_t>(seq_num + 1U);
  InsSequenceUpdateSnapshot(seq_num, &snapshot);
  EXPECT_EQ(seq_num, snapshot.seq_num);
  EXPECT_EQ(0x1, snapshot.seq_bitmask);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

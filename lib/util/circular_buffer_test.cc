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
#include <stdio.h>

#include "lib/util/circular_buffer.h"

class CircularBufferTest : public ::testing::Test {
 protected:
  virtual void TestDataReadyEmptyFull(const int32_t size,
                                      const int32_t fill_count) {
    CircularBuffer<int32_t> buffer(size);
    for (int32_t j = 0; j < fill_count; ++j) {
      for (int32_t i = 0; i < size; ++i) {
        ASSERT_EQ(buffer.Write(i), BufferWriteInterface<int32_t>::NO_ERROR);
        ASSERT_EQ(buffer.DataReady(), i + 1);
      }
      ASSERT_EQ(buffer.Write(22), BufferWriteInterface<int32_t>::BUFFER_FULL);
      // Read out each value, write the negative value in its place.
      int32_t value;
      for (int32_t i = 0; i < size; ++i) {
        ASSERT_EQ(buffer.Read(&value), BufferReadInterface<int32_t>::NO_ERROR);
        ASSERT_EQ(value, i);
        ASSERT_EQ(buffer.DataReady(), size - 1);
        ASSERT_EQ(buffer.Write(-i), BufferWriteInterface<int32_t>::NO_ERROR);
        ASSERT_EQ(buffer.Write(-i), BufferWriteInterface<int32_t>::BUFFER_FULL);
      }
      for (int32_t i = 0; i < size; ++i) {
        ASSERT_EQ(buffer.Read(&value), BufferReadInterface<int32_t>::NO_ERROR);
        ASSERT_EQ(value, -i);
        ASSERT_EQ(buffer.DataReady(), size - i - 1);
      }
      ASSERT_EQ(buffer.Read(&value),
                BufferReadInterface<int32_t>::BUFFER_EMPTY);
    }
  }
};

TEST_F(CircularBufferTest, TestDataReadyEmptyFull) {
  TestDataReadyEmptyFull(10, 3);
  TestDataReadyEmptyFull(4, 3);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

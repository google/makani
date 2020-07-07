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
#include <string.h>

#include "common/ring.h"

TEST(RingInit, Normal) {
  // Corrupt data structure and verify that all elements have been initialized.
  RingBuffer ring_5a, ring_a5;
  memset(&ring_5a, 0x5A, sizeof(ring_5a));
  memset(&ring_a5, 0xA5, sizeof(ring_a5));

  // Check for initialization of all elements.
  RingInit(128, &ring_5a);
  RingInit(128, &ring_a5);
  EXPECT_EQ(0, memcmp(&ring_5a, &ring_a5, sizeof(ring_5a)));

  // Check for proper initialization.
  EXPECT_EQ(0U, ring_5a.tail);
  EXPECT_EQ(0U, ring_5a.new_head);
  EXPECT_EQ(128U, ring_5a.elements);
}

TEST(RingGetUsed, Normal) {
  RingBuffer ring;

  // Check initialization.
  RingInit(128, &ring);
  EXPECT_EQ(0, RingGetUsed(&ring));

  // Check unwrapped case.
  ring.tail = 0U;
  ring.new_head = ring.tail + 1U;
  EXPECT_EQ(1, RingGetUsed(&ring));
  ring.tail = 100U;
  ring.new_head = ring.tail + 100U;
  EXPECT_EQ(100, RingGetUsed(&ring));
  ring.tail = UINT32_MAX - 100U;
  ring.new_head = ring.tail + 100U;
  EXPECT_EQ(100, RingGetUsed(&ring));

  // Check wrapped case.
  ring.tail = UINT32_MAX - 99U;
  ring.new_head = ring.tail + 100U;
  EXPECT_EQ(100, RingGetUsed(&ring));
  ring.tail = UINT32_MAX - 50U;
  ring.new_head = ring.tail + 100U;
  EXPECT_EQ(100, RingGetUsed(&ring));
}

TEST(RingIsEmpty, Normal) {
  RingBuffer ring;

  // Check initialization.
  RingInit(128, &ring);
  EXPECT_TRUE(RingIsEmpty(&ring));

  // Check empty buffer.
  ring.tail = 100U;
  ring.new_head = ring.tail;
  EXPECT_TRUE(RingIsEmpty(&ring));
  ring.tail = 200U;
  ring.new_head = ring.tail;
  EXPECT_TRUE(RingIsEmpty(&ring));

  // Check partial buffer.
  ring.tail = 0U;
  ring.new_head = ring.tail + 1U;
  EXPECT_FALSE(RingIsEmpty(&ring));
  ring.tail = 100U;
  ring.new_head = ring.tail + 1U;
  EXPECT_FALSE(RingIsEmpty(&ring));
  ring.tail = 100U;
  ring.new_head = ring.tail + ring.elements - 1U;
  EXPECT_FALSE(RingIsEmpty(&ring));

  // Check full buffer;
  ring.tail = 0U;
  ring.new_head = ring.tail + ring.elements;
  EXPECT_FALSE(RingIsEmpty(&ring));
  ring.tail = 100U;
  ring.new_head = ring.tail + ring.elements;
  EXPECT_FALSE(RingIsEmpty(&ring));
  ring.tail = 100U;
  ring.new_head = ring.tail + ring.elements + 1U;
  EXPECT_FALSE(RingIsEmpty(&ring));
}

TEST(RingIsFull, Normal) {
  RingBuffer ring;

  // Check initialization.
  RingInit(128, &ring);
  EXPECT_FALSE(RingIsFull(&ring));

  // Check partial buffer.
  ring.tail = 0U;
  ring.new_head = ring.tail + 1U;
  EXPECT_FALSE(RingIsFull(&ring));
  ring.tail = 100U;
  ring.new_head = ring.tail + 1U;
  EXPECT_FALSE(RingIsFull(&ring));
  ring.tail = 100U;
  ring.new_head = ring.tail + ring.elements - 1U;
  EXPECT_FALSE(RingIsFull(&ring));

  // Check full buffer;
  ring.tail = 0U;
  ring.new_head = ring.tail + ring.elements;
  EXPECT_TRUE(RingIsFull(&ring));
  ring.tail = 100U;
  ring.new_head = ring.tail + ring.elements;
  EXPECT_TRUE(RingIsFull(&ring));
  ring.tail = 100U;
  ring.new_head = ring.tail + ring.elements + 1U;
  EXPECT_TRUE(RingIsFull(&ring));
}

TEST(RingPushNewHead, Normal) {
  RingBuffer ring;
  RingInit(128, &ring);

  // Start with empty buffer with iterators near UINT32_MAX.
  ring.tail = UINT32_MAX - 4U;
  ring.new_head = ring.tail;

  // Check that new_head rolls over at UINT32_MAX.
  RingPushNewHead(&ring);
  EXPECT_EQ(UINT32_MAX - 3U, ring.new_head);
  RingPushNewHead(&ring);
  EXPECT_EQ(UINT32_MAX - 2U, ring.new_head);
  RingPushNewHead(&ring);
  EXPECT_EQ(UINT32_MAX - 1U, ring.new_head);
  RingPushNewHead(&ring);
  EXPECT_EQ(UINT32_MAX, ring.new_head);
  RingPushNewHead(&ring);
  EXPECT_EQ(0U, ring.new_head);
  RingPushNewHead(&ring);
  EXPECT_EQ(1U, ring.new_head);
}

TEST(RingPopTail, Normal) {
  RingBuffer ring;
  RingInit(128, &ring);

  ring.tail = UINT32_MAX;
  ring.new_head = ring.tail + 10U;
  RingPopTail(&ring);
  EXPECT_EQ(0U, ring.tail);
  RingPopTail(&ring);
  EXPECT_EQ(1U, ring.tail);
  RingPopTail(&ring);
  EXPECT_EQ(2U, ring.tail);
}

TEST(RingCompareGe, Normal) {
  EXPECT_TRUE(RingCompareGe(0U, 0U));
  EXPECT_TRUE(RingCompareGe(1U, 0U));
  EXPECT_TRUE(RingCompareGe(0U, UINT32_MAX));
  EXPECT_FALSE(RingCompareGe(0U, 1U));
}

TEST(RingCompareGt, Normal) {
  EXPECT_FALSE(RingCompareGt(0U, 0U));
  EXPECT_TRUE(RingCompareGt(1U, 0U));
  EXPECT_TRUE(RingCompareGt(0U, UINT32_MAX));
  EXPECT_FALSE(RingCompareGt(0U, 1U));
}

TEST(RingCompareLe, Normal) {
  EXPECT_TRUE(RingCompareLe(0U, 0U));
  EXPECT_TRUE(RingCompareLe(0U, 1U));
  EXPECT_TRUE(RingCompareLe(UINT32_MAX, 0U));
  EXPECT_FALSE(RingCompareLe(1U, 0U));
}

TEST(RingCompareLt, Normal) {
  EXPECT_FALSE(RingCompareLt(0U, 0U));
  EXPECT_TRUE(RingCompareLt(0U, 1U));
  EXPECT_TRUE(RingCompareLt(UINT32_MAX, 0U));
  EXPECT_FALSE(RingCompareLt(1U, 0U));
}

TEST(RingGetHead, Normal) {
  RingBuffer ring;
  RingInit(128, &ring);

  ring.tail = UINT32_MAX - 9U;
  ring.new_head = ring.tail + 10U;
  EXPECT_EQ(ring.new_head - 1U, RingGetHead(&ring));
}

TEST(RingGetNewHead, Normal) {
  RingBuffer ring;
  RingInit(128, &ring);

  ring.tail = UINT32_MAX;
  ring.new_head = ring.tail + 1U;
  EXPECT_EQ(ring.new_head, RingGetNewHead(&ring));
}

TEST(RingGetTail, Normal) {
  RingBuffer ring;
  RingInit(128, &ring);

  ring.tail = UINT32_MAX;
  ring.new_head = ring.tail + 10U;
  EXPECT_EQ(ring.tail, RingGetTail(&ring));
}

TEST(RingGetIndex, Normal) {
  RingBuffer ring;
  RingInit(128, &ring);
  int32_t index;

  // Test empty ring.
  EXPECT_FALSE(RingGetIndex(&ring, ring.tail - 1U, &index));
  EXPECT_FALSE(RingGetIndex(&ring, ring.tail, &index));
  EXPECT_FALSE(RingGetIndex(&ring, ring.new_head - 1U, &index));
  EXPECT_FALSE(RingGetIndex(&ring, ring.new_head, &index));

  // Test ring where iterators are greater than the number of elements (to
  // verify modulus).
  ring.tail = 100U;
  ring.new_head = ring.tail + 100U;
  EXPECT_FALSE(RingGetIndex(&ring, ring.tail - 1U, &index));  // Out of range.
  EXPECT_TRUE(RingGetIndex(&ring, 100U, &index));
  EXPECT_EQ(100U, index);
  EXPECT_TRUE(RingGetIndex(&ring, 150U, &index));
  EXPECT_EQ(22U, index);
  EXPECT_TRUE(RingGetIndex(&ring, 199U, &index));
  EXPECT_EQ(71U, index);
  EXPECT_TRUE(RingGetIndex(&ring, ring.new_head - 1U, &index));
  EXPECT_EQ(71U, index);
  EXPECT_FALSE(RingGetIndex(&ring, ring.new_head, &index));  // Out of range.
}

TEST(RingGetHeadIndex, Normal) {
  RingBuffer ring;
  RingInit(128, &ring);
  int32_t index;

  // Test empty ring.
  EXPECT_FALSE(RingGetHeadIndex(&ring, &index));

  // Test ring where iterators are greater than the number of elements (to
  // verify modulus).
  ring.tail = 100U;
  ring.new_head = ring.tail + 100U;
  EXPECT_TRUE(RingGetHeadIndex(&ring, &index));
  EXPECT_EQ(71U, index);
}

TEST(RingGetNewHeadIndex, Normal) {
  RingBuffer ring;
  RingInit(128, &ring);
  int32_t index;

  // Test empty ring.
  EXPECT_TRUE(RingGetNewHeadIndex(&ring, &index));
  EXPECT_EQ(0U, index);

  // Test full ring.
  ring.tail = 100U;
  ring.new_head = ring.tail + ring.elements;
  EXPECT_FALSE(RingGetNewHeadIndex(&ring, &index));

  // Test ring where iterators are greater than the number of elements (to
  // verify modulus).
  ring.tail = 100U;
  ring.new_head = ring.tail + 100U;
  EXPECT_TRUE(RingGetNewHeadIndex(&ring, &index));
  EXPECT_EQ(72U, index);
}

TEST(RingGetTailIndex, Normal) {
  RingBuffer ring;
  RingInit(128, &ring);
  int32_t index;

  // Test empty ring.
  EXPECT_FALSE(RingGetTailIndex(&ring, &index));

  // Test ring where iterators are greater than the number of elements (to
  // verify modulus).
  ring.tail = 200U;
  ring.new_head = ring.tail + 100U;
  EXPECT_TRUE(RingGetTailIndex(&ring, &index));
  EXPECT_EQ(72U, index);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

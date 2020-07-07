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

#include <vector>

#include "common/ring.h"
#include "nav/ins/inertial/inertial.h"
#include "nav/ins/messages/labels.h"
#include "nav/ins/messages/messages.h"

namespace {

typedef std::vector<InsMessageHeader> InsMessageHeaderList;

void AppendInsMessageHeader(const InsMessageHeader *header,
                            const InsInertialMessage * /* data */, void *arg) {
  InsMessageHeaderList *list = reinterpret_cast<InsMessageHeaderList *>(arg);
  list->push_back(*header);
}

void InsInertialMessageHeaderInit(uint16_t seq_num, InsMessageHeader *header) {
  memset(header, 0, sizeof(*header));
  header->type = kInsMessageTypeInertial;
  header->seq_num = seq_num;
  header->label = kInsImuLabelPrimary;
}

}  // namespace

TEST(InsInertialInit, Normal) {
  InsInertial inertial;
  memset(&inertial, 0x5A, sizeof(inertial));

  InsInertialInit(&inertial);
  EXPECT_TRUE(RingIsEmpty(&inertial.buffer.ring));
}

TEST(InsInertialGetNewHead, Normal) {
  InsInertial inertial;
  InsInertialInit(&inertial);
  InsMessageHeader header;
  InsInertialMessage *data;

  // Insert first message.
  InsInertialMessageHeaderInit(0U, &header);
  EXPECT_TRUE(InsInertialGetNewHead(&header, &inertial, &data));
  EXPECT_NE(nullptr, data);
  InsInertialPushNewHead(&inertial);

  // Insert second message.
  InsInertialMessageHeaderInit(1U, &header);
  EXPECT_TRUE(InsInertialGetNewHead(&header, &inertial, &data));
  EXPECT_NE(nullptr, data);
  InsInertialPushNewHead(&inertial);

  // Test de-duplication. Re-insert first message.
  InsInertialMessageHeaderInit(0U, &header);
  EXPECT_FALSE(InsInertialGetNewHead(&header, &inertial, &data));

  // Test de-duplication. Re-insert second message.
  InsInertialMessageHeaderInit(1U, &header);
  EXPECT_FALSE(InsInertialGetNewHead(&header, &inertial, &data));

  // Fill buffer to test overflow condition.
  for (int32_t i = 2; i < INS_INERTIAL_RING_ELEMENTS; ++i) {
    InsInertialMessageHeaderInit(static_cast<uint16_t>(i), &header);
    EXPECT_TRUE(InsInertialGetNewHead(&header, &inertial, &data));
    EXPECT_NE(nullptr, data);
    InsInertialPushNewHead(&inertial);
  }

  // Test overflow condition.
  ++header.seq_num;
  EXPECT_FALSE(InsInertialGetNewHead(&header, &inertial, &data));
}

TEST(InsInertialRemoveTail, Normal) {
  InsInertial inertial;
  InsInertialInit(&inertial);

  // Test empty buffer.
  InsInertialRemoveTail(INT64_MIN, &inertial);
  EXPECT_EQ(0, RingGetUsed(&inertial.buffer.ring));

  // Fill buffer.
  for (int32_t i = 0; i < INS_INERTIAL_RING_ELEMENTS; ++i) {
    InsInertialMessage *data;
    InsMessageHeader header;
    InsInertialMessageHeaderInit(static_cast<uint16_t>(i), &header);
    header.timestamp = i;
    EXPECT_TRUE(InsInertialGetNewHead(&header, &inertial, &data));
    InsInertialPushNewHead(&inertial);
  }
  EXPECT_EQ(INS_INERTIAL_RING_ELEMENTS, RingGetUsed(&inertial.buffer.ring));

  // Test min_time less than oldest message condition.
  InsInertialRemoveTail(INT64_MIN, &inertial);
  EXPECT_EQ(INS_INERTIAL_RING_ELEMENTS, RingGetUsed(&inertial.buffer.ring));

  // Test min_time equal to oldest message condition.
  InsInertialRemoveTail(0, &inertial);
  EXPECT_EQ(INS_INERTIAL_RING_ELEMENTS - 1, RingGetUsed(&inertial.buffer.ring));

  // Test removal of multiple messages.
  InsInertialRemoveTail(INS_INERTIAL_RING_ELEMENTS / 2 - 1, &inertial);
  EXPECT_EQ(INS_INERTIAL_RING_ELEMENTS / 2, RingGetUsed(&inertial.buffer.ring));

  // Test removal of all messages.
  InsInertialRemoveTail(INT64_MAX, &inertial);
  EXPECT_EQ(0, RingGetUsed(&inertial.buffer.ring));
}

TEST(InsInertialIterateToSeqNum, Normal) {
  InsInertial inertial;
  InsInertialInit(&inertial);

  // Force sequence number rollover condition.
  const uint16_t seq_num0 = UINT16_MAX - INS_INERTIAL_RING_ELEMENTS / 2;
  const uint16_t seq_num_last =
      static_cast<uint16_t>(seq_num0 + INS_INERTIAL_RING_ELEMENTS - 1);

  // Fill buffer.
  for (int32_t i = 0; i < INS_INERTIAL_RING_ELEMENTS; ++i) {
    InsInertialMessage *data;
    InsMessageHeader header;
    InsInertialMessageHeaderInit(static_cast<uint16_t>(seq_num0 + i), &header);
    EXPECT_TRUE(InsInertialGetNewHead(&header, &inertial, &data));
    InsInertialPushNewHead(&inertial);
  }
  EXPECT_EQ(INS_INERTIAL_RING_ELEMENTS, RingGetUsed(&inertial.buffer.ring));

  // Test processing of tail.
  InsMessageHeaderList list;
  uint32_t iter = InsInertialGetTailIter(&inertial);
  InsInertialIterateToSeqNum(&inertial, seq_num0, AppendInsMessageHeader,
                             &list, &iter);
  EXPECT_EQ(seq_num0, list.back().seq_num);
  EXPECT_EQ(1, list.size());

  // Test processing of tail+1 to tail+3.
  list.clear();
  InsInertialIterateToSeqNum(&inertial, static_cast<uint16_t>(seq_num0 + 2),
                             AppendInsMessageHeader, &list, &iter);
  EXPECT_EQ(static_cast<uint16_t>(seq_num0 + 1), list.front().seq_num);
  EXPECT_EQ(static_cast<uint16_t>(seq_num0 + 2), list.back().seq_num);
  EXPECT_EQ(2, list.size());

  // Test processing of tail to head.
  list.clear();
  iter = InsInertialGetTailIter(&inertial);
  InsInertialIterateToSeqNum(&inertial, seq_num_last, AppendInsMessageHeader,
                             &list, &iter);
  EXPECT_EQ(seq_num0, list.front().seq_num);
  EXPECT_EQ(seq_num_last, list.back().seq_num);
  EXPECT_EQ(INS_INERTIAL_RING_ELEMENTS, list.size());
}

TEST(InsInertialIterateToHead, Normal) {
  InsInertial inertial;
  InsInertialInit(&inertial);

  // Force sequence number rollover condition.
  const uint16_t seq_num0 = UINT16_MAX - INS_INERTIAL_RING_ELEMENTS / 2;
  const uint16_t seq_num_last =
      static_cast<uint16_t>(seq_num0 + INS_INERTIAL_RING_ELEMENTS - 1);

  // Fill buffer.
  for (int32_t i = 0; i < INS_INERTIAL_RING_ELEMENTS; ++i) {
    InsInertialMessage *data;
    InsMessageHeader header;
    InsInertialMessageHeaderInit(static_cast<uint16_t>(seq_num0 + i), &header);
    EXPECT_TRUE(InsInertialGetNewHead(&header, &inertial, &data));
    InsInertialPushNewHead(&inertial);
  }
  EXPECT_EQ(INS_INERTIAL_RING_ELEMENTS, RingGetUsed(&inertial.buffer.ring));

  // Test processing from tail.
  InsMessageHeaderList list;
  uint32_t iter = InsInertialGetTailIter(&inertial);
  InsInertialIterateToHead(&inertial, AppendInsMessageHeader, &list, &iter);
  EXPECT_EQ(seq_num0, list.front().seq_num);
  EXPECT_EQ(seq_num_last, list.back().seq_num);
  EXPECT_EQ(INS_INERTIAL_RING_ELEMENTS, list.size());

  // Test processing from mid buffer.
  list.clear();
  iter = InsInertialGetTailIter(&inertial) + INS_INERTIAL_RING_ELEMENTS / 2;
  InsInertialIterateToHead(&inertial, AppendInsMessageHeader,
                           &list, &iter);
  EXPECT_EQ(static_cast<uint16_t>(seq_num0 + INS_INERTIAL_RING_ELEMENTS / 2),
            list.front().seq_num);
  EXPECT_EQ(seq_num_last, list.back().seq_num);
  EXPECT_EQ(INS_INERTIAL_RING_ELEMENTS / 2, list.size());
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

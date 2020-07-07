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

#include "nav/ins/estimate/estimate.h"
#include "nav/ins/inertial/inertial.h"
#include "nav/ins/messages/labels.h"
#include "nav/ins/messages/messages.h"

TEST(InsEstimatePropagateForward, Sequence) {
  InsMessageHeader header;
  InsInertialMessage data;
  InsEstimate xhat_1, xhat;

  // Initialize input data.
  memset(&header, 0, sizeof(header));
  header.seq_num = 921U;
  memset(&data, 0, sizeof(data));

  // Initialize previous estimate.
  InsEstimateInit(&xhat_1);
  xhat_1.inertial_iter = 102412U;
  xhat_1.inertial_seq_num = 916U;
  xhat_1.filter_seq_num = 910U;

  // Propagate estimate one step using input data.
  InsEstimatePropagateForward(&header, &data, &xhat_1, &xhat);
  EXPECT_EQ(xhat_1.inertial_iter + 1U, xhat.inertial_iter);
  EXPECT_EQ(header.seq_num, xhat.inertial_seq_num);
  EXPECT_EQ(xhat_1.filter_seq_num, xhat.filter_seq_num);

  // Next input data.
  memset(&header, 0, sizeof(header));
  header.seq_num = 923U;  // Sequence numbers may not be contiguous.

  // Propagate estimate one step using input data.
  xhat_1 = xhat;
  InsEstimatePropagateForward(&header, &data, &xhat_1, &xhat);
  EXPECT_EQ(xhat_1.inertial_iter + 1U, xhat.inertial_iter);
  EXPECT_EQ(header.seq_num, xhat.inertial_seq_num);
  EXPECT_EQ(xhat_1.filter_seq_num, xhat.filter_seq_num);
}

TEST(InsEstimatePropagateForwardToSeqNum, Normal) {
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
    header.label = kInsImuLabelPrimary;
    header.seq_num = static_cast<uint16_t>(seq_num0 + i);
    EXPECT_TRUE(InsInertialGetNewHead(&header, &inertial, &data));
    InsInertialPushNewHead(&inertial);
  }
  EXPECT_EQ(INS_INERTIAL_RING_ELEMENTS, RingGetUsed(&inertial.buffer.ring));

  InsEstimateBuffer estimate;
  uint32_t iter0 = InsInertialGetTailIter(&inertial);
  InsEstimateBufferInit(iter0, &estimate);
  InsEstimateGetCurrent(&estimate)->filter_seq_num = 5U;

  // Test processing of tail.
  InsEstimatePropagateForwardToSeqNum(&inertial, seq_num0, &estimate);
  EXPECT_EQ(iter0 + 1U, InsEstimateGetCurrent(&estimate)->inertial_iter);
  EXPECT_EQ(seq_num0, InsEstimateGetCurrent(&estimate)->inertial_seq_num);
  EXPECT_EQ(5U, InsEstimateGetCurrent(&estimate)->filter_seq_num);

  // Test processing of tail+1 to head.
  InsEstimatePropagateForwardToSeqNum(&inertial, seq_num_last, &estimate);
  EXPECT_EQ(iter0 + INS_INERTIAL_RING_ELEMENTS,
            InsEstimateGetCurrent(&estimate)->inertial_iter);
  EXPECT_EQ(seq_num_last, InsEstimateGetCurrent(&estimate)->inertial_seq_num);
  EXPECT_EQ(5U, InsEstimateGetCurrent(&estimate)->filter_seq_num);

  // Test processing past head.
  InsEstimatePropagateForwardToSeqNum(&inertial, seq_num_last, &estimate);
  EXPECT_EQ(iter0 + INS_INERTIAL_RING_ELEMENTS,
            InsEstimateGetCurrent(&estimate)->inertial_iter);
  EXPECT_EQ(seq_num_last, InsEstimateGetCurrent(&estimate)->inertial_seq_num);
  EXPECT_EQ(5U, InsEstimateGetCurrent(&estimate)->filter_seq_num);
}

TEST(InsEstimatePropagateForwardToHead, Normal) {
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
    header.label = kInsImuLabelPrimary;
    header.seq_num = static_cast<uint16_t>(seq_num0 + i);
    EXPECT_TRUE(InsInertialGetNewHead(&header, &inertial, &data));
    InsInertialPushNewHead(&inertial);
  }
  EXPECT_EQ(INS_INERTIAL_RING_ELEMENTS, RingGetUsed(&inertial.buffer.ring));

  InsEstimateBuffer estimate;
  uint32_t iter0 = InsInertialGetTailIter(&inertial);
  InsEstimateBufferInit(iter0, &estimate);
  InsEstimateGetCurrent(&estimate)->filter_seq_num = 5U;

  // Test processing from tail.
  InsEstimatePropagateForwardToHead(&inertial, &estimate);
  EXPECT_EQ(iter0 + INS_INERTIAL_RING_ELEMENTS,
            InsEstimateGetCurrent(&estimate)->inertial_iter);
  EXPECT_EQ(seq_num_last, InsEstimateGetCurrent(&estimate)->inertial_seq_num);
  EXPECT_EQ(5U, InsEstimateGetCurrent(&estimate)->filter_seq_num);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

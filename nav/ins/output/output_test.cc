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

#include "common/ring.h"
#include "nav/ins/estimate/estimate.h"
#include "nav/ins/inertial/inertial.h"
#include "nav/ins/messages/messages.h"
#include "nav/ins/output/output.h"

TEST(InsOutputUpdateFilterEstimate, Normal) {
  InsInertial inertial;
  InsOutput output;

  InsInertialInit(&inertial);
  InsOutputInit(&inertial, &output);

  EXPECT_TRUE(RingIsEmpty(&output.filter.ring));

  InsEstimate estimate;
  InsOutputUpdateFilterEstimate(&estimate, &output);
  EXPECT_EQ(1, RingGetUsed(&output.filter.ring));
}

TEST(InsOutputPropagateInertialEstimate, Normal) {
  InsInertial inertial;
  InsOutput output;

  InsInertialInit(&inertial);
  InsOutputInit(&inertial, &output);

  // Insert 10 updates into inertial buffer.
  InsMessageHeader header;
  memset(&header, 0, sizeof(header));
  for (int32_t i = 0; i < 10; ++i) {
    InsInertialMessage *data;
    ++header.seq_num;
    EXPECT_TRUE(InsInertialGetNewHead(&header, &inertial, &data));
    memset(data, 0, sizeof(*data));
    InsInertialPushNewHead(&inertial);
  }

  // Propagate output to the head of the inertial buffer.
  InsOutputPropagateInertialEstimate(&inertial, &output);
  EXPECT_EQ(10U, InsOutputGetInertialEstimate(&output)->inertial_iter);
  EXPECT_EQ(10U, InsOutputGetInertialEstimate(&output)->inertial_seq_num);
  EXPECT_EQ(0U, InsOutputGetInertialEstimate(&output)->filter_seq_num);

  // Apply filter update from inertial seq_num=2.
  InsEstimate filter;
  memset(&filter, 0, sizeof(filter));
  filter.inertial_iter = 2U;
  filter.inertial_seq_num = 2U;
  filter.filter_seq_num = 2U;
  InsOutputUpdateFilterEstimate(&filter, &output);
  EXPECT_EQ(10U, InsOutputGetInertialEstimate(&output)->inertial_iter);
  EXPECT_EQ(10U, InsOutputGetInertialEstimate(&output)->inertial_seq_num);
  EXPECT_EQ(0U, InsOutputGetInertialEstimate(&output)->filter_seq_num);

  // Insert 2 updates into inertial buffer.
  for (int32_t i = 0; i < 2; ++i) {
    InsInertialMessage *data;
    ++header.seq_num;
    EXPECT_TRUE(InsInertialGetNewHead(&header, &inertial, &data));
    memset(data, 0, sizeof(*data));
    InsInertialPushNewHead(&inertial);
  }

  // Propagate output to the head of the inertial buffer.
  InsOutputPropagateInertialEstimate(&inertial, &output);
  EXPECT_EQ(12U, InsOutputGetInertialEstimate(&output)->inertial_iter);
  EXPECT_EQ(12U, InsOutputGetInertialEstimate(&output)->inertial_seq_num);
  EXPECT_EQ(2U, InsOutputGetInertialEstimate(&output)->filter_seq_num);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

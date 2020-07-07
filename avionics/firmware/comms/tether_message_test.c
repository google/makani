/*
 * Copyright 2020 Makani Technologies LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "avionics/firmware/comms/tether_message_test.h"

#include <stdint.h>
#include <string.h>

#include "avionics/common/avionics_messages.h"
#include "avionics/common/cvt.h"
#include "avionics/common/cvt_avionics_messages.h"
#include "avionics/common/tether_convert.h"
#include "avionics/common/tether_message.h"
#include "avionics/firmware/test/test.h"
#include "avionics/network/aio_node.h"
#include "common/macros.h"

static struct {
  uint8_t before[128];
  TetherDownMergeState state;
  uint8_t after[128];
} g_tether_down;

static struct {
  uint8_t before[128];
  TetherUpMergeState state;
  uint8_t after[128];
} g_tether_up;

static void TestSetup(void) {
  TetherDownMergeStateInit(&g_tether_down.state);
  TetherUpMergeStateInit(&g_tether_up.state);
}

static void TestTeardown(void) {
}

static void CheckCanary(int32_t length, const uint8_t *data,
                        uint8_t expected_value) {
  for (int32_t i = 0; i < length; ++i) {
    EXPECT_EQ(data[i], expected_value);
  }
}

static const TetherDownMessage *RunTetherDownMergeCvtGet(uint8_t canary) {
  memset(g_tether_down.before, canary, sizeof(g_tether_down.before));
  memset(g_tether_down.after, canary, sizeof(g_tether_down.after));

  const TetherDownMessage *merged = TetherDownMergeCvtGet(&g_tether_down.state);

  CheckCanary(ARRAYSIZE(g_tether_down.before), g_tether_down.before, canary);
  CheckCanary(ARRAYSIZE(g_tether_down.after), g_tether_down.after, canary);

  return merged;
}

static void TestTetherDownMergeCvtGetCanary(void) {
  const uint8_t canaries[] = {0x00, 0xFF};

  // Test canary with empty CVT.
  for (int32_t i = 0; i < kNumTetherDownSources; ++i) {
    AioNode source = TetherDownSourceToAioNode((TetherDownSource)i);
    CvtClearMessage(source, kMessageTypeTetherDown);
  }
  for (int32_t i = 0; i < ARRAYSIZE(canaries); ++i) {
    RunTetherDownMergeCvtGet(canaries[i]);
  }

  // Test canary with one new message.
  for (int32_t i = 0; i < kNumTetherDownSources; ++i) {
    TetherDownMessage in;
    memset(&in, 0, sizeof(in));
    AioNode source = TetherDownSourceToAioNode((TetherDownSource)i);
    for (int32_t j = 0; j < ARRAYSIZE(canaries); ++j) {
      CvtClearMessage(source, kMessageTypeTetherDown);
      EXPECT_EQ(kCvtEventNone, CvtPutTetherDownMessage(source, &in, 1U, 1));
      RunTetherDownMergeCvtGet(canaries[j]);
    }
  }

  // Test canary with all new messages.
  for (int32_t i = 0; i < ARRAYSIZE(canaries); ++i) {
    for (int32_t j = 0; j < kNumTetherDownSources; ++j) {
      TetherDownMessage in;
      memset(&in, 0, sizeof(in));
      AioNode source = TetherDownSourceToAioNode((TetherDownSource)j);
      CvtClearMessage(source, kMessageTypeTetherDown);
      EXPECT_EQ(kCvtEventNone, CvtPutTetherDownMessage(source, &in, 1U, 1));
    }
    RunTetherDownMergeCvtGet(canaries[i]);
  }
}

static const TetherUpMessage *RunTetherUpMergeCvtGet(uint8_t canary) {
  memset(g_tether_up.before, canary, sizeof(g_tether_up.before));
  memset(g_tether_up.after, canary, sizeof(g_tether_up.after));

  const TetherUpMessage *merged = TetherUpMergeCvtGet(&g_tether_up.state);

  CheckCanary(ARRAYSIZE(g_tether_up.before), g_tether_up.before, canary);
  CheckCanary(ARRAYSIZE(g_tether_up.after), g_tether_up.after, canary);

  return merged;
}

static void TestTetherUpMergeCvtGetCanary(void) {
  const uint8_t canaries[] = {0x00, 0xFF};

  // Test canary with empty CVT.
  for (int32_t i = 0; i < kNumTetherUpSources; ++i) {
    AioNode source = TetherUpSourceToAioNode((TetherUpSource)i);
    CvtClearMessage(source, kMessageTypeTetherUp);
  }
  for (int32_t i = 0; i < ARRAYSIZE(canaries); ++i) {
    RunTetherUpMergeCvtGet(canaries[i]);
  }

  // Test canary with one new message.
  for (int32_t i = 0; i < kNumTetherUpSources; ++i) {
    TetherUpMessage in;
    memset(&in, 0, sizeof(in));
    AioNode source = TetherUpSourceToAioNode((TetherUpSource)i);
    for (int32_t j = 0; j < ARRAYSIZE(canaries); ++j) {
      CvtClearMessage(source, kMessageTypeTetherUp);
      EXPECT_EQ(kCvtEventNone, CvtPutTetherUpMessage(source, &in, 1U, 1));
      RunTetherUpMergeCvtGet(canaries[j]);
    }
  }

  // Test canary with all new messages.
  for (int32_t i = 0; i < ARRAYSIZE(canaries); ++i) {
    for (int32_t j = 0; j < kNumTetherUpSources; ++j) {
      TetherUpMessage in;
      memset(&in, 0, sizeof(in));
      AioNode source = TetherUpSourceToAioNode((TetherUpSource)j);
      CvtClearMessage(source, kMessageTypeTetherUp);
      EXPECT_EQ(kCvtEventNone, CvtPutTetherUpMessage(source, &in, 1U, 1));
    }
    RunTetherUpMergeCvtGet(canaries[i]);
  }
}

static const TestConfig kTetherMessageTests[] = {
  TEST_CONFIG_INIT(TestTetherDownMergeCvtGetCanary, 10000),
  TEST_CONFIG_INIT(TestTetherUpMergeCvtGetCanary, 10000),
};

const TestSuite kTetherMessageTest =
    TEST_SUITE_INIT(kTetherMessageTests, TestSetup, TestTeardown);

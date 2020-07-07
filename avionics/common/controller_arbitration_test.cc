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

#include "avionics/common/controller_arbitration.h"

#include <gtest/gtest.h>

#include "avionics/common/cvt_avionics_messages.h"
#include "avionics/network/aio_labels.h"
#include "common/macros.h"

extern "C" {

int32_t ControllerArbitrationGetTimeoutUsec(void);

}

namespace {

int64_t g_last_timestamp = 0;
#ifdef USE_REAL_CONTROLLER_ARBITRATION
// TODO: Turn this back on when we're ready for it.
uint8_t g_seq = 0;
#endif  // USE_REAL_CONTROLLER_ARBITRATION

// ClearCvt doesn't clear the sequence or timestamp stored in the CVT entries,
// so we use the above variables to make sure each test's messages are accepted.
void ClearCvt() {
  for (int32_t i = 0; i < kNumControllers; ++i) {
    ControllerLabel label = static_cast<ControllerLabel>(i);
    CvtClearMessage(ControllerLabelToControllerAioNode(label),
                    kMessageTypeControllerCommand);
  }
}

bool NullValidator(const ControllerCommandMessage * /* msg */) {
  return true;
}

}  // namespace

TEST(ControllerArbitration, NoData) {
  ControllerArbitrationState state;
  ControllerLabel source;
  ControllerArbitrationInit(g_last_timestamp, &state);

  ClearCvt();
  EXPECT_EQ(nullptr, ControllerArbitrationGetCommand(g_last_timestamp,
                                                     NullValidator, &state,
                                                     &source));

  ControllerArbitrationUpdateFromCvt(&state);
  EXPECT_EQ(nullptr, ControllerArbitrationGetCommand(g_last_timestamp,
                                                     NullValidator, &state,
                                                     &source));
}

#ifdef USE_REAL_CONTROLLER_ARBITRATION
// TODO: Turn this back on when we're ready for it.

TEST(ControllerArbitration, Expected) {
  struct {
    bool update[kNumControllers];
    ControllerLabel leader;
  } expected[] = {{{true, true, true}, kControllerA},
                  {{true, true, false}, kControllerA},
                  {{true, false, true}, kControllerA},
                  {{true, false, false}, kControllerA},
                  {{false, true, true}, kControllerB},
                  {{false, true, false}, kControllerB},
                  {{false, false, true}, kControllerC}};

  for (int32_t i = 0; i < ARRAYSIZE(expected); ++i) {
    ControllerArbitrationState state;
    ControllerLabel source;

    ClearCvt();
    ControllerArbitrationInit(g_last_timestamp, &state);
    ++g_seq;
    ++g_last_timestamp;
    ControllerCommandMessage messages[kNumControllers];
    for (int32_t c = 0; c < kNumControllers; ++c) {
      memset(&messages[c], 0, sizeof(messages[c]));
      messages[c].light = static_cast<uint8_t>(c);
      if (expected[i].update[c]) {
        ControllerLabel label = static_cast<ControllerLabel>(c);
        CvtPutControllerCommandMessage(
            ControllerLabelToControllerAioNode(label),
            &messages[c], g_seq, g_last_timestamp);
      }
    }

    ControllerArbitrationUpdateFromCvt(&state);

    const ControllerCommandMessage *message
        = ControllerArbitrationGetCommand(g_last_timestamp, NullValidator,
                                          &state, &source);
    EXPECT_NE(nullptr, message);
    EXPECT_EQ(source, expected[i].leader);
    if (message != nullptr) {
      EXPECT_EQ(messages[expected[i].leader].light, message->light);
    }

    // Validate expiry.
    const ControllerCommandMessage *old_message
        = ControllerArbitrationGetCommand(g_last_timestamp, NullValidator,
                                          &state, &source);
    EXPECT_NE(nullptr, old_message);
    g_last_timestamp += ControllerArbitrationGetTimeoutUsec() - 1;
    message = ControllerArbitrationGetCommand(
        g_last_timestamp, NullValidator, &state, &source);
    EXPECT_EQ(message, old_message);
    ++g_last_timestamp;
    message = ControllerArbitrationGetCommand(
        g_last_timestamp, NullValidator, &state, &source);
    EXPECT_EQ(nullptr, message);
  }
}

bool TetherReleaseValidator(const ControllerCommandMessage *msg) {
  return msg->tether_release;
}

// In this test, we always update all messages, but only some of them are valid.
TEST(ControllerArbitration, Invalid) {
  struct {
    bool valid[kNumControllers];
    ControllerLabel leader;
  } expected[] = {{{true, true, true}, kControllerA},
                  {{true, true, false}, kControllerA},
                  {{true, false, true}, kControllerA},
                  {{true, false, false}, kControllerA},
                  {{false, true, true}, kControllerB},
                  {{false, true, false}, kControllerB},
                  {{false, false, true}, kControllerC}};

  for (int32_t i = 0; i < ARRAYSIZE(expected); ++i) {
    ControllerArbitrationState state;
    ControllerLabel source;

    ClearCvt();
    ControllerArbitrationInit(g_last_timestamp, &state);
    ++g_seq;
    ++g_last_timestamp;
    ControllerCommandMessage messages[kNumControllers];
    for (int32_t c = 0; c < kNumControllers; ++c) {
      memset(&messages[c], 0, sizeof(messages[c]));
      // Use the tether_release flag to indicate validity.
      messages[c].tether_release = expected[i].valid[c];
      messages[c].light = static_cast<uint8_t>(c);
      ControllerLabel label = static_cast<ControllerLabel>(c);
      CvtPutControllerCommandMessage(
          ControllerLabelToControllerAioNode(label),
          &messages[c], g_seq, g_last_timestamp);
    }

    ControllerArbitrationUpdateFromCvt(&state);

    const ControllerCommandMessage *message
        = ControllerArbitrationGetCommand(g_last_timestamp,
                                          TetherReleaseValidator, &state,
                                          &source);
    EXPECT_NE(nullptr, message);
    EXPECT_EQ(source, expected[i].leader);
    if (message != nullptr) {
      EXPECT_EQ(messages[expected[i].leader].light, message->light);
    }
  }
}
#endif  // USE_REAL_CONTROLLER_ARBITRATION

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

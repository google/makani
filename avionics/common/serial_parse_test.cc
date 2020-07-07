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

#include <stdint.h>
#include <string.h>

#include <gtest/gtest.h>

extern "C" {

#include "avionics/common/serial_parse.h"
#include "common/macros.h"

}  // extern "C"

namespace {

typedef struct {
  int32_t parsed;
  bool sync;
  bool called_after_complete;
} TestParserContext;

#define WORD_SIZE 5

bool ParseWord(const char word[WORD_SIZE], int32_t length, const uint8_t *data,
               TestParserContext *ctx, int32_t *parsed) {
  assert(length > 0);
  assert(data != NULL);
  assert(ctx != NULL);
  assert(parsed != NULL);

  ctx->called_after_complete = (length > 1 && (!ctx->sync || ctx->parsed));
  ctx->parsed = 0;
  if (0 < length && length <= WORD_SIZE) {
    ctx->sync = (data[length - 1] == word[length - 1]);
  } else {
    ctx->sync = false;
  }
  if (ctx->sync && length == WORD_SIZE) {
    ctx->parsed = length;
  }
  *parsed = ctx->parsed;
  return ctx->sync;
}

bool ParseABCDE(uint32_t sync_flags, int32_t length, const uint8_t *data,
                void *context, int32_t *parsed) {
  (void)sync_flags;
  TestParserContext *ctx = reinterpret_cast<TestParserContext *>(context);
  return ParseWord("ABCDE", length, data, ctx, parsed);
}

bool ParseABCED(uint32_t sync_flags, int32_t length, const uint8_t *data,
                void *context, int32_t *parsed) {
  (void)sync_flags;
  TestParserContext *ctx = reinterpret_cast<TestParserContext *>(context);
  return ParseWord("ABCED", length, data, ctx, parsed);
}

bool ParseBCDEA(uint32_t sync_flags, int32_t length, const uint8_t *data,
                void *context, int32_t *parsed) {
  (void)sync_flags;
  TestParserContext *ctx = reinterpret_cast<TestParserContext *>(context);
  return ParseWord("BCDEA", length, data, ctx, parsed);
}

bool ParseForever(uint32_t sync_flags, int32_t length, const uint8_t *data,
                  void *context, int32_t *parsed) {
  assert(context != NULL);
  assert(parsed != NULL);

  (void)sync_flags;
  (void)length;
  (void)data;

  TestParserContext *ctx = reinterpret_cast<TestParserContext *>(context);
  ctx->called_after_complete = (length > 1 && (!ctx->sync || ctx->parsed));
  ctx->parsed = 0;
  ctx->sync = true;

  *parsed = ctx->parsed;
  return ctx->sync;
}

}  // namespace

TEST(SerialParse, Normal) {
  TestParserContext abcde_context;
  TestParserContext abced_context;
  TestParserContext bcdea_context;
  TestParserContext forever_context;
  const SerialParser parsers[] = {
    {ParseABCDE, &abcde_context},
    {ParseABCED, &abced_context},
    {ParseBCDEA, &bcdea_context},
    {ParseForever, &forever_context}
  };

  int32_t protocol, length;
  const uint8_t *data;
  SerialReceiveBuffer buf;
  SerialParseInit(&buf);

  // Init parser contexts.
  memset(&abcde_context, 0, sizeof(abcde_context));
  memset(&abced_context, 0, sizeof(abced_context));
  memset(&bcdea_context, 0, sizeof(bcdea_context));
  memset(&forever_context, 0, sizeof(forever_context));

  // Send ABCDE.
  const uint8_t abcde_data[] = "ABCDE";
  // Send first byte. BCDEA should drop out.
  EXPECT_EQ(SerialReadData(1, &abcde_data[0], &buf), 1);
  data = SerialParse(ARRAYSIZE(parsers), parsers, &buf, &protocol, &length);
  EXPECT_EQ(data, nullptr);
  EXPECT_TRUE(abcde_context.sync);
  EXPECT_TRUE(abced_context.sync);
  EXPECT_FALSE(bcdea_context.sync);
  EXPECT_TRUE(forever_context.sync);
  EXPECT_FALSE(abcde_context.called_after_complete);
  EXPECT_FALSE(abced_context.called_after_complete);
  EXPECT_FALSE(bcdea_context.called_after_complete);
  EXPECT_FALSE(forever_context.called_after_complete);
  // Send bytes 2 and 3. No additional protocols should drop out.
  EXPECT_EQ(SerialReadData(2, &abcde_data[1], &buf), 2);
  data = SerialParse(ARRAYSIZE(parsers), parsers, &buf, &protocol, &length);
  EXPECT_EQ(data, nullptr);
  EXPECT_TRUE(abcde_context.sync);
  EXPECT_TRUE(abced_context.sync);
  EXPECT_FALSE(bcdea_context.sync);
  EXPECT_TRUE(forever_context.sync);
  EXPECT_FALSE(abcde_context.called_after_complete);
  EXPECT_FALSE(abced_context.called_after_complete);
  EXPECT_FALSE(bcdea_context.called_after_complete);
  EXPECT_FALSE(forever_context.called_after_complete);
  // Send byte 4. ABCED should drop out.
  EXPECT_EQ(SerialReadData(1, &abcde_data[3], &buf), 1);
  data = SerialParse(ARRAYSIZE(parsers), parsers, &buf, &protocol, &length);
  EXPECT_EQ(data, nullptr);
  EXPECT_TRUE(abcde_context.sync);
  EXPECT_FALSE(abced_context.sync);
  EXPECT_FALSE(bcdea_context.sync);
  EXPECT_TRUE(forever_context.sync);
  EXPECT_FALSE(abcde_context.called_after_complete);
  EXPECT_FALSE(abced_context.called_after_complete);
  EXPECT_FALSE(bcdea_context.called_after_complete);
  EXPECT_FALSE(forever_context.called_after_complete);
  // Send byte 5. ABCDE should succeed.
  EXPECT_EQ(SerialReadData(1, &abcde_data[4], &buf), 1);
  data = SerialParse(ARRAYSIZE(parsers), parsers, &buf, &protocol, &length);
  ASSERT_NE(data, nullptr);
  EXPECT_EQ(length, 5);
  EXPECT_EQ(memcmp(abcde_data, data, length), 0);
  EXPECT_TRUE(abcde_context.sync);
  EXPECT_FALSE(abced_context.sync);
  EXPECT_FALSE(bcdea_context.sync);
  EXPECT_TRUE(forever_context.sync);
  EXPECT_FALSE(abcde_context.called_after_complete);
  EXPECT_FALSE(abced_context.called_after_complete);
  EXPECT_FALSE(bcdea_context.called_after_complete);
  EXPECT_FALSE(forever_context.called_after_complete);
  EXPECT_EQ(protocol, 0);

  // Send garbage (ignore forever parser). All parsers drop out.
  const uint8_t garbage_data[] = "ABCX";
  EXPECT_EQ(SerialReadData(4, garbage_data, &buf), 4);
  data = SerialParse(ARRAYSIZE(parsers) - 1, parsers, &buf, &protocol, &length);
  EXPECT_EQ(data, nullptr);
  EXPECT_FALSE(abcde_context.sync);
  EXPECT_FALSE(abced_context.sync);
  EXPECT_FALSE(bcdea_context.sync);
  EXPECT_FALSE(abcde_context.called_after_complete);
  EXPECT_FALSE(abced_context.called_after_complete);
  EXPECT_FALSE(bcdea_context.called_after_complete);

  // Send ABCED. ABCED should succeed.
  const uint8_t abced_data[] = "ABCED";
  EXPECT_EQ(SerialReadData(5, abced_data, &buf), 5);
  data = SerialParse(ARRAYSIZE(parsers), parsers, &buf, &protocol, &length);
  ASSERT_NE(data, nullptr);
  EXPECT_EQ(length, 5);
  EXPECT_EQ(memcmp(abced_data, data, length), 0);
  EXPECT_FALSE(abcde_context.sync);
  EXPECT_TRUE(abced_context.sync);
  EXPECT_FALSE(bcdea_context.sync);
  EXPECT_FALSE(abcde_context.called_after_complete);
  EXPECT_FALSE(abced_context.called_after_complete);
  EXPECT_FALSE(bcdea_context.called_after_complete);
  EXPECT_FALSE(forever_context.called_after_complete);
  EXPECT_EQ(protocol, 1);

  // Send BCDEA. BCDEA should succeed.
  const uint8_t bcdea_data[] = "BCDEA";
  EXPECT_EQ(SerialReadData(5, bcdea_data, &buf), 5);
  data = SerialParse(ARRAYSIZE(parsers), parsers, &buf, &protocol, &length);
  ASSERT_NE(data, nullptr);
  EXPECT_EQ(length, 5);
  EXPECT_EQ(memcmp(bcdea_data, data, length), 0);
  EXPECT_FALSE(abcde_context.sync);
  EXPECT_FALSE(abced_context.sync);
  EXPECT_TRUE(bcdea_context.sync);
  EXPECT_FALSE(abcde_context.called_after_complete);
  EXPECT_FALSE(abced_context.called_after_complete);
  EXPECT_FALSE(bcdea_context.called_after_complete);
  EXPECT_FALSE(forever_context.called_after_complete);
  EXPECT_EQ(protocol, 2);
}

TEST(SerialParse, Overflow) {
  TestParserContext abcde_context;
  TestParserContext abced_context;
  TestParserContext bcdea_context;
  TestParserContext forever_context;
  const SerialParser parsers[] = {
    {ParseABCDE, &abcde_context},
    {ParseABCED, &abced_context},
    {ParseBCDEA, &bcdea_context},
    {ParseForever, &forever_context}
  };

  int32_t protocol, length;
  const uint8_t *data;
  SerialReceiveBuffer buf;
  SerialParseInit(&buf);

  // Init parser contexts.
  memset(&abcde_context, 0, sizeof(abcde_context));
  memset(&abced_context, 0, sizeof(abced_context));
  memset(&bcdea_context, 0, sizeof(bcdea_context));
  memset(&forever_context, 0, sizeof(forever_context));

  // Send one garbage byte to eliminate parsers ABCDE, ABCED, and BCDEA, but
  // keep the forever parser.
  const uint8_t zero = 0;
  EXPECT_EQ(SerialReadData(1, &zero, &buf), 1);
  data = SerialParse(ARRAYSIZE(parsers), parsers, &buf, &protocol, &length);
  EXPECT_EQ(data, nullptr);
  EXPECT_FALSE(abcde_context.sync);
  EXPECT_FALSE(abced_context.sync);
  EXPECT_FALSE(bcdea_context.sync);
  EXPECT_TRUE(forever_context.sync);
  EXPECT_FALSE(abcde_context.called_after_complete);
  EXPECT_FALSE(abced_context.called_after_complete);
  EXPECT_FALSE(bcdea_context.called_after_complete);
  EXPECT_FALSE(forever_context.called_after_complete);

  // Send valid message. The forever parser prevents this message from parsing
  // until overflow.
  const uint8_t abced_data[] = "ABCED";
  EXPECT_EQ(SerialReadData(5, abced_data, &buf), 5);
  data = SerialParse(ARRAYSIZE(parsers), parsers, &buf, &protocol, &length);
  EXPECT_EQ(data, nullptr);
  EXPECT_FALSE(abcde_context.sync);
  EXPECT_FALSE(abced_context.sync);
  EXPECT_FALSE(bcdea_context.sync);
  EXPECT_TRUE(forever_context.sync);
  EXPECT_FALSE(abcde_context.called_after_complete);
  EXPECT_FALSE(abced_context.called_after_complete);
  EXPECT_FALSE(bcdea_context.called_after_complete);
  EXPECT_FALSE(forever_context.called_after_complete);

  // Send data until one shy of overflow.
  int32_t avail = SerialReadGetAvailableBytes(&buf);
  ASSERT_GE(avail, 1);
  for (int32_t i = 1; i < avail; ++i) {
    EXPECT_EQ(SerialReadData(1, &zero, &buf), 1);
    data = SerialParse(ARRAYSIZE(parsers), parsers, &buf, &protocol, &length);
    EXPECT_EQ(data, nullptr);
    ASSERT_EQ(SerialReadGetAvailableBytes(&buf), avail - i);
  }
  EXPECT_FALSE(abcde_context.sync);
  EXPECT_FALSE(abced_context.sync);
  EXPECT_FALSE(bcdea_context.sync);
  EXPECT_TRUE(forever_context.sync);
  EXPECT_FALSE(abcde_context.called_after_complete);
  EXPECT_FALSE(abced_context.called_after_complete);
  EXPECT_FALSE(bcdea_context.called_after_complete);
  EXPECT_FALSE(forever_context.called_after_complete);

  // Send one more to cause overflow. Overflow drops first byte and reparses
  // remaining sequence again, starting at the second byte. This process
  // should re-enable all parsers and parse ABCED.
  EXPECT_EQ(SerialReadData(1, &zero, &buf), 1);
  data = SerialParse(ARRAYSIZE(parsers), parsers, &buf, &protocol, &length);
  ASSERT_NE(data, nullptr);
  EXPECT_EQ(length, 5);
  EXPECT_EQ(memcmp(abced_data, data, length), 0);
  EXPECT_FALSE(abcde_context.sync);
  EXPECT_TRUE(abced_context.sync);
  EXPECT_FALSE(bcdea_context.sync);
  EXPECT_TRUE(forever_context.sync);
  EXPECT_FALSE(abcde_context.called_after_complete);
  EXPECT_FALSE(abced_context.called_after_complete);
  EXPECT_FALSE(bcdea_context.called_after_complete);
  EXPECT_FALSE(forever_context.called_after_complete);
  EXPECT_EQ(memcmp(abced_data, data, ARRAYSIZE(abced_data)), 0);
  EXPECT_EQ(protocol, 1);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

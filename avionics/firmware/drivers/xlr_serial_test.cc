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

#include <gtest/gtest.h>

#include "avionics/firmware/drivers/xlr_api.h"
#include "avionics/firmware/drivers/xlr_serial.h"
#include "common/macros.h"

// TODO: Add proper unit tests. These tests only ensure basic
// functionality.

namespace {

void InitTest(SerialReceiveBuffer *buffer, XlrReceive *rx) {
  memset(rx, 0, sizeof(*rx));
  SerialParseInit(buffer);
}

void RunTest(int32_t length, const uint8_t *data, SerialReceiveBuffer *buffer,
             XlrReceive *rx) {
  for (int32_t i = 0; i < length; ++i) {
    SerialReadData(1, &data[i], buffer);
    bool expected = (i + 1 == length);
    bool new_message = XlrParse(buffer, rx);
    EXPECT_EQ(expected, new_message);
  }
}

}  // namespace

TEST(XlrParse, NonEscapedData) {
  // Example message from documentation.
  const uint8_t kData[] = {0x7E, 0x00, 0x02, 0x23, 0x11, 0xCB};
  XlrReceive rx;
  SerialReceiveBuffer buffer;
  InitTest(&buffer, &rx);
  RunTest(ARRAYSIZE(kData), kData, &buffer, &rx);
  EXPECT_EQ(kXlrProtoApiNonEscaped, rx.proto);
}

TEST(XlrParse, EscapedData) {
  // Example message from documentation.
  const uint8_t kData[] = {0x7E, 0x00, 0x02, 0x23, 0x7D, 0x31, 0xCB};
  XlrReceive rx;
  SerialReceiveBuffer buffer;
  InitTest(&buffer, &rx);
  RunTest(ARRAYSIZE(kData), kData, &buffer, &rx);
  EXPECT_EQ(kXlrProtoApiEscaped, rx.proto);
}

TEST(XlrParse, AsciiData) {
  const uint8_t kData[] = {'1', '\r'};
  XlrReceive rx;
  SerialReceiveBuffer buffer;
  InitTest(&buffer, &rx);
  RunTest(ARRAYSIZE(kData), kData, &buffer, &rx);
  EXPECT_EQ(kXlrProtoAscii, rx.proto);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

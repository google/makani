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

#include "avionics/common/novatel_serial.h"
#include "avionics/common/novatel_types.h"
#include "avionics/common/serial_parse.h"
#include "common/macros.h"

}  // extern "C"

namespace {

static const uint8_t kBestXyzBinaryData[] = {
  0xAA, 0x44, 0x12, 0x1C, 0xF1, 0x00, 0x02, 0x40, 0x70, 0x00, 0x00, 0x00,
  0xA6, 0x14, 0x00, 0x00, 0xD4, 0x32, 0x08, 0x00, 0x00, 0x00, 0x4C, 0x00,
  0x21, 0xD8, 0x9E, 0x18, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x59, 0x40, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0xFD, 0x4D, 0xE6
};

static const uint8_t kBestXyzAsciiData[] = {
  "#BESTXYZA,COM1,0,55.0,FINESTEERING,1419,340033.000,00000040,d821,2724;"
  "SOL_COMPUTED,NARROW_INT,-1634531.5683,-3664618.0326,4942496.3270,"
  "0.0099,0.0219,0.0115,SOL_COMPUTED,NARROW_INT,0.0011,-0.0049,-0.0001,"
  "0.0199,0.0439,0.0230,\"AAAA\",0.250,1.000,0.000,12,11,11,11,0,01,0,33"
  "*e9eafeca\r"
};

static const uint8_t kLogCommand[] = {
  0xAA, 0x44, 0x12, 0x1C, 0x01, 0x00, 0x02, 0x40, 0x20, 0x00, 0x00, 0x00,
  0x1D, 0x1D, 0x00, 0x00, 0x29, 0x16, 0x00, 0x00, 0x00, 0x00, 0x4C, 0x00,
  0x55, 0x52, 0x5A, 0x80, 0x20, 0x00, 0x00, 0x00, 0x2A, 0x00, 0x00, 0x00,
  0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF0, 0x3F,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x23, 0x04, 0xB3, 0xF1
};

void InitTest(SerialReceiveBuffer *buffer, NovAtelReceive *rx) {
  memset(rx, 0, sizeof(*rx));
  SerialParseInit(buffer);
}

void RunTest(int32_t length, const uint8_t *data, SerialReceiveBuffer *buffer,
             NovAtelReceive *rx) {
  for (int32_t i = 0; i < length; ++i) {
    SerialReadData(1, &data[i], buffer);
    bool expected = (i + 1 == length);
    bool new_message = NovAtelParse(buffer, rx);
    EXPECT_EQ(expected, new_message);
  }
}

}  // namespace

TEST(NovAtelParseMessage, Command) {
  NovAtelReceive rx;
  SerialReceiveBuffer buffer;
  InitTest(&buffer, &rx);
  RunTest(ARRAYSIZE(kLogCommand), kLogCommand, &buffer, &rx);
  EXPECT_EQ(rx.binary.header.header_length, 28);
  EXPECT_EQ(rx.binary.header.message_length, 32);
  EXPECT_EQ(rx.binary.header.message_id, kNovAtelMessageIdLog);
  EXPECT_EQ(rx.binary.header.format, kNovAtelFormatBinary);
  EXPECT_EQ(rx.binary.header.port, kNovAtelPortCom2);
  EXPECT_EQ(rx.binary.header.response, kNovAtelResponseNone);
  EXPECT_EQ(rx.binary.header.timestamp.time_status, 29);
  EXPECT_EQ(rx.binary.header.timestamp.week, 0);
  EXPECT_EQ(rx.binary.header.timestamp.tow, 5673);
  EXPECT_EQ(rx.binary.header.sequence, 0);
  EXPECT_EQ(rx.binary.header.idle_time, 29);
  EXPECT_EQ(rx.binary.header.receiver_status, 0x4C0000);
  EXPECT_EQ(rx.binary.header.receiver_sw_version, 0x805A);
}

TEST(NovAtelReceive, Binary) {
  NovAtelReceive rx;
  SerialReceiveBuffer buffer;
  InitTest(&buffer, &rx);
  RunTest(ARRAYSIZE(kBestXyzBinaryData), kBestXyzBinaryData, &buffer, &rx);
  EXPECT_EQ(kNovAtelProtoBinary, rx.proto);
  EXPECT_EQ(kNovAtelMessageIdBestXyz, rx.binary.header.message_id);
  EXPECT_EQ(kNovAtelFormatBinary, rx.binary.header.format);
  EXPECT_EQ(rx.binary.header.header_length, 28);
  EXPECT_EQ(rx.binary.header.message_length, 112);
  EXPECT_EQ(rx.binary.header.message_id, kNovAtelMessageIdBestXyz);
  EXPECT_EQ(rx.binary.header.format, kNovAtelFormatBinary);
  EXPECT_EQ(rx.binary.header.port, kNovAtelPortCom2);
  EXPECT_EQ(rx.binary.header.response, kNovAtelResponseNone);
  EXPECT_EQ(rx.binary.header.timestamp.time_status, 20);
  EXPECT_EQ(rx.binary.header.timestamp.week, 0);
  EXPECT_EQ(rx.binary.header.timestamp.tow, 537300);
  EXPECT_EQ(rx.binary.header.sequence, 0);
  EXPECT_EQ(rx.binary.header.idle_time, 166);
  EXPECT_EQ(rx.binary.header.receiver_status, 0x4C0000);
  EXPECT_EQ(rx.binary.header.receiver_sw_version, 0x189E);
}

TEST(NovAtelReceive, Ascii) {
  NovAtelReceive rx;
  SerialReceiveBuffer buffer;
  InitTest(&buffer, &rx);
  RunTest(ARRAYSIZE(kBestXyzAsciiData) - 1, kBestXyzAsciiData, &buffer, &rx);
  EXPECT_EQ(kNovAtelProtoAscii, rx.proto);
  EXPECT_EQ(38, rx.ascii.fields);
  EXPECT_EQ(10, rx.ascii.header_fields);

  // Verify first field.
  int32_t field_length, field_start;
  NovAtelAsciiGetField(&rx, 0, &field_length, &field_start);
  EXPECT_EQ(field_start, 0);
  EXPECT_EQ(field_length, 9);
  EXPECT_EQ(memcmp("#BESTXYZA", &rx.data[field_start], field_length), 0);

  // Verify second field.
  NovAtelAsciiGetField(&rx, 1, &field_length, &field_start);
  EXPECT_EQ(field_start, 10);
  EXPECT_EQ(field_length, 4);
  EXPECT_EQ(memcmp("COM1", &rx.data[field_start], field_length), 0);

  // Verify last field.
  NovAtelAsciiGetField(&rx, rx.ascii.fields - 1, &field_length, &field_start);
  EXPECT_EQ(field_start, 268);
  EXPECT_EQ(field_length, 2);
  EXPECT_EQ(memcmp("33", &rx.data[field_start], field_length), 0);

  // Verify invalid field.
  NovAtelAsciiGetField(&rx, rx.ascii.fields, &field_length, &field_start);
  EXPECT_EQ(field_start, 0);
  EXPECT_EQ(field_length, 0);
}

TEST(NovAtelReceive, MixedBinaryAndAscii) {
  NovAtelReceive rx;
  SerialReceiveBuffer buffer;
  InitTest(&buffer, &rx);

  // Ascii.
  RunTest(ARRAYSIZE(kBestXyzAsciiData) - 1, kBestXyzAsciiData, &buffer, &rx);

  // Ascii -> binary.
  RunTest(ARRAYSIZE(kBestXyzBinaryData), kBestXyzBinaryData, &buffer, &rx);

  // Binary -> ascii.
  RunTest(ARRAYSIZE(kBestXyzAsciiData) - 1, kBestXyzAsciiData, &buffer, &rx);

  // Ascii -> ascii.
  RunTest(ARRAYSIZE(kBestXyzAsciiData) - 1, kBestXyzAsciiData, &buffer, &rx);

  // Ascii -> binary.
  RunTest(ARRAYSIZE(kBestXyzBinaryData), kBestXyzBinaryData, &buffer, &rx);

  // Binary -> binary.
  RunTest(ARRAYSIZE(kBestXyzBinaryData), kBestXyzBinaryData, &buffer, &rx);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

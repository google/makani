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

#include <float.h>
#include <stdint.h>
#include <string.h>

#include <gtest/gtest.h>

extern "C" {

#include "avionics/common/gill_ascii.h"
#include "avionics/common/gill_binary.h"
#include "avionics/common/gill_serial.h"
#include "common/macros.h"

}  // extern "C"

namespace {

static const uint8_t kAsciiData[] = {
  "\002Q,170,000.08,1019.5,035.0,+024.7,+008.3,+04.9,00,,LAST\00373\n"
};

static const uint8_t kBinaryData[] = {
  0xB4, 0xB4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x00, 0x81, 0x87,
  0x50, 0x2F, 0x44, 0x2F, 0x50, 0x2F, 0x50, 0x2F, 0xFF, 0x7F, 0x97
};

static const uint8_t kLineData[] = {
  "Hello world!\n"
};

static const uint8_t kNmeaData[] = {
  "$WIXDR,C,+023.0,C,TEMP,P,1.0199,B,PRESS,H,039.6,P,RH*33\n"
};

void InitTest(SerialReceiveBuffer *buffer, GillReceive *rx) {
  memset(rx, 0, sizeof(*rx));
  SerialParseInit(buffer);
}

void RunTest(int32_t length, const uint8_t *data, SerialReceiveBuffer *buffer,
             GillReceive *rx) {
  for (int32_t i = 0; i < length; ++i) {
    SerialReadData(1, &data[i], buffer);
    bool expected = (i + 1 == length);
    bool new_message = GillParse(buffer, rx);
    EXPECT_EQ(expected, new_message);
  }
}

}  // namespace

TEST(GillParseMessage, AsciiData) {
  GillReceive rx;
  SerialReceiveBuffer buffer;
  InitTest(&buffer, &rx);
  RunTest(ARRAYSIZE(kAsciiData) - 1, kAsciiData, &buffer, &rx);
  EXPECT_EQ(rx.proto, kGillProtoAscii);
  EXPECT_EQ(rx.ascii.fields, 11);
  EXPECT_EQ(rx.ascii.field_delim[0], 2);
  EXPECT_EQ(rx.ascii.field_delim[1], 6);
  EXPECT_EQ(rx.ascii.field_delim[2], 13);
  EXPECT_EQ(rx.ascii.field_delim[3], 20);
  EXPECT_EQ(rx.ascii.field_delim[4], 26);
  EXPECT_EQ(rx.ascii.field_delim[5], 33);
  EXPECT_EQ(rx.ascii.field_delim[6], 40);
  EXPECT_EQ(rx.ascii.field_delim[7], 46);
  EXPECT_EQ(rx.ascii.field_delim[8], 49);
  EXPECT_EQ(rx.ascii.field_delim[9], 50);
  EXPECT_EQ(rx.ascii.field_delim[10], 55);
}

TEST(GillParseMessage, BinaryData) {
  GillReceive rx;
  SerialReceiveBuffer buffer;
  InitTest(&buffer, &rx);
  RunTest(ARRAYSIZE(kBinaryData), kBinaryData, &buffer, &rx);
  EXPECT_EQ(rx.binary.id, kGillBinaryIdWindmasterMode10);
  EXPECT_EQ(rx.binary.length, 23);
  EXPECT_EQ(rx.proto, kGillProtoBinary);
}

TEST(GillParseMessage, LineData) {
  GillReceive rx;
  SerialReceiveBuffer buffer;
  InitTest(&buffer, &rx);
  RunTest(ARRAYSIZE(kLineData) - 1, kLineData, &buffer, &rx);
  EXPECT_EQ(rx.proto, kGillProtoLine);
}

TEST(GillParseMessage, NmeaData) {
  GillReceive rx;
  SerialReceiveBuffer buffer;
  InitTest(&buffer, &rx);
  RunTest(ARRAYSIZE(kNmeaData) - 1, kNmeaData, &buffer, &rx);
  EXPECT_EQ(rx.proto, kGillProtoNmea);
}

TEST(GillAsciiGetField, Normal) {
  GillReceive rx;
  SerialReceiveBuffer buffer;
  InitTest(&buffer, &rx);
  RunTest(ARRAYSIZE(kAsciiData) - 1, kAsciiData, &buffer, &rx);
  EXPECT_EQ(rx.proto, kGillProtoAscii);

  // Get first field, Q.
  int32_t field_length, field_start;
  GillAsciiGetField(&rx.ascii, 0, &field_length, &field_start);
  EXPECT_EQ(field_length, 1);
  EXPECT_EQ(field_start, 1);
  EXPECT_EQ(memcmp("Q", &rx.data[field_start], field_length), 0);

  // Get second field, 170.
  GillAsciiGetField(&rx.ascii, 1, &field_length, &field_start);
  EXPECT_EQ(field_length, 3);
  EXPECT_EQ(field_start, 3);
  EXPECT_EQ(memcmp("170", &rx.data[field_start], field_length), 0);

  // Get last field, ''.
  GillAsciiGetField(&rx.ascii, rx.ascii.fields - 1, &field_length,
                    &field_start);
  EXPECT_EQ(field_length, 4);
  EXPECT_EQ(field_start, 51);
  EXPECT_EQ(memcmp("LAST", &rx.data[field_start], field_length), 0);

  // Get an invalid field.
  GillAsciiGetField(&rx.ascii, rx.ascii.fields, &field_length, &field_start);
  EXPECT_EQ(field_length, 0);
  EXPECT_EQ(field_start, 0);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

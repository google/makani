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

#include "avionics/common/septentrio_serial.h"
#include "avionics/common/serial_parse.h"
#include "common/macros.h"

}  // extern "C"

namespace {

static const uint8_t kSbfBlock[] = {
  0x24, 0x40, 0x44, 0xD6, 0x13, 0x23, 0x08, 0x00
};

static const uint8_t kSnmpPdu[] = {
  0x24, 0x26, 0x01, 0x52, 0x04, 0x00, 0x00, 0x00, 0x52, 0x00, 0x00, 0x00
};

static const uint8_t kAsciiReply[] = {
  "$R: setCOMSettings, all, baud115200\n"
  "  COMSettings, COM1, baud115200, bits8, No, bit1, none\n"
  "  COMSettings, COM2, baud115200, bits8, No, bit1, none\n"
  "  COMSettings, COM3, baud115200, bits8, No, bit1, none\n"
  "COM1>"
};

static const uint8_t kAsciiReplyList[] = {
  "$R; lai, Overview\n"
  "<?xml version=\"1.0\" encoding=\"ISO-8859-1\" ?>\n"
  "<AntennaInfo version=\"0.1\">\n"
  "    <Antenna ID=\"AERAT1675_29    NONE\"/>\n"
  "    <Antenna ID=\"AERAT2775_150   NONE\"/>\n"
  "    <Antenna ID=\"AERAT2775_159       \"/>\n"
  "    <Antenna ID=\"AERAT2775_159   SPKE\"/>\n"
  "    <Antenna ID=\"AERAT2775_160       \"/>\n"
  "    <Antenna ID=\"TRM_R8_GNSS         \"/>\n"
  "</AntennaInfo>\n"
  "COM1>"
};

static const uint8_t kAsciiReplyListBlock[] = {
  "$R; lcf, Current\n"
  "$-- BLOCK 1 / 1\n"
  "  setMarkerParameters, \"TestMarker\"\n"
  "COM1>"
};

static const uint8_t kAsciiReplyBlock[] = {
  "$R; lif, Permissions\n"
  "---->"
};

static const uint8_t kAsciiBlockContinue[] = {
  "$-- BLOCK 1 / 0\n"
  "  ... block text ...\n"
  "---->"
};

static const uint8_t kAsciiBlockEnd[] = {
  "$-- BLOCK 2 / 0\n"
  "  ... block text ...\n"
  "COM1>"
};

static const uint8_t kAsciiTransmission[] = {
  "$TE ResetReceiver Hard\n"
  "STOP>"
};

void InitTest(SerialReceiveBuffer *buffer, SeptentrioReceive *rx) {
  memset(rx, 0, sizeof(*rx));
  SerialParseInit(buffer);
}

void RunTest(int32_t length, const uint8_t *data, SerialReceiveBuffer *buffer,
             SeptentrioReceive *rx) {
  for (int32_t i = 0; i < length; ++i) {
    SerialReadData(1, &data[i], buffer);
    bool expected = (i + 1 == length);
    bool new_message = SeptentrioParse(buffer, rx);
    EXPECT_EQ(expected, new_message);
  }
}

}  // namespace

TEST(SeptentrioParseBuffer, NormalSbfBlock) {
  SeptentrioReceive rx;
  SerialReceiveBuffer buffer;
  InitTest(&buffer, &rx);
  RunTest(ARRAYSIZE(kSbfBlock), kSbfBlock, &buffer, &rx);
}

TEST(SeptentrioParseBuffer, NormalSnmpPdu) {
  SeptentrioReceive rx;
  SerialReceiveBuffer buffer;
  InitTest(&buffer, &rx);
  RunTest(ARRAYSIZE(kSnmpPdu), kSnmpPdu, &buffer, &rx);
}

TEST(SeptentrioParseBuffer, NormalAsciiReply) {
  SeptentrioReceive rx;
  SerialReceiveBuffer buffer;
  InitTest(&buffer, &rx);
  RunTest(ARRAYSIZE(kAsciiReply) - 1, kAsciiReply, &buffer, &rx);
}

TEST(SeptentrioParseBuffer, NormalAsciiReplyList) {
  SeptentrioReceive rx;
  SerialReceiveBuffer buffer;
  InitTest(&buffer, &rx);
  RunTest(ARRAYSIZE(kAsciiReplyList) - 1, kAsciiReplyList, &buffer, &rx);
}

TEST(SeptentrioParseBuffer, NormalAsciiReplyListBlock) {
  SeptentrioReceive rx;
  SerialReceiveBuffer buffer;
  InitTest(&buffer, &rx);
  RunTest(ARRAYSIZE(kAsciiReplyListBlock) - 1, kAsciiReplyListBlock, &buffer,
          &rx);
}

TEST(SeptentrioParseBuffer, NormalAsciiReplyBlock) {
  SeptentrioReceive rx;
  SerialReceiveBuffer buffer;
  InitTest(&buffer, &rx);
  RunTest(ARRAYSIZE(kAsciiReplyBlock) - 1, kAsciiReplyBlock, &buffer, &rx);
}

TEST(SeptentrioParseBuffer, NormalAsciiBlockContinue) {
  SeptentrioReceive rx;
  SerialReceiveBuffer buffer;
  InitTest(&buffer, &rx);
  RunTest(ARRAYSIZE(kAsciiBlockContinue) - 1, kAsciiBlockContinue, &buffer,
          &rx);
}

TEST(SeptentrioParseBuffer, NormalAsciiBlockEnd) {
  SeptentrioReceive rx;
  SerialReceiveBuffer buffer;
  InitTest(&buffer, &rx);
  RunTest(ARRAYSIZE(kAsciiBlockEnd) - 1, kAsciiBlockEnd, &buffer, &rx);
}

TEST(SeptentrioParseBuffer, NormalAsciiTransmission) {
  SeptentrioReceive rx;
  SerialReceiveBuffer buffer;
  InitTest(&buffer, &rx);
  RunTest(ARRAYSIZE(kAsciiTransmission) - 1, kAsciiTransmission, &buffer, &rx);
}

TEST(SeptentrioParseBuffer, NormalMixed) {
  SeptentrioReceive rx;
  SerialReceiveBuffer buffer;
  InitTest(&buffer, &rx);

  // ASCII reply.
  RunTest(ARRAYSIZE(kAsciiReply) - 1, kAsciiReply, &buffer, &rx);

  // SNMP.
  RunTest(ARRAYSIZE(kSnmpPdu), kSnmpPdu, &buffer, &rx);

  // ASCII reply block.
  RunTest(ARRAYSIZE(kAsciiReplyBlock) - 1, kAsciiReplyBlock, &buffer, &rx);

  // SBF.
  RunTest(ARRAYSIZE(kSbfBlock), kSbfBlock, &buffer, &rx);

  // ASCII block continue.
  RunTest(ARRAYSIZE(kAsciiBlockContinue) - 1, kAsciiBlockContinue, &buffer,
          &rx);

  // SNMP.
  RunTest(ARRAYSIZE(kSnmpPdu), kSnmpPdu, &buffer, &rx);

  // ASCII block end.
  RunTest(ARRAYSIZE(kAsciiBlockEnd) - 1, kAsciiBlockEnd, &buffer, &rx);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

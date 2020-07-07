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

#include "avionics/firmware/drivers/xlr_api.h"
#include "common/macros.h"

TEST(XlrApiWriteAtCommand, Normal) {
  uint8_t out[8];

  EXPECT_EQ(8, XlrApiWriteAtCommand(kXlrApiFrameTypeCommand, 0x52,
                                    kXlrAtCommandNetworkId, 0, NULL,
                                    ARRAYSIZE(out), out));
  EXPECT_EQ(XLR_FRAME_DELIM, out[0]);
  EXPECT_EQ(0x00, out[1]);  // Length, most significant byte.
  EXPECT_EQ(0x04, out[2]);  // Length, least significant byte.
  EXPECT_EQ(0x08, out[3]);  // Frame type.
  EXPECT_EQ(0x52, out[4]);  // Frame id.
  EXPECT_EQ(0x49, out[5]);  // ASCII 'I'.
  EXPECT_EQ(0x44, out[6]);  // ASCII 'D'.
  EXPECT_EQ(0x18, out[7]);  // Checksum.
}

TEST(XlrApiWriteTxRequest, Normal) {
  uint8_t data[4] = {0x32, 0x11, 0x11, 0xAB};
  uint8_t out[22];

  EXPECT_EQ(22, XlrApiWriteTxRequest(0x52, 0x0123456789ABCDEFUL, 2U, 0x0,
                                     ARRAYSIZE(data), data, ARRAYSIZE(out),
                                     out));
  EXPECT_EQ(XLR_FRAME_DELIM, out[0]);
  EXPECT_EQ(0x00, out[1]);  // Length, most significant byte.
  EXPECT_EQ(0x12, out[2]);  // Length, least significant byte.
  EXPECT_EQ(0x10, out[3]);  // Frame type.
  EXPECT_EQ(0x52, out[4]);  // Frame id.
  EXPECT_EQ(0x01, out[5]);  // Destination address, byte 0.
  EXPECT_EQ(0x23, out[6]);  // Destination address, byte 1.
  EXPECT_EQ(0x45, out[7]);  // Destination address, byte 2.
  EXPECT_EQ(0x67, out[8]);  // Destination address, byte 3.
  EXPECT_EQ(0x89, out[9]);  // Destination address, byte 4.
  EXPECT_EQ(0xAB, out[10]);  // Destination address, byte 5.
  EXPECT_EQ(0xCD, out[11]);  // Destination address, byte 6.
  EXPECT_EQ(0xEF, out[12]);  // Destination address, byte 7.
  EXPECT_EQ(0xFF, out[13]);  // Reserved.
  EXPECT_EQ(0xFE, out[14]);  // Reserved.
  EXPECT_EQ(0x02, out[15]);  // Broadcast radius (hops).
  EXPECT_EQ(0x00, out[16]);  // Transmit options.
  EXPECT_EQ(0x32, out[17]);  // RF data, byte 0.
  EXPECT_EQ(0x11, out[18]);  // RF data, escaped byte 1.
  EXPECT_EQ(0x11, out[19]);  // RF data, escaped byte 2.
  EXPECT_EQ(0xAB, out[20]);  // RF data, byte 3.
  EXPECT_EQ(0xDF, out[21]);  // Checksum.
}

TEST(XlrApiParseData, CommandResponse) {
  const uint8_t data[] = {
    XLR_FRAME_DELIM, 0x00, 0x00, kXlrApiFrameTypeCommandResponse,
    0x23, 0xAB, 0xCD, 0x05, 0x10, 0x11, 0x12
  };
  XlrApiData out;

  memset(&out, 0x0, sizeof(out));
  EXPECT_TRUE(XlrApiParseData(ARRAYSIZE(data), data, &out));
  EXPECT_EQ(kXlrApiFrameTypeCommandResponse, out.frame_type);
  EXPECT_EQ(0x23, out.u.command_response.frame_id);
  EXPECT_EQ(0xABCD, out.u.command_response.command);
  EXPECT_EQ(0x05, out.u.command_response.status);
  EXPECT_EQ(2, out.u.command_response.length);
  EXPECT_EQ(0x10, out.u.command_response.data[0]);
  EXPECT_EQ(0x11, out.u.command_response.data[1]);

  memset(&out, 0x0, sizeof(out));
  EXPECT_TRUE(XlrApiParseData(ARRAYSIZE(data) - 1, data, &out));
  EXPECT_EQ(kXlrApiFrameTypeCommandResponse, out.frame_type);
  EXPECT_EQ(0x23, out.u.command_response.frame_id);
  EXPECT_EQ(0xABCD, out.u.command_response.command);
  EXPECT_EQ(0x05, out.u.command_response.status);
  EXPECT_EQ(1, out.u.command_response.length);
  EXPECT_EQ(0x10, out.u.command_response.data[0]);

  memset(&out, 0x0, sizeof(out));
  EXPECT_TRUE(XlrApiParseData(ARRAYSIZE(data) - 2, data, &out));
  EXPECT_EQ(kXlrApiFrameTypeCommandResponse, out.frame_type);
  EXPECT_EQ(0x23, out.u.command_response.frame_id);
  EXPECT_EQ(0xABCD, out.u.command_response.command);
  EXPECT_EQ(0x05, out.u.command_response.status);
  EXPECT_EQ(0, out.u.command_response.length);

  memset(&out, 0x0, sizeof(out));
  EXPECT_FALSE(XlrApiParseData(ARRAYSIZE(data) - 3, data, &out));  // Too short.
}

TEST(XlrApiParseData, ModemStatus) {
  const uint8_t data[] = {
    XLR_FRAME_DELIM, 0x00, 0x00, kXlrApiFrameTypeModemStatus, 0x23, 0x00, 0x00
  };
  XlrApiData out;

  memset(&out, 0x0, sizeof(out));
  EXPECT_FALSE(XlrApiParseData(ARRAYSIZE(data), data, &out));  // Too long.
  EXPECT_FALSE(XlrApiParseData(ARRAYSIZE(data) - 2, data, &out));  // Too short.
  EXPECT_TRUE(XlrApiParseData(ARRAYSIZE(data) - 1, data, &out));
  EXPECT_EQ(kXlrApiFrameTypeModemStatus, out.frame_type);
  EXPECT_EQ(0x23, out.u.modem_status.status);
}

TEST(XlrApiParseData, TransmitStatus) {
  const uint8_t data[] = {
    XLR_FRAME_DELIM, 0x00, 0x00, kXlrApiFrameTypeTransmitStatus,
    0x23, 0x00, 0x00, 0x04, 0x05, 0x06, 0x12, 0x00
  };
  XlrApiData out;

  memset(&out, 0x0, sizeof(out));
  EXPECT_FALSE(XlrApiParseData(ARRAYSIZE(data), data, &out));  // Too long.
  EXPECT_FALSE(XlrApiParseData(ARRAYSIZE(data) - 2, data, &out));  // Too short.
  EXPECT_TRUE(XlrApiParseData(ARRAYSIZE(data) - 1, data, &out));
  EXPECT_EQ(kXlrApiFrameTypeTransmitStatus, out.frame_type);
  EXPECT_EQ(0x23, out.u.transmit_status.frame_id);
  EXPECT_EQ(0x04, out.u.transmit_status.retries);
  EXPECT_EQ(0x05, out.u.transmit_status.delivery_status);
  EXPECT_EQ(0x06, out.u.transmit_status.discovery_status);
}

TEST(XlrApiParseData, RxIndicator) {
  const uint8_t data[] = {
    XLR_FRAME_DELIM, 0x00, 0x00, kXlrApiFrameTypeRxIndicator,
    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,  // Source.
    0x00, 0x00,  // Reserved.
    0x12,  // Options.
    0x0A, 0x0B,  // Data.
    0x00,  // Checksum.
  };
  XlrApiData out;

  memset(&out, 0x0, sizeof(out));
  EXPECT_TRUE(XlrApiParseData(ARRAYSIZE(data), data, &out));
  EXPECT_EQ(kXlrApiFrameTypeRxIndicator, out.frame_type);
  EXPECT_EQ(0x0102030405060708, out.u.rx_indicator.source);
  EXPECT_EQ(0x12, out.u.rx_indicator.options);
  EXPECT_EQ(2, out.u.rx_indicator.length);
  EXPECT_EQ(0x0A, out.u.rx_indicator.data[0]);
  EXPECT_EQ(0x0B, out.u.rx_indicator.data[1]);

  memset(&out, 0x0, sizeof(out));
  EXPECT_TRUE(XlrApiParseData(ARRAYSIZE(data) - 1, data, &out));
  EXPECT_EQ(kXlrApiFrameTypeRxIndicator, out.frame_type);
  EXPECT_EQ(0x0102030405060708, out.u.rx_indicator.source);
  EXPECT_EQ(0x12, out.u.rx_indicator.options);
  EXPECT_EQ(1, out.u.rx_indicator.length);
  EXPECT_EQ(0x0A, out.u.rx_indicator.data[0]);

  memset(&out, 0x0, sizeof(out));
  EXPECT_TRUE(XlrApiParseData(ARRAYSIZE(data) - 2, data, &out));
  EXPECT_EQ(kXlrApiFrameTypeRxIndicator, out.frame_type);
  EXPECT_EQ(0x0102030405060708, out.u.rx_indicator.source);
  EXPECT_EQ(0x12, out.u.rx_indicator.options);
  EXPECT_EQ(0, out.u.rx_indicator.length);

  memset(&out, 0x0, sizeof(out));
  EXPECT_FALSE(XlrApiParseData(ARRAYSIZE(data) - 3, data, &out));  // Too short.
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

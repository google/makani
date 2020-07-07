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

#include "avionics/common/rtcm3.h"
#include "avionics/common/serial_parse.h"
#include "common/macros.h"

}  // extern "C"

namespace {

bool RunTest(int32_t length, const uint8_t *data, Rtcm3Receive *rtcm) {
  memset(rtcm, 0, sizeof(*rtcm));
  bool sync = true;
  int32_t parsed = 0;
  for (int32_t i = 0; i < length && sync && parsed == 0; ++i) {
    sync = Rtcm3Parse(i + 1, data, rtcm, &parsed);
  }
  return sync && parsed > 0;
}

}  // namespace

TEST(Rtcm3Parse, Message1005) {
  uint8_t rtcm1005[] = {
    0xD3, 0x00, 0x13, 0x3E, 0xD7, 0xD3, 0x02, 0x02, 0x98, 0x0E, 0xDE, 0xEF,
    0x34, 0xB4, 0xBD, 0x62, 0xAC, 0x09, 0x41, 0x98, 0x6F, 0x33, 0x36, 0x0B,
    0x98
  };
  Rtcm3Receive rtcm;

  // Parse message with valid checksum.
  EXPECT_TRUE(RunTest(ARRAYSIZE(rtcm1005), rtcm1005, &rtcm));

  // Parse message with invalid checksum.
  rtcm1005[10] = 0xFF;
  EXPECT_FALSE(RunTest(ARRAYSIZE(rtcm1005), rtcm1005, &rtcm));
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

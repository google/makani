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

#include "avionics/common/network_config.h"
#include "avionics/network/aio_node.h"
#include "avionics/network/aio_node_to_ip_address.h"

static bool IpAddressMatch(const IpAddress ip,
                           uint8_t a, uint8_t b, uint8_t c, uint8_t d) {
  return ip.a == a && ip.b == b && ip.c == c && ip.d == d;
}

TEST(IpSanityCheckFcA, Normal) {
  IpAddress ip = AioNodeToIpAddress(kAioNodeFcA);
  EXPECT_TRUE(IpAddressMatch(ip, 192, 168, 1, 6));
}

TEST(IpSanityCheckMotorSbo, Normal) {
  IpAddress ip = AioNodeToIpAddress(kAioNodeMotorSbo);
  EXPECT_TRUE(IpAddressMatch(ip, 192, 168, 1, 11));
}

TEST(IpSanityCheckGpsBaseStation, Normal) {
  IpAddress ip = AioNodeToIpAddress(kAioNodeGpsBaseStation);
  EXPECT_TRUE(IpAddressMatch(ip, 192, 168, 1, 38));
}

TEST(IpSanityCheckServoA1, Normal) {
  IpAddress ip = AioNodeToIpAddress(kAioNodeServoA1);
  EXPECT_TRUE(IpAddressMatch(ip, 192, 168, 1, 19));
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

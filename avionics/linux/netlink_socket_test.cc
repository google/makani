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

#include <net/if.h>
#include <netinet/in.h>
#include <stdint.h>
#include <sys/socket.h>

#include <linux/netlink.h>
#include <linux/rtnetlink.h>

#include "avionics/common/network_config.h"
#include "avionics/linux/netlink_socket.h"

namespace {

void SafeCopy(size_t in_size, const void *in_data, size_t out_size,
              const void *out_data, void *out_ptr) {
  ASSERT_LE(out_data, out_ptr);

  ASSERT_GE(reinterpret_cast<const uint8_t *>(out_data) + out_size,
            reinterpret_cast<const uint8_t *>(out_ptr) + in_size);

  memcpy(out_ptr, in_data, in_size);
}

}  // namespace

TEST(NetlinkSocket, GetInterfaceByIp) {
  union {
    struct nlmsghdr nh;
    uint8_t data[NLMSG_SPACE(sizeof(struct ifaddrmsg))
                 + RTA_SPACE(sizeof(IpAddress))];  /* NOLINT */
  } u;

  memset(&u, 0, sizeof(u));
  u.nh.nlmsg_len = ((uint32_t)NLMSG_SPACE(sizeof(struct ifaddrmsg))
                    + (uint32_t)RTA_SPACE(sizeof(IpAddress)));
  u.nh.nlmsg_type = RTM_GETADDR;

  struct ifaddrmsg ifaddr;
  memset(&ifaddr, 0, sizeof(ifaddr));
  ifaddr.ifa_index = 1234;
  void *u_ifaddr = NLMSG_DATA(&u.nh);
  SafeCopy(sizeof(ifaddr), &ifaddr, sizeof(u), &u, u_ifaddr);

  struct rtattr rt;
  memset(&rt, 0, sizeof(rt));
  rt.rta_type = IFA_LOCAL;
  rt.rta_len = RTA_LENGTH(sizeof(IpAddress));
  void *u_rt = IFA_RTA(u_ifaddr);
  SafeCopy(sizeof(rt), &rt, sizeof(u), &u, u_rt);

  IpAddress ip = {192, 168, 1, 2};
  SafeCopy(sizeof(ip), &ip, sizeof(u), &u, RTA_DATA(u_rt));

  // Test expected result.
  int32_t if_index;
  EXPECT_TRUE(NetlinkGetInterfaceIndexByIp(sizeof(u), &u, &ip, &if_index));
  EXPECT_EQ(ifaddr.ifa_index, if_index);

  // Test invalid IP address.
  IpAddress invalid_ip = {192, 168, 255, 255};
  EXPECT_FALSE(NetlinkGetInterfaceIndexByIp(sizeof(u), &u, &invalid_ip,
                                            &if_index));
}

TEST(NetlinkSocket, GetInterfaceStats) {
  union {
    struct nlmsghdr nh;
    uint8_t data[NLMSG_SPACE(sizeof(struct ifinfomsg))
                 + RTA_SPACE(sizeof(struct rtnl_link_stats))];  /* NOLINT */
  } u;

  memset(&u, 0, sizeof(u));
  u.nh.nlmsg_len = ((uint32_t)NLMSG_LENGTH(sizeof(struct ifinfomsg))
                    + (uint32_t)RTA_LENGTH(sizeof(struct rtnl_link_stats)));
  u.nh.nlmsg_type = RTM_GETLINK;

  struct ifinfomsg ifinfo;
  memset(&ifinfo, 0, sizeof(ifinfo));
  ifinfo.ifi_index = 1234;
  void *u_ifinfo = NLMSG_DATA(&u.nh);
  SafeCopy(sizeof(ifinfo), &ifinfo, sizeof(u), &u, u_ifinfo);

  struct rtattr rt;
  memset(&rt, 0, sizeof(rt));
  rt.rta_type = IFLA_STATS;
  rt.rta_len = RTA_LENGTH(sizeof(struct rtnl_link_stats));
  void *u_rt = IFLA_RTA(u_ifinfo);
  SafeCopy(sizeof(rt), &rt, sizeof(u), &u, u_rt);

  struct rtnl_link_stats in_stats;
  memset(&in_stats, 0, sizeof(in_stats));
  in_stats.rx_packets = 1U;
  in_stats.tx_packets = 2U;
  in_stats.rx_bytes = 3U;
  in_stats.tx_bytes = 4U;
  in_stats.rx_errors = 5U;
  in_stats.tx_errors = 6U;
  in_stats.rx_dropped = 7U;
  in_stats.tx_dropped = 8U;
  in_stats.multicast = 9U;
  in_stats.collisions = 10U;

  // Detailed rx_errors.
  in_stats.rx_length_errors = 11U;
  in_stats.rx_over_errors = 12U;
  in_stats.rx_crc_errors = 13U;
  in_stats.rx_frame_errors = 14U;
  in_stats.rx_fifo_errors  = 15U;
  in_stats.rx_missed_errors = 16U;

  // Detailed tx_errors.
  in_stats.tx_aborted_errors = 17U;
  in_stats.tx_carrier_errors = 18U;
  in_stats.tx_fifo_errors = 19U;
  in_stats.tx_heartbeat_errors = 20U;
  in_stats.tx_window_errors = 21U;
  SafeCopy(sizeof(in_stats), &in_stats, sizeof(u), &u, RTA_DATA(u_rt));

  // Test expected result.
  NetlinkInterfaceStats out_stats;
  EXPECT_TRUE(NetlinkGetInterfaceStats(sizeof(u), &u, ifinfo.ifi_index,
                                       &out_stats));

  EXPECT_EQ(in_stats.rx_packets, out_stats.rx_packets);
  EXPECT_EQ(in_stats.tx_packets, out_stats.tx_packets);
  EXPECT_EQ(in_stats.rx_bytes, out_stats.rx_bytes);
  EXPECT_EQ(in_stats.tx_bytes, out_stats.tx_bytes);
  EXPECT_EQ(in_stats.rx_errors, out_stats.rx_errors);
  EXPECT_EQ(in_stats.tx_errors, out_stats.tx_errors);
  EXPECT_EQ(in_stats.rx_dropped, out_stats.rx_dropped);
  EXPECT_EQ(in_stats.tx_dropped, out_stats.tx_dropped);
  EXPECT_EQ(in_stats.multicast, out_stats.multicast);
  EXPECT_EQ(in_stats.collisions, out_stats.collisions);

  // Detailed rx_errors.
  EXPECT_EQ(in_stats.rx_length_errors, out_stats.rx_length_errors);
  EXPECT_EQ(in_stats.rx_over_errors, out_stats.rx_over_errors);
  EXPECT_EQ(in_stats.rx_crc_errors, out_stats.rx_crc_errors);
  EXPECT_EQ(in_stats.rx_frame_errors, out_stats.rx_frame_errors);
  EXPECT_EQ(in_stats.rx_fifo_errors , out_stats.rx_fifo_errors);
  EXPECT_EQ(in_stats.rx_missed_errors, out_stats.rx_missed_errors);

  // Detailed tx_errors.
  EXPECT_EQ(in_stats.tx_aborted_errors, out_stats.tx_aborted_errors);
  EXPECT_EQ(in_stats.tx_carrier_errors, out_stats.tx_carrier_errors);
  EXPECT_EQ(in_stats.tx_fifo_errors, out_stats.tx_fifo_errors);
  EXPECT_EQ(in_stats.tx_heartbeat_errors, out_stats.tx_heartbeat_errors);
  EXPECT_EQ(in_stats.tx_window_errors, out_stats.tx_window_errors);

  // Test invalid index failure.
  EXPECT_FALSE(NetlinkGetInterfaceStats(sizeof(u), &u, ifinfo.ifi_index + 1,
                                        &out_stats));
}

TEST(NetlinkSocket, GetInterfaceFlags) {
  union {
    struct nlmsghdr nh;
    uint8_t data[NLMSG_SPACE(sizeof(struct ifinfomsg))];  /* NOLINT */
  } u;

  memset(&u, 0, sizeof(u));
  u.nh.nlmsg_len = (uint32_t)NLMSG_LENGTH(sizeof(struct ifinfomsg));
  u.nh.nlmsg_type = RTM_GETLINK;

  struct ifinfomsg ifinfo;
  memset(&ifinfo, 0, sizeof(ifinfo));
  ifinfo.ifi_index = 1234;
  ifinfo.ifi_flags = 0xABCDEF01;
  SafeCopy(sizeof(ifinfo), &ifinfo, sizeof(u), &u, NLMSG_DATA(&u.nh));

  // Test expected result.
  uint32_t ifi_flags;
  EXPECT_TRUE(NetlinkGetInterfaceFlags(sizeof(u), &u, ifinfo.ifi_index,
                                       &ifi_flags));
  EXPECT_EQ(ifinfo.ifi_flags, ifi_flags);

  // Test invalid index failure.
  EXPECT_FALSE(NetlinkGetInterfaceFlags(sizeof(u), &u, ifinfo.ifi_index + 1,
                                        &ifi_flags));
}

TEST(NetlinkSocket, CheckRouteUpdateByDst) {
  union {
    struct nlmsghdr nh;
    uint8_t data[NLMSG_SPACE(sizeof(struct rtmsg))
                 + RTA_SPACE(sizeof(IpAddress))];  /* NOLINT */
  } u;

  memset(&u, 0, sizeof(u));
  u.nh.nlmsg_len = ((uint32_t)NLMSG_SPACE(sizeof(struct rtmsg))
                    + (uint32_t)RTA_SPACE(sizeof(IpAddress)));
  u.nh.nlmsg_type = RTM_NEWROUTE;

  struct rtmsg rtm;
  memset(&rtm, 0, sizeof(rtm));
  void *u_rtm = NLMSG_DATA(&u.nh);
  SafeCopy(sizeof(rtm), &rtm, sizeof(u), &u, u_rtm);

  struct rtattr rt;
  memset(&rt, 0, sizeof(rt));
  rt.rta_type = RTA_DST;
  rt.rta_len = RTA_LENGTH(sizeof(IpAddress));
  void *u_rt = RTM_RTA(u_rtm);
  SafeCopy(sizeof(rt), &rt, sizeof(u), &u, u_rt);

  IpAddress dst = {239, 0, 0, 0};
  uint32_t ndst = htonl(IpAddressToUint32(dst));
  SafeCopy(sizeof(ndst), &ndst, sizeof(u), &u, RTA_DATA(u_rt));

  // Test expected result.
  EXPECT_TRUE(NetlinkCheckRouteUpdateByDst(sizeof(u), &u, &dst));

  // Test invalid destination address.
  IpAddress invalid_dst = {239, 0, 0, 1};
  EXPECT_FALSE(NetlinkCheckRouteUpdateByDst(sizeof(u), &u, &invalid_dst));
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

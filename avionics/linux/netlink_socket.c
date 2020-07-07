/*
 * Copyright 2020 Makani Technologies LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "avionics/linux/netlink_socket.h"

#include <assert.h>
#include <errno.h>
#include <netinet/in.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#include <linux/if.h>
#include <linux/netlink.h>
#include <linux/rtnetlink.h>

#include "avionics/common/network_addresses.h"

// Used only as a sanity check on the message size.
#define MAX_MESSAGE_SIZE (1 << 16)

#define NETLINK_SOCKET_IGNORE_ERROR(e) (                                \
    (e) == EAGAIN || (EWOULDBLOCK != EAGAIN && (e) == EWOULDBLOCK) ||   \
    (e) == EINTR)

static void PrintError(int error, const char *func, int32_t line) {
  fprintf(stderr, __FILE__ ":%d %s: %s\n", line, func, strerror(error));
}

static void CopyInterfaceStats(const struct rtnl_link_stats *in,
                               NetlinkInterfaceStats *out) {
  out->rx_packets = (uint32_t)in->rx_packets;
  out->tx_packets = (uint32_t)in->tx_packets;
  out->rx_bytes = (uint32_t)in->rx_bytes;
  out->tx_bytes = (uint32_t)in->tx_bytes;
  out->rx_errors = (uint32_t)in->rx_errors;
  out->tx_errors = (uint32_t)in->tx_errors;
  out->rx_dropped = (uint32_t)in->rx_dropped;
  out->tx_dropped = (uint32_t)in->tx_dropped;
  out->multicast = (uint32_t)in->multicast;
  out->collisions = (uint32_t)in->collisions;

  // Detailed rx_errors.
  out->rx_length_errors = (uint32_t)in->rx_length_errors;
  out->rx_over_errors = (uint32_t)in->rx_over_errors;
  out->rx_crc_errors = (uint32_t)in->rx_crc_errors;
  out->rx_frame_errors = (uint32_t)in->rx_frame_errors;
  out->rx_fifo_errors  = (uint32_t)in->rx_fifo_errors;
  out->rx_missed_errors = (uint32_t)in->rx_missed_errors;

  // detailed tx_errors.
  out->tx_aborted_errors = (uint32_t)in->tx_aborted_errors;
  out->tx_carrier_errors = (uint32_t)in->tx_carrier_errors;
  out->tx_fifo_errors = (uint32_t)in->tx_fifo_errors;
  out->tx_heartbeat_errors = (uint32_t)in->tx_heartbeat_errors;
  out->tx_window_errors = (uint32_t)in->tx_window_errors;
}

// Handle ifinfomsg messages.

static bool IsIfInfoMsg(const struct nlmsghdr *nh) {
  return nh->nlmsg_type == RTM_NEWLINK
      || nh->nlmsg_type == RTM_DELLINK
      || nh->nlmsg_type == RTM_GETLINK;
}

static bool IfInfoGetInterfaceStats(int32_t rt_len,
                                    struct ifinfomsg *rt_msg,
                                    NetlinkInterfaceStats *out) {
  assert(0 <= rt_len && rt_len < MAX_MESSAGE_SIZE);
  assert(rt_msg != NULL);
  assert(out != NULL);

  // See rtnetlink(7). Macro RTA_NEXT() updates rt_len.
  for (struct rtattr *rt = IFLA_RTA(rt_msg); RTA_OK(rt, rt_len);
       rt = RTA_NEXT(rt, rt_len)) {
    if (rt->rta_type == IFLA_STATS
        && RTA_PAYLOAD(rt) == sizeof(struct rtnl_link_stats)) {
      CopyInterfaceStats((const struct rtnl_link_stats *)RTA_DATA(rt), out);
      return true;
    }
  }
  return false;
}

// Handle ifaddrmsg messages.

static bool IsIfAddrMsg(const struct nlmsghdr *nh) {
  return nh->nlmsg_type == RTM_NEWADDR
      || nh->nlmsg_type == RTM_DELADDR
      || nh->nlmsg_type == RTM_GETADDR;
}

static bool IfAddrGetIndexByIp(int32_t rt_len, struct ifaddrmsg *rt_msg,
                               const IpAddress *ip, int32_t *if_index) {
  assert(0 <= rt_len && rt_len < MAX_MESSAGE_SIZE);
  assert(rt_msg != NULL);
  assert(ip != NULL);
  assert(if_index != NULL);

  // See rtnetlink(7). Macro RTA_NEXT() updates rt_len.
  for (struct rtattr *rt = IFA_RTA(rt_msg); RTA_OK(rt, rt_len);
       rt = RTA_NEXT(rt, rt_len)) {
    if (rt->rta_type == IFA_LOCAL
        && RTA_PAYLOAD(rt) == sizeof(*ip)
        && memcmp(RTA_DATA(rt), ip, sizeof(*ip)) == 0) {
      *if_index = (int32_t)rt_msg->ifa_index;
      return true;
    }
  }
  return false;
}

// Handle rtmsg messages.

static bool IsRtMsg(const struct nlmsghdr *nh) {
  return nh->nlmsg_type == RTM_NEWROUTE
      || nh->nlmsg_type == RTM_DELROUTE
      || nh->nlmsg_type == RTM_GETROUTE;
}

static bool RtMsgCheckDst(int32_t rt_len, struct rtmsg *rt_msg,
                          const IpAddress *dst) {
  assert(0 <= rt_len && rt_len < MAX_MESSAGE_SIZE);
  assert(rt_msg != NULL);

  // See rtnetlink(7). Macro RTA_NEXT() updates rt_len.
  for (struct rtattr *rt = RTM_RTA(rt_msg); RTA_OK(rt, rt_len);
       rt = RTA_NEXT(rt, rt_len)) {
    if (rt->rta_type == RTA_DST
        && RTA_PAYLOAD(rt) == sizeof(*dst)
        && memcmp(RTA_DATA(rt), dst, sizeof(*dst)) == 0) {
      return true;
    }
  }
  return false;
}

// Handle nlmsghdr messages.

static bool NlGetIfInfoByIndex(int32_t nl_len, struct nlmsghdr *nl_msg,
                               int32_t if_index, int32_t *rt_len,
                               struct ifinfomsg **rt_msg) {
  assert(0 <= nl_len && nl_len < MAX_MESSAGE_SIZE);
  assert(nl_msg != NULL);
  assert(rt_len != NULL);
  assert(rt_msg != NULL);

  // See netlink(3) and netlink(7). Macro NLMSG_NEXT() updates nl_len.
  for (struct nlmsghdr *nh = nl_msg;
       NLMSG_OK(nh, (uint32_t)nl_len) && nh->nlmsg_type != NLMSG_DONE;
       nh = NLMSG_NEXT(nh, nl_len)) {
    if (IsIfInfoMsg(nh)) {
      *rt_len = (int32_t)NLMSG_PAYLOAD(nh, sizeof(struct ifinfomsg));
      *rt_msg = (struct ifinfomsg *)NLMSG_DATA(nh);
      if ((*rt_msg)->ifi_index == if_index) {
        return true;
      }
    }
  }
  return false;
}

static bool NlGetIndexByIp(int32_t nl_len, struct nlmsghdr *nl_msg,
                           const IpAddress *ip, int32_t *if_index) {
  assert(0 <= nl_len && nl_len < MAX_MESSAGE_SIZE);
  assert(nl_msg != NULL);
  assert(ip != NULL);
  assert(if_index != NULL);

  // See netlink(3) and netlink(7). Macro NLMSG_NEXT() updates nl_len.
  for (struct nlmsghdr *nh = nl_msg;
       NLMSG_OK(nh, (uint32_t)nl_len) && nh->nlmsg_type != NLMSG_DONE;
       nh = NLMSG_NEXT(nh, nl_len)) {
    if (IsIfAddrMsg(nh)) {
      int32_t rt_len = (int32_t)NLMSG_PAYLOAD(nh, sizeof(struct ifaddrmsg));
      struct ifaddrmsg *rt_msg = (struct ifaddrmsg *)NLMSG_DATA(nh);
      if (IfAddrGetIndexByIp(rt_len, rt_msg, ip, if_index)) {
        return true;
      }
    }
  }
  return false;
}

static bool NlCheckRouteUpdateByDst(int32_t nl_len,
                                    struct nlmsghdr *nl_msg,
                                    const IpAddress *dst) {
  assert(0 <= nl_len && nl_len < MAX_MESSAGE_SIZE);
  assert(nl_msg != NULL);

  // See netlink(3) and netlink(7). Macro NLMSG_NEXT() updates nl_len.
  for (struct nlmsghdr *nh = nl_msg;
       NLMSG_OK(nh, (uint32_t)nl_len) && nh->nlmsg_type != NLMSG_DONE;
       nh = NLMSG_NEXT(nh, nl_len)) {
    if (IsRtMsg(nh)) {
      int32_t rt_len = (int32_t)NLMSG_PAYLOAD(nh, sizeof(struct rtmsg));
      struct rtmsg *rt_msg = (struct rtmsg *)NLMSG_DATA(nh);
      if (RtMsgCheckDst(rt_len, rt_msg, dst)) {
        return true;
      }
    }
  }
  return false;
}

// Handle send request.

static int32_t SendRequest(int32_t fd, uint32_t seq_num, uint16_t type) {
  // See rtnetlink(7).
  union {
    struct nlmsghdr nh;
    uint8_t data[NLMSG_SPACE(sizeof(struct rtgenmsg))];  /* NOLINT */
  } u;

  memset(&u, 0, sizeof(u));
  u.nh.nlmsg_len = NLMSG_LENGTH(sizeof(struct rtgenmsg));
  u.nh.nlmsg_type = type;
  u.nh.nlmsg_flags = NLM_F_DUMP | NLM_F_REQUEST;
  u.nh.nlmsg_seq = seq_num;
  u.nh.nlmsg_pid = (uint32_t)getpid();

  struct rtgenmsg rtgen = {
    .rtgen_family = AF_UNSPEC
  };
  memcpy(NLMSG_DATA(&u.nh), &rtgen, sizeof(rtgen));

  int32_t rc = (int32_t)send(fd, &u, sizeof(u), 0);
  if (rc < 0) {
    PrintError(errno, __func__, __LINE__);
  }
  return rc;
}

// Handle socket.

static int32_t OpenSocket(void) {
  int32_t fd = socket(AF_NETLINK, SOCK_RAW | SOCK_NONBLOCK, NETLINK_ROUTE);
  if (fd < 0) {
    PrintError(errno, __func__, __LINE__);
  }
  return fd;
}

static bool BindSocket(int32_t fd) {
  // See netlink(7).
  struct sockaddr_nl local_addr;
  memset(&local_addr, 0, sizeof(local_addr));
  local_addr.nl_family = AF_NETLINK;
  local_addr.nl_pid = (uint32_t)getpid();
  local_addr.nl_groups = RTMGRP_LINK | RTMGRP_IPV4_IFADDR | RTMGRP_IPV4_ROUTE;
  if (bind(fd, (struct sockaddr *)&local_addr, sizeof(local_addr)) < 0) {
    PrintError(errno, __func__, __LINE__);
    return false;
  }
  return true;
}

// Public functions.

int32_t NetlinkSocketOpen(void) {
  int32_t fd = OpenSocket();
  if (fd >= 0 && !BindSocket(fd)) {
    NetlinkSocketClose(fd);
    fd = -1;
  }
  return fd;
}

int32_t NetlinkSocketClose(int32_t fd) {
  int32_t rc = 0;
  if (fd >= 0) {
    rc = close(fd);
    if (rc < 0) {
      PrintError(errno, __func__, __LINE__);
    }
  }
  return rc;
}

int32_t NetlinkSendGetLink(int32_t fd, uint32_t seq_num) {
  return SendRequest(fd, seq_num, RTM_GETLINK);
}

int32_t NetlinkSendGetAddr(int32_t fd, uint32_t seq_num) {
  return SendRequest(fd, seq_num, RTM_GETADDR);
}

int32_t NetlinkReceive(int32_t fd, int32_t data_length, void *data) {
  assert(data_length >= 0 && data != NULL);

  ssize_t rc = recv(fd, data, (size_t)data_length, MSG_DONTWAIT);
  if (rc < 0) {
    if (NETLINK_SOCKET_IGNORE_ERROR(errno)) {
      rc = 0;  // Ignore these errors.
    } else {
      PrintError(errno, __func__, __LINE__);
    }
  }
  return (int32_t)rc;
}

bool NetlinkGetInterfaceIndexByIp(int32_t nl_len, void *nl_msg,
                                  const IpAddress *ip, int32_t *if_index) {
  assert(0 <= nl_len && nl_len < MAX_MESSAGE_SIZE);
  assert(nl_msg != NULL && (size_t)nl_msg % NLMSG_ALIGNTO == 0);
  assert(ip != NULL);
  assert(if_index != NULL);

  struct nlmsghdr *nh = (struct nlmsghdr *)nl_msg;
  return NlGetIndexByIp(nl_len, nh, ip, if_index);
}

bool NetlinkGetInterfaceStats(int32_t nl_len, void *nl_msg,
                              int32_t if_index,
                              NetlinkInterfaceStats *if_stats) {
  assert(0 <= nl_len && nl_len < MAX_MESSAGE_SIZE);
  assert(nl_msg != NULL && (size_t)nl_msg % NLMSG_ALIGNTO == 0);
  assert(if_stats != NULL);

  struct nlmsghdr *nh = (struct nlmsghdr *)nl_msg;
  int32_t rt_len;
  struct ifinfomsg *rt_msg;
  if (NlGetIfInfoByIndex(nl_len, nh, if_index, &rt_len, &rt_msg)) {
    return IfInfoGetInterfaceStats(rt_len, rt_msg, if_stats);
  }
  return false;
}

bool NetlinkGetInterfaceFlags(int32_t nl_len, void *nl_msg,
                              int32_t if_index, uint32_t *ifi_flags) {
  assert(0 <= nl_len && nl_len < MAX_MESSAGE_SIZE);
  assert(nl_msg != NULL && (size_t)nl_msg % NLMSG_ALIGNTO == 0);
  assert(ifi_flags != NULL);

  struct nlmsghdr *nh = (struct nlmsghdr *)nl_msg;
  int32_t rt_len;
  struct ifinfomsg *rt_msg;
  if (NlGetIfInfoByIndex(nl_len, nh, if_index, &rt_len, &rt_msg)) {
    *ifi_flags = rt_msg->ifi_flags;  // See netdevice(7).
    return true;
  }
  return false;
}

bool NetlinkCheckRouteUpdateByDst(int32_t nl_len, void *nl_msg,
                                  const IpAddress *dst) {
  assert(0 <= nl_len && nl_len < MAX_MESSAGE_SIZE);
  assert(nl_msg != NULL && (size_t)nl_msg % NLMSG_ALIGNTO == 0);
  assert(dst != NULL);

  struct nlmsghdr *nh = (struct nlmsghdr *)nl_msg;
  return NlCheckRouteUpdateByDst(nl_len, nh, dst);
}

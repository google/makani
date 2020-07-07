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

#ifndef AVIONICS_LINUX_NETLINK_SOCKET_H_
#define AVIONICS_LINUX_NETLINK_SOCKET_H_

#include <stdbool.h>
#include <stdint.h>

#include "avionics/common/network_addresses.h"
#include "avionics/linux/netlink_stats.h"

#ifdef __cplusplus
extern "C" {
#endif

int32_t NetlinkSocketOpen(void);
int32_t NetlinkSocketClose(int32_t fd);

int32_t NetlinkSendGetAddr(int32_t fd, uint32_t seq_num);
int32_t NetlinkSendGetLink(int32_t fd, uint32_t seq_num);
int32_t NetlinkReceive(int32_t fd, int32_t data_length, void *data);

bool NetlinkGetInterfaceIndexByIp(int32_t nl_len, void *nl_msg,
                                  const IpAddress *ip, int32_t *if_index);

bool NetlinkGetInterfaceStats(int32_t nl_len, void *nl_msg,
                              int32_t if_index,
                              NetlinkInterfaceStats *if_stats);

bool NetlinkGetInterfaceFlags(int32_t nl_len, void *nl_msg,
                              int32_t if_index, uint32_t *ifi_flags);

bool NetlinkCheckRouteUpdateByDst(int32_t nl_len, void *nl_msg,
                                  const IpAddress *dst);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // AVIONICS_LINUX_NETLINK_SOCKET_H_

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

#ifndef AVIONICS_LINUX_AIO_CLIENT_STATS_H_
#define AVIONICS_LINUX_AIO_CLIENT_STATS_H_

#include <stdint.h>

#include "avionics/linux/netlink_stats.h"

typedef struct {
  int32_t received_valid_packets;
  int32_t received_invalid_packets;
  int32_t sent_packets;
  int32_t unsent_packets;
  int32_t idle_time;
  int32_t total_time;
  int32_t select_errors;
  int32_t aio_socket_reopened;
  int32_t aio_socket_errors;
  int32_t netlink_socket_errors;
  int32_t netlink_stats_updates;
  int32_t netlink_interface_updates;
  int32_t netlink_route_updates;

  // Check netlink_stats_updates to determine updates to this field.
  NetlinkInterfaceStats netlink;
} AioClientStats;

#endif  // AVIONICS_LINUX_AIO_CLIENT_STATS_H_

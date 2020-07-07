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

#ifndef AVIONICS_LINUX_NETLINK_STATS_H_
#define AVIONICS_LINUX_NETLINK_STATS_H_

#include <stdint.h>

typedef struct {
  uint32_t rx_packets;  // Total packets received.
  uint32_t tx_packets;  // Total packets transmitted.
  uint32_t rx_bytes;    // Total bytes received.
  uint32_t tx_bytes;    // Total bytes transmitted.
  uint32_t rx_errors;   // Bad packets received.
  uint32_t tx_errors;   // Packet transmit problems.
  uint32_t rx_dropped;  // No space in linux buffers.
  uint32_t tx_dropped;  // No space available in linux.
  uint32_t multicast;   // Multicast packets received.
  uint32_t collisions;  // Packet collisions.

  // Detailed rx_errors.
  uint32_t rx_length_errors;  // Received length errors.
  uint32_t rx_over_errors;    // Receiver ring buffer overflow.
  uint32_t rx_crc_errors;     // Received pkt with crc error.
  uint32_t rx_frame_errors;   // Received frame alignment error.
  uint32_t rx_fifo_errors;    // Receiver fifo overrun.
  uint32_t rx_missed_errors;  // Receiver missed packet.

  // Detailed tx_errors.
  uint32_t tx_aborted_errors;
  uint32_t tx_carrier_errors;
  uint32_t tx_fifo_errors;
  uint32_t tx_heartbeat_errors;
  uint32_t tx_window_errors;
} NetlinkInterfaceStats;

#endif  // AVIONICS_LINUX_NETLINK_STATS_H_

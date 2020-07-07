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

#ifndef AVIONICS_FIRMWARE_NETWORK_NET_MON_TYPES_H_
#define AVIONICS_FIRMWARE_NETWORK_NET_MON_TYPES_H_

#include <stdint.h>

#include "avionics/common/network_config.h"

typedef struct {
  uint16_t rx_multicast_packet_rate;
  uint16_t tx_multicast_packet_rate;
  uint32_t rx_octet_rate;
  uint32_t tx_octet_rate;
  uint16_t rx_fragment_errors;
  uint16_t rx_alignment_errors;
  uint16_t rx_fcs_errors;
  uint16_t rx_symbol_errors;
  uint16_t rx_jabber_errors;
  uint16_t rx_in_range_errors;
  uint32_t rx_good_octet_rate;
  uint16_t rx_dropped_packets;
  uint16_t tx_dropped_packets;
  uint16_t rx_pause_packets;
  uint16_t tx_pause_packets;
  uint16_t rx_route_discard;
} EthernetStats;

typedef struct {
  EthernetStats stats[NUM_ACCESS_SWITCH_PORTS];
  uint32_t link_status_bits;
  uint32_t segment_status_bits;
  uint32_t reconfigured_status_bits;
  uint32_t reconfigured_events;
  uint16_t sequence_num;
} AccessSwitchStats;

typedef struct {
  EthernetStats stats[NUM_CORE_SWITCH_PORTS];
  uint32_t link_status_bits;
  uint32_t segment_status_bits;
  uint32_t reconfigured_status_bits;
  uint32_t reconfigured_events;
  uint16_t sequence_num;
} CoreSwitchStats;

#endif  // AVIONICS_FIRMWARE_NETWORK_NET_MON_TYPES_H_

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

#ifndef AVIONICS_COMMON_EOP_MESSAGES_H_
#define AVIONICS_COMMON_EOP_MESSAGES_H_

#include <stdint.h>

// This file specifies the message types communicated between the Eop modem
// and the core switch. It does not specify AIO message types. Updating this
// file means updating the Eop modem software and reprogramming the Eop
// modems.

typedef struct {
  uint16_t sequence;
  int8_t type;
  int8_t reserved;
  uint32_t version;  // See eop_version.h.
} EopHeader;

typedef struct {
  int32_t data_lpdu_big_sent;
  int32_t data_lpdu_small_sent;
  int32_t bytes_received;
  int32_t packets_sent;
  int32_t packets_received;
  int32_t errors_received;
  int32_t unicast_packets_sent;
  int32_t unicast_packets_received;
  int32_t discard_packets_sent;
  int32_t discard_packets_received;
  int32_t multicast_packets_sent;
  int32_t multicast_packets_received;
  int32_t broadcast_packets_sent;
  int32_t broadcast_packets_received;
  int32_t mgmt_lpdu_big_sent;
  int32_t mgmt_lpdu_small_sent;
  int32_t mgmt_bytes_received;
  int32_t blocks_resent;
} EopGhnCounters;

typedef struct {
  int32_t rx_packets;
  int32_t rx_bytes;
  int32_t tx_packets;
  int32_t tx_bytes;
  int32_t rx_drop_overflow;
  int32_t rx_drop_rx_error;
  int32_t rx_drop_collision;
  int32_t rx_drop_length;
  int32_t rx_drop_no_cell;
  int32_t rx_drop;
  int32_t rx_bad_crc;
  int32_t tx_drop;
  int32_t tx_drop_collision;
} EopEthCounters;

typedef struct {
  uint32_t agc_overflows;
  uint16_t rms_power1;
  uint16_t rms_power2;
  uint8_t rx_agc_enabled;
  uint8_t rx_gain1;
  uint8_t rx_gain2;
  uint8_t rx_gain;
  uint8_t rx_gain_min;
  uint8_t rx_gain_max;
  uint8_t rx_gains;
  uint8_t tx_agc_enabled;
  uint8_t tx_gain;
  uint8_t tx_gains;
} EopAgcStatus;

typedef struct {
  uint8_t version[20];  // Git hash of marvell repository.
  int8_t phy_temperature;  // [C]
  EopGhnCounters ghn_counters;
  EopEthCounters eth_counters;
  EopAgcStatus agc_status;
} EopModemStatusMessage;

#endif  // AVIONICS_COMMON_EOP_MESSAGES_H_

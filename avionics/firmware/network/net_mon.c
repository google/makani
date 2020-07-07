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

#include "avionics/firmware/network/net_mon.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "avionics/firmware/drivers/bcm.h"
#include "avionics/firmware/drivers/bcm53101.h"
#include "avionics/firmware/drivers/bcm53284.h"
#include "avionics/firmware/network/net_probe.h"
#include "avionics/network/switch_config.h"
#include "avionics/network/switch_types.h"
#include "common/macros.h"

// Highest-level state variable for NetMonPoll FSM.
typedef enum {
  kNetPollPhaseStart,
  kNetPollPhaseReadData,
  kNetPollPhaseReadLinkStatus,
  kNetPollPhaseIdle
} NetPollPhase;

// State vector for NetMonPoll FSM.
static struct {
  NetPollPhase current_phase;
  int32_t current_port;
  int32_t current_port_index;
  PortStatistic current_stat;
  bool stats_ready_flag;
  uint32_t sequence_num;
} g_net_poll_state;

static struct {
  int32_t port_list[NUM_SWITCH_PORTS_MAX];
  int32_t number_of_ports;
  bool (*read_mib_fn)(PortStatistic statistic, int32_t port,
                      MibCounts *counters);
  bool (*read_link_fn)(uint32_t *link_status_bits,
                       uint32_t *down_ports_changed,
                       uint32_t *down_port_events);
} g_net_poll_config;

static MibCounts g_raw_stats_a[NUM_SWITCH_PORTS_MAX];
static MibCounts g_raw_stats_b[NUM_SWITCH_PORTS_MAX];
static MibCounts *g_raw_stats_current_ptr, *g_raw_stats_last_ptr;
static uint32_t g_link_status_bits;
static uint32_t g_down_ports_changed;
static uint32_t g_down_port_events;

// TODO: Move config argument to the poll function.
void NetMonInit(const SwitchInfo *info) {
  GetActivePortList(info, g_net_poll_config.port_list,
                    &g_net_poll_config.number_of_ports);
  switch (info->type) {
    case kSwitchTypeBcm53284:
      g_net_poll_config.read_mib_fn = Bcm53284ReadMibCounter;
      g_net_poll_config.read_link_fn = Bcm53284ReadLinkStatus;
      break;
    case kSwitchTypeBcm53101:
      g_net_poll_config.read_mib_fn = Bcm53101ReadMibCounter;
      g_net_poll_config.read_link_fn = Bcm53101ReadLinkStatus;
      break;
    default:
      assert(false);
      break;
  }
  memset(&g_net_poll_state, 0, sizeof(g_net_poll_state));
  memset(g_raw_stats_a, 0, sizeof(g_raw_stats_a));
  memset(g_raw_stats_b, 0, sizeof(g_raw_stats_b));
  g_raw_stats_current_ptr = g_raw_stats_a;
  g_raw_stats_last_ptr = g_raw_stats_b;
  g_net_poll_state.stats_ready_flag = false;
}

// NetMonPoll is a sequencer that reads the per port statistics of the ports
// in use on each access switch. The sequencer advances to the next state once
// one statistic for one port has been read.
// Parameters are read in the order defined by the enum PortStatistic.
// The sequencer is a hierarchial state machine with three state variables:
//   current_phase - indicates initializing, reading statistics or idling,
//   current_port  - the port on the access switch being read,
//   current_stat  - the port statistics being read.
void NetMonPoll(void) {
  switch (g_net_poll_state.current_phase) {
    case kNetPollPhaseStart:
      // Propagate the state.
      g_net_poll_state.current_port_index = 0;
      g_net_poll_state.current_phase = kNetPollPhaseReadData;
      g_net_poll_state.current_port =
          g_net_poll_config.port_list[g_net_poll_state.current_port_index];
      g_net_poll_state.current_stat = 0;
      break;
    case kNetPollPhaseReadData:
      // Read all the per-port stats for fault detection.
      // Break out of the loop if reading is not finished.
      if (!g_net_poll_config.read_mib_fn(g_net_poll_state.current_stat,
                                         g_net_poll_state.current_port,
                                         g_raw_stats_current_ptr)) {
        break;
      }
      // Propagate the state when read has completed.
      // First iterate all the stats for the current port.
      if (g_net_poll_state.current_stat < kNumPortStatistics - 1) {
        g_net_poll_state.current_stat++;
      } else {
        // Then iterate all the ports.
        if (g_net_poll_state.current_port_index <
            g_net_poll_config.number_of_ports - 1) {
          g_net_poll_state.current_port_index++;
          g_net_poll_state.current_port =
              g_net_poll_config.port_list[g_net_poll_state.current_port_index];
          g_net_poll_state.current_stat = 0;
        } else {
          g_net_poll_state.current_phase = kNetPollPhaseReadLinkStatus;
        }
      }
      break;
    case kNetPollPhaseReadLinkStatus:
      if (g_net_poll_config.read_link_fn(&g_link_status_bits,
                                         &g_down_ports_changed,
                                         &g_down_port_events)) {
        g_net_poll_state.current_phase = kNetPollPhaseIdle;
      }
      break;
    case kNetPollPhaseIdle:
      g_net_poll_state.stats_ready_flag = true;
      break;
    default:
      g_net_poll_state.current_phase = kNetPollPhaseIdle;
      assert(false);
      break;
  }
}

static void PackPortStats(int32_t port, EthernetStats *output) {
  MibCounts *current = &g_raw_stats_current_ptr[port];
  MibCounts *last = &g_raw_stats_last_ptr[port];
  // Compute packet rate since last read and transfer to message struct.
  output->rx_multicast_packet_rate =
      (uint16_t)(current->rx_multicast_packets -
                 last->rx_multicast_packets);
  output->tx_multicast_packet_rate =
      (uint16_t)(current->tx_multicast_packets -
                 last->tx_multicast_packets);
  output->rx_octet_rate =
      (uint32_t)(current->rx_octets -
                 last->rx_octets);
  output->tx_octet_rate =
      (uint32_t)(current->tx_octets -
                 last->tx_octets);
  output->rx_fragment_errors = (uint16_t)current->rx_fragment_errors;
  output->rx_alignment_errors = (uint16_t)current->rx_alignment_errors;
  output->rx_fcs_errors = (uint16_t)current->rx_fcs_errors;
  output->rx_symbol_errors = (uint16_t)current->rx_symbol_errors;
  output->rx_jabber_errors = (uint16_t)current->rx_jabber_errors;
  output->rx_in_range_errors = (uint16_t)current->rx_in_range_errors;
  output->rx_good_octet_rate =
      (uint32_t)(current->rx_good_octets -
                 last->rx_good_octets);
  output->rx_dropped_packets = (uint16_t)current->rx_dropped_packets;
  output->tx_dropped_packets = (uint16_t)current->tx_dropped_packets;
  output->rx_pause_packets = (uint16_t)current->rx_pause_packets;
  output->tx_pause_packets = (uint16_t)current->tx_pause_packets;
  output->rx_route_discard = (uint16_t)current->rx_route_discard;
}

static void SwapStats(void) {
  // Ping-pong between g_raw_stats_a and g_raw_stats_b for capturing data.
  // (Not thread safe.)
  if (g_raw_stats_current_ptr == g_raw_stats_a) {
    g_raw_stats_last_ptr = g_raw_stats_a;
    g_raw_stats_current_ptr = g_raw_stats_b;
  } else {
    g_raw_stats_last_ptr = g_raw_stats_b;
    g_raw_stats_current_ptr = g_raw_stats_a;
  }
  g_net_poll_state.stats_ready_flag = false;
  g_net_poll_state.current_phase = kNetPollPhaseStart;
}

// The Ethernet statistics gathered by NetMonPoll are filled into the status
// message once all the counts have been read. The function is intended to
// be called at a periodic rate; multicast rates and byte (octet) rates are
// computed with regard to rate at which this function is called.
static void PackEthernetStats(EthernetStats *stats, uint32_t *link_status_bits,
                              uint32_t *segment_status_bits,
                              uint32_t *reconfigured_status_bits,
                              uint32_t *reconfigured_events,
                              uint16_t *sequence_num) {
  // Update message if new counts have been read.
  if (g_net_poll_state.stats_ready_flag) {
    for (int32_t i = 0; i < g_net_poll_config.number_of_ports; ++i) {
      int32_t port = g_net_poll_config.port_list[i];
      PackPortStats(port, &stats[port]);
    }
    *link_status_bits = g_link_status_bits;
    *sequence_num = (uint16_t)(++g_net_poll_state.sequence_num);
    *segment_status_bits = NetProbeGetSegmentStatus();
    *reconfigured_status_bits = g_down_ports_changed;
    *reconfigured_events = g_down_port_events;

    SwapStats();
  }
}

void NetMonGetAccessSwitchStats(AccessSwitchStats *switch_stats) {
  PackEthernetStats(switch_stats->stats, &switch_stats->link_status_bits,
                    &switch_stats->segment_status_bits,
                    &switch_stats->reconfigured_status_bits,
                    &switch_stats->reconfigured_events,
                    &switch_stats->sequence_num);
}

void NetMonGetCoreSwitchStats(CoreSwitchStats *switch_stats) {
  PackEthernetStats(switch_stats->stats, &switch_stats->link_status_bits,
                    &switch_stats->segment_status_bits,
                    &switch_stats->reconfigured_status_bits,
                    &switch_stats->reconfigured_events,
                    &switch_stats->sequence_num);
}

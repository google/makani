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

// The net_probe module uses specially defined segment VLANs to verify the wing
// network connectivity.  We transmit special probe packets which are designed
// to propogate one hop on the network and ensure that switches are connected
// as specified in network.yaml.

// Every segment on the network is assigned a unique ID.  This ID is known by
// both switches attached to that segment.  TMS570s will send out probe packets
// with each ID periodically (specified in the VLAN tag).  The switches are
// configured to route each ID to the TMS570 and one of the switch ports.
// The tagged packet hits the switch and is forwarded to a specific egress port.
// The tagged packet hits an ingress port on the opposite switch.  If the
// packet's VLAN ID matches the ingress port, the packet will be forwarded to
// that TMS570.  In this manner, the fact of receiving a probe message with
// a specific ID means that that segment is connected to the correct switch
// ports on the correct nodes on both sides.

#include "avionics/firmware/network/net_probe.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "avionics/common/endian.h"
#include "avionics/firmware/cpu/clock.h"
#include "avionics/firmware/identity/identity.h"
#include "avionics/firmware/network/net.h"
#include "avionics/firmware/network/pack_net_probe_message.h"
#include "avionics/network/switch_config.h"
#include "avionics/network/switch_info.h"

#define PROBE_SEND_INTERVAL CLOCK32_MSEC_TO_CYCLES(250)
#define PROBE_EXPIRE CLOCK32_MSEC_TO_CYCLES(10000)

static struct {
  int16_t send_port;
  uint32_t receive_cycles[NUM_SWITCH_PORTS_MAX];
  uint32_t send_timeout;
  uint32_t segment_status;
} g_state;

static const SwitchInfo *g_info;

static bool SendNetProbeMessage(int16_t send_port) {
  assert(0 <= send_port && send_port < g_info->num_ports);

  if (g_info->segment_vlans[send_port] == 0) {
    return true;
  }

  NetProbeMessage probe;
  uint8_t buf[PACK_NETPROBEMESSAGE_SIZE];
  memset(buf, 0, sizeof(buf));
  IpAddress dest_ip = {192, 168, 1, 255};
  EthernetAddress dest_mac = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  VlanTag vlan = {.drop_eligible = false, .priority = 0,
                  .vlan_id = g_info->segment_vlans[send_port]};
  probe.segment_id = g_info->segment_vlans[send_port];
  probe.source_node = AppConfigGetAioNode();
  PackNetProbeMessage(&probe, 1, buf);
  return NetSendUdp(dest_ip, dest_mac, UDP_PORT_NET_PROBE, UDP_PORT_NET_PROBE,
                    buf, sizeof(buf), &vlan);
}

void NetProbeInit(const SwitchInfo *switch_info) {
  memset(&g_state, 0, sizeof(g_state));
  g_state.segment_status = 0;
  g_state.send_timeout = Clock32GetCycles();
  g_info = switch_info;
}

void NetProbePoll(void) {
  uint32_t now = Clock32GetCycles();
  if (CLOCK32_GE(now, g_state.send_timeout)) {
    if (SendNetProbeMessage(g_state.send_port)) {
      g_state.send_port = (g_state.send_port + 1) % g_info->num_ports;
      g_state.send_timeout += PROBE_SEND_INTERVAL;
    }
  }
  for (int32_t i = 0; i < g_info->num_ports; i++) {
    if (CLOCK32_GE(now, (g_state.receive_cycles[i] + PROBE_EXPIRE))) {
      g_state.segment_status &= ~(1 << i);
    }
  }
  // Show correct segment for TMS570 port.
  g_state.segment_status |= 1 << g_info->host_port;
}

bool NetProbeReceive(int32_t msg_len, const uint8_t *msg) {
  NetProbeMessage probe;
  if (msg_len < (int32_t)sizeof(probe)) return false;
  UnpackNetProbeMessage(msg, 1, &probe);
  for (int32_t i = 0; i < g_info->num_ports; i++) {
    if (probe.segment_id == g_info->segment_vlans[i]) {
      g_state.segment_status |= 1 << i;
      g_state.receive_cycles[i] = Clock32GetCycles();
      return true;
    }
  }
  return false;
}

uint32_t NetProbeGetSegmentStatus(void) {
  return g_state.segment_status;
}

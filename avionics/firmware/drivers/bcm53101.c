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

#include "avionics/firmware/drivers/bcm53101.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "avionics/common/network_config.h"
#include "avionics/firmware/util/state_machine.h"
#include "avionics/network/switch_config.h"
#include "avionics/network/switch_types.h"
#include "common/macros.h"

typedef enum {
  kStateInit,
  kStateResetDelay,
  kStateResetDone,
  kStateDisableForwarding,
  kStateDisableStp,
  kStateEnableVlans,
  kStateWriteVlans,
  kStateSetPortBasedVlans,
  kStateSetDefaultVlans,
  kStateWriteMulticastTable,
  kStateSetFailedForwardMap,
  kStatePortForwardControl,
  kStateDisableLearning,
  kStateDisableUnusedPorts,
  kStatePortStateOverride,
  kStateFiberMode,
  kStateMirrorControl,
  kStateEnableForwarding,
  kStateStartupWaitLink,
  kStateStartupDelay,
  kStateReady,
  kStateDumpRoutes,
  kStateReconfigureUnusedPorts,
  kStateReconfigureFiberMode,
  kStateReconfigureDisableLearning,
  kStateReconfigurePortForwardControl,
} BcmState;

// The default config is meant to ensure delivery of traffic in the bootloader
// or the bootloader application.  Unicast traffic will still flow over the A
// network ports.  Both A and B network ports will still forward multicast
// traffic.
#define HOST_PORT 5
#define MASK_ALL ((1 << NUM_SWITCH_PORTS_BCM53101) - 1)
#define MASK_A ((1 << 0) | (1 << 2) | (1 << 4) | (1 << 5))
#define MASK_B ((1 << 1) | (1 << 3) | (1 << 4) | (1 << 5))
static const SwitchInfo kDefaultSwitchInfo = {
  .type = kSwitchTypeBcm53101,
  .num_ports = NUM_SWITCH_PORTS_BCM53101,
  .num_fiber_ports = NUM_SWITCH_FIBER_PORTS_BCM53101,
  .host_port = HOST_PORT,
  .mirror_port = -1,
  .forward_mask_a = MASK_A,
  .forward_mask_b = MASK_B,
  .isolate_mask = 0,
  .unicast_mask = MASK_A,
  .segment_vlans = NULL,
  .trunk = {0, 0, 0, 0, NULL}
};

static const SwitchOptions kDefaultSwitchOptions = {
  .unicast_flood_mask_current = MASK_A,
  .unicast_learn_mask_current = MASK_A,
  .mirror_enable = false,
  .num_static_addresses = 0,
  .static_addresses = NULL,
  .port_disable_mask_current = 0,
};

static const SwitchInfo *g_info;
static const SwitchOptions *g_options;
static const MulticastTable *g_mcast_table;
static VlanTable g_vlan_table;
static bool g_perform_reset;
static bool g_reconfigure_options;
static bool g_dump_routes;
static bool g_dump_routes_finished;
static bool g_dump_routes_new_entry;
static AddressRouteEntry g_dump_routes_entry;
static uint8_t g_initialize_fiber_ports;

typedef enum {
  kPageControl = 0x00,
  kPageStatus = 0x01,
  kPageManagementMirror = 0x02,
  kPageArlControl = 0x04,
  kPageArlVtblAccess = 0x05,
  kPageExternalPhy0 = 0x10,
  kPageExternalPhy1 = 0x11,
  kPageExternalPhy2 = 0x12,
  kPageExternalPhy3 = 0x13,
  kPageExternalPhy4 = 0x14,
  kPageMibPort0 = 0x20,
  kPageMibPort1 = 0x21,
  kPageMibPort2 = 0x22,
  kPageMibPort3 = 0x23,
  kPageMibPort4 = 0x24,
  kPageMibPort5 = 0x25,
  kPageMibPortJmp = 0x28,
  kPageQos = 0x30,
  kPagePortVlan = 0x31,
  kPageTrunk = 0x32,
  kPageTagVlan = 0x34,
  kPageDos = 0x36,
  kPageMaxFrame = 0x40,
  kPageBroadcastStorm = 0x41,
  kPageEap = 0x42,
  kPageMstp = 0x43,
  kPageMibSnapshotControl = 0x70,
  kPageMibSnapshot = 0x71,
  kPageWanInterfacePhy = 0x85,
  kPageImpPortPhy = 0x88,
  kPageBroadsync = 0x90,
  kPageTrafficRemarking = 0x91,
} RegPage;

static bool StateInit(FsmState *state) {
  // Get config.
  GenerateVlanTable(g_info, &g_vlan_table);

  // Initialize low-level switch interface.
  BcmInit();

  g_initialize_fiber_ports = (1U << g_info->num_fiber_ports) - 1;

  if (g_perform_reset) {
    // Reset switch.
    BcmReset(true);
  } else {
    state->next = kStateResetDone;
  }

  return true;
}

static bool StateResetDone(FsmState *state) {
  (void)state;
  BcmReset(false);
  return true;
}

static bool StateDisableForwarding(FsmState *state) {
  (void)state;
  return BcmWriteUint8(kPageControl, 0x0B, 0x00);
}

static bool StateDisableStp(FsmState *state) {
  static int32_t port = 0;
  if (state->first_entry) port = 0;
  return BcmWriteUint8(kPageControl, port, 0x00) && ++port >= g_info->num_ports;
}

static bool StateEnableVlans(FsmState *state) {
  (void)state;
  // Turn on 802.1Q VLAN support.
  return BcmWriteUint8(kPageTagVlan, 0x00, 0xe3);
}

static bool StateWriteVlans(FsmState *state) {
  static int32_t i = 0, vlan = 0;
  if (state->first_entry) {
    i = 0;
    vlan = 0;
  }
  VlanEntry *entry = &g_vlan_table.vlan[vlan];
  uint32_t vlan_config;
  uint8_t read_value;
  if (vlan >= g_vlan_table.num_vlans) {
    return true;
  }
  switch (i) {
    case 0:
      // Set VLAN ID.
      if (BcmWriteUint16(kPageArlVtblAccess, 0x81, entry->vlan_id)) ++i;
      break;
    case 1:
      // Set VLAN entry.
      vlan_config = ((entry->vlan_forward_mode ? 1 : 0) << 21)
          | (entry->untag_map << 9)
          | entry->forward_map;
      if (BcmWriteUint32(kPageArlVtblAccess, 0x83, vlan_config)) ++i;
      break;
    case 2:
      // Begin write operation.
      if (BcmWriteUint8(kPageArlVtblAccess, 0x80, 0x80)) ++i;
      break;
    case 3:
      // Wait for write completion.
      if (BcmReadUint8(kPageArlVtblAccess, 0x80, &read_value)
          && (read_value & 0x80) == 0) {
        i = 0;
        ++vlan;
      }
      break;
    default:
      assert(false);
      state->next = state->error;
      return true;
  }
  return false;
}

static bool ArlInsert(bool first_entry, uint16_t vlan,
                      const EthernetAddress *mac, uint32_t arl_entry_data) {
  static int32_t i = 0;
  static int32_t bucket = 0;
  uint32_t entry;
  uint8_t read_value;
  uint64_t mac_addr = ((uint64_t)mac->a << 40) | ((uint64_t)mac->b << 32)
      | ((uint64_t)mac->c << 24) | ((uint64_t)mac->d << 16)
      | ((uint64_t)mac->e << 8) | mac->f;
  if (first_entry) {
    i = 0;
    bucket = 0;
  }
  switch (i) {
    case 0:
      // Set memory key to MAC address.
      if (BcmWriteUint48(kPageArlVtblAccess, 0x02, mac_addr)) ++i;
      break;
    case 1:
      // Set VLAN ID.
      if (BcmWriteUint16(kPageArlVtblAccess, 0x08, vlan)) ++i;
      break;
    case 2:
      // Read data.
      if (BcmWriteUint8(kPageArlVtblAccess, 0x00, 0x81)) ++i;
      break;
    case 3:
      if (BcmReadUint8(kPageArlVtblAccess, 0x00, &read_value)
          && (read_value & 0x80) == 0) {
        ++i;
      }
      break;
    case 4:
      // Find empty bucket.
      if (BcmReadUint32(kPageArlVtblAccess, 0x18 + 0x10 * bucket, &entry)) {
        if ((entry & 0x10000) == 0) {
          ++i;
        } else if (++bucket >= 4) {
          assert(false);
          return true;
        }
      }
      break;
    case 5:
      // Set ARL address.
      if (BcmWriteUint64(kPageArlVtblAccess, 0x10 + 0x10 * bucket,
                         mac_addr | ((uint64_t)vlan << 48))) {
        ++i;
      }
      break;
    case 6:
      // Set memory data to ARL entry.
      if (BcmWriteUint32(kPageArlVtblAccess, 0x18 + 0x10 * bucket,
                         arl_entry_data)) {
        ++i;
      }
      break;
    case 7:
      // Write data.
      if (BcmWriteUint8(kPageArlVtblAccess, 0x00, 0x80)) ++i;
      break;
    case 8:
      if (BcmReadUint8(kPageArlVtblAccess, 0x00, &read_value)
          && (read_value & 0x80) == 0) {
        return true;
      }
      break;
    default:
      assert(false);
      return true;
  }
  return false;
}

static bool StateWriteMulticastTable(FsmState *state) {
  static bool multicast_first_entry = true;
  static int32_t i = 0;
  if (state->first_entry) {
    multicast_first_entry = true;
    i = 0;
  }
  if (g_mcast_table == NULL || i >= g_mcast_table->num_entries) {
    return true;
  }
  const MulticastEntry *entry = &g_mcast_table->entry[i];
  uint32_t data = 0x18000 | entry->forward_map;
  // Multicast routes are added with the default VLAN for untagged traffic (1).
  multicast_first_entry = ArlInsert(multicast_first_entry, 1, &entry->mac,
                                    data);
  if (multicast_first_entry) ++i;
  return false;
}

static bool StateSetPortBasedVlans(FsmState *state) {
  static int32_t port = 0;
  if (state->first_entry) port = 0;
  // Set the port-based VLAN for each port.
  return BcmWriteUint16(kPagePortVlan, 2 * port,
                        g_vlan_table.port_based_vlan[port])
      && ++port >= g_info->num_ports;
}

static bool StateSetDefaultVlans(FsmState *state) {
  static int32_t port = 0;
  if (state->first_entry) port = 0;
  // Set the default VLAN for each port.
  return BcmWriteUint16(kPageTagVlan, 0x10 + 2 * port,
                        g_vlan_table.default_vlan[port])
      && ++port >= g_info->num_ports;
}

static bool StateSetFailedForwardMap(FsmState *state) {
  // Direct unresolved unicast destinations to all but B-network facing ports.
  (void)state;
  return BcmWriteUint16(kPageControl, 0x32,
                        g_options->unicast_flood_mask_current);
}

static bool StatePortForwardControl(FsmState *state) {
  // Enable failed forward map for unicast.  Enable failed forward map for
  // multicast if a multicast table is provided.  Default failed forward map
  // will drop all multicast.
  // See p.147 and p.150 of 53101M-DS05-R for bit mask definitions.
  (void)state;
  uint8_t port_forward = 0x41;
  if (g_mcast_table != NULL) {
    port_forward |= 0x80;
  }
  return BcmWriteUint8(kPageControl, 0x21, port_forward);
}

static bool StateDisableLearning(FsmState *state) {
  (void)state;
  uint16_t mask = ~g_options->unicast_learn_mask_current;
  return BcmWriteUint16(kPageControl, 0x3C, mask);
}

static bool StateDisableUnusedPorts(FsmState *state) {
  static uint32_t enabled_ports = 0;
  static uint32_t old_disable_mask = 0;
  static uint32_t disable_mask = 0;
  static uint32_t reset_time_cycles = 0;
  static int32_t i = 0;
  if (state->first_entry) {
    i = 0;
    disable_mask = 0x1F & g_options->port_disable_mask_current;
  }
  uint32_t old_power_down = 0;
  uint32_t power_down = disable_mask | (disable_mask << 5) |
      (disable_mask << 10) | (disable_mask << 15) | (disable_mask << 20);
  switch (i) {
    case 0:
      if (BcmReadUint32(kPageControl, 0x4A, &old_power_down)) {
        if (old_power_down == power_down) return true;
        old_disable_mask = 0x1F &
            (old_power_down | (old_power_down >> 5)
             | (old_power_down >> 10) | (old_power_down >> 15)
             | (old_power_down >> 20));
        ++i;
      }
      break;
    case 1:
      if (BcmWriteUint32(kPageControl, 0x4A, power_down)) {
        enabled_ports = 0x1F & ~disable_mask & old_disable_mask;
        g_initialize_fiber_ports |= enabled_ports;
        if (enabled_ports == 0) return true;
        ++i;
      }
      break;
    case 2:
      // Reset the PHYs of ports which were re-enabled.
      if (BcmWriteUint16(kPageControl, 0x48, (uint16_t)enabled_ports)) {
        reset_time_cycles = Clock32GetCycles();
        ++i;
      }
      break;
    case 3:
      if (CLOCK32_LE(reset_time_cycles + CLOCK32_USEC_TO_CYCLES(1),
                     Clock32GetCycles())) ++i;
      break;
    case 4:
      if (BcmWriteUint16(kPageControl, 0x48, 0)) ++i;
      break;
    default:
      return true;
  }
  return false;
}

static bool StatePortStateOverride(FsmState *state) {
  // Set RvMII port to 100Mbps, full duplex, and link up.
  (void)state;
  // Force host port to full-duplex 100Mbps, link up.
  return BcmWriteUint8(kPageControl, 0x58 + g_info->host_port, 0x47);
}

// Enable 100BASE-FX mode on fiber ports.
static bool StateFiberMode(FsmState *state) {
  static uint16_t read_value = 0;
  static int32_t port = 0, i = 0;
  if (state->first_entry) {
    port = 0;
    i = 0;
  }
  while (!(g_initialize_fiber_ports & (1 << port))) {
    ++port;
    if (port >= g_info->num_fiber_ports) {
      return true;
    }
  }
  uint8_t page = kPageExternalPhy0 + port;
  switch (i) {
    case 0:
      // Set port to 100M full-duplex without auto-negotiation.
      if (BcmWriteUint16(page, 0x00, 0x2100)) ++i;
      break;
    case 1:
      // Bypass scrambler and descrambler blocks.
      if (BcmReadUint16(page, 0x20, &read_value)) ++i;
      break;
    case 2:
      if (BcmWriteUint16(page, 0x20, read_value | 0x0220)) ++i;
      break;
    case 3:
      // Change three-level MLT-3 code to two-level binary.
      if (BcmReadUint16(page, 0x2E, &read_value)) ++i;
      break;
    case 4:
      if (BcmWriteUint16(page, 0x2E, read_value | 0x0020)) ++i;
      break;
    case 5:
      // Enable FX signal detect.
      if (BcmWriteUint16(page, 0x3E, 0x008B)) ++i;
      break;
    case 6:
      if (BcmWriteUint16(page, 0x32, 0x0200)) ++i;
      break;
    case 7:
      if (BcmWriteUint16(page, 0x3A, 0x0084)) ++i;
      break;
    case 8:
      // Set receiver in a half threshold mode. (see 5328X-AN103-R page 24)
      if (BcmWriteUint16(page, 0x22, 0x0A01)) ++i;
      break;
    case 9:
      if (BcmWriteUint16(page, 0x3E, 0x000B)) ++i;
      break;
    default:
      i = 0;
      g_initialize_fiber_ports &= ~(1 << port);
  }
  return false;
}

static bool StateMirrorControl(FsmState *state) {
  static int32_t i = 0;
  if (state->first_entry) i = 0;
  switch (i) {
    case 0:
      if (BcmWriteUint16(kPageManagementMirror, 0x12, 0x003F)) ++i;
      break;
    case 1:
      if (BcmWriteUint16(kPageManagementMirror, 0x1C, 0x003F)) ++i;
      break;
    default:
      return true;
  }
  return false;
}


static bool StateEnableForwarding(FsmState *state) {
  (void)state;
  return BcmWriteUint8(kPageControl, 0x0B, 0x02);
}

static bool StateStartupWaitLink(FsmState *state) {
  uint32_t link_status;
  uint32_t port_speed;
  static int32_t i = 0;
  if (state->first_entry) i = 0;
  switch (i) {
    case 0:
      // Verify that the host port and at least one other port is up before
      // declaring the switch operational.
      if (Bcm53101ReadLinkStatus(&link_status, NULL, NULL)
          && (link_status & (1 << g_info->host_port)) != 0
          && (link_status & ~(1 << g_info->host_port)) != 0) {
        ++i;
      }
      break;
    case 1:
      // Verify that the host port rate is 100 Mbps or 200 Mbps.
      if (BcmReadUint32(kPageStatus, 0x04, &port_speed)
          && (port_speed & (1 << (g_info->host_port * 2))) != 0) {
        ++i;
      }
      break;
    default:
      return true;
  }
  return false;
}

static bool StateDumpRoutes(FsmState *state) {
  static int32_t i = 0;
  static int32_t arl_index = 0;
  if (state->first_entry) {
    i = 0;
    arl_index = 0;
    g_dump_routes_finished = false;
    g_dump_routes_new_entry = false;
  }
  uint8_t search_status;
  uint64_t arl_addr;
  uint32_t arl_data;
  switch (i) {
    case 0:
      if (BcmWriteUint8(kPageArlVtblAccess, 0x50, 0x80)) ++i;
      break;
    case 1:
      if (!g_dump_routes_new_entry &&
          BcmReadUint8(kPageArlVtblAccess, 0x50, &search_status)) {
        if ((search_status & 0x80) == 0) {
          g_dump_routes_finished = true;
          return true;
        } else if (search_status & 0x01) {
          arl_index = 0;
          ++i;
        }
      }
      break;
    case 2:
      if (BcmReadUint64(kPageArlVtblAccess, 0x60 + (arl_index * 0x10),
                        &arl_addr)) {
        g_dump_routes_entry.vlan_id = (uint16_t)(arl_addr >> 48);
        g_dump_routes_entry.ethernet_address.a = (uint8_t)(arl_addr >> 40);
        g_dump_routes_entry.ethernet_address.b = (uint8_t)(arl_addr >> 32);
        g_dump_routes_entry.ethernet_address.c = (uint8_t)(arl_addr >> 24);
        g_dump_routes_entry.ethernet_address.d = (uint8_t)(arl_addr >> 16);
        g_dump_routes_entry.ethernet_address.e = (uint8_t)(arl_addr >> 8);
        g_dump_routes_entry.ethernet_address.f = (uint8_t)arl_addr;
        ++i;
      }
      break;
    case 3:
      if (BcmReadUint32(kPageArlVtblAccess, 0x68 + (arl_index * 0x10),
                        &arl_data)) {
        g_dump_routes_entry.valid = (arl_data & (1U << 16)) != 0;
        g_dump_routes_entry.static_entry = (arl_data & (1U << 15)) != 0;
        g_dump_routes_entry.age = (arl_data & (1U << 14)) != 0;
        g_dump_routes_entry.priority = (arl_data >> 11) & 0x7;
        g_dump_routes_entry.arl_con = (arl_data >> 9) & 0x3;
        g_dump_routes_entry.port_map = arl_data & 0x1FF;
        if (g_dump_routes_entry.valid) {
          g_dump_routes_new_entry = true;
        }
        if (arl_index == 0) {
          arl_index = 1;
          i = 2;
        } else {
          arl_index = 0;
          i = 1;
        }
      }
      break;
    default:
      assert(false);
      return true;
  }
  return false;
}

static const StateTransition kFsmTrans[] = {
  STATE_TRANS_DEFAULT(kStateInit, StateInit, kStateResetDelay),
  STATE_TRANS_DELAY(kStateResetDelay, kStateResetDone, 100e3),
  STATE_TRANS_DEFAULT(kStateResetDone, StateResetDone, kStateDisableForwarding),
  STATE_TRANS_DEFAULT(kStateDisableForwarding, StateDisableForwarding,
                      kStateDisableStp),
  STATE_TRANS_DEFAULT(kStateDisableStp, StateDisableStp, kStateEnableVlans),
  STATE_TRANS_DEFAULT(kStateEnableVlans, StateEnableVlans, kStateWriteVlans),
  STATE_TRANS_DEFAULT(kStateWriteVlans, StateWriteVlans,
                      kStateSetPortBasedVlans),
  STATE_TRANS_DEFAULT(kStateSetPortBasedVlans, StateSetPortBasedVlans,
                      kStateSetDefaultVlans),
  STATE_TRANS_DEFAULT(kStateSetDefaultVlans, StateSetDefaultVlans,
                      kStateWriteMulticastTable),
  STATE_TRANS_DEFAULT(kStateWriteMulticastTable, StateWriteMulticastTable,
                      kStateSetFailedForwardMap),
  STATE_TRANS_DEFAULT(kStateSetFailedForwardMap, StateSetFailedForwardMap,
                      kStatePortForwardControl),
  STATE_TRANS_DEFAULT(kStatePortForwardControl, StatePortForwardControl,
                      kStateDisableLearning),
  STATE_TRANS_DEFAULT(kStateDisableLearning, StateDisableLearning,
                      kStateDisableUnusedPorts),
  STATE_TRANS_DEFAULT(kStateDisableUnusedPorts, StateDisableUnusedPorts,
                      kStatePortStateOverride),
  STATE_TRANS_DEFAULT(kStatePortStateOverride, StatePortStateOverride,
                      kStateFiberMode),
  STATE_TRANS_DEFAULT(kStateFiberMode, StateFiberMode, kStateMirrorControl),
  STATE_TRANS_DEFAULT(kStateMirrorControl, StateMirrorControl,
                      kStateEnableForwarding),
  STATE_TRANS_DEFAULT(kStateEnableForwarding, StateEnableForwarding,
                      kStateStartupWaitLink),
  STATE_TRANS(kStateStartupWaitLink, StateStartupWaitLink, kStateStartupDelay,
              STATE_INVALID, kStateStartupDelay, 1e6),
  // TODO: Get rid of this state.
  STATE_TRANS_DELAY(kStateStartupDelay, kStateReady, 2e6),
  STATE_TRANS_IDLE(kStateReady),
  STATE_TRANS_DEFAULT(kStateDumpRoutes, StateDumpRoutes, kStateReady),
  STATE_TRANS_DEFAULT(kStateReconfigureUnusedPorts, StateDisableUnusedPorts,
                      kStateReconfigureFiberMode),
  STATE_TRANS_DEFAULT(kStateReconfigureFiberMode, StateFiberMode,
                      kStateReconfigureDisableLearning),
  STATE_TRANS_DEFAULT(kStateReconfigureDisableLearning, StateDisableLearning,
                      kStateReconfigurePortForwardControl),
  STATE_TRANS_DEFAULT(kStateReconfigurePortForwardControl,
                      StatePortForwardControl, kStateReady),
};

static const StateMachine kFsm = {
  .num_states = ARRAYSIZE(kFsmTrans),
  .init_state = kStateInit,
  .states = kFsmTrans
};

static FsmState g_state;

void Bcm53101Init(bool reset) {
  g_info = NULL;
  g_options = NULL;
  g_mcast_table = NULL;
  g_perform_reset = false;
  g_reconfigure_options = false;
  g_dump_routes = false;
  g_dump_routes_finished = false;
  g_dump_routes_new_entry = false;

  memset(&g_vlan_table, 0, sizeof(g_vlan_table));
  memset(&g_dump_routes_entry, 0, sizeof(g_dump_routes_entry));

  g_perform_reset = reset;
  StateMachineInit(&kFsm, &g_state);
}

void Bcm53101Poll(const SwitchConfig *config) {
  if (config == NULL) {
    g_info = &kDefaultSwitchInfo;
    g_options = &kDefaultSwitchOptions;
    g_mcast_table = NULL;
  } else {
    g_info = config->info;
    g_options = config->options;
    g_mcast_table = config->mcast_table;
  }
  if (Bcm53101Ready()) {
    if (g_dump_routes) {
      g_dump_routes = false;
      SetState(kStateDumpRoutes, &g_state);
    } else if (g_reconfigure_options) {
      g_reconfigure_options = false;
      SetState(kStateReconfigureUnusedPorts, &g_state);
    }
  }
  StateMachinePoll(&kFsm, &g_state);
  BcmPoll();
}

bool Bcm53101Ready(void) {
  return GetState(&g_state) == kStateReady;
}

bool Bcm53101MirrorEnable(bool enable) {
  return BcmWriteUint16(kPageManagementMirror, 0x10,
                        enable << 15 | g_info->mirror_port);
}

bool Bcm53101ReadMibCounter(PortStatistic statistic, int32_t port,
                            MibCounts *access_switch_counters) {
  assert(0 <= port && port < NUM_SWITCH_PORTS_BCM53101);

  // See Table 160, page 211, in 53101-DS05-R for MIB register pages.
  uint8_t page = kPageMibPort0 + (uint8_t)port;
  // See Table 161, pages 211 to 214 in 53101-DS05-R for MIB register addresses.
  switch (statistic) {
    case kPortStatisticRxAlignmentError:
      return BcmReadUint32(page, 0x80,
                           &access_switch_counters[port].rx_alignment_errors);
    case kPortStatisticRxDiscard:
      return BcmReadUint32(page, 0xC0,
                           &access_switch_counters[port].rx_route_discard);
    case kPortStatisticRxDroppedPackets:
      return BcmReadUint32(page, 0x90,
                           &access_switch_counters[port].rx_dropped_packets);
    case kPortStatisticRxFcsError:
      return BcmReadUint32(page, 0x84,
                           &access_switch_counters[port].rx_fcs_errors);
    case kPortStatisticRxFragmentError:
      return BcmReadUint32(page, 0xA4,
                           &access_switch_counters[port].rx_fragment_errors);
    case kPortStatisticRxGoodOctets:
      return BcmReadUint64(page, 0x88,
                           &access_switch_counters[port].rx_good_octets);
    case kPortStatisticRxInRangeError:
      return BcmReadUint32(page, 0xB0,
                           &access_switch_counters[port].rx_in_range_errors);
    case kPortStatisticRxJabberError:
      return BcmReadUint32(page, 0x7C,
                           &access_switch_counters[port].rx_jabber_errors);
    case kPortStatisticRxMulticastPackets:
      return BcmReadUint32(page, 0x98,
                           &access_switch_counters[port].rx_multicast_packets);
    case kPortStatisticRxOctets:
      return BcmReadUint64(page, 0x50, &access_switch_counters[port].rx_octets);
    case kPortStatisticRxPausePackets:
      return BcmReadUint32(page, 0x5C,
                           &access_switch_counters[port].rx_pause_packets);
    case kPortStatisticRxSymbolError:
      return BcmReadUint32(page, 0xAC,
                           &access_switch_counters[port].rx_symbol_errors);
    case kPortStatisticTxDroppedPackets:
      return BcmReadUint32(page, 0x08,
                           &access_switch_counters[port].tx_dropped_packets);
    case kPortStatisticTxMulticastPackets:
      return BcmReadUint32(page, 0x14,
                           &access_switch_counters[port].tx_multicast_packets);
    case kPortStatisticTxOctets:
      return BcmReadUint64(page, 0x00, &access_switch_counters[port].tx_octets);
    case kPortStatisticTxPausePackets:
      return BcmReadUint32(page, 0x38,
                           &access_switch_counters[port].tx_pause_packets);
    default:
      assert(false);
      break;
  }
  return false;
}

void Bcm53101ReconfigureOptions(void) {
  g_reconfigure_options = true;
}

// See 53101M-DS05-R.pdf page 159, "Link Status Summary Register".
bool Bcm53101ReadLinkStatus(uint32_t *link_status_bits,
                            uint32_t *down_ports_changed,
                            uint32_t *down_event_counter) {
  (void)down_ports_changed;
  (void)down_event_counter;
  // TODO: Support down_ports_changed.
  uint16_t temp;
  if (!BcmReadUint16(kPageStatus, 0x00, &temp)) {
    return false;
  }
  *link_status_bits = temp & 0x013f;
  return true;
}

const AddressRouteEntry *Bcm53101DumpRoutes(bool *finished) {
  *finished = g_dump_routes_finished;
  if (!g_dump_routes_finished && GetState(&g_state) != kStateDumpRoutes) {
    g_dump_routes = true;
  } else {
    g_dump_routes_finished = false;
    if (g_dump_routes_new_entry) {
      g_dump_routes_new_entry = false;
      return &g_dump_routes_entry;
    }
  }
  return NULL;
}

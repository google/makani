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

#include "avionics/firmware/drivers/bcm53284.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "avionics/common/network_config.h"
#include "avionics/firmware/drivers/bcm.h"
#include "avionics/firmware/drivers/bcm5482s.h"
#include "avionics/firmware/util/state_machine.h"
#include "avionics/network/route_config.h"
#include "avionics/network/switch_config.h"
#include "avionics/network/switch_types.h"
#include "common/macros.h"

typedef enum {
  kStateInit,
  kStateResetDelay,
  kStateResetDone,
  kStateBcm5482SInit,
  kStateDisableForwarding,
  kStateInitDelay,
  kStatePortRvMii,
  kStateFiberMode,
  kStateRadioMode,
  kStateEnableVlans,
  kStateWriteVlans,
  kStateSetPortMask,
  kStateSetDefaultVlans,
  kStateSetMulticastRoutes,
  kStateAddMulticastRoutes,
  kStateAddStaticAddresses,
  kStateSetMirror,
  kStateLedInit,
  kStateTrafficControl,
  kStateSetUnicastAgingTimer,
  kStateEnableForwarding,
  kStateStartupDelay,
  kStateReady,
  kStateUpdateMulticastRoutes,
  kStateUpdatePortMask,
  kStateDisablePorts,
  kStateFastAgeUnicast,
  kStateMirrorEnable,
  kStateReadLinkStatus,
  kStateReadMibCounter,
  kStateReadFinished,
  kStateCheckDownPorts,
  kStateDumpRoutes,
} BcmState;

// The default config is meant to ensure delivery of traffic in the bootloader
// or the bootloader application.  All ports will forward unicast and multicast
// traffic.  The trunk links need to be reflected here in ISOLATE_MASK to
// prevent network loops from forming.
#define RADIO_PORT 23
#define HOST_PORT 24
#define MASK_ALL ((1 << NUM_SWITCH_PORTS_BCM53284) - 1)
#define ISOLATE_MASK ((1ul << 18) | (1ul << 20) | (1ul << 22) | (1ul << 23))
static const SwitchInfo kDefaultSwitchInfo = {
  .type = kSwitchTypeBcm53284,
  .num_ports = NUM_SWITCH_PORTS_BCM53284,
  .num_fiber_ports = NUM_SWITCH_FIBER_PORTS_BCM53284,
  .host_port = HOST_PORT,
  .mirror_port = -1,
  .forward_mask_a = MASK_ALL,
  .forward_mask_b = 0,
  .isolate_mask = ISOLATE_MASK,
  .unicast_mask = MASK_ALL,
  .segment_vlans = NULL,
  .trunk = {0, 0, 0, 0, NULL}
};
static const SwitchOptions kDefaultSwitchOptions = {
  .unicast_flood_mask_current = MASK_ALL,
  .unicast_learn_mask_current = 1ul << 20,  // Unicast learning on for POF only.
  .mirror_enable = false,
  .num_static_addresses = 0,
  .static_addresses = NULL,
};

static const SwitchInfo *g_info;
static const SwitchOptions *g_options;
static const MulticastTable *g_mcast_table;
static VlanTable g_vlan_table;
static bool g_perform_reset;

static struct {
  bool reconfigure_options;
  bool read_mib_counter;
  PortStatistic read_statistic;
  int32_t read_port;
  MibCounts *read_counters;
  bool read_link_status;
  uint32_t link_status_bits;
  bool read_finished;
  uint32_t down_ports_changed;
  uint32_t down_port_events;
  bool dump_routes;
  bool dump_routes_finished;
  bool dump_routes_new_entry;
  AddressRouteEntry dump_routes_entry;
} g_cmd;

typedef enum {
  kPageControl0 = 0x00,
  kPageControl1 = 0x01,
  kPageStatus = 0x02,
  kPageManagementMode = 0x03,
  kPageArlControl = 0x04,
  kPageMemorySearch = 0x07,
  kPageMemoryAccess = 0x08,
  kPageCongestionControl = 0x0A,
  kPagePhyInfo = 0x10,
  kPageCfpControl = 0x21,
  kPageCfpUserControl = 0x22,
  kPageDefaultQos = 0x28,
  kPageSwitchFilterControl = 0x2C,
  kPageQos = 0x30,
  kPageTrunk = 0x31,
  kPageVlan = 0x34,
  kPageSecurity = 0x40,
  kPageMibControl = 0x50,
  kPageMibRx = 0x51,
  kPageMibTx = 0x52,
  kPageSnapshotMibRx = 0x53,
  kPageSnapshotMibTx = 0x54,
  kPagePort0Mii = 0xA0,
  kPageExternalPhy0 = 0xD8,
  kPageExternalPhy1 = 0xD9,
  kPageExternalPhy2 = 0xDA,
  kPageExternalPhy3 = 0xDB,
  kPageExternalPhy4 = 0xDC,
} RegPage;

typedef enum {
  kMemTableArl = 0x01,
  kMemTableMulticast = 0x02,
  kMemTableVlan = 0x03,
  kMemTableMst = 0x04,
  kMemTableCfpTcam = 0x10,
  kMemTableCfpAction = 0x11,
  kMemTableCfpRate = 0x12,
  kMemTableCfpStatistic = 0x13,
  kMemTableIvmKey = 0x20,
  kMemTableIvmAction = 0x21,
  kMemTableEvmKey = 0x30,
  kMemTableEvmAction = 0x31,
  kMemTableIngressRateControl = 0x40,
  kMemTableEgressRateControl = 0x41,
  kMemTable1PToTcdp = 0x50,
  kMemTableTcdpTo1P = 0x51,
  kMemTablePortMask = 0x52,
  kMemTableSaLearning = 0x53,
  kMemTableDscpEcnToTcdp = 0x54,
  kMemTableTcdpToDscpEcn = 0x55,
  kMemTableMulticastGroupVirtualPortId = 0x60,
  kMemTableVirtualPortVid = 0x61,
} MemTable;

typedef enum {
  kMemRegIndex = 0x00,
  kMemRegCtrl = 0x08,
  kMemRegGroupCtrl = 0x09,
  kMemRegAddr0 = 0x10,
  kMemRegAddr1 = 0x12,
  kMemRegAddr2 = 0x14,
  kMemRegAddr3 = 0x16,
  kMemRegData0 = 0x20,
  kMemRegData1 = 0x28,
  kMemRegData2 = 0x30,
  kMemRegData3 = 0x38,
  kMemRegData4 = 0x40,
  kMemRegData5 = 0x48,
  kMemRegData6 = 0x50,
  kMemRegData7 = 0x58,
  kMemRegKey0 = 0x80,
  kMemRegKey1 = 0x88,
  kMemRegKey2 = 0x90,
  kMemRegKey3 = 0x98,
  kMemRegKey4 = 0xA0,
  kMemRegKey5 = 0xA8,
  kMemRegKey6 = 0xB0,
  kMemRegKey7 = 0xB8,
} MemReg;

typedef enum {
  kMemOpRead = 0x01,
  kMemOpWrite = 0x02,
  kMemOpIndexRead = 0x03,
  kMemOpIndexWrite = 0x04,
  kMemOpValidScan = 0x05,
} MemOp;

#define MEM_STDN 0x80

typedef enum {
  kLinkTypeHalfDuplex10M = 0x0000,
  kLinkTypeFullDuplex10M = 0x0100,
  kLinkTypeHalfDuplex100M = 0x2000,
  kLinkTypeFullDuplex100M = 0x2100,
  kLinkTypeAutoNegotiate = 0x1200,  // Table 277 in BCM532XM Data Sheet.
} LinkType;

typedef enum {
  kPortModeBasic,
  kPortModeFiber
} PortMode;

static bool BcmMemoryOp(MemOp cmd) {
  return BcmWriteUint8(kPageMemoryAccess, kMemRegCtrl, cmd | MEM_STDN);
}

static bool BcmMemoryOpFinished(void) {
  uint8_t value;
  return BcmReadUint8(kPageMemoryAccess, kMemRegCtrl, &value)
      && (value & MEM_STDN) == 0;
}

static bool SetPortForwarding(int32_t port, bool forwarding_enable) {
  // See 5328XM-DS303-R.pdf page 284.
  return BcmWriteUint8(kPageControl1, 0x90 + port,
                       forwarding_enable ? 0xC0: 0x03);
}

static bool ArlInsert(bool first_entry, uint16_t vlan,
                      const EthernetAddress *mac,
                      uint8_t arl_entry_high, uint64_t arl_entry_low) {
  static int32_t i = 0;
  static int32_t bucket = 0;
  uint8_t valid;
  uint64_t mac_addr = ((uint64_t)mac->a << 40) | ((uint64_t)mac->b << 32)
      | ((uint64_t)mac->c << 24) | ((uint64_t)mac->d << 16)
      | ((uint64_t)mac->e << 8) | mac->f;
  if (first_entry) {
    i = 0;
    bucket = 0;
  }
  switch (i) {
    case 0:
      // Set memory index to ARL table.
      if (BcmWriteUint8(kPageMemoryAccess, kMemRegIndex, kMemTableArl)) ++i;
      break;
    case 1:
      // Set memory key to MAC address.
      if (BcmWriteUint48(kPageMemoryAccess, kMemRegKey0, mac_addr)) ++i;
      break;
    case 2:
      // Set VLAN ID.
      if (BcmWriteUint16(kPageMemoryAccess, kMemRegKey1, vlan)) ++i;
      break;
    case 3:
      // Read data.
      if (BcmMemoryOp(kMemOpIndexRead)) ++i;
      break;
    case 4:
      if (BcmMemoryOpFinished()) ++i;
      break;
    case 5:
      // Find empty bucket.
      if (BcmReadUint8(kPageMemoryAccess, kMemRegData1 + 0x10 * bucket,
                       &valid)) {
        if ((valid & 0x3) == 0) {
          ++i;
        } else if (++bucket >= 4) {
          assert(false);
          return true;
        }
      }
      break;
    case 6:
      // Set memory data to ARL entry.
      if (BcmWriteUint8(kPageMemoryAccess, kMemRegData1 + 0x10 * bucket,
                        arl_entry_high)) {
        ++i;
      }
      break;
    case 7:
      if (BcmWriteUint64(kPageMemoryAccess, kMemRegData0 + 0x10 * bucket,
                         arl_entry_low)) {
        ++i;
      }
      break;
    case 8:
      // Write data.
      if (BcmMemoryOp(kMemOpIndexWrite)) ++i;
      break;
    case 9:
      if (BcmMemoryOpFinished()) return true;
      break;
    default:
      assert(false);
      return true;
  }
  return false;
}

static bool AddStaticAddress(bool first_entry, const StaticAddress *entry) {
  if (entry->port < 0 || entry->port >= g_info->num_ports)
    return true;
  uint64_t arl_entry = (0x09ull << 60) | ((uint64_t)entry->port << 36)
      | ((uint64_t)entry->mac.a << 28) | ((uint32_t)entry->mac.b << 20)
      | ((uint32_t)entry->mac.c << 12) | ((uint32_t)entry->mac.d << 4)
      | (entry->mac.e >> 4);
  return ArlInsert(first_entry, 0, &entry->mac, 0x03, arl_entry);
}

static bool AddMulticastRoute(bool first_entry, const MulticastEntry *entry) {
  uint64_t arl_entry = (0x09ull << 60) | ((uint64_t)entry->group_id << 36)
      | ((uint64_t)entry->mac.a << 28) | ((uint32_t)entry->mac.b << 20)
      | ((uint32_t)entry->mac.c << 12) | ((uint32_t)entry->mac.d << 4)
      | (entry->mac.e >> 4);
  return ArlInsert(first_entry, 0, &entry->mac, 0x03, arl_entry);
}

static bool SetMulticastRoute(bool first_entry, const MulticastEntry *entry) {
  static int32_t i = 0;
  if (first_entry) {
    i = 0;
  }
  switch (i) {
    case 0:
      // Set memory index to multicast table.
      if (BcmWriteUint8(kPageMemoryAccess, kMemRegIndex, kMemTableMulticast)) {
        ++i;
      }
      break;
    case 1:
      // Set memory address to multicast group number.
      if (BcmWriteUint16(kPageMemoryAccess, kMemRegAddr0, entry->group_id)) ++i;
      break;
    case 2:
      // Set memory data to forwarding mask.
      if (BcmWriteUint32(kPageMemoryAccess, kMemRegData0, entry->forward_map)) {
        ++i;
      }
      break;
    case 3:
      // Write data.
      if (BcmMemoryOp(kMemOpWrite)) ++i;
      break;
    case 4:
      if (BcmMemoryOpFinished()) return true;
      break;
    default:
      assert(false);
      return true;
  }
  return false;
}

static bool SetVlan(bool first_entry, const VlanEntry *vlan) {
  static int32_t i = 0;
  if (first_entry) i = 0;
  uint64_t vlan_h;
  uint64_t vlan_l;

  switch (i) {
    case 0:
      // Set memory index to VLAN table.
      if (BcmWriteUint8(kPageMemoryAccess, kMemRegIndex, kMemTableVlan)) ++i;
      break;
    case 1:
      if (BcmWriteUint16(kPageMemoryAccess, kMemRegAddr0, vlan->vlan_id)) ++i;
      break;
    case 2:
      vlan_h = vlan->untag_map >> (64 - 45);
      if (BcmWriteUint48(kPageMemoryAccess, kMemRegData1, vlan_h)) ++i;
      break;
    case 3:
      vlan_l = (((uint64_t)vlan->untag_map) << 45)
          | (vlan->vlan_forward_mode ? (1ULL << 38) : 0)
          | vlan->forward_map;
      if (BcmWriteUint64(kPageMemoryAccess, kMemRegData0, vlan_l)) ++i;
      break;
    case 4:
      if (BcmMemoryOp(kMemOpWrite)) ++i;
      break;
    case 5:
      if (BcmMemoryOpFinished()) return true;
      break;
    default:
      assert(false);
      return true;
  }
  return false;
}

static bool CheckSetValueUint16(uint8_t page, uint8_t address,
                                uint16_t read, uint16_t expect, bool *changed) {
  if (read == expect) {
    return true;
  } else {
    if (changed != NULL) {
      *changed = true;
    }
    return BcmWriteUint16(page, address, expect);
  }
}

static bool SetPortMode(bool first_entry, int32_t port, LinkType link,
                        PortMode mode, bool *changed) {
  static int32_t i = 0;
  if (first_entry) {
    i = 0;
  }
  uint8_t page = kPagePort0Mii + port;

  static uint16_t read_value = 0;
  switch (i) {
    case 0:
      if (BcmWriteUint16(page, 0x00, link)) {
        if (mode == kPortModeBasic) {
          return true;
        } else if (mode == kPortModeFiber) {
          ++i;
        } else {
          assert(false);
          return true;
        }
      }
      break;
      // These settings are documented in 5328X-AN103-RDS.pdf pgs. 23-24.
    case 1:
      // Bypass scrambler and descrambler blocks.
      if (BcmReadUint16(page, 0x20, &read_value)) ++i;
      break;
    case 2:
      if (CheckSetValueUint16(page, 0x20, read_value, read_value | 0x0220,
                              changed)) ++i;
      break;
    case 3:
      // Change three-level MLT-3 code to two-level binary.
      // This register is undocumented in BCM53101 as well.
      if (BcmReadUint16(page, 0x2E, &read_value)) ++i;
      break;
    case 4:
      if (CheckSetValueUint16(page, 0x2E, read_value, read_value | 0x0020,
                              changed)) ++i;
      break;
    case 5:
      // Enable FX signal detect.
      // Access shadow registers.
      if (BcmWriteUint16(page, 0x3E, 0x008B)) ++i;
      break;
    case 6:
      if (BcmReadUint16(page, 0x32, &read_value)) ++i;
      break;
    case 7:
      // This register is undocumented in BCM53101 as well.
      if (CheckSetValueUint16(page, 0x32, read_value, 0x0200, changed)) ++i;
      break;
    case 8:
      if (BcmReadUint16(page, 0x3A, &read_value)) ++i;
      break;
    case 9:
      // The set bit (0x80) is reserved.
      if (CheckSetValueUint16(page, 0x3A, read_value, 0x0084, changed)) ++i;
      break;
    case 10:
      // Access normal registers.
      if (BcmWriteUint16(page, 0x3E, 0x000B)) return true;
      break;
    default:
      assert(false);
      return true;
  }
  return false;
}

static bool StateInit(FsmState *state) {
  // Get config.
  GenerateVlanTable(g_info, &g_vlan_table);

  // Initialize low-level switch interface.
  BcmInit();

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

// The BCM5482S is an external PHY with two GMACS mapped to ports 25 and 26
// on the BCM53284M.
static bool StateBcm5482SInit(FsmState *state) {
  static int32_t i = 0;
  if (state->first_entry) i = 0;
  switch (i) {
    case 0:
      Bcm5482SInit();
      i++;
      break;
    case 1:
      Bcm5482SPoll();
      if (!Bcm5482SReady()) {
        return false;
      }
      i++;
    case 2:  // Fall through.
      return true;
    default:
      assert(false);
      return true;
  }
  return false;
}

static bool StateDisableForwarding(FsmState *state) {
  (void)state;
  return BcmWriteUint8(kPageControl0, 0x00, 0x34);
}

static bool StatePortRvMii(FsmState *state) {
  (void)state;
  return BcmWriteUint8(kPageControl1, 0x28, 0x47);
}

// Enable 100BASE-FX mode on fiber ports.
static bool StateFiberMode(FsmState *state) {
  static bool first_entry = true;
  static int32_t port = 0;
  if (state->first_entry) {
    first_entry = true;
    port = 0;
  }
  first_entry = SetPortMode(first_entry, port, kLinkTypeFullDuplex100M,
                            kPortModeFiber, NULL);
  if (first_entry) {
    return ++port >= g_info->num_fiber_ports;
  }
  return false;
}

static bool StateCheckDownPorts(FsmState *state) {
  static bool first_entry = true;
  static bool changed = false;
  static int32_t port = 0;
  if (state->first_entry) {
    first_entry = true;
    changed = false;
    port = 0;
  }
  if (first_entry) {
    while ((g_cmd.link_status_bits & (1 << port)) != 0) {
      ++port;
      if (port >= g_info->num_fiber_ports) {
        return true;
      }
    }
  }
  first_entry = SetPortMode(first_entry, port, kLinkTypeFullDuplex100M,
                            kPortModeFiber, &changed);
  if (first_entry) {
    if (changed) {
      g_cmd.down_ports_changed |= (1 << port);
      g_cmd.down_port_events++;
    }
    changed = false;
    ++port;
    return port >= g_info->num_fiber_ports;
  }
  return false;
}

// Set radio port to Autonegotiate and re-start auto-negotionation process.
static bool StateRadioMode(FsmState *state) {
  return SetPortMode(state->first_entry, RADIO_PORT, kLinkTypeAutoNegotiate,
                     kPortModeBasic, NULL);
}

static bool StateEnableVlans(FsmState *state) {
  static int32_t i = 0;
  if (state->first_entry) i = 0;
  switch (i) {
    case 0:
      // Set VT_DOMAIN = 0, VT_ENABLE = 0, SVL mode.
      if (BcmWriteUint8(kPageVlan, 0x00, 0x00)) ++i;
      break;
    case 1:
      // Drop unregistered packets.
      if (BcmWriteUint8(kPageVlan, 0x48, 0x1A)) ++i;
      break;
    case 2:
      // Set trust CVID register to all ports.
      if (BcmWriteUint32(kPageVlan, 0x10, (1 << g_info->num_ports) - 1)) ++i;
      break;
    case 3:
      // Set all ports to non-ISP ports.
      if (BcmWriteUint32(kPageVlan, 0x08, 0)) return true;
      break;
    default:
      assert(false);
      return true;
  }
  return false;
}

static bool StateWriteVlans(FsmState *state) {
  static int32_t vlan = 0;
  static bool init_write = true;
  if (state->first_entry) {
    vlan = 0;
    init_write = true;
  }
  if (vlan >= g_vlan_table.num_vlans) {
    return true;
  }
  const VlanEntry *entry = &g_vlan_table.vlan[vlan];
  if (SetVlan(init_write, entry)) {
    init_write = true;
    vlan++;
  } else {
    init_write = false;
  }
  return false;
}

static bool StateSetMirror(FsmState *state) {
  static int32_t i = 0;
  if (state->first_entry) i = 0;
  switch (i) {
    case 0:
      // Enable mirror filters.
      if (BcmWriteUint8(kPageManagementMode, 0x10, 0x70)) ++i;
      break;
    case 1:
      // Set ingress filter.
      if (BcmWriteUint32(kPageManagementMode, 0x20,
                         (1 << g_info->num_ports) - 1)) {
        ++i;
      }
      break;
    case 2:
      // Set egress filter.
      if (BcmWriteUint32(kPageManagementMode, 0x28,
                         (1 << g_info->num_ports) - 1)) {
        return true;
      }
      break;
    default:
      assert(false);
      return true;
  }
  return false;
}

static bool StateSetPortMask(FsmState *state) {
  // For each port, set the port forwarding mask to only A ports from an A
  // port, only B ports from a B port, or A and B ports from an A+B port.
  // Do not forward unicast packets to any port which is not in unicast_mask.
  // Disable learning unicast addresses for any port which is not in
  // unicast_mask.
  // See 5328XM-AN101-RDS.pdf section "Port Mask Table".
  static int32_t port = 0, i = 0;
  static bool mem_addr = true;
  if (state->first_entry) {
    port = 0;
    i = 0;
    mem_addr = true;
  }
  uint32_t mask_all = ((1 << g_info->num_ports) - 1);
  uint32_t drop_all = ~g_vlan_table.port_based_vlan[port] & mask_all;
  bool disable_learn =
      ((1 << port) & g_options->unicast_learn_mask_current) == 0;
  bool drop_vlan_non_member = true;
  uint32_t drop_dlf_unicast =
      (((1 << port) & g_options->unicast_flood_mask_current) == 0) ?
      mask_all : (~g_options->unicast_flood_mask_current & mask_all);
  uint32_t drop_dlf_multicast_layer3 = mask_all;
  uint32_t drop_dlf_multicast_layer2 = mask_all;
  uint32_t drop_broadcast = 0;
  // If the multicast table isn't provided, allow multicast traffic to route
  // only to or from the TMS570.
  if (g_mcast_table == NULL) {
    if (g_info->host_port == port) {
      drop_dlf_multicast_layer3 = 0;
      drop_dlf_multicast_layer2 = 0;
    } else {
      drop_dlf_multicast_layer3 = mask_all & ~(1 << g_info->host_port);
      drop_dlf_multicast_layer2 = mask_all & ~(1 << g_info->host_port);
    }
  }
  if (mem_addr) {
    if (BcmWriteUint8(kPageMemoryAccess, kMemRegIndex, kMemTablePortMask)) {
      mem_addr = false;
    }
  } else {
    switch (i) {
      case 0:
        if (BcmWriteUint16(kPageMemoryAccess, kMemRegAddr0, port)) ++i;
        break;
      case 1:
        if (BcmWriteUint64(kPageMemoryAccess, kMemRegData0, drop_all
                           | (((uint64_t)drop_dlf_unicast) << 29)
                           | (((uint64_t)drop_dlf_multicast_layer2) << 58))) {
          ++i;
        }
        break;
      case 2:
        if (BcmWriteUint64(
                kPageMemoryAccess, kMemRegData1,
                (((uint64_t)drop_dlf_multicast_layer2) >> (64 - 58))
                | (((uint64_t)drop_dlf_multicast_layer3) << (87 - 64))
                | (((uint64_t)drop_broadcast) << (116 - 64)))) ++i;
        break;
      case 3:
        if (BcmWriteUint32(
                kPageMemoryAccess, kMemRegData2,
                ((uint32_t)drop_broadcast) >> (128 - 116)
                | (drop_vlan_non_member ? 1UL << (149 - 128) : 0)
                | (disable_learn ? 1UL << (151 - 128) : 0))) {
          ++i;
        }
        break;
      case 4:
        if (BcmMemoryOp(kMemOpWrite)) ++i;
        break;
      case 5:
        if (BcmMemoryOpFinished()) ++i;
        break;
      default:
        i = 0;
        return ++port >= g_info->num_ports;
    }
  }
  return false;
}

static bool StateSetDefaultVlans(FsmState *state) {
  static int32_t port = 0;
  if (state->first_entry) {
    port = 0;
  }
  if (BcmWriteUint48(kPageDefaultQos, 0x06 * port,
                     ((uint64_t)g_vlan_table.default_vlan[port]) << 24
                     | (0x001 << 8) | (0x1 << 6))) {
    return ++port >= g_info->num_ports;
  }
  return false;
}

static bool StateDisablePorts(FsmState *state) {
  static int32_t i = 0;
  static int32_t port = 0;
  static uint8_t port_override;
  if (state->first_entry) {
    i = 0;
    port = 0;
  }
  // Don't allow disabling the host port.
  if (port == g_info->host_port) {
    ++port;
  }
  // Only disable POF ports.
  if (port >= g_info->num_fiber_ports) {
    return true;
  }
  bool disable_port = (g_options->port_disable_mask_current & (1 << port)) != 0;
  switch (i) {
    case 0:
      if (BcmReadUint8(kPageControl1, 0x10 + port, &port_override)) ++i;
      break;
    case 1:
      if (disable_port) {
        port_override = (port_override | 0x40) & ~0x01;
      } else {
        port_override = (port_override & ~0x40) | 0x01;
      }
      if (BcmWriteUint8(kPageControl1, 0x10 + port, port_override)) ++i;
      break;
    case 2:
      if (BcmWriteUint8(kPageControl1, 0x90 + port,
                        disable_port ? 0xC3 : 0xC0)) ++i;
      break;
    default:
      i = 0;
      ++port;
  }
  return false;
}

static bool StateFastAgeUnicast(FsmState *state) {
  // TODO: Technically we only need to age the ports which are being
  // disabled for unicast.  Consider tracking this in the future.
  static int32_t port = 0;
  static int32_t i = 0;
  uint8_t read_value;
  if (state->first_entry) {
    port = 0;
    i = 0;
  }
  switch (i) {
    case 0:
      // Set age out port.
      if (BcmWriteUint32(kPageArlControl, 0x0C, port)) ++i;
      break;
    case 1:
      // Enable fast aging by port, age out all dynamic entries.
      if (BcmWriteUint8(kPageArlControl, 0x0B, 0xE0)) ++i;
      break;
    case 2:
      // Wait for fast aging to complete.
      if (BcmReadUint8(kPageArlControl, 0x0B, &read_value)
          && ((read_value & 0x80) == 0)) ++i;
      break;
    default:
      i = 0;
      return ++port >= g_info->num_ports;
  }
  return false;
}

static bool StateSetMulticastRoutes(FsmState *state) {
  static bool multicast_first_entry = true;
  static int32_t i = 0;
  if (state->first_entry) {
    multicast_first_entry = true;
    i = 0;
  }
  if (g_mcast_table == NULL || i >= g_mcast_table->num_entries) {
    return true;
  }
  multicast_first_entry = SetMulticastRoute(multicast_first_entry,
                                            &g_mcast_table->entry[i]);
  if (multicast_first_entry) {
    ++i;
  }
  return false;
}

static bool StateAddMulticastRoutes(FsmState *state) {
  static bool multicast_first_entry = true;
  static int32_t i = 0;
  if (state->first_entry) {
    multicast_first_entry = true;
    i = 0;
  }
  if (g_mcast_table == NULL || i >= g_mcast_table->num_entries) {
    return true;
  }
  multicast_first_entry = AddMulticastRoute(multicast_first_entry,
                                            &g_mcast_table->entry[i]);
  if (multicast_first_entry) {
    ++i;
  }
  return false;
}

static bool StateAddStaticAddresses(FsmState *state) {
  static bool addr_first_entry = true;
  static int32_t i = 0;
  if (state->first_entry) {
    addr_first_entry = true;
    i = 0;
  }
  if (g_options->static_addresses == NULL
      || i >= g_options->num_static_addresses) {
    return true;
  }
  addr_first_entry = AddStaticAddress(addr_first_entry,
                                      &g_options->static_addresses[i]);
  if (addr_first_entry) {
    ++i;
  }
  return false;
}

static bool StateLedInit(FsmState *state) {
  static int32_t i = 0;
  if (state->first_entry) i = 0;
  switch (i) {
    case 0:
      // LED control register.
      if (BcmWriteUint16(kPageControl0, 0x5A, 0x0583)) ++i;
      break;
    case 1:
      // LED function 0 control register = LNK/ACT.
      if (BcmWriteUint16(kPageControl0, 0x5C, 0x0100)) ++i;
      break;
    case 2:
      // LED function 1 control register = 1G/ACT 10/100M/ACT.
      if (BcmWriteUint16(kPageControl0, 0x5E, 0x3000)) ++i;
      break;
    case 3:
      // LED function map register.
      if (BcmWriteUint32(kPageControl0, 0x60, 0x1E000000)) ++i;
      break;
    case 4:
      // LED enable map register.
      if (BcmWriteUint32(kPageControl0, 0x68, 0x06FFFFFF)) ++i;
      break;
    case 5:
      // LED mode map 0.
      if (BcmWriteUint32(kPageControl0, 0x70, 0x1FFFFFFF)) ++i;
      break;
    case 6:
      // LED mode map 1.
      if (BcmWriteUint32(kPageControl0, 0x78, 0x1FFFFFFF)) return true;
      break;
    default:
      assert(false);
      return true;
  }
  return false;
}

// Starting the switch strapped with Hardware Forwarding Disabled
// also initializes the Spanning Tree Protocol to the "Disable State" in
// which frames are not forwarded (see page 76 of 5328XM-DS303-R).
// Set STP to forwarding state on all ports to the "Forwarding State".
// The port control registers are at page 0x01 from 0x90 to 0xAA
// for FE ports 0-23, the MII port for the TMS570 and GE ports 0 and 1.
// See datasheet 5328XM-DS303-R pages 284, 286 for bit field definitions.
static bool StateTrafficControl(FsmState *state) {
  static int32_t port = 0;
  if (state->first_entry) port = 0;
  if (SetPortForwarding(port, true)) port++;
  return port >= NUM_SWITCH_PORTS_BCM53284;
}

static bool StateSetUnicastAgingTimer(FsmState *state) {
  (void)state;
  // Set aging timeout to 5 seconds.
  return BcmWriteUint32(kPageManagementMode, 0x04, 0x05);
}

static bool StateEnableForwarding(FsmState *state) {
  (void)state;
  return BcmWriteUint8(kPageControl0, 0x00, 0x36);
}

static bool StateReadMibCounter(FsmState *state) {
  (void)state;
  static int32_t port_selected = -1;
  int32_t port = g_cmd.read_port;
  if (port_selected != port) {
    if (BcmWriteUint8(kPageMibControl, 0x00, (uint8_t)port)) {
      port_selected = port;
    }
  } else {
    switch (g_cmd.read_statistic) {
      case kPortStatisticRxAlignmentError:
        return BcmReadUint32(kPageMibRx, 0x34,
                             &g_cmd.read_counters[port].rx_alignment_errors);
      case kPortStatisticRxDiscard:
        return BcmReadUint32(kPageMibRx, 0x60,
                             &g_cmd.read_counters[port].rx_route_discard);
      case kPortStatisticRxDroppedPackets:
        // Not implemented - No equivalent counter found.
        return true;
      case kPortStatisticRxFcsError:
        return BcmReadUint32(kPageMibRx, 0x38,
                             &g_cmd.read_counters[port].rx_fcs_errors);
      case kPortStatisticRxFragmentError:
        return BcmReadUint32(kPageMibRx, 0x54,
                             &g_cmd.read_counters[port].rx_fragment_errors);
      case kPortStatisticRxGoodOctets:
        return BcmReadUint64(kPageMibRx, 0x3C,
                             &g_cmd.read_counters[port].rx_good_octets);
      case kPortStatisticRxInRangeError:
        return BcmReadUint32(kPageMibRx, 0x64,
                             &g_cmd.read_counters[port].rx_in_range_errors);
      case kPortStatisticRxJabberError:
        return BcmReadUint32(kPageMibRx, 0x30,
                             &g_cmd.read_counters[port].rx_jabber_errors);
      case kPortStatisticRxMulticastPackets:
        return BcmReadUint32(kPageMibRx, 0x4C,
                             &g_cmd.read_counters[port].rx_multicast_packets);
      case kPortStatisticRxOctets:
        return BcmReadUint64(kPageMibRx, 0x1C,
                             &g_cmd.read_counters[port].rx_octets);
      case kPortStatisticRxPausePackets:
        return BcmReadUint32(kPageMibRx, 0x28,
                             &g_cmd.read_counters[port].rx_pause_packets);
      case kPortStatisticRxSymbolError:
        return BcmReadUint32(kPageMibRx, 0x5C,
                             &g_cmd.read_counters[port].rx_symbol_errors);
      case kPortStatisticTxDroppedPackets:
        return BcmReadUint32(kPageMibTx, 0x50,
                             &g_cmd.read_counters[port].tx_dropped_packets);
      case kPortStatisticTxMulticastPackets:
        return BcmReadUint32(kPageMibTx, 0x30,
                             &g_cmd.read_counters[port].tx_multicast_packets);
      case kPortStatisticTxOctets:
        return BcmReadUint64(kPageMibTx, 0x1C,
                             &g_cmd.read_counters[port].tx_octets);
      case kPortStatisticTxPausePackets:
        return BcmReadUint32(kPageMibTx, 0x28,
                             &g_cmd.read_counters[port].tx_pause_packets);
      default:
        assert(false);
        break;
    }
  }
  return false;
}

static bool StateMirrorEnable(FsmState *state) {
  (void)state;
  uint32_t mirror = g_options->mirror_enable ? 1 << g_info->mirror_port : 0;
  return BcmWriteUint32(kPageManagementMode, 0x18, mirror);
}

static bool StateReadLinkStatus(FsmState *state) {
  (void)state;
  uint32_t temp;
  if (!BcmReadUint32(kPageStatus, 0x10, &temp)) {
    return false;
  }
  g_cmd.link_status_bits = temp & 0x1FFFFFFF;
  return true;
}

static bool StateReadFinished(FsmState *state) {
  (void)state;
  g_cmd.read_finished = true;
  return true;
}

static bool StateDumpRoutes(FsmState *state) {
  static enum {
    kReadStateInit,
    kReadStateTableRead,
    kReadStateTableReadRun,
    kReadStateTableReadWait,
    kReadStateTableReadEntryLow,
    kReadStateTableReadEntryHigh,
    kReadStateTableReadMacLow,
    kReadStateOutput,
  } read_state = kReadStateInit;
  static int32_t scan_block;
  static int32_t block_index;
  static int32_t bin_index;
  static uint64_t arl_entry_low;
  static uint8_t arl_entry_high;
  static uint16_t arl_entry_mac_low;
  if (state->first_entry) {
    read_state = kReadStateInit;
    scan_block = 0;
    block_index = 0;
    bin_index = 0;
    arl_entry_low = 0;
    arl_entry_high = 0;
    arl_entry_mac_low = 0;
  }
  // TODO: Figure out how to fix valid scan to make this operation
  // much faster.
  switch (read_state) {
    case kReadStateInit:
      // Set memory index to ARL table.
      if (BcmWriteUint8(kPageMemoryAccess, kMemRegIndex, kMemTableArl)) {
        read_state = kReadStateTableRead;
        block_index = 0;
      }
      break;
    case kReadStateTableRead:
      // Find next valid block index.
      if (block_index >= 64) {
        scan_block += 1;
        block_index = 0;
        if (scan_block >= 64) {
          g_cmd.dump_routes_finished = true;
          return true;
        }
        break;
      }
      // Set memory address to ARL address.
      if (BcmWriteUint16(kPageMemoryAccess, kMemRegAddr0,
                         scan_block * 64 + block_index)) {
        read_state = kReadStateTableReadRun;
      }
      break;
    case kReadStateTableReadRun:
      // Read contents of table at address.
      if (BcmMemoryOp(kMemOpRead)) read_state = kReadStateTableReadWait;
      break;
    case kReadStateTableReadWait:
      if (BcmMemoryOpFinished()) {
        bin_index = 0;
        read_state = kReadStateTableReadEntryLow;
      }
      break;
    case kReadStateTableReadEntryLow:
      if (BcmReadUint64(kPageMemoryAccess, kMemRegData0 + 0x10 * bin_index,
                        &arl_entry_low)) {
        read_state = kReadStateTableReadEntryHigh;
      }
      break;
    case kReadStateTableReadEntryHigh:
      if (BcmReadUint8(kPageMemoryAccess, kMemRegData1 + 0x10 * bin_index,
                       &arl_entry_high)) {
        read_state = kReadStateTableReadMacLow;
      }
      break;
    case kReadStateTableReadMacLow:
      if (BcmReadUint16(kPageMemoryAccess, kMemRegKey2 + 0x08 * bin_index,
                        &arl_entry_mac_low)) {
        read_state = kReadStateOutput;
      }
      break;
    case kReadStateOutput:
      if ((arl_entry_high & 0x3) == 0x3) {
        g_cmd.dump_routes_entry.static_entry = (arl_entry_low >> 60) & 1;
        g_cmd.dump_routes_entry.vlan_id = (arl_entry_low >> 48) & 0xFFF;
        g_cmd.dump_routes_entry.ethernet_address.a = (uint8_t)(
            arl_entry_low >> 28);
        g_cmd.dump_routes_entry.ethernet_address.b = (uint8_t)(
            arl_entry_low >> 20);
        g_cmd.dump_routes_entry.ethernet_address.c = (uint8_t)(
            arl_entry_low >> 12);
        g_cmd.dump_routes_entry.ethernet_address.d = (uint8_t)(
            arl_entry_low >> 4);
        g_cmd.dump_routes_entry.ethernet_address.e =
            (uint8_t)((arl_entry_low << 4) | ((arl_entry_mac_low >> 8) & 0x0F));
        g_cmd.dump_routes_entry.ethernet_address.f = (uint8_t)(
            arl_entry_mac_low);
        // TODO: Add aging counter.
        // TODO: Add multicast route dump support.
        // For now, send the value out of our multicast table instead.
        if (g_cmd.dump_routes_entry.ethernet_address.a & 1) {
          int32_t group_id = (arl_entry_low >> 36) & 0xFFF;
          g_cmd.dump_routes_entry.port_map =
              g_mcast_table->entry[group_id].forward_map;
        } else {
          g_cmd.dump_routes_entry.port_map = (arl_entry_low >> 36) & 0x1F;
        }
        g_cmd.dump_routes_entry.valid = (arl_entry_high & 0x3) == 0x3;
        g_cmd.dump_routes_new_entry = true;
      }
      if (++bin_index >= 4) {
        ++block_index;
        read_state = kReadStateTableRead;
      } else {
        read_state = kReadStateTableReadEntryLow;
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
  STATE_TRANS_DELAY(kStateResetDelay, kStateResetDone, 50e3),
  STATE_TRANS_DEFAULT(kStateResetDone, StateResetDone, kStateBcm5482SInit),
  STATE_TRANS_DEFAULT(kStateBcm5482SInit, StateBcm5482SInit,
                      kStateDisableForwarding),
  STATE_TRANS_DEFAULT(kStateDisableForwarding, StateDisableForwarding,
                      kStateInitDelay),
  // TODO: Figure out how to manage or get rid of this delay.
  STATE_TRANS_DELAY(kStateInitDelay, kStatePortRvMii, 500e3),
  STATE_TRANS_DEFAULT(kStatePortRvMii, StatePortRvMii, kStateFiberMode),
  STATE_TRANS_DEFAULT(kStateFiberMode, StateFiberMode, kStateRadioMode),
  STATE_TRANS_DEFAULT(kStateRadioMode, StateRadioMode, kStateEnableVlans),
  STATE_TRANS_DEFAULT(kStateEnableVlans, StateEnableVlans, kStateWriteVlans),
  STATE_TRANS_DEFAULT(kStateWriteVlans, StateWriteVlans, kStateSetPortMask),
  STATE_TRANS_DEFAULT(kStateSetPortMask, StateSetPortMask,
                      kStateSetMulticastRoutes),
  STATE_TRANS_DEFAULT(kStateSetMulticastRoutes, StateSetMulticastRoutes,
                      kStateAddMulticastRoutes),
  STATE_TRANS_DEFAULT(kStateAddMulticastRoutes, StateAddMulticastRoutes,
                      kStateAddStaticAddresses),
  STATE_TRANS_DEFAULT(kStateAddStaticAddresses, StateAddStaticAddresses,
                      kStateSetMirror),
  STATE_TRANS_DEFAULT(kStateSetMirror, StateSetMirror, kStateLedInit),
  STATE_TRANS_DEFAULT(kStateLedInit, StateLedInit, kStateTrafficControl),
  STATE_TRANS_DEFAULT(kStateTrafficControl, StateTrafficControl,
                      kStateSetUnicastAgingTimer),
  STATE_TRANS_DEFAULT(kStateSetUnicastAgingTimer, StateSetUnicastAgingTimer,
                      kStateEnableForwarding),
  STATE_TRANS_DEFAULT(kStateEnableForwarding, StateEnableForwarding,
                      kStateStartupDelay),
  // TODO: Figure out how to manage or get rid of this delay.
  STATE_TRANS_DELAY(kStateStartupDelay, kStateReady, 3e6),
  STATE_TRANS_IDLE(kStateReady),
  STATE_TRANS_DEFAULT(kStateUpdateMulticastRoutes, StateSetMulticastRoutes,
                      kStateUpdatePortMask),
  STATE_TRANS_DEFAULT(kStateUpdatePortMask, StateSetPortMask,
                      kStateSetDefaultVlans),
  STATE_TRANS_DEFAULT(kStateSetDefaultVlans, StateSetDefaultVlans,
                      kStateDisablePorts),
  STATE_TRANS_DEFAULT(kStateDisablePorts, StateDisablePorts,
                      kStateFastAgeUnicast),
  STATE_TRANS_DEFAULT(kStateFastAgeUnicast, StateFastAgeUnicast,
                      kStateMirrorEnable),
  STATE_TRANS_DEFAULT(kStateMirrorEnable, StateMirrorEnable, kStateReady),
  STATE_TRANS_DEFAULT(kStateReadLinkStatus, StateReadLinkStatus,
                      kStateCheckDownPorts),
  STATE_TRANS_DEFAULT(kStateCheckDownPorts, StateCheckDownPorts,
                      kStateReadFinished),
  STATE_TRANS_DEFAULT(kStateReadMibCounter, StateReadMibCounter,
                      kStateReadFinished),
  STATE_TRANS_DEFAULT(kStateReadFinished, StateReadFinished, kStateReady),
  STATE_TRANS_DEFAULT(kStateDumpRoutes, StateDumpRoutes, kStateReady),
};

static const StateMachine kFsm = {
  .num_states = ARRAYSIZE(kFsmTrans),
  .init_state = kStateInit,
  .states = kFsmTrans
};

static FsmState g_state;

void Bcm53284Init(bool reset) {
  memset(&g_cmd, 0, sizeof(g_cmd));
  g_perform_reset = reset;
  StateMachineInit(&kFsm, &g_state);
}

void Bcm53284Poll(const SwitchConfig *config) {
  if (config == NULL) {
    g_info = &kDefaultSwitchInfo;
    g_mcast_table = NULL;
    g_options = &kDefaultSwitchOptions;
  } else {
    assert(config->options != NULL);
    g_info = config->info;
    g_mcast_table = config->mcast_table;
    g_options = config->options;
  }
  if (Bcm53284Ready()) {
    if (g_cmd.reconfigure_options) {
      g_cmd.reconfigure_options = false;
      SetState(kStateUpdateMulticastRoutes, &g_state);
    } else if (g_cmd.read_link_status) {
      g_cmd.read_link_status = false;
      SetState(kStateReadLinkStatus, &g_state);
    } else if (g_cmd.read_mib_counter) {
      g_cmd.read_mib_counter = false;
      SetState(kStateReadMibCounter, &g_state);
    } else if (g_cmd.dump_routes) {
      g_cmd.dump_routes = false;
      SetState(kStateDumpRoutes, &g_state);
    }
  }
  StateMachinePoll(&kFsm, &g_state);
  BcmPoll();
}

bool Bcm53284Ready(void) {
  return GetState(&g_state) == kStateReady;
}

bool Bcm53284ReadMibCounter(PortStatistic statistic, int32_t port,
                            MibCounts *core_switch_counters) {
  assert(0 <= port && port < g_info->num_ports);
  assert(core_switch_counters != NULL);

  if (g_cmd.read_finished) {
    g_cmd.read_finished = false;
    return true;
  } else if (Bcm53284Ready()) {
    g_cmd.read_mib_counter = true;
    g_cmd.read_statistic = statistic;
    g_cmd.read_port = port;
    g_cmd.read_counters = core_switch_counters;
  }
  return false;
}

void Bcm53284ReconfigureOptions(void) {
  g_cmd.reconfigure_options = true;
}

bool Bcm53284ReadLinkStatus(uint32_t *link_status_bits,
                            uint32_t *down_ports_changed,
                            uint32_t *down_port_events) {
  assert(link_status_bits != NULL);
  assert(down_ports_changed != NULL);

  if (g_cmd.read_finished) {
    g_cmd.read_finished = false;
    *link_status_bits = g_cmd.link_status_bits;
    *down_ports_changed = g_cmd.down_ports_changed;
    *down_port_events = g_cmd.down_port_events;
    return true;
  } else if (Bcm53284Ready()) {
    g_cmd.read_link_status = true;
  }
  return false;
}

const AddressRouteEntry *Bcm53284DumpRoutes(bool *finished) {
  *finished = g_cmd.dump_routes_finished;
  if (!g_cmd.dump_routes_finished && GetState(&g_state) != kStateDumpRoutes) {
    g_cmd.dump_routes = true;
  } else {
    g_cmd.dump_routes_finished = false;
    if (g_cmd.dump_routes_new_entry) {
      g_cmd.dump_routes_new_entry = false;
      return &g_cmd.dump_routes_entry;
    }
  }
  return NULL;
}

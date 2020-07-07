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

#ifndef AVIONICS_NETWORK_SWITCH_TYPES_H_
#define AVIONICS_NETWORK_SWITCH_TYPES_H_

#include <stdbool.h>
#include <stdint.h>

#include "avionics/common/network_addresses.h"
#include "avionics/common/network_config.h"
#include "avionics/network/aio_node.h"
#include "avionics/network/eop_message_type.h"
#include "avionics/network/message_type.h"
#include "avionics/network/switch_def.h"
#include "avionics/network/winch_message_type.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  kSwitchTypeUnknown,
  kSwitchTypeBcm53101,
  kSwitchTypeBcm53284,
  kNumSwitchTypes,
} SwitchType;

typedef struct {
  uint16_t vlan_id;
  uint32_t forward_map;
  uint32_t untag_map;
  bool vlan_forward_mode;
} VlanEntry;

typedef struct {
  int32_t num_vlans;
  VlanEntry vlan[NUM_SWITCH_PORTS_MAX + 2];
  uint16_t default_vlan[NUM_SWITCH_PORTS_MAX];
  uint32_t port_based_vlan[NUM_SWITCH_PORTS_MAX];
} VlanTable;

typedef struct {
  const EthernetAddress mac;
  const uint32_t retain_mask;
} TrunkMulticastOverride;

typedef struct {
  const uint32_t trunk_mask;
  const uint32_t select_default_mask;
  const uint32_t unicast_learning_mask;
  const int16_t num_multicast_overrides;
  const TrunkMulticastOverride *multicast_overrides;
} TrunkInfo;

typedef struct {
  const SwitchType type;
  const int16_t num_ports;
  const int16_t num_fiber_ports;
  const int16_t host_port;
  const int16_t mirror_port;
  const uint32_t forward_mask_a;
  const uint32_t forward_mask_b;
  const uint32_t forward_mask_c;
  const uint32_t egress_mask_c;
  const uint32_t isolate_mask;
  const uint32_t unicast_mask;
  const uint16_t *segment_vlans;
  const TrunkInfo trunk;
} SwitchInfo;

typedef struct {
  int16_t port;
  EthernetAddress mac;
} StaticAddress;

typedef struct {
  uint32_t unicast_flood_mask_current;
  uint32_t unicast_learn_mask_current;
  uint32_t port_disable_mask_current;
  bool mirror_enable;
  int16_t num_static_addresses;
  StaticAddress *static_addresses;
} SwitchOptions;

typedef struct {
  EthernetAddress mac;
  uint16_t group_id;
  uint32_t forward_map;
} MulticastEntry;

// TODO: Generate from network.yaml.
#define MAX_MULTICAST_ENTRIES (kNumMessageTypes         \
                               + kNumEopMessageTypes    \
                               + kNumWinchMessageTypes)

typedef struct {
  int32_t num_entries;
  MulticastEntry entry[MAX_MULTICAST_ENTRIES];
} MulticastTable;

typedef struct {
  const SwitchInfo *info;
  const SwitchOptions *options;
  const MulticastTable *mcast_table;
} SwitchConfig;

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // AVIONICS_NETWORK_SWITCH_TYPES_H_

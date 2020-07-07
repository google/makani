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

#include "avionics/network/switch_config.h"

#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "avionics/network/switch_types.h"

#define VLAN_A_B 1
#define VLAN_C 3

void GenerateVlanTable(const SwitchInfo *info, VlanTable *table) {
  assert(info != NULL && table != NULL);
  int32_t index = 0;

  // Generate port-based VLANs.
  for (int32_t port = 0; port < info->num_ports; port++) {
    uint32_t mask = 0;
    if (((1U << port) & info->forward_mask_a) != 0) {
      mask |= info->forward_mask_a;
    }
    if (((1U << port) & info->forward_mask_b) != 0) {
      mask |= info->forward_mask_b;
    }
    if (((1U << port) & info->forward_mask_c) != 0) {
      mask |= info->forward_mask_c;
    }
    if (((1U << port) & info->isolate_mask) != 0) {
      mask &= ~info->isolate_mask;
      mask |= 1U << port;
    }
    table->port_based_vlan[port] = mask;
  }

  // Generate default VLAN for untagged traffic.
  table->vlan[index].vlan_id = VLAN_A_B;
  table->vlan[index].forward_map =
      info->forward_mask_a | info->forward_mask_b;
  table->vlan[index].untag_map =
      info->forward_mask_a | info->forward_mask_b;
  table->vlan[index].vlan_forward_mode = false;
  for (int32_t port = 0; port < info->num_ports; port++) {
    table->default_vlan[port] = VLAN_A_B;
  }
  index++;

  // Generate network C VLAN.
  table->vlan[index].vlan_id = VLAN_C;
  table->vlan[index].forward_map = info->forward_mask_c;
  table->vlan[index].untag_map = info->egress_mask_c;
  table->vlan[index].vlan_forward_mode = false;
  for (int32_t port = 0; port < info->num_ports; port++) {
    if (info->egress_mask_c & (1U << port)) {
      table->default_vlan[port] = VLAN_C;
    }
  }
  index++;

  // Generate segment VLANs.
  if (info->segment_vlans != NULL) {
    for (int32_t port = 0; port < info->num_ports; port++) {
      if (info->segment_vlans[port] == 0) continue;
      table->vlan[index].vlan_id = info->segment_vlans[port];
      table->vlan[index].forward_map =
          (1U << port) | (1U << info->host_port);
      table->vlan[index].untag_map = 0;
      table->vlan[index].vlan_forward_mode = true;
      index++;
    }
  }

  table->num_vlans = index;
}

void GetActivePortList(const SwitchInfo *info, int32_t *active_ports,
                       int32_t *number_of_ports) {
  assert(info != NULL && active_ports != NULL && number_of_ports != NULL);
  uint32_t mask = info->forward_mask_a | info->forward_mask_b
      | info->unicast_mask;
  for (int32_t i = 0; i < info->num_ports; i++) {
    if (((1U << i) & mask) != 0) {
      active_ports[*number_of_ports] = i;
      (*number_of_ports)++;
    }
  }
}

void GetDefaultSwitchOptions(const SwitchInfo *info,
                             SwitchOptions *switch_options) {
  assert(info != NULL && switch_options != NULL);
  memset(switch_options, 0, sizeof(*switch_options));
  switch_options->mirror_enable = false;
  switch_options->port_disable_mask_current = ((1U << info->num_ports) - 1)
      & ~(info->forward_mask_a | info->forward_mask_b | info->unicast_mask);
  // Disable unicast learning on trunk ports.
  switch_options->unicast_flood_mask_current = info->unicast_mask;
  switch_options->unicast_learn_mask_current =
      (~info->trunk.trunk_mask & info->unicast_mask) |
      info->trunk.unicast_learning_mask;
  switch_options->num_static_addresses = 0;
}

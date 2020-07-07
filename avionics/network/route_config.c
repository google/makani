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

#include "avionics/network/route_config.h"

#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "avionics/network/routes.h"
#include "avionics/network/switch_config.h"
#include "avionics/network/switch_info.h"
#include "common/macros.h"

static bool MacEqual(const EthernetAddress *a, const EthernetAddress *b) {
  return a->a == b->a && a->b == b->b && a->c == b->c && a->d == b->d
      && a->e == b->e && a->f == b->f;
}

static uint32_t GetOverrideMask(const SwitchInfo *switch_info,
                                const EthernetAddress *mac) {
  int32_t num_overrides = switch_info->trunk.num_multicast_overrides;
  const TrunkMulticastOverride *overrides =
      switch_info->trunk.multicast_overrides;
  uint32_t trunk_mask = switch_info->trunk.trunk_mask;
  uint32_t override_mask = trunk_mask;
  if (num_overrides > 0) {
    assert(overrides != NULL);
    for (int32_t j = 0; j < num_overrides; j++) {
      if (MacEqual(&overrides[j].mac, mac)) {
        override_mask = overrides[j].retain_mask;
        break;
      }
    }
  }
  return override_mask | ~trunk_mask;
}

typedef const uint32_t * (* const MessageTypeForwardMapFunction)(
    AioNode node, int32_t *size);
typedef EthernetAddress (* const MessageTypeToMulticastFunction)(int32_t type);

static EthernetAddress AioMessageTypeToMulticastAddress(int32_t type) {
  return AioMessageTypeToEthernetAddress((MessageType)type);
}

static EthernetAddress EopMessageTypeToMulticastAddress(int32_t type) {
  return EopMessageTypeToEthernetAddress((EopMessageType)type);
}

static EthernetAddress WinchMessageTypeToMulticastAddress(int32_t type) {
  return WinchMessageTypeToEthernetAddress((WinchMessageType)type);
}

static void AppendMessageRoutes(
    AioNode node,
    MessageTypeForwardMapFunction get_forward_map,
    MessageTypeToMulticastFunction type_to_mcast_address,
    MulticastTable *table) {
  int32_t size;
  const uint32_t *forward_map = get_forward_map(node, &size);
  const SwitchInfo *switch_info = GetSwitchInfo(node);

  // TODO: The get_forward_map functions should set size to 0 in the
  // case of a null table.  Currently they don't but this check will prevent
  // garbage route data from being set.
  if (forward_map == NULL) return;

  for (int32_t i = 0; i < size; ++i) {
    if (forward_map[i] == 0) continue;
    assert(table->num_entries < ARRAYSIZE(table->entry));
    EthernetAddress mac = type_to_mcast_address(i);
    uint32_t override_mask = GetOverrideMask(switch_info, &mac);
    table->entry[table->num_entries] = (MulticastEntry) {
      .forward_map = forward_map[i] & override_mask,
      .mac = mac,
      .group_id = (uint16_t)table->num_entries};
    ++table->num_entries;
  }
}

void GenerateMulticastRoutingTable(AioNode node,
                                   MulticastTable *table) {
  // Note: Multicast table group_id is expected to remain stable across updates.
  table->num_entries = 0;
  AppendMessageRoutes(node, AioMessageForwardingMap,
                      AioMessageTypeToMulticastAddress, table);
  AppendMessageRoutes(node, EopMessageForwardingMap,
                      EopMessageTypeToMulticastAddress, table);
  AppendMessageRoutes(node, WinchMessageForwardingMap,
                      WinchMessageTypeToMulticastAddress, table);
}

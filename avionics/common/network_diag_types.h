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

#ifndef AVIONICS_COMMON_NETWORK_DIAG_TYPES_H_
#define AVIONICS_COMMON_NETWORK_DIAG_TYPES_H_

#include <stdbool.h>
#include <stdint.h>

#include "avionics/common/network_addresses.h"

typedef struct {
  uint16_t vlan_id;
  EthernetAddress ethernet_address;
  bool valid;
  bool static_entry;
  bool age;
  int8_t priority;
  int8_t arl_con;
  uint32_t port_map;
} AddressRouteEntry;

#endif  // AVIONICS_COMMON_NETWORK_DIAG_TYPES_H_

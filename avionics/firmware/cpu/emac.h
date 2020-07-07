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

// Asynchronous EMAC driver.

#ifndef AVIONICS_FIRMWARE_CPU_EMAC_H_
#define AVIONICS_FIRMWARE_CPU_EMAC_H_

#include <stdbool.h>
#include <stdint.h>

#include "avionics/common/network_config.h"

// Ethernet frame sizes.
#define MIN_ETHERNET_LENGTH 60
#define MAX_ETHERNET_LENGTH 1514
// Maximum size of a 802.1Q tagged ethernet frame.
#define MAX_ETHERNET_LENGTH_VLAN 1518

// Initializes the EMAC with the given MAC address.
void EmacInit(EthernetAddress mac);

// Sends an Ethernet frame. Returns true on success.
bool EmacSend(const void *buf, int32_t len);

// Receives an Ethernet frame. Returns true on success. len is an in/out
// parameter: the caller supplies the length in bytes of buf and it returns the
// number of bytes written to buf.
bool EmacReceive(void *buf, int32_t *len);

#endif  // AVIONICS_FIRMWARE_CPU_EMAC_H_

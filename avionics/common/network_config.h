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

#ifndef AVIONICS_COMMON_NETWORK_CONFIG_H_
#define AVIONICS_COMMON_NETWORK_CONFIG_H_

#include <stdint.h>

#include "avionics/common/network_addresses.h"
#include "avionics/network/aio_node.h"
#include "avionics/network/eop_message_type.h"
#include "avionics/network/message_type.h"
#include "avionics/network/switch_def.h"
#include "avionics/network/winch_message_type.h"

#ifdef __cplusplus
extern "C" {
#endif

// The wing network configuration is summarized as follows:
//
// - Each AIO node is given a fixed IP address (see AioNodeToIpAddress).
// - Each IP address is given a fixed Ethernet (MAC) address (see
//       IpAddressToEthernetAddress).
// - Each MessageType is given a fixed multicast address (see
//       AioMessageTypeToIpAddress).
//
// The IP and MAC addresses assigned to AIO nodes are used for unicast
// traffic such as bootloading and ping requests.  They should not be
// used for in-flight messages.
//
// The fact that the network is dual redundant (consisting of the A
// and B networks) is transparent to the end nodes, EXCEPT that
// unicast traffic is restricted to the A network.
//
// Multicast messages on the wing are forward by each core switch to
// fixed links based on their multicast IP address.

// UDP ports.
#define UDP_PORT_AIO                40000
#define UDP_PORT_AIO_TUNNEL         40010
#define UDP_PORT_BOOTLOADER         40667
#define UDP_PORT_BOOTLOADER_REQUEST 40668
#define UDP_PORT_WINCH              40670
#define UDP_PORT_EOP                40700
#define UDP_PORT_NET_PROBE          41337

#define NUM_CORE_SWITCH_PORTS       NUM_SWITCH_PORTS_BCM53284
#define NUM_ACCESS_SWITCH_PORTS     NUM_SWITCH_PORTS_BCM53101

// Maximum size of a non-fragmented UDP frame.
#define MAX_UDP_PAYLOAD_SIZE 1472

// Returns multicast route bitmask for a given MessageType.
uint32_t MessageForwardingMap(AioNode node, MessageType type);

// Produces the multicast IP address for a given message type.
IpAddress AioMessageTypeToIpAddress(MessageType type);

// Produces the multicast MAC address for a given message type.
EthernetAddress AioMessageTypeToEthernetAddress(MessageType type);

// Produces the multicast IP address for a given winch message type.
IpAddress WinchMessageTypeToIpAddress(WinchMessageType type);

// Produces the multicast IP address for a given Eop message type.
IpAddress EopMessageTypeToIpAddress(EopMessageType type);

// Produces the multicast MAC address for a given winch message type.
EthernetAddress WinchMessageTypeToEthernetAddress(WinchMessageType type);

// Produces the multicast MAC address for a given Eop message type.
EthernetAddress EopMessageTypeToEthernetAddress(EopMessageType type);

// Produces the MAC address for a given IP address on the wing network.
// WARNING: This function should only be used by generate_image and identity
// for its self test.  Nodes should not send unprompted unicast traffic.  If
// responding to an incoming message, use the sender's MAC for the reply.
// Return value is in network byte order.
EthernetAddress IpAddressToEthernetAddress(IpAddress ip);

// Converts an IP address in to a uint32_t in the host byte order.
uint32_t IpAddressToUint32(IpAddress ip);

// Converts a uint32_t in host byte order to an IP address.
IpAddress Uint32ToIpAddress(uint32_t u32);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // AVIONICS_COMMON_NETWORK_CONFIG_H_

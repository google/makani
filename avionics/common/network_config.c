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

#include "avionics/common/network_config.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

#include "avionics/network/aio_node.h"
#include "avionics/network/message_type.h"

// Returns the multicast IP address for a given MessageType.
IpAddress AioMessageTypeToIpAddress(MessageType type) {
  return (IpAddress) {239, 0, 0, (uint8_t)type};  // NOLINT(readability/braces)
}

// Produces the multicast MAC address for a given message type.
EthernetAddress AioMessageTypeToEthernetAddress(MessageType type) {
  IpAddress ip = AioMessageTypeToIpAddress(type);
  return IpAddressToEthernetAddress(ip);
}

// Returns the multicast IP address for a given winch message type.
IpAddress WinchMessageTypeToIpAddress(WinchMessageType type) {
  return (IpAddress) {239, 0, 1, (uint8_t)type};  // NOLINT(readability/braces)
}

// Returns the multicast IP address for a given Eop message type.
IpAddress EopMessageTypeToIpAddress(EopMessageType type) {
  return (IpAddress) {239, 0, 2, (uint8_t)type};  // NOLINT(readability/braces)
}

// Produces the multicast MAC address for a given winch message type.
EthernetAddress WinchMessageTypeToEthernetAddress(WinchMessageType type) {
  IpAddress ip = WinchMessageTypeToIpAddress(type);
  return IpAddressToEthernetAddress(ip);
}

// Produces the multicast MAC address for a given Eop message type.
EthernetAddress EopMessageTypeToEthernetAddress(EopMessageType type) {
  IpAddress ip = EopMessageTypeToIpAddress(type);
  return IpAddressToEthernetAddress(ip);
}

// Returns the MAC address for a given IP address according to the static
// address mapping.
EthernetAddress IpAddressToEthernetAddress(IpAddress ip) {
  // Multicast address.
  if ((ip.a & 0xF0) == 0xE0) {
    // NOLINTNEXTLINE(readability/braces)
    return (EthernetAddress) {0x01, 0x00, 0x5E, ip.b & 0x7F, ip.c, ip.d};
  }

  // Unicast address.
  if (ip.a == 192 && ip.b == 168 && ip.c == 1) {
    // Special case for broadcast address.
    if (ip.d == 255) {
      // NOLINTNEXTLINE(readability/braces)
      return (EthernetAddress) {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    }
    // NOLINTNEXTLINE(readability/braces)
    return (EthernetAddress) {0x02, 0x00, 0x00, 0x00, 0x00, ip.d};
  }

  assert(!(bool)"Invalid IP.");
  // NOLINTNEXTLINE(readability/braces)
  return (EthernetAddress) {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
}

uint32_t IpAddressToUint32(IpAddress ip) {
  return (uint32_t)(ip.a << 24U | ip.b << 16U | ip.c << 8U | ip.d);
}

IpAddress Uint32ToIpAddress(uint32_t u32) {
  IpAddress ip = {
    ip.a = (uint8_t)(u32 >> 24U),
    ip.b = (uint8_t)(u32 >> 16U),
    ip.c = (uint8_t)(u32 >> 8U),
    ip.d = (uint8_t)(u32)
  };
  return ip;
}

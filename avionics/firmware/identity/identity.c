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

#include "avionics/firmware/identity/identity.h"

#include <assert.h>
#include <stdint.h>
#include <string.h>

#include "avionics/firmware/identity/identity_types.h"
#include "avionics/firmware/startup/ldscript.h"
#include "avionics/network/aio_node.h"
#include "avionics/network/aio_node_to_ip_address.h"

#define LDSCRIPT_BOOT_BLOCK_VERSION_FLAG_MASK 0xFFFF0000
#define LDSCRIPT_BOOT_BLOCK_VERSION_FLAG 0xDE100000
#define LDSCRIPT_BOOT_BLOCK_VERSION 1

HardwareType BootConfigGetHardwareType(void) {
  // Do not call assert here; it uses this function.
  return LdscriptReadInt16(&ldscript_boot_config.hardware_type);
}

IpAddress BootConfigGetIpAddress(void) {
  if (BootConfigIsValid()) {
    IpAddress address = {
      .a = LdscriptReadUint8(&ldscript_boot_config.ip_address[0]),
      .b = LdscriptReadUint8(&ldscript_boot_config.ip_address[1]),
      .c = LdscriptReadUint8(&ldscript_boot_config.ip_address[2]),
      .d = LdscriptReadUint8(&ldscript_boot_config.ip_address[3])
    };
    return address;
  } else {
    return AioNodeToIpAddress(kAioNodeUnknown);
  }
}

EthernetAddress BootConfigGetMacAddress(void) {
  if (BootConfigIsValid()) {
    EthernetAddress address = {
      .a = LdscriptReadUint8(&ldscript_boot_config.mac_address[0]),
      .b = LdscriptReadUint8(&ldscript_boot_config.mac_address[1]),
      .c = LdscriptReadUint8(&ldscript_boot_config.mac_address[2]),
      .d = LdscriptReadUint8(&ldscript_boot_config.mac_address[3]),
      .e = LdscriptReadUint8(&ldscript_boot_config.mac_address[4]),
      .f = LdscriptReadUint8(&ldscript_boot_config.mac_address[5])
    };
    return address;
  } else {
    return IpAddressToEthernetAddress(BootConfigGetIpAddress());
  }
}

bool BootConfigIsValid(void) {
  int32_t version = LdscriptReadInt32(&ldscript_boot_config.version);

  // TODO: Remove ugly transition hack.
  if ((version & LDSCRIPT_BOOT_BLOCK_VERSION_FLAG_MASK) !=
      LDSCRIPT_BOOT_BLOCK_VERSION_FLAG) {
    return false;
  }
  // So far we only need to support a single version.
  version = version & ~LDSCRIPT_BOOT_BLOCK_VERSION_FLAG_MASK;
  return version == LDSCRIPT_BOOT_BLOCK_VERSION;
}

int32_t AppConfigGetIndex(void) {
  return LdscriptReadInt32(&ldscript_app_config.node_index);
}

AioNode AppConfigGetAioNode(void) {
  return (AioNode)LdscriptReadInt32(&ldscript_app_config.aio_node);
}

bool AppConfigIsAioNodeValid(void) {
  AioNode node = AppConfigGetAioNode();
  IpAddress boot_ip = BootConfigGetIpAddress();
  IpAddress node_ip = AioNodeToIpAddress(node);
  EthernetAddress boot_mac = BootConfigGetMacAddress();
  EthernetAddress node_mac = IpAddressToEthernetAddress(node_ip);
  return BootConfigIsValid()
      && node != kAioNodeUnknown
      && BootConfigGetHardwareType() != kHardwareTypeUnknown
      && memcmp(&boot_ip, &node_ip, sizeof(boot_ip)) == 0
      && memcmp(&boot_mac, &node_mac, sizeof(boot_mac)) == 0;
}

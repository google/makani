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

#include "avionics/cs/firmware/eop_interface.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "avionics/common/avionics_messages.h"
#include "avionics/common/eop_version.h"
#include "avionics/common/network_config.h"
#include "avionics/common/pack_avionics_messages.h"
#include "avionics/common/pack_eop_messages.h"
#include "avionics/firmware/identity/identity.h"
#include "avionics/firmware/network/net.h"
#include "avionics/network/aio_node.h"
#include "avionics/network/aio_node_to_ip_address.h"
#include "avionics/network/eop_message_type.h"
#include "avionics/network/message_type.h"

// Avoid placing large data structures on the stack.
static EopModemStatusMessage g_in;
static EopSlowStatusMessage g_out;

static AioNode GetEopSourceNode(void) {
  AioNode me = AppConfigGetAioNode();
  if (me == kAioNodeCsB) {
    return kAioNodeEopWingB;
  } else if (me == kAioNodeCsGsB) {
    return kAioNodeEopGsB;
  } else {
    return kAioNodeUnknown;
  }
}

static bool IsEopSourceIp(const IpAddress *source_ip) {
  AioNode eop_node = GetEopSourceNode();
  IpAddress eop_ip = AioNodeToIpAddress(eop_node);
  return eop_node != kAioNodeUnknown
      && memcmp(&eop_ip, source_ip, sizeof(eop_ip)) == 0;
}

static bool IsEopMessage(const IpAddress *source_ip, uint16_t source_port,
                         uint16_t dest_port, int32_t length,
                         const uint8_t *data, EopHeader *header) {
  return source_port == UDP_PORT_EOP
      && dest_port == UDP_PORT_EOP
      && IsEopSourceIp(source_ip)
      && length >= PACK_EOPHEADER_SIZE
      && UnpackEopHeader(data, 1, header) == PACK_EOPHEADER_SIZE
      && header->type >= 0
      && header->type < kNumEopMessageTypes
      && header->version == EOP_VERSION;
}

static bool IsEopModemStatusMessage(const EopHeader *header, int32_t length) {
  return header->type == kEopMessageTypeEopModemStatus
      && length == PACK_EOPMODEMSTATUSMESSAGE_SIZE;
}

static void HandleEopModemStatus(const EopHeader *header, const uint8_t *data) {
  UnpackEopModemStatusMessage(data, 1, &g_in);
  g_out = (EopSlowStatusMessage) {
    .modem = g_in
  };
  // Use EopMessage's sequence number.
  NetSendSpoofedAio(
      AppConfigGetAioNode(), kMessageTypeEopSlowStatus, header->sequence,
      (PackAioMessageFunction)PackEopSlowStatusMessage, &g_out);
}

bool HandleEopMessage(const IpAddress *source_ip, uint16_t source_port,
                      uint16_t dest_port, int32_t length, const uint8_t *data) {
  EopHeader header;
  if (IsEopMessage(source_ip, source_port, dest_port, length, data, &header)) {
    int32_t payload_length = length - PACK_EOPHEADER_SIZE;
    const uint8_t *payload_data = &data[PACK_EOPHEADER_SIZE];

    if (IsEopModemStatusMessage(&header, payload_length)) {
      HandleEopModemStatus(&header, payload_data);
    }
    return true;
  }
  return false;
}

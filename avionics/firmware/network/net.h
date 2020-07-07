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

// Network stack.

#ifndef AVIONICS_FIRMWARE_NETWORK_NET_H_
#define AVIONICS_FIRMWARE_NETWORK_NET_H_

#include <stdbool.h>
#include <stddef.h>  // For size_t.
#include <stdint.h>

#include "avionics/common/avionics_messages.h"
#include "avionics/common/network_config.h"
#include "avionics/common/pack_aio_header.h"
#include "avionics/network/aio_node.h"
#include "avionics/network/message_type.h"

// Maximum size of a non-fragmented AIO payload.
#define MAX_AIO_PAYLOAD_SIZE (MAX_UDP_PAYLOAD_SIZE - PACK_AIOHEADER_SIZE)

// Definition
typedef struct {
  bool drop_eligible;
  uint16_t vlan_id;
  uint8_t priority;
} VlanTag;

// Initializes the network stack for a given node.
void NetInit(AioNode node);

// Handles received frames. If a UDP frame is received but not handled and
// buffer is not NULL, the frame's payload is copied into buffer, truncating if
// necessary, and returning the size of the UDP payload before any truncation.
// source_ip is set to the sender IP address, and source_port and
// destination_port are set to the source and destination ports respectively.
// Each of the output parameters is optional, except that buffer may not be NULL
// if buffer_len is > 0.  However, if buffer is NULL, no other parameter will be
// filled in.
int32_t NetPollUdp(int32_t buffer_len, void *buffer, IpAddress *source_ip,
                   EthernetAddress *source_mac, uint16_t *source_port,
                   uint16_t *destination_port);

// Equivalent to NetPollUDP(0, NULL, NULL, NULL, NULL, NULL).
void NetPoll(void);

// Sends a UDP frame to a given address and port. Returns true on success.
bool NetSendUdp(IpAddress address, EthernetAddress mac, uint16_t source_port,
                uint16_t destination_port, const void *buffer, int32_t len,
                const VlanTag *vlan);

// Sends an AIO frame of a given message type to all subscribed nodes. Returns
// true on success.
bool NetSendAio(MessageType type, const void *buffer, int32_t len);

// General pack function prototype for pack_avionics_messages.h.
typedef size_t (* const PackAioMessageFunction)(const void *msg, size_t num,
                                                uint8_t *out);

// Packs and sends an AIO message to all subscribed nodes. The caller should use
// PackAioMessageFunction to cast a pack function from pack_avionics_messages.h.
// Returns true on success.
bool NetSendAioPacked(MessageType type, PackAioMessageFunction pack_func,
                      const void *msg);

// Packs and sends an AIO message as if it were sent by 'node' with sequence
// number 'sequence'. The caller should use PackAioMessageFunction to cast a
// pack function from pack_avionics_messages.h. Returns true on success.
bool NetSendSpoofedAio(AioNode node, MessageType type, uint16_t sequence,
                       PackAioMessageFunction pack_func, const void *msg);

// Get AIO statistics since last call.
void AioGetStats(AioStats *stats);

#endif  // AVIONICS_FIRMWARE_NETWORK_NET_H_

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

#include "avionics/firmware/network/net.h"

#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "avionics/common/aio_header.h"
#include "avionics/common/aio_version_code.h"
#include "avionics/common/cvt.h"
#include "avionics/common/endian.h"
#include "avionics/common/network_config.h"
#include "avionics/firmware/cpu/clock.h"
#include "avionics/firmware/cpu/emac.h"
#include "avionics/firmware/cpu/memcpy.h"
#include "avionics/firmware/cpu/registers.h"
#include "avionics/firmware/identity/identity.h"
#include "avionics/firmware/network/net_aio.h"
#include "avionics/firmware/network/net_probe.h"
#include "avionics/network/aio_node_to_ip_address.h"

static int32_t g_received_network_packets = 0;
static int32_t g_received_valid_aio_packets = 0;
static int32_t g_received_invalid_aio_packets = 0;
static int32_t g_received_probe_packets = 0;
static int32_t g_received_arp_request_packets = 0;
static int32_t g_received_icmp_request_packets = 0;
static int32_t g_sent_aio_packets = 0;

// Ethernet definitions.

#define ETHERTYPE_ARP 0x0806
#define ETHERTYPE_IP 0x0800
#define ETHERTYPE_VLAN 0x8100

typedef struct {
  EthernetAddress destination_mac;
  EthernetAddress source_mac;
  uint16_t        type;
} __attribute__((packed)) EthernetHeader;

#define MIN_PAYLOAD_LENGTH (MIN_ETHERNET_LENGTH - sizeof(EthernetHeader))

// 802.1Q VLAN definitions.

typedef struct {
  unsigned int priority_code_point:3;
  unsigned int drop_eligibility_indicator:1;
  unsigned int vlan_id:12;
  uint16_t     inner_type;
} __attribute__((packed)) VlanHeader;

#define MIN_PAYLOAD_LENGTH_VLAN (MIN_PAYLOAD_LENGTH - sizeof(VlanHeader))

// IP definitions.

#define IP_PROTOCOL_ICMP 1
#define IP_PROTOCOL_UDP  17

typedef struct {
  unsigned int version:4;
  unsigned int ihl:4;
  uint8_t      type_of_service;
  uint16_t     total_length;
  uint16_t     identification;
  unsigned int flags:3;
  unsigned int fragment_offset:13;
  uint8_t      time_to_live;
  uint8_t      protocol;
  uint16_t     header_checksum;
  IpAddress    source_ip;
  IpAddress    destination_ip;
} __attribute__((packed)) IpHeader;

// ICMP definitions.

#define ICMP_TYPE_ECHO_REQUEST 8
#define ICMP_TYPE_ECHO_REPLY   0

typedef struct {
  uint8_t  type;
  uint8_t  code;
  uint16_t checksum;
  uint16_t identifier;
  uint16_t sequence_number;
} __attribute__((packed)) IcmpHeader;

// UDP definitions.

typedef struct {
  uint16_t source_port;
  uint16_t destination_port;
  uint16_t length;
  uint16_t checksum;
} __attribute__((packed)) UdpHeader;

#define BOOTLOADER_RESET_CODE 0xDE10FE1F
typedef struct {
  uint32_t code;
} __attribute__((packed)) BootloaderResetMessage;

// ARP definitions.

#define ARP_HARDWARE_TYPE_ETHERNET 1
#define ARP_PROTOCOL_TYPE_IP 0x0800

#define ARP_OPERATION_REQUEST 1
#define ARP_OPERATION_REPLY 2

typedef struct {
  uint16_t hardware_type;
  uint16_t protocol_type;
  uint8_t hardware_address_length;
  uint8_t protocol_address_length;
  uint16_t operation_code;
  EthernetAddress source_hardware_address;
  IpAddress source_protocol_address;
  EthernetAddress target_hardware_address;
  IpAddress target_protocol_address;
} __attribute__((packed)) ArpMessage;

// Generic Ethernet payload format.
typedef union {
  uint8_t                  raw[MAX_ETHERNET_LENGTH - sizeof(EthernetHeader)];
  struct {
    IpHeader               ip_header;
    uint8_t                ip_payload[MAX_ETHERNET_LENGTH
                                      - sizeof(EthernetHeader)
                                      - sizeof(IpHeader)];
  };
  struct {
    IpHeader               __icmp_ip_header_padding;
    IcmpHeader             icmp_header;
    uint8_t                icmp_payload[MAX_ETHERNET_LENGTH
                                        - sizeof(EthernetHeader)
                                        - sizeof(IpHeader)
                                        - sizeof(IcmpHeader)];
  };
  struct {
    IpHeader               __udp_ip_header_padding;
    UdpHeader              udp_header;
    uint8_t                udp_payload[MAX_ETHERNET_LENGTH
                                       - sizeof(EthernetHeader)
                                       - sizeof(IpHeader)
                                       - sizeof(UdpHeader)];
  };
  struct {
    IpHeader               __aio_ip_header_padding;
    UdpHeader              __aio_udp_header_padding;
    AioHeader              aio_header;
    uint8_t                aio_payload[MAX_ETHERNET_LENGTH
                                       - sizeof(EthernetHeader)
                                       - sizeof(IpHeader)
                                       - sizeof(UdpHeader)
                                       - sizeof(AioHeader)];
  };
  struct {
    IpHeader               __bootloader_reset_ip_header_padding;
    UdpHeader              __bootloader_reset_udp_header_padding;
    BootloaderResetMessage bootloader_reset_message;
  };
  ArpMessage               arp_message;
} __attribute__((packed)) EthernetPayload;

// Generic Ethernet frame format.
typedef union {
  uint8_t                  raw[MAX_ETHERNET_LENGTH_VLAN];
  struct {
    EthernetHeader         ethernet_header;
    EthernetPayload        ethernet_payload;
  };
  struct {
    EthernetHeader         __vlan_ethernet_header_padding;
    VlanHeader             vlan_header;
    EthernetPayload        tagged_ethernet_payload;
  };
} __attribute__((packed)) Frame;

// Context for returning UDP data.
typedef struct {
  void *buffer;
  int32_t buffer_len;
  int32_t *data_len;
  IpAddress *source_ip;
  EthernetAddress *source_mac;
  uint16_t *source_port;
  uint16_t *destination_port;
} UdpContext;

// Source addresses.
static AioNode g_node;
static IpAddress g_source_ip;
static EthernetAddress g_source_mac;

// Frame buffer used for transmits and receives.
static Frame g_send, g_receive;

// Returns true if the given MAC address matches our host.
static bool IsMacForUs(EthernetAddress mac) {
  // AIO multicast address.
  EthernetAddress aio_base = AioMessageTypeToEthernetAddress(0);
  if (!memcmp(&aio_base, &mac, sizeof(EthernetAddress) - 1)) {
    return true;
  }

  // Winch multicast address.
  EthernetAddress winch_base = WinchMessageTypeToEthernetAddress(0);
  if (!memcmp(&winch_base, &mac, sizeof(EthernetAddress) - 1)) {
    return true;
  }

  // Eop multicast address.
  EthernetAddress eop_base = EopMessageTypeToEthernetAddress(0);
  if (!memcmp(&eop_base, &mac, sizeof(EthernetAddress) - 1)) {
    return true;
  }

  // Unicast address.
  if (!memcmp(&g_source_mac, &mac, sizeof(EthernetAddress))) {
    return true;
  }

  // Broadcast address.
  EthernetAddress broadcast = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  if (!memcmp(&broadcast, &mac, sizeof(EthernetAddress))) {
    return true;
  }

  return false;
}

// Returns true if the given IP address matches our host.
static bool IsIpForUs(IpAddress ip) {
  // AIO multicast address.
  IpAddress aio_base = AioMessageTypeToIpAddress(0);
  if (!memcmp(&aio_base, &ip, sizeof(IpAddress) - 1)) {
    return true;
  }

  // Winch multicast address.
  IpAddress winch_base = WinchMessageTypeToIpAddress(0);
  if (!memcmp(&winch_base, &ip, sizeof(IpAddress) - 1)) {
    return true;
  }

  // Eop multicast address.
  IpAddress eop_base = EopMessageTypeToIpAddress(0);
  if (!memcmp(&eop_base, &ip, sizeof(IpAddress) - 1)) {
    return true;
  }

  // Unicast address.
  if (!memcmp(&g_source_ip, &ip, sizeof(IpAddress))) {
    return true;
  }

  // Broadcast address.
  IpAddress global_bcast = {255, 255, 255, 255};
  IpAddress local_bcast = {192, 168, 1, 255};
  if (!memcmp(&global_bcast, &ip, sizeof(IpAddress))
      || !memcmp(&local_bcast, &ip, sizeof(IpAddress))) {
    return true;
  }

  return false;
}

// Computes the Internet checksum of a buffer and writes it to 'out'. 'out' is
// zeroed before computation.  'out' is typically a pointer to data inside the
// buffer, thus the zeroing is essential to correctly compute the checksum.
static void InternetChecksum(uint16_t *out, const void *buffer, int32_t len) {
  const uint8_t *start = buffer;
  const uint8_t *end = start + len;
  uint32_t checksum = 0;

  *out = 0;

  if (((uintptr_t)start & 1) && (start < end)) {
    checksum += *start++;
  }

  if (((uintptr_t)end & 1) && (start < end)) {
    checksum += *--end << 8;
  }

  while (start < end) {
    uint16_t x;
    memcpy(&x, start, sizeof(uint16_t));
    checksum += x;
    start += sizeof(uint16_t);
  }

  while (checksum >> 16) {
    checksum = (checksum & 0xFFFF) + (checksum >> 16);
  }

  *out = ~checksum;
}

// Fills an Ethernet header with the given values.
static void FillEthernetHeader(EthernetAddress destination_mac,
                               uint16_t type, EthernetHeader *h) {
  h->destination_mac = destination_mac;
  h->source_mac      = g_source_mac;
  h->type            = type;
}

// Fills a VLAN header with the given values.
static void FillVlanHeader(const VlanTag *tag, uint16_t inner_type,
                           VlanHeader *h) {
  h->priority_code_point = tag->priority;
  h->drop_eligibility_indicator = tag->drop_eligible ? 1 : 0;
  h->vlan_id = tag->vlan_id;
  h->inner_type = inner_type;
}

// Fills an IP header with the given values.
static void FillIpHeader(uint16_t total_length, uint8_t protocol,
                         IpAddress destination_ip, IpHeader *h) {
  h->version         = 4;
  h->ihl             = sizeof(IpHeader) / sizeof(uint32_t);
  h->type_of_service = 0;
  h->total_length    = total_length;
  h->identification  = 0;
  h->flags           = 1 << 1;  // Don't Fragment.
  h->fragment_offset = 0;
  h->time_to_live    = 8;
  h->protocol        = protocol;
  h->source_ip       = g_source_ip;
  h->destination_ip  = destination_ip;
  InternetChecksum(&h->header_checksum, h, sizeof(IpHeader));
}

// Fills a UDP header with the given values.
static void FillUdpHeader(uint16_t source_port, uint16_t destination_port,
                          uint16_t length, UdpHeader *h) {
  h->source_port      = source_port;
  h->destination_port = destination_port;
  h->length           = length;
  h->checksum         = 0;
}

// Fills an AIO header with the given values.
static void FillAioHeader(AioNode source, MessageType type, uint16_t sequence,
                          AioHeader *h) {
  h->version = AIO_VERSION;
  h->source = source;
  h->type = type;
  h->sequence = sequence;
  h->timestamp = (uint32_t)ClockGetUs();
}

// Handles an AIO frame. If the frame is valid, the data is inserted into the
// CVT.
static bool HandleAio(const EthernetPayload *p, int32_t len) {
  // Invalid length.
  if (len < (int32_t)offsetof(EthernetPayload, aio_payload)) {
    ++g_received_invalid_aio_packets;
    return false;
  }

  // Invalid version.
  if (p->aio_header.version != AIO_VERSION) {
    ++g_received_invalid_aio_packets;
    return false;
  }

  // Total received valid AIO packets. CvtStats tracks the number of packets
  // put into the CVT, so we do not need to test the return code of CvtPut().
  ++g_received_valid_aio_packets;

  HandleAioFrame(p->aio_header.source,
                 p->aio_header.type,
                 p->aio_payload,
                 p->udp_header.length - sizeof(UdpHeader) - sizeof(AioHeader),
                 p->aio_header.sequence,
                 ClockGetUs());

  return true;
}

// Handles a request to reboot into the bootloader.
static bool HandleBootloaderRequest(const EthernetPayload *p, int32_t len) {
  // Invalid length.
  if (len < (int32_t)(offsetof(EthernetPayload, bootloader_reset_message) +
                      sizeof(BootloaderResetMessage))) {
    return false;
  }

  uint32_t code = 0;
  ReadUint32Be(&p->bootloader_reset_message.code, &code);
  if (code != BOOTLOADER_RESET_CODE) {
    return false;
  }

  SYS.ECR.RESET = 2;  // Software reset.

  return true;
}

// Unpack a VLAN header into a VlanTag structure.
static void UnpackVlanHeader(const VlanHeader *h, VlanTag *tag) {
  memset(tag, 0, sizeof(*tag));
  if (h != NULL) {
    tag->priority = h->priority_code_point;
    tag->drop_eligible = h->drop_eligibility_indicator != 0;
    tag->vlan_id = h->vlan_id;
  }
}

// Handles a net probe message.
static bool HandleNetProbe(const EthernetPayload *p, int32_t len,
                           const VlanHeader *vlan) {
  // TODO: Pass vlan info to net_probe module.
  VlanTag vlan_tag;
  UnpackVlanHeader(vlan, &vlan_tag);
  if (NetProbeReceive(len - offsetof(EthernetPayload, udp_payload),
                      p->udp_payload)) {
    ++g_received_probe_packets;
    return true;
  }
  return false;
}

// Handles a UDP frame.
static bool HandleUdp(const Frame *f, const EthernetPayload *p, int32_t len,
                      const UdpContext *udp, const VlanHeader *vlan) {
  // Invalid length.
  if (len < (int32_t)offsetof(EthernetPayload, udp_payload)) {
    return false;
  }

  // Length mismatch. Ethernet frames are padded to a minimum of 60 bytes. For
  // smaller frames, check that the received length is 60 bytes and that the
  // IP header and UDP header have consistent lengths.
  int32_t expected_length = offsetof(EthernetPayload, udp_header)
      + p->udp_header.length;
  int32_t min_length = (int32_t)(vlan != NULL ? MIN_PAYLOAD_LENGTH_VLAN :
                                 MIN_PAYLOAD_LENGTH);
  if (expected_length < min_length) {
    if (len != min_length) {
      return false;
    }
    if (p->udp_header.length != p->ip_header.total_length - sizeof(IpHeader)) {
      return false;
    }
  } else {
    if (len != expected_length) {
      return false;
    }
  }

  // Pass to upper layers.
  switch (p->udp_header.destination_port) {
    case UDP_PORT_AIO:
      return HandleAio(p, len);
    case UDP_PORT_BOOTLOADER_REQUEST:
      return HandleBootloaderRequest(p, len);
    case UDP_PORT_NET_PROBE:
      return HandleNetProbe(p, len, vlan);
    default:
      break;
  }

  // Pass to application.
  if (udp->buffer != NULL) {
    // Get length to copy.
    int32_t udp_length = p->udp_header.length - sizeof(UdpHeader);
    int32_t copy_length =
        udp->buffer_len < udp_length ? udp->buffer_len : udp_length;
    // Copy data.
    FastCopy(copy_length, p->udp_payload, udp->buffer);
    // Copy metadata.
    *udp->data_len = udp_length;
    if (udp->source_ip) {
      *udp->source_ip = p->ip_header.source_ip;
    }
    if (udp->source_mac) {
      *udp->source_mac = f->ethernet_header.source_mac;
    }
    if (udp->source_port) {
      *udp->source_port = p->udp_header.source_port;
    }
    if (udp->destination_port) {
      *udp->destination_port = p->udp_header.destination_port;
    }
    return true;
  }
  return false;
}

// Handles an ICMP frame. If the frame is an echo request, it is converted
// in-place into a reply and sent.
static bool HandleIcmp(int32_t len, EthernetPayload *p, Frame *f) {
  // Invalid length.
  if (len < (int32_t)offsetof(EthernetPayload, icmp_payload)) {
    return false;
  }

  // Not an echo request.
  if (p->icmp_header.type != ICMP_TYPE_ECHO_REQUEST) {
    return false;
  }

  ++g_received_icmp_request_packets;

  int32_t icmp_length     = p->ip_header.total_length - sizeof(IpHeader);
  int32_t ip_length       = sizeof(IpHeader)       + icmp_length;
  int32_t ethernet_length = sizeof(EthernetHeader) + ip_length;
  uint16_t outer_type;
  if (f->ethernet_header.type == ETHERTYPE_VLAN) {
    ethernet_length += sizeof(VlanHeader);
    outer_type = ETHERTYPE_VLAN;
  } else {
    outer_type = ETHERTYPE_IP;
  }

  p->icmp_header.type = ICMP_TYPE_ECHO_REPLY;
  p->icmp_header.code = 0;
  InternetChecksum(&p->icmp_header.checksum, &p->icmp_header, icmp_length);

  FillIpHeader(ip_length, IP_PROTOCOL_ICMP, p->ip_header.source_ip,
               &p->ip_header);

  // VLAN tag will remain unmodified.

  FillEthernetHeader(f->ethernet_header.source_mac, outer_type,
                     &f->ethernet_header);

  if (ethernet_length < MIN_ETHERNET_LENGTH) {
    ethernet_length = MIN_ETHERNET_LENGTH;
  }

  EmacSend(f, ethernet_length);

  return true;
}

// Handles an IP frame.
static bool HandleIp(const UdpContext *udp, const VlanHeader *vlan,
                     int32_t len, EthernetPayload *p, Frame *f) {
  // Invalid length.
  if (len < (int32_t)offsetof(EthernetPayload, ip_payload)) {
    return false;
  }

  // Not IPv4.
  if (p->ip_header.version != 4) {
    return false;
  }

  // Invalid header size.
  if (p->ip_header.ihl != sizeof(IpHeader) / sizeof(uint32_t)) {
    return false;
  }

  // Length mismatch. Ethernet frames are padded to a minimum of 60 bytes. For
  // smaller frames, check that the received length is 60 bytes.
  int32_t expected_length
      = offsetof(EthernetPayload, ip_header) + p->ip_header.total_length;
  int32_t min_length = (int32_t)(vlan != NULL ? MIN_PAYLOAD_LENGTH_VLAN :
                                 MIN_PAYLOAD_LENGTH);
  if (expected_length < min_length) {
    if (len != min_length) {
      return false;
    }
  } else {
    if (len != expected_length) {
      return false;
    }
  }

  // Fragmented.
  if (p->ip_header.flags & (1 << 0) || p->ip_header.fragment_offset != 0) {
    return false;
  }

  // Destination IP mismatch.
  if (!IsIpForUs(p->ip_header.destination_ip)) {
    return false;
  }

  // Pass to upper layers.
  switch (p->ip_header.protocol) {
    case IP_PROTOCOL_ICMP:
      return HandleIcmp(len, p, f);
    case IP_PROTOCOL_UDP:
      return HandleUdp(f, p, len, udp, vlan);
    default:
      return false;
  }
}

// Handles an ARP frame. If the frame is a request, it is converted
// in-place into a reply and sent.
static bool HandleArp(int32_t len, EthernetPayload *p, Frame *f) {
  // Invalid length.
  if (len < (int32_t)sizeof(ArpMessage)) {
    return false;
  }

  ArpMessage *arp = &p->arp_message;

  // Protocol and length validation.
  if (arp->hardware_type != ARP_HARDWARE_TYPE_ETHERNET ||
      arp->protocol_type != ARP_PROTOCOL_TYPE_IP ||
      arp->hardware_address_length != sizeof(EthernetAddress) ||
      arp->protocol_address_length != sizeof(IpAddress) ||
      arp->operation_code != ARP_OPERATION_REQUEST) {
    return false;
  }

  // Is the request for our unicast IP?
  if (memcmp(&g_source_ip, &arp->target_protocol_address, sizeof(IpAddress))) {
    return false;
  }

  ++g_received_arp_request_packets;

  int32_t ethernet_length = sizeof(EthernetHeader) + sizeof(ArpMessage);
  uint16_t outer_type;
  if (f->ethernet_header.type == ETHERTYPE_VLAN) {
    ethernet_length += sizeof(VlanHeader);
    outer_type = ETHERTYPE_VLAN;
  } else {
    outer_type = ETHERTYPE_ARP;
  }

  // Rewrite message as a reply.
  arp->operation_code = ARP_OPERATION_REPLY;
  arp->target_hardware_address = arp->source_hardware_address;
  arp->target_protocol_address = arp->source_protocol_address;
  arp->source_hardware_address = g_source_mac;
  arp->source_protocol_address = g_source_ip;

  // VLAN tag will remain unmodified.

  FillEthernetHeader(f->ethernet_header.source_mac, outer_type,
                     &f->ethernet_header);

  if (ethernet_length < MIN_ETHERNET_LENGTH) {
    ethernet_length = MIN_ETHERNET_LENGTH;
  }

  EmacSend(f, ethernet_length);

  return true;
}

// Handles a VLAN-tagged Ethernet frame.
static bool HandleVlan(const UdpContext *udp, int32_t len, Frame *f) {
  // Invalid length.
  if (len < (int32_t)offsetof(Frame, tagged_ethernet_payload)) {
    return false;
  }

  // TODO: Implement processing and rejection of messages based on VID.

  // Pass to upper layers.
  switch (f->vlan_header.inner_type) {
    case ETHERTYPE_ARP:
      return HandleArp(len - offsetof(Frame, tagged_ethernet_payload),
                       &f->tagged_ethernet_payload, f);
    case ETHERTYPE_IP:
      return HandleIp(udp, &f->vlan_header,
                      len - offsetof(Frame, tagged_ethernet_payload),
                      &f->tagged_ethernet_payload, f);
    case ETHERTYPE_VLAN:
      // Currently not supporting double-tagged frames.
      return false;
    default:
      return false;
  }
}

// Handles an Ethernet frame.
static bool HandleEthernet(const UdpContext *udp, int32_t len, Frame *f) {
  // Invalid length.
  if (len < (int32_t)offsetof(Frame, ethernet_payload)) {
    return false;
  }

  // Destination MAC mismatch.
  if (!IsMacForUs(f->ethernet_header.destination_mac)) {
    return false;
  }

  // Pass to upper layers.
  switch (f->ethernet_header.type) {
    case ETHERTYPE_ARP:
      return HandleArp(len - offsetof(Frame, ethernet_payload),
                       &f->ethernet_payload, f);
    case ETHERTYPE_IP:
      return HandleIp(udp, NULL, len - offsetof(Frame, ethernet_payload),
                      &f->ethernet_payload, f);
    case ETHERTYPE_VLAN:
      return HandleVlan(udp, len, f);
    default:
      return false;
  }
}

// Sends a UDP frame. Assumes the UDP payload has been filled.
static bool SendUdp(IpAddress ip, EthernetAddress mac, uint16_t source_port,
                    uint16_t destination_port, int32_t len, const VlanTag *vlan,
                    Frame *f) {
  int32_t udp_length      = sizeof(UdpHeader)      + len;
  int32_t ip_length       = sizeof(IpHeader)       + udp_length;
  int32_t ethernet_length = sizeof(EthernetHeader) + ip_length;

  uint16_t outer_type;
  EthernetPayload *payload;
  if (vlan != NULL) {
    ethernet_length += sizeof(VlanHeader);
    outer_type = ETHERTYPE_VLAN;
    payload = &f->tagged_ethernet_payload;
    FillVlanHeader(vlan, ETHERTYPE_IP, &f->vlan_header);
  } else {
    outer_type = ETHERTYPE_IP;
    payload = &f->ethernet_payload;
  }

  FillUdpHeader(source_port, destination_port, udp_length,
                &payload->udp_header);

  FillIpHeader(ip_length, IP_PROTOCOL_UDP, ip, &payload->ip_header);

  FillEthernetHeader(mac, outer_type, &f->ethernet_header);

  if (ethernet_length < MIN_ETHERNET_LENGTH) {
    ethernet_length = MIN_ETHERNET_LENGTH;
  }

  return EmacSend(f, ethernet_length);
}

// Sends an AIO frame. Assumes the AIO payload has been filled.
static bool SendAioHelper(AioNode node, MessageType type, uint16_t sequence,
                          int32_t len, Frame *f) {
  int32_t aio_length = sizeof(AioHeader) + len;

  FillAioHeader(node, type, sequence, &f->ethernet_payload.aio_header);

  bool sent = SendUdp(AioMessageTypeToIpAddress(type),
                      AioMessageTypeToEthernetAddress(type), UDP_PORT_AIO,
                      UDP_PORT_AIO, aio_length, NULL, f);
  g_sent_aio_packets += sent;
  return sent;
}

// Sends an AIO frame. Assumes the AIO payload has been filled.
static bool SendAio(MessageType type, int32_t len, Frame *f) {
  static uint16_t sequence_numbers[kNumMessageTypes] = {0U};
  return SendAioHelper(g_node, type, ++sequence_numbers[type], len, f);
}

void NetInit(AioNode node) {
  assert(0 <= node && node < kNumAioNodes);
  g_node       = node;
  g_source_ip  = BootConfigGetIpAddress();
  g_source_mac = BootConfigGetMacAddress();
  EmacInit(g_source_mac);
}

void NetPoll(void) {
  NetPollUdp(0, NULL, NULL, NULL, NULL, NULL);
}

int32_t NetPollUdp(int32_t buffer_len, void *buffer, IpAddress *source_ip,
                   EthernetAddress *source_mac, uint16_t *source_port,
                   uint16_t *destination_port) {
  if (buffer_len < 0 || ((buffer_len > 0) != (buffer != NULL))) {
    return 0;
  }

  // Get the next received message.
  int32_t received_length = sizeof(g_receive);
  if (!EmacReceive(&g_receive, &received_length)) {
    return 0;
  }

  // Pass message through protocol handlers.
  int32_t data_len = 0;
  const UdpContext udp = {
    buffer, buffer_len, &data_len, source_ip, source_mac, source_port,
    destination_port};
  HandleEthernet(&udp, received_length, &g_receive);
  ++g_received_network_packets;
  return data_len;
}

bool NetSendUdp(IpAddress address, EthernetAddress mac, uint16_t source_port,
                uint16_t destination_port, const void *buffer, int32_t len,
                const VlanTag *vlan) {
  assert(buffer != NULL);
  EthernetPayload *p = &g_send.ethernet_payload;
  if (vlan != NULL) {
    p = &g_send.tagged_ethernet_payload;
  }

  assert(0 <= len && len <= (int32_t)sizeof(p->udp_payload));

  FastCopy(len, buffer, p->udp_payload);
  return SendUdp(address, mac, source_port, destination_port, len, vlan,
                 &g_send);
}

bool NetSendAio(MessageType type, const void *buffer, int32_t len) {
  EthernetPayload *p = &g_send.ethernet_payload;
  assert(type < kNumMessageTypes);
  assert(buffer != NULL);
  assert(0 <= len && len <= (int32_t)sizeof(p->aio_payload));
  FastCopy(len, buffer, p->aio_payload);
  return SendAio(type, len, &g_send);
}

bool NetSendAioPacked(MessageType type, PackAioMessageFunction pack_func,
                      const void *msg) {
  EthernetPayload *p = &g_send.ethernet_payload;
  assert(type < kNumMessageTypes);
  assert(pack_func != NULL);
  assert(msg != NULL);
  int32_t len = (int32_t)pack_func(msg, 1, p->aio_payload);
  return SendAio(type, len, &g_send);
}

bool NetSendSpoofedAio(AioNode node, MessageType type, uint16_t sequence,
                       PackAioMessageFunction pack_func, const void *msg) {
  EthernetPayload *p = &g_send.ethernet_payload;
  assert(type < kNumMessageTypes);
  assert(pack_func != NULL);
  assert(msg != NULL);
  int32_t len = (int32_t)pack_func(msg, 1, p->aio_payload);
  return SendAioHelper(node, type, sequence, len, &g_send);
}

void AioGetStats(AioStats *stats) {
  int32_t received_aio_packets = g_received_valid_aio_packets
      + g_received_invalid_aio_packets;

  stats->received_valid_aio_packets = (int16_t)g_received_valid_aio_packets;
  stats->received_invalid_aio_packets = (int16_t)g_received_invalid_aio_packets;
  stats->received_probe_packets = (int16_t)g_received_probe_packets;
  stats->received_arp_request_packets =
      (int16_t)g_received_arp_request_packets;
  stats->received_icmp_request_packets =
      (int16_t)g_received_icmp_request_packets;
  stats->received_non_routine_packets = (int16_t)(g_received_network_packets
                                                  - received_aio_packets
                                                  - g_received_probe_packets);
  stats->sent_aio_packets = (int16_t)g_sent_aio_packets;

  g_received_network_packets = 0;
  g_received_valid_aio_packets = 0;
  g_received_invalid_aio_packets = 0;
  g_received_probe_packets = 0;
  g_received_arp_request_packets = 0;
  g_received_icmp_request_packets = 0;
  g_sent_aio_packets = 0;
}

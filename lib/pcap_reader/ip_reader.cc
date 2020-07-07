// Copyright 2020 Makani Technologies LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "lib/pcap_reader/ip_reader.h"

#include <netinet/if_ether.h>
#include <netinet/ip.h>
#include <pcap/pcap.h>
#include <pcap/sll.h>
#include <stdint.h>
#include <string.h>

#include <limits>

namespace lib {
namespace pcap_reader {

IpReader::IpReader(void)
    : ip_header_(), payload_len_(-1), payload_(nullptr), defrag_() {
  Reset();
}

IpStatus IpReader::Failure(IpStatus status) {
  payload_len_ = -1;
  return status;
}

void IpReader::Reset(void) {
  Failure(IpStatus::kBadIpHeader);
  defrag_.ClearPackets();
}

bool IpReader::HandleIpHeader(int32_t packet_len,
                              const uint8_t *packet,
                              struct iphdr *header) const {
  // Check packet length.
  if (packet_len < static_cast<int32_t>(sizeof(*header))) return false;
  memcpy(header, packet, sizeof(*header));

  // Check for IPv4.
  if (header->version != 4) return false;

  // Check header length.
  if (packet_len < header->ihl * 4) return false;

  // Compute checksum.
  uint32_t checksum = 0U;
  const uint16_t *raw = reinterpret_cast<const uint16_t *>(header);
  for (size_t i = 0; i < sizeof(*header) / sizeof(uint16_t); ++i) {
    checksum = static_cast<uint32_t>(checksum + raw[i]);
  }
  checksum = (checksum & 0xFFFF) + (checksum >> 16);

  // Validate IP header checksum.
  return static_cast<uint16_t>(~checksum) == 0x0;
}

IpStatus IpReader::HandleIpPacket(const timeval &ts, int32_t packet_len,
                                  const uint8_t *packet) {
  if (HandleIpHeader(packet_len, packet, &ip_header_)) {
    const uint8_t *fragment = &packet[ip_header_.ihl * 4];
    payload_len_ = defrag_.Defragment(ts, ip_header_, fragment, &payload_);
    return payload_len_ < 0 ? Failure(IpStatus::kFragmented) : IpStatus::kValid;
  }
  return Failure(IpStatus::kBadIpHeader);
}

int32_t IpReader::HandleEthernetHeader(int32_t packet_len,
                                       const uint8_t *packet,
                                       struct ethhdr *header,
                                       const uint8_t **payload) const {
  // Check packet length.
  int32_t header_len = static_cast<int32_t>(sizeof(*header));
  if (packet_len < header_len) return -1;
  memcpy(header, packet, sizeof(*header));

  // Check for 802.1Q header (used by probe packets).
  // See https://en.wikipedia.org/wiki/EtherType
  if (ntohs(header->h_proto) == ETHERTYPE_VLAN) {
    header_len += 4;  // { uint16_t vid; uint16_t h_proto; }
    if (packet_len < header_len) return -1;
    memcpy(&header->h_proto, &packet[header_len - 2], sizeof(header->h_proto));
  }

  // Valid Ethernet packet.
  *payload = &packet[header_len];
  return packet_len - header_len;
}

IpStatus IpReader::HandleEthernetPacket(const timeval &ts, int32_t packet_len,
                                        const uint8_t *packet) {
  struct ethhdr header;
  payload_len_ = HandleEthernetHeader(packet_len, packet, &header, &payload_);
  if (payload_len_ >= 0) {
    switch (ntohs(header.h_proto)) {
      case ETHERTYPE_IP:
        return HandleIpPacket(ts, payload_len_, payload_);
      default:
        return Failure(IpStatus::kBadEthernetProtocol);
    }
  }
  return Failure(IpStatus::kBadEthernetHeader);
}

int32_t IpReader::HandleSllHeader(int32_t packet_len, const uint8_t *packet,
                                  struct sll_header *header,
                                  const uint8_t **payload) const {
  // Check packet length.
  if (packet_len < static_cast<int32_t>(sizeof(*header))) return -1;
  memcpy(header, packet, sizeof(*header));

  // TODO: Evaluate sll_pkttype and sll_protocol.

  // Valid SLL packet.
  *payload = &packet[sizeof(*header)];
  return packet_len - static_cast<int32_t>(sizeof(*header));
}

IpStatus IpReader::HandleSllPacket(const timeval &ts, int32_t packet_len,
                                   const uint8_t *packet) {
  struct sll_header header;
  payload_len_ = HandleSllHeader(packet_len, packet, &header, &payload_);
  if (payload_len_ >= 0) {
    return HandleIpPacket(ts, payload_len_, payload_);
  }
  return Failure(IpStatus::kBadSllHeader);
}

int32_t IpReader::HandlePcapHeader(const pcap_pkthdr &header,
                                   const uint8_t *packet,
                                   const uint8_t **payload) const {
  if (header.caplen == header.len
      && header.caplen <= std::numeric_limits<int32_t>::max()) {
    *payload = packet;
    return header.len;
  }
  return -1;
}

IpStatus IpReader::HandlePcapPacket(const pcap_pkthdr &header,
                                    const uint8_t *packet, pcap_t *handle) {
  payload_len_ = HandlePcapHeader(header, packet, &payload_);
  if (payload_len_ >= 0) {
    switch (pcap_datalink(handle)) {
      case DLT_EN10MB:
        return HandleEthernetPacket(header.ts, payload_len_, payload_);
      case DLT_LINUX_SLL:
        return HandleSllPacket(header.ts, payload_len_, payload_);
      default:
        return Failure(IpStatus::kBadLinkProtocol);
    }
  }
  return Failure(IpStatus::kBadPcapHeader);
}

}  // namespace pcap_reader
}  // namespace lib

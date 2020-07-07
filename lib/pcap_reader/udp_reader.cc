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

#include "lib/pcap_reader/udp_reader.h"

#include <netinet/ip.h>
#include <netinet/udp.h>
#include <stdint.h>
#include <string.h>

#include "lib/pcap_reader/ip_reader.h"

namespace lib {
namespace pcap_reader {

UdpReader::UdpReader(void)
    : udp_header_(), payload_len_(-1), payload_(nullptr) {
  Reset();
}

UdpStatus UdpReader::Failure(UdpStatus status) {
  payload_len_ = -1;
  return status;
}

void UdpReader::Reset(void) {
  Failure(UdpStatus::kBadUdpHeader);
}

int32_t UdpReader::HandleUdpHeader(int32_t packet_len,
                                   const uint8_t *packet,
                                   struct udphdr *header,
                                   const uint8_t **payload) const {
  // Check packet length.
  if (packet_len < static_cast<int32_t>(sizeof(*header))) return -1;
  memcpy(header, packet, sizeof(*header));

  // Check header length.
  if (ntohs(header->len) != packet_len) return -1;

  // TODO: Verify CRC.

  // Valid UDP packet.
  *payload = &packet[sizeof(*header)];
  return packet_len - static_cast<int32_t>(sizeof(*header));
}

UdpStatus UdpReader::HandleUdpPacket(int32_t packet_len,
                                     const uint8_t *packet) {
  payload_len_ = HandleUdpHeader(packet_len, packet, &udp_header_, &payload_);
  return payload_len_ < 0 ? Failure(UdpStatus::kBadUdpHeader)
      : UdpStatus::kValid;
}

UdpStatus UdpReader::HandleIpPacket(const IpReader &reader) {
  payload_len_ = -1;
  if (reader.GetProtocol() == IPPROTO_UDP) {
    return HandleUdpPacket(reader.GetPayloadLength(), reader.GetPayload());
  }
  return Failure(UdpStatus::kBadIpProtocol);
}

}  // namespace pcap_reader
}  // namespace lib

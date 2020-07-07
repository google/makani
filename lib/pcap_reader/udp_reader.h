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

#ifndef LIB_PCAP_READER_UDP_READER_H_
#define LIB_PCAP_READER_UDP_READER_H_

#include <netinet/udp.h>
#include <stdint.h>

#include "lib/pcap_reader/ip_reader.h"

namespace lib {
namespace pcap_reader {

// Enum to interpret reason for processing failure.
enum class UdpStatus {
  kBadIpProtocol,
  kBadUdpHeader,
  kValid,
};

// Class to read and validate UDP network packets.
class UdpReader {
 public:
  UdpReader(void);
  ~UdpReader(void) {}

  // Reset stored state. Call this function before parsing unrelated files to
  // clear the internal state.
  void Reset(void);

  UdpStatus HandleUdpPacket(int32_t packet_len, const uint8_t *packet);
  UdpStatus HandleIpPacket(const IpReader &reader);

  // UDP header accessor functions.
  const struct udphdr &GetUdpHeader(void) const { return udp_header_; }
  uint16_t GetDestination(void) const { return ntohs(udp_header_.dest); }

  // UDP payload accessor functions.
  int32_t GetPayloadLength(void) const { return payload_len_; }
  const uint8_t *GetPayload(void) const {
    return payload_len_ < 0 ? nullptr : payload_;
  }

 private:
  UdpStatus Failure(UdpStatus status);
  int32_t HandleUdpHeader(int32_t packet_len, const uint8_t *packet,
                          struct udphdr *header, const uint8_t **payload) const;

  struct udphdr udp_header_;
  int32_t payload_len_;
  const uint8_t *payload_;

  DISALLOW_COPY_AND_ASSIGN(UdpReader);
};

}  // namespace pcap_reader
}  // namespace lib

#endif  // LIB_PCAP_READER_UDP_READER_H_

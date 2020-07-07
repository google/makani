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

#ifndef LIB_PCAP_READER_IP_READER_H_
#define LIB_PCAP_READER_IP_READER_H_

#include <netinet/if_ether.h>
#include <netinet/ip.h>
#include <pcap/pcap.h>
#include <pcap/sll.h>
#include <stdint.h>

#include "lib/pcap_reader/ip_defragmenter.h"

namespace lib {
namespace pcap_reader {

// Enum to interpret reason for processing failure.
enum class IpStatus {
  kBadEthernetHeader,
    kBadEthernetProtocol,
    kBadIpHeader,
    kBadLinkProtocol,
    kBadPcapHeader,
    kBadSllHeader,
    kFragmented,
    kValid,
    };

// Class to read and validate IP network packets.
class IpReader {
 public:
  IpReader(void);
  ~IpReader(void) {}

  // Reset stored state. Call this function before parsing unrelated files to
  // clear the internal state.
  void Reset(void);

  IpStatus HandleIpPacket(const timeval &ts, int32_t packet_len,
                          const uint8_t *packet);
  IpStatus HandleEthernetPacket(const timeval &ts, int32_t packet_len,
                                const uint8_t *packet);
  IpStatus HandleSllPacket(const timeval &ts, int32_t packet_len,
                           const uint8_t *packet);
  IpStatus HandlePcapPacket(const pcap_pkthdr &header, const uint8_t *packet,
                            pcap_t *handle);

  // IP header accessor functions.
  const struct iphdr &GetIpHeader(void) const { return ip_header_; }
  int8_t GetProtocol(void) const { return ip_header_.protocol; }
  uint32_t GetSource(void) const { return ntohl(ip_header_.saddr); }
  uint32_t GetDestination(void) const { return ntohl(ip_header_.daddr); }

  // IP payload accessor functions.
  int32_t GetPayloadLength(void) const { return payload_len_; }
  const uint8_t *GetPayload(void) const {
    return payload_len_ < 0 ? nullptr : payload_;
  }

 private:
  IpStatus Failure(IpStatus status);
  bool HandleIpHeader(int32_t packet_len, const uint8_t *packet,
                      struct iphdr *header) const;
  int32_t HandleEthernetHeader(int32_t packet_len, const uint8_t *packet,
                               struct ethhdr *header,
                               const uint8_t **payload) const;
  int32_t HandleSllHeader(int32_t packet_len, const uint8_t *packet,
                          struct sll_header *header,
                          const uint8_t **payload) const;
  int32_t HandlePcapHeader(const pcap_pkthdr &header, const uint8_t *packet,
                           const uint8_t **payload) const;

  struct iphdr ip_header_;
  int32_t payload_len_;
  const uint8_t *payload_;
  IpDefragmenter defrag_;

  DISALLOW_COPY_AND_ASSIGN(IpReader);
};

}  // namespace pcap_reader
}  // namespace lib

#endif  // LIB_PCAP_READER_IP_READER_H_

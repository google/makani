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

#ifndef LIB_PCAP_READER_IP_DEFRAGMENTER_H_
#define LIB_PCAP_READER_IP_DEFRAGMENTER_H_

#include <netinet/ip.h>  // For iphdr.
#include <stdint.h>
#include <sys/time.h>

#include <deque>
#include <list>
#include <utility>

#include "common/macros.h"

class FragmentedPacket {
 public:
  FragmentedPacket(const timeval &ts, const iphdr &header);
  ~FragmentedPacket() {}

  // Copies the payload from the current fragmented packet into the
  // appropriate position in the reassembled packet.
  void Fill(const iphdr &header, const uint8_t *fragment_payload);

  // Returns true when all the fragments of the original packet have
  // been received.
  bool IsComplete() const { return holes_.empty(); }

  // Returns packet age in microseconds.
  int64_t GetAge(const timeval &now) const;

  const iphdr &ip_header() const { return ip_header_; }
  int32_t payload_size() const { return payload_size_; }
  const uint8_t *payload() const { return payload_; }

 private:
  // The maximum size of the payload of an IP packet is determined by
  // the maximum value of the iphdr.tot_len field and the size of the
  // IP header itself.
  static const int32_t kMaxPayloadSize = 65535 - sizeof(iphdr);

  // The packet timestamp is used to expire old fragments.
  timeval timestamp_;

  // The IP header includes an id field, which is used to identify
  // which fragments belong to the same packet.
  iphdr ip_header_;

  // The hole structure describes a section of the payload with an
  // inclusive start and exclusive end byte indices: [start, end).
  // The list of holes keeps track of which parts of the fragmented
  // packet have not been filled in.  The list of holes is initialized
  // to a single hole that covers the largest possible packet.  Holes
  // may be split, truncated, or removed as fragments arrive.  The
  // list of holes is guaranteed to be non-overlapping and in
  // increasing order of start index.
  struct Hole {
    int32_t start, end;
  };
  std::list<Hole> holes_;

  // payload_size_ is the total number of bytes of the payloads of all
  // the packet fragments.  The total size of the payload is not known
  // until the last fragment packet arrives.
  int32_t payload_size_;
  uint8_t payload_[kMaxPayloadSize];
};

// Holds the state of the incomplete packets during the IP
// defragmentation process.  Returns full payloads when they are
// complete.  Discards incomplete packets once the fragmented packet
// container reaches a maximum size.
class IpDefragmenter {
 public:
  IpDefragmenter()
      : packets_(), completed_packet_(packets_.end(), false) {}
  ~IpDefragmenter() {}

  // Defragments IP packets.  Once the packet has been completely
  // defragmented, returns the total length of the payload, and a
  // pointer to the payload.  You can continue to use the payload
  // pointer until the next call to Defragment.  If the packet is not
  // defragmented, returns -1.
  int32_t Defragment(const timeval &ts, const iphdr &header,
                     const uint8_t *fragment_payload,
                     const uint8_t **full_payload);

  // Returns the number of partially completed packets, which may
  // include the last completed packet, currently stored in the
  // container.
  int32_t Length() const { return static_cast<int32_t>(packets_.size()); }

  // Clear stored fragmented packets from buffer.
  void ClearPackets(void);

 private:
  typedef std::deque<FragmentedPacket>::iterator iterator;

  // The maximum number of incomplete packets allowed in the deque.
  // Once we reach this limit, we remove a quarter of the incomplete
  // packets starting at the front of the deque.
  static const int32_t kMaxFragmentedPackets = 100;

  // The maximum number of microseconds to keep a fragmented packet.
  static const int64_t kMaxFragmentTimeUsec = 30000000LL;

  // Returns an iterator to the partially filled out FragmentedPacket
  // that has the same ID as the packet with the given header.
  iterator FindPacket(const timeval &ts, const iphdr &header);

  // Adds a new fragmented packet to the back of the deque.
  void AddPacket(const timeval &ts, const iphdr &header,
                 const uint8_t *fragment_payload);

  // Returns true if the container has reached the maximum allowed size.
  bool IsFull() const { return packets_.size() >= kMaxFragmentedPackets; }

  // Partially completed packets are stored in a deque.
  std::deque<FragmentedPacket> packets_;

  // Iterator to the completed packet from the last call to Defragment
  // and a Boolean that is true when the iterator is valid.
  std::pair<iterator, bool> completed_packet_;

  DISALLOW_COPY_AND_ASSIGN(IpDefragmenter);
};

#endif  // LIB_PCAP_READER_IP_DEFRAGMENTER_H_

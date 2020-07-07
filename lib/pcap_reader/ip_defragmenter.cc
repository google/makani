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

#include "lib/pcap_reader/ip_defragmenter.h"

#include <glog/logging.h>
#include <netinet/in.h>  // For htons and ntohs.
#include <netinet/ip.h>  // For iphdr, IP_MF, and IP_OFFMASK.
#include <stdint.h>
#include <string.h>
#include <sys/time.h>

#include <deque>
#include <limits>
#include <list>

namespace {

// Returns true if the IP packet is fragmented.
bool IsFragmented(const iphdr &header) {
  return header.frag_off & htons(IP_MF | IP_OFFMASK);
}

// Returns true if this is the fragment at the end of the fragmented
// packet.
bool IsFinalFragment(const iphdr &header) {
  return !(header.frag_off & htons(IP_MF));
}

// Returns true if the two headers are fragments from the same packet.
bool IsPacketFragment(const iphdr &a, const iphdr &b) {
  return a.id == b.id
      && a.protocol == b.protocol
      && a.saddr == b.saddr
      && a.daddr == b.daddr;
}

// Returns the offset of the current payload data in the original
// unfragmented packet.
int32_t GetOffset(const iphdr &header) {
  // Clang raises -Wshadow if the htons call is nested in ntohs.
  uint16_t ip_offmask = htons(IP_OFFMASK);
  return ntohs(header.frag_off & ip_offmask) << 3;
}

// Returns the number of bytes in the current fragment's payload.
int32_t GetPayloadLength(const iphdr &header) {
  return ntohs(header.tot_len) - static_cast<int32_t>(sizeof(header));
}

}  // namespace

FragmentedPacket::FragmentedPacket(const timeval &ts, const iphdr &header)
    : timestamp_(ts),
      ip_header_(header),
      holes_({{0, kMaxPayloadSize}}),
      payload_size_(-1),
      payload_() {}

void FragmentedPacket::Fill(const iphdr &header,
                            const uint8_t *fragment_payload) {
  int32_t fragment_start = GetOffset(header);
  int32_t fragment_length = GetPayloadLength(header);
  int32_t fragment_end = fragment_start + fragment_length;

  // Check if this is the final fragmented packet.  If so, set the
  // payload_size_ and truncate the end of the last hole.
  if (IsFinalFragment(header)) {
    payload_size_ = fragment_end;
    holes_.back().end = fragment_end;
  }

  // Update list of holes by erasing, splitting, or truncating holes.
  std::list<Hole>::iterator hole = holes_.begin();
  while (hole != holes_.end() && hole->start < fragment_end) {
    if (hole->start >= fragment_start && hole->end <= fragment_end) {
      // Erase hole.
      hole = holes_.erase(hole);
    } else {
      if (hole->start < fragment_start && hole->end > fragment_end) {
        // Split hole for non-zero length fragments.
        if (fragment_start != fragment_end) {
          holes_.insert(next(hole), {fragment_end, hole->end});
          hole->end = fragment_start;
        }
      } else if (hole->start < fragment_start && hole->end > fragment_start) {
        // Truncate hole end.
        hole->end = fragment_start;
      } else if (hole->start < fragment_end && hole->end > fragment_end) {
        // Truncate hole start.
        hole->start = fragment_end;
      }
      ++hole;
    }
  }

  // Copy the payload fragment into the full payload.
  memcpy(&payload_[fragment_start], fragment_payload, fragment_length);
}

int64_t FragmentedPacket::GetAge(const timeval &now) const {
  timeval dt;
  timersub(&now, &timestamp_, &dt);
  return dt.tv_sec * 1000000LL + dt.tv_usec;
}

int32_t IpDefragmenter::Defragment(const timeval &ts, const iphdr &header,
                                   const uint8_t *fragment_payload,
                                   const uint8_t **full_payload) {
  CHECK_NOTNULL(fragment_payload);
  CHECK_NOTNULL(full_payload);

  // Remove completed packet from last call.
  if (completed_packet_.second) {
    packets_.erase(completed_packet_.first);
    completed_packet_.second = false;
  }

  // If the packet isn't fragmented, then we are done.
  if (!IsFragmented(header)) {
    *full_payload = fragment_payload;
    return GetPayloadLength(header);
  }

  // Clear out oldest packet if the container is full.
  while (IsFull()) {
    packets_.pop_front();
  }

  // Search for a matching packet; add a packet if one doesn't exist.
  iterator packet = FindPacket(ts, header);
  if (packet == packets_.end()) {
    AddPacket(ts, header, fragment_payload);
  } else {
    // Add the current fragment to the full payload.
    packet->Fill(header, fragment_payload);

    // If the packet is complete, return the length of the full
    // payload and a pointer to it.  Mark the packet for deletion on
    // the next defragment call.
    if (packet->IsComplete()) {
      completed_packet_.first = packet;
      completed_packet_.second = true;
      *full_payload = packet->payload();
      return packet->payload_size();
    }
  }

  return -1;
}

void IpDefragmenter::ClearPackets(void) {
  packets_.clear();
  completed_packet_.second = false;
}

IpDefragmenter::iterator IpDefragmenter::FindPacket(const timeval &ts,
                                                    const iphdr &header) {
  iterator packet;
  for (packet = packets_.begin(); packet != packets_.end(); ) {
    if (packet->GetAge(ts) > kMaxFragmentTimeUsec) {
      packet = packets_.erase(packet);
    } else if (IsPacketFragment(packet->ip_header(), header)) {
      break;
    } else {
      ++packet;
    }
  }
  return packet;
}

void IpDefragmenter::AddPacket(const timeval &ts, const iphdr &header,
                               const uint8_t *fragment_payload) {
  packets_.emplace_back(FragmentedPacket(ts, header));
  packets_.back().Fill(header, fragment_payload);
}

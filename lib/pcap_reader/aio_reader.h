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

#ifndef LIB_PCAP_READER_AIO_READER_H_
#define LIB_PCAP_READER_AIO_READER_H_

#include <stdint.h>
#include <sys/time.h>

#include "avionics/common/aio_header.h"
#include "avionics/network/aio_node.h"
#include "avionics/network/message_info.h"
#include "avionics/network/message_type.h"
#include "lib/pcap_reader/udp_reader.h"

namespace lib {
namespace pcap_reader {

// Enum to interpret reason for processing failure.
enum class AioStatus {
  kBadAioHeader,
    kBadAioVersion,
    kBadIpProtocol,
    kBadMessageType,
    kBadSource,
    kBadUdpPort,
    kDuplicate,
    kValid,
    };

// Returns true for any bad status (kBad*).
bool IsBadStatus(AioStatus status);
bool IsValid(AioStatus status);
bool IsDuplicate(AioStatus status);

// Class to read and validate AIO network packets.
class AioReader {
 public:
  AioReader(void);
  ~AioReader(void) {}

  // Reset stored state. Call this function before parsing unrelated files to
  // clear the internal state.
  void Reset(void);

  AioStatus HandleAioPacket(const timeval &ts, int32_t packet_len,
                            const uint8_t *packet, bool ignore_aio_version);
  AioStatus HandleUdpPacket(const timeval &ts, const UdpReader &reader,
                            bool ignore_aio_version);

  // AIO header accessor functions.
  const AioHeader &GetHeader(void) const {
    return aio_header_;
  }
  AioNode GetSource(void) const {
    return static_cast<AioNode>(aio_header_.source);
  }
  MessageType GetMessageType(void) const {
    return static_cast<MessageType>(aio_header_.type);
  }
  uint16_t GetSequence(void) const {
    return aio_header_.sequence;
  }

  // AIO payload accessor functions.
  int32_t GetPayloadLength(void) const { return payload_len_; }
  const uint8_t *GetPayload(void) const {
    return payload_len_ < 0 ? nullptr : payload_;
  }

  // Unpack AIO message data.
  bool UnpackData(AioMessageData *data) const;
  bool UnpackMessage(AioMessage *message) const;

 private:
  AioStatus Failure(AioStatus status);
  int64_t TimevalToUsec(const struct timeval &ts) const;
  bool IsBootloaderMessage(MessageType type) const;
  AioStatus HandleAioHeader(const struct timeval &ts, int32_t packet_len,
                            const uint8_t *packet, bool ignore_aio_version,
                            AioHeader *header);

  AioHeader aio_header_;
  AioDeduplicate dedup_[kNumAioNodes][kNumMessageTypes];
  int32_t payload_len_;
  const uint8_t *payload_;

  DISALLOW_COPY_AND_ASSIGN(AioReader);
};

}  // namespace pcap_reader
}  // namespace lib

#endif  // LIB_PCAP_READER_AIO_READER_H_

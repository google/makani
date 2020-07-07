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

#include "lib/pcap_reader/aio_reader.h"

#include <stdint.h>
#include <sys/time.h>
#include <string.h>

#include "avionics/common/aio_header.h"
#include "avionics/common/aio_version_code.h"
#include "avionics/common/pack_aio_header.h"
#include "avionics/common/network_config.h"
#include "avionics/network/message_info.h"

namespace lib {
namespace pcap_reader {

bool IsBadStatus(AioStatus status) {
  return status != AioStatus::kDuplicate && status != AioStatus::kValid;
}

bool IsValid(AioStatus status) {
  return status == AioStatus::kValid;
}

bool IsDuplicate(AioStatus status) {
  return status == AioStatus::kDuplicate;
}

AioReader::AioReader(void)
    : aio_header_(), dedup_(), payload_len_(-1), payload_(nullptr) {
  Reset();
}

AioStatus AioReader::Failure(AioStatus status) {
  payload_ = nullptr;
  payload_len_ = -1;
  return status;
}

void AioReader::Reset(void) {
  Failure(AioStatus::kBadAioHeader);
  memset(&dedup_, 0, sizeof(dedup_));
}

int64_t AioReader::TimevalToUsec(const struct timeval &ts) const {
  return ts.tv_sec * 1000000LL + ts.tv_usec;
}

bool AioReader::IsBootloaderMessage(MessageType type) const {
  return type == kMessageTypeBootloaderSlowStatus;
}

AioStatus AioReader::HandleAioHeader(const struct timeval &ts,
                                     int32_t packet_len, const uint8_t *packet,
                                     bool ignore_aio_version,
                                     AioHeader *header) {
  // Check packet length.
  if (packet_len < static_cast<int32_t>(sizeof(*header))) {
    return Failure(AioStatus::kBadAioHeader);
  }
  if (UnpackAioHeader(packet, 1U, header) != PACK_AIOHEADER_SIZE) {
    return Failure(AioStatus::kBadAioHeader);
  }

  // Check version.
  if (!ignore_aio_version && header->version != AIO_VERSION
      && !IsBootloaderMessage(static_cast<MessageType>(header->type))) {
    return Failure(AioStatus::kBadAioVersion);
  }

  // Check source.
  if (header->source < 0 || kNumAioNodes <= header->source) {
    return Failure(AioStatus::kBadSource);
  }

  // Check message type.
  if (kNumMessageTypes <= header->type) {
    return Failure(AioStatus::kBadMessageType);
  }

  // Check for duplicates.
  if (AioHeaderIsDuplicate(TimevalToUsec(ts), header,
                           &dedup_[header->source][header->type])) {
    return AioStatus::kDuplicate;
  }

  // Valid AIO header.
  return AioStatus::kValid;
}

AioStatus AioReader::HandleAioPacket(const struct timeval &ts,
                                     int32_t packet_len, const uint8_t *packet,
                                     bool ignore_aio_version) {
  AioStatus status = HandleAioHeader(ts, packet_len, packet, ignore_aio_version,
                                     &aio_header_);
  if (IsValid(status)) {
    AioDeduplicate *dedup = &dedup_[aio_header_.source][aio_header_.type];
    dedup->sequence = aio_header_.sequence;
    dedup->timestamp = TimevalToUsec(ts);
  }
  if (!IsBadStatus(status)) {
    payload_ = &packet[sizeof(aio_header_)];
    payload_len_ = packet_len - static_cast<int32_t>(sizeof(aio_header_));
  }
  return status;
}

AioStatus AioReader::HandleUdpPacket(const timeval &ts,
                                     const UdpReader &reader,
                                     bool ignore_aio_version) {
  if (reader.GetDestination() == UDP_PORT_AIO) {
    return HandleAioPacket(ts, reader.GetPayloadLength(), reader.GetPayload(),
                           ignore_aio_version);
  }
  return Failure(AioStatus::kBadIpProtocol);
}

bool AioReader::UnpackData(AioMessageData *data) const {
  if (payload_ != nullptr && payload_len_ >= 0) {
    MessageType type = static_cast<MessageType>(aio_header_.type);
    size_t unpacked_length = UnpackAioMessageData(type, payload_, data);
    return static_cast<int32_t>(unpacked_length) == payload_len_;
  }
  return false;
}

bool AioReader::UnpackMessage(AioMessage *message) const {
  message->header = aio_header_;
  return UnpackData(&message->u);
}

}  // namespace pcap_reader
}  // namespace lib

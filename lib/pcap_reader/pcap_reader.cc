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

#include "lib/pcap_reader/pcap_reader.h"

#include <errno.h>
#include <glog/logging.h>
#include <pcap/pcap.h>
#include <stdint.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/time.h>

#include <string>

#include "lib/pcap_reader/aio_reader.h"
#include "lib/pcap_reader/ip_reader.h"
#include "lib/pcap_reader/udp_reader.h"

namespace lib {
namespace pcap_reader {

PcapReader::PcapReader(void)
    : min_pcap_time_(),
      pcap_handle_(nullptr),
      pcap_header_(),
      pcap_payload_(nullptr),
      pcap_errbuf_(),
      ip_reader_(),
      ip_status_(IpStatus::kValid),
      ip_status_valid_(false),
      udp_reader_(),
      udp_status_(UdpStatus::kValid),
      udp_status_valid_(false),
      aio_reader_(),
      aio_status_(AioStatus::kValid),
      aio_status_valid_(false) {
  Reset();
}

void PcapReader::ResetFlags(void) {
  pcap_payload_ = nullptr;
  ip_status_valid_ = false;
  udp_status_valid_ = false;
  aio_status_valid_ = false;
}

void PcapReader::UpdateMinPcapTime(const timeval &pcap_time) {
  if (!timerisset(&min_pcap_time_)
      || timercmp(&min_pcap_time_, &pcap_time, >)) {
    min_pcap_time_ = pcap_time;
  }
}

bool PcapReader::OpenOffline(const std::string &filename) {
  ResetFlags();
  Close();
  if (filename.length() > 0) {
    pcap_handle_ = pcap_open_offline(filename.c_str(), pcap_errbuf_);
  } else {
    pcap_handle_ = nullptr;
  }
  return pcap_handle_ != nullptr;
}

void PcapReader::Close(void) {
  ResetFlags();
  if (pcap_handle_ != nullptr) {
    pcap_close(pcap_handle_);
    pcap_handle_ = nullptr;
  }
}

void PcapReader::Reset(void) {
  ResetFlags();
  ip_reader_.Reset();
  udp_reader_.Reset();
  aio_reader_.Reset();
  timerclear(&min_pcap_time_);
}

bool PcapReader::ReadNext(void) {
  ResetFlags();
  if (pcap_handle_ != nullptr) {
    pcap_payload_ = pcap_next(pcap_handle_, &pcap_header_);
    if (pcap_payload_ != nullptr) {
      UpdateMinPcapTime(pcap_header_.ts);
    }
  } else {
    pcap_payload_ = nullptr;
  }
  return pcap_payload_ != nullptr;
}

IpStatus PcapReader::ReadIp(void) {
  if (!ip_status_valid_) {
    CHECK(pcap_handle_ != nullptr);
    CHECK(pcap_payload_ != nullptr);
    if (pcap_handle_ != nullptr && pcap_payload_ != nullptr) {
      ip_status_ = ip_reader_.HandlePcapPacket(pcap_header_, pcap_payload_,
                                               pcap_handle_);
    } else {
      ip_status_ = IpStatus(IpStatus::kBadPcapHeader);
    }
    ip_status_valid_ = true;
  }
  return ip_status_;
}

UdpStatus PcapReader::ReadUdp(void) {
  if (!udp_status_valid_) {
    if (ReadIp() == IpStatus::kValid) {
      udp_status_ = udp_reader_.HandleIpPacket(ip_reader_);
    } else {
      udp_status_ = UdpStatus(UdpStatus::kBadUdpHeader);
    }
    udp_status_valid_ = true;
  }
  return udp_status_;
}

AioStatus PcapReader::ReadAio(bool ignore_aio_version) {
  if (!aio_status_valid_) {
    if (ReadUdp() == UdpStatus::kValid) {
      aio_status_ = aio_reader_.HandleUdpPacket(pcap_header_.ts, udp_reader_,
                                                ignore_aio_version);
    } else {
      aio_status_ = AioStatus(AioStatus::kBadAioHeader);
    }
    aio_status_valid_ = true;
  }
  return aio_status_;
}

bool PcapReader::CheckIfAllFilesExist(int32_t num_filenames,
                                      char *filenames[]) {
  for (int32_t i = 0; i < num_filenames; ++i) {
    struct stat s;
    if (stat(filenames[i], &s) < 0) {
      snprintf(pcap_errbuf_, sizeof(pcap_errbuf_), "%s: %s", filenames[i],
               strerror(errno));
      return false;
    }
  }
  return true;
}

}  // namespace pcap_reader
}  // namespace lib

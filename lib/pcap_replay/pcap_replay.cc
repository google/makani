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

#include "lib/pcap_replay/pcap_replay.h"

#include <assert.h>
#include <stdint.h>
#include <gflags/gflags.h>
#include <glog/logging.h>

#include <set>
#include <vector>

#include "avionics/common/aio_header.h"
#include "avionics/linux/aio_socket.h"
#include "avionics/linux/clock.h"
#include "avionics/network/message_type.h"
#include "lib/pcap_reader/aio_reader.h"
#include "lib/pcap_reader/pcap_reader.h"
#include "lib/util/string_util.h"

DEFINE_double(speedup, 1.0, "The replay speed of the telemetry.");

DEFINE_string(message_types, "",
              "Comma-separated list of message types to replay.");

namespace {

// Checks that the speedup is larger than 0.
bool ValidateSpeedup(const char * /* flag_name */, double value) {
  return value > 0.0;
}

bool HandlePcapMessageWrapper(lib::pcap_reader::PcapReader *reader,
                              PcapReplay *ths) {
  return ths->HandlePcapFile(reader);
}

}  // namespace

PcapReplay::~PcapReplay(void) {
  if (fd_ < 0) {
    AioClose();
  }
}

void PcapReplay::AioSetup(void) {
  AioSocketInitConfig(0, nullptr, &socket_config_);
  AioSocketSetReuseSocket(true, &socket_config_);
  AioSocketSetLoopback(true, &socket_config_);
  got_first_message_ = false;

  // Open AIO socket.
  fd_ = AioSocketOpen(&socket_config_);
  if (fd_ < 0) {
    assert(false);
  }
}

void PcapReplay::AioClose(void) {
  int32_t fd = fd_;
  fd_ = -1;
  AioSocketClose(fd);
}

int64_t PcapReplay::WaitUntil(int64_t next_message_us) {
  if (!got_first_message_) {
    local_start_time_us_ = ClockGetUs();
    log_start_time_us_ = next_message_us;
    got_first_message_ = true;
  }
  int64_t log_offset_us = next_message_us - log_start_time_us_;
  int64_t local_offset_us = static_cast<int64_t>(
      static_cast<double>(log_offset_us) / speedup_);
  int64_t wait_until_us = local_start_time_us_ + local_offset_us;
  int64_t wait_for_us = wait_until_us - ClockGetUs();
  return wait_for_us;
}

bool PcapReplay::HandlePcapFile(lib::pcap_reader::PcapReader *reader) {
  if (!IsBadStatus(reader->ReadAio())) {
    const lib::pcap_reader::AioReader* aio_reader = &reader->GetAio();
    MessageType msg_type =
        static_cast<MessageType>(aio_reader->GetHeader().type);
    if (message_types_.size() > 0 &&
        message_types_.find(msg_type) == message_types_.end()) {
      return true;
    }
    int64_t interval_us = WaitUntil(reader->GetAbsoluteTimestamp());
    if (interval_us > 0) {
      usleep(static_cast<unsigned int>(interval_us));
    }
    AioSocketSend(&socket_config_, msg_type,
                  reader->GetUdp().GetPayloadLength(),
                  reader->GetUdp().GetPayload(),
                  fd_);
  } else {
    LOG(WARNING) << "Cannot read AIO. Bad AIO version?";
  }
  return true;
}

int main(int argc, char *argv[]) {
  google::InitGoogleLogging(argv[0]);
  google::SetUsageMessage(
      "\nUsage: pcap_replay [-speedup=N] [-message_tyes=...] "
      "1.pcap [2.pcap] ...\n");
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::RegisterFlagValidator(&FLAGS_speedup, &ValidateSpeedup);

  if (argc < 2) {
    google::ShowUsageWithFlagsRestrict(argv[0], "pcap_replay");
    return EXIT_FAILURE;
  }

  // Process consecutive log files.
  lib::pcap_reader::PcapReader pcap_reader;
  std::vector<std::string> message_names =
      SplitString(FLAGS_message_types, ',');
  std::set<MessageType> message_types;
  for (auto message_name : message_names) {
    message_types.insert(
        StringToMessageType(message_name.c_str()));
  }
  PcapReplay pcap_replay(FLAGS_speedup, message_types);

  pcap_replay.AioSetup();
  if (pcap_reader.ForEach(argc - 1, argv + 1,
                          HandlePcapMessageWrapper,
                          &pcap_replay) < 0) {
    LOG(FATAL) << "Pcap error: " << pcap_reader.GetError();
  }
  pcap_replay.AioClose();

  return EXIT_SUCCESS;
}

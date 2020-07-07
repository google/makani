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

#ifndef LIB_PCAP_REPLAY_PCAP_REPLAY_H_
#define LIB_PCAP_REPLAY_PCAP_REPLAY_H_

#include <stdint.h>

#include <set>
#include <string>

#include "avionics/linux/aio_socket.h"
#include "avionics/network/message_type.h"
#include "lib/pcap_reader/pcap_reader.h"

class PcapReplay {
 public:
  explicit PcapReplay(double speedup,
                      const std::set<MessageType> &message_types)
      : fd_(-1),
        got_first_message_(false),
        local_start_time_us_(-1),
        log_start_time_us_(-1),
        speedup_(speedup),
        message_types_(message_types),
        socket_config_() {}

  ~PcapReplay(void);

  void AioSetup(void);
  void AioClose(void);

  bool HandlePcapFile(lib::pcap_reader::PcapReader *reader);

 private:
  int32_t fd_;
  bool got_first_message_;
  int64_t local_start_time_us_;
  int64_t log_start_time_us_;
  double speedup_;
  std::set<MessageType> message_types_;
  AioSocketConfig socket_config_;

  int64_t WaitUntil(int64_t next_message_us);

  DISALLOW_COPY_AND_ASSIGN(PcapReplay);
};

#endif  // LIB_PCAP_REPLAY_PCAP_REPLAY_H_

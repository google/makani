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

// Generate AIO traffic.
// This program uses getopt to avoid dependency on gflags for ARM portability.

#include <getopt.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <cmath>
#include <iomanip>
#include <iostream>
#include <queue>

#include "avionics/common/aio_version_code.h"
#include "avionics/linux/aio.h"
#include "avionics/network/message_info.h"
#include "avionics/network/message_stats.h"

namespace {

enum NetworkSubset {
  kNetworkSubsetAll,
  kNetworkSubsetWing,
  kNetworkSubsetGround,
};

static struct {
  int32_t rand;
  uint16_t aio_version;
  double rate;
  int32_t subset;
} g_options;

static struct option g_arguments[] = {
  {"rand", no_argument, &g_options.rand, 1},
  {"aio_version", required_argument, NULL, 'v'},
  {"rate", required_argument, NULL, 'r'},
  {"wing", no_argument, &g_options.subset, kNetworkSubsetWing},
  {"ground", no_argument, &g_options.subset, kNetworkSubsetGround},
  {NULL, 0, NULL, 0}
};

void PrintUsage() {
  std::cerr << "Generate AIO message traffic." << std::endl;
  std::cerr << "Options:" << std::endl;
  std::cerr << "  --aio_version: Override AIO version. (default "
            << std::hex << AIO_VERSION << ")" << std::endl;
  std::cerr << "  --rand: Randomize data in messages." << std::endl;
  std::cerr << "  --rate: Set rate to this factor relative to nominal traffic."
            << " (default 1.0)" << std::endl;
  std::cerr << "  --ground: Only send ground traffic (except core switches)."
            << std::endl;
  std::cerr << "  --wing: Only send wing traffic (except core switches)."
            << std::endl;
}

bool ParseArguments(int argc, char *argv[]) {
  memset(&g_options, 0, sizeof(g_options));
  g_options.aio_version = AIO_VERSION;
  g_options.rand = 0;
  g_options.rate = 1.0;
  g_options.subset = kNetworkSubsetAll;

  int32_t option_index = 0;
  int32_t c;
  while ((c = getopt_long(argc, argv, "v:r:", g_arguments, &option_index))
         != -1) {
    char *endptr = optarg;
    int64_t version;
    double rate;
    switch (c) {
      case 'v':
        version = strtol(optarg, &endptr, 16);
        if (*endptr != '\0' || version < 0 || version > 0xFFFF) {
          std::cerr << "AIO verson must be a hexadecimal integer between 0 and "
                    << "0xFFFF." << std::endl;
          return false;
        }
        g_options.aio_version = static_cast<uint16_t>(version);
        break;
      case 'r':
        rate = strtod(optarg, &endptr);
        if (*endptr != '\0' || rate <= 0) {
          std::cerr << "Rate must be a positive floating point value."
                    << std::endl;
          return false;
        }
        g_options.rate = rate;
        break;
      case 0:
        break;
      case '?':
      default:
        PrintUsage();
        return false;
    }
  }
  return true;
}

uint32_t g_seed;

struct ScheduleItem {
  MessageType message;
  AioNode source;
  int32_t period_us;
  uint16_t sequence_id;
  int64_t schedule_time;
  friend bool operator<(const ScheduleItem &a, const ScheduleItem &b) {
    return a.schedule_time > b.schedule_time;
  }
};

void SendMessage(AioNode source, MessageType type, uint16_t sequence) {
  uint8_t message_buf[AIO_MAX_MESSAGE_LEN] = {0};
  int32_t pack_size = static_cast<int32_t>(kAioMessageInfo[type].pack_size);

  if (g_options.rand) {
    for (int32_t i = 0; i < pack_size; i++) {
      message_buf[i] = static_cast<uint8_t>(rand_r(&g_seed) % 256);
    }
  }

  AioHeader header;
  memset(&header, 0, sizeof(header));
  header.version = g_options.aio_version;
  header.source = source;
  header.type = type;
  header.sequence = sequence;
  header.timestamp = static_cast<uint32_t>(ClockGetUs());

  AioSendInternal(type, message_buf, pack_size, &header);
}

bool TrafficSelected(NetworkSubset subset, AioNode source) {
  if (subset == kNetworkSubsetAll) {
    return true;
  } else if (subset == kNetworkSubsetWing) {
    return IsWingNode(source) && !IsCoreSwitchNode(source);
  } else if (subset == kNetworkSubsetGround) {
    return IsGroundStationNode(source) && !IsCoreSwitchNode(source);
  }
  return false;
}

void Run(void) {
  g_seed = static_cast<uint32_t>(ClockGetUs());

  // Initialize schedule.
  std::priority_queue<ScheduleItem> schedule;
  int64_t current_time = ClockGetUs();
  int32_t num_nodes = 0;
  double packets_per_sec = 0.0;
  double bytes_per_sec = 0.0;
  // Total AIO header size is 14 (ethernet) + 20 (IP) + 8 (UDP) + AIO header +
  // 4 (FCS).
  const size_t header_size = 14 + 20 + 8 + sizeof(AioHeader) + 4;
  for (int32_t i = 0; i < kNumAioNodes; ++i) {
    AioNode source = static_cast<AioNode>(i);
    if (!TrafficSelected(static_cast<NetworkSubset>(g_options.subset),
                         source)) {
      continue;
    }
    num_nodes++;
    const MessageSenderInfo *sender_info = &kMessageSenderInfo[source];
    for (int32_t j = 0; j < sender_info->num_messages; j++) {
      const MessageFrequencyInfo *freq_info =
          &sender_info->frequency_info[j];
      MessageType type = freq_info->message;
      if (freq_info->period_us <= 0) continue;
      double period_scaled = round(freq_info->period_us / g_options.rate);
      int32_t period_us = static_cast<int32_t>(period_scaled);
      if (period_us <= 0) {
        period_us = 1;
        std::cerr << "WARNING: Messages are limited to 1M packets per second "
                  << "... and you're requesting that I go over that?"
                  << std::endl;
      }
      packets_per_sec += 1e6 / period_scaled;
      bytes_per_sec += (1e6 / period_scaled) * static_cast<double>(
          kAioMessageInfo[type].pack_size + header_size);
      ScheduleItem item;
      item.message = type;
      item.source = source;
      item.period_us = period_us;
      item.sequence_id = 0U;
      item.schedule_time = current_time + rand_r(&g_seed) % period_us;
      schedule.push(item);
    }
  }

  std::cout << "Sending " << schedule.size() << " total messages from "
            << num_nodes << " AIO nodes.  "
            << "Rate is " << g_options.rate << "x normal speed." << std::endl;
  std::cout << "Traffic rate is " << (bytes_per_sec / 1e6 * 8) << " Mbps ("
            << round(packets_per_sec) << " packets per second)." << std::endl;

  // Send traffic.
  while (true) {
    ScheduleItem item = schedule.top();
    if (item.schedule_time > ClockGetUs()) {
      usleep(5);
    } else {
      schedule.pop();
      item.schedule_time += item.period_us;
      SendMessage(item.source, item.message, item.sequence_id++);
      schedule.push(item);
    }
  }
}

}  // namespace.

int main(int argc, char *argv[]) {
  if (!ParseArguments(argc, argv)) {
    return EXIT_FAILURE;
  }
  AioSetup(kAioNodeUnknown, UDP_PORT_AIO, NULL, 0);
  std::cout << "Opened AIO connection." << std::endl;
  Run();
  return EXIT_SUCCESS;
}

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

#include <arpa/inet.h>
#include <assert.h>
#include <ifaddrs.h>
#include <inttypes.h>
#include <netinet/in.h>
#include <stdio.h>
#include <string.h>
#include <sys/sysinfo.h>
#include <unistd.h>

#include <cstdint>
#include <fstream>
#include <iostream>
#include <string>

#include "avionics/linux/q7_slow_status.h"

#include "avionics/common/avionics_messages.h"
#include "avionics/common/build_info.h"
#include "avionics/common/strings.h"
#include "avionics/network/aio_node.h"
#include "avionics/network/aio_node_to_ip_address.h"
#include "common/macros.h"

namespace {

AioNode GetAioNode() {
  struct ifaddrs *head, *current;

  if (getifaddrs(&head)) {
    return kAioNodeUnknown;
  }

  for (current = head; current != NULL; current = current->ifa_next) {
    if (!current->ifa_addr) continue;
    if (current->ifa_addr->sa_family != AF_INET) continue;
    void *silence_alignment_warning =
        reinterpret_cast<void *>(current->ifa_addr);
    struct sockaddr_in *sai =
        reinterpret_cast<struct sockaddr_in *>(silence_alignment_warning);
    uint32_t addr = ntohl(sai->sin_addr.s_addr);
    if (((addr >> 24) & 0xff) != 192 || ((addr >> 16) & 0xff) != 168 ||
        ((addr >>  8) & 0xff) != 1) {
      continue;
    }
    return FinalOctetToAioNode(addr & 0xff);
  }
  return kAioNodeUnknown;
}

bool GetGitHash(GitHash git_hash) {
  std::ifstream timestamp_file("/TIMESTAMP");
  if (timestamp_file.is_open()) {
    std::string line;
    std::getline(timestamp_file, line);
    timestamp_file.close();
    const char hash_tag[] = "at hash ";
    size_t hash = line.find(hash_tag);
    if (hash == std::string::npos) {
      return false;
    }
    hash += strlen(hash_tag);
    line = line.substr(hash, 40);
    if (line.size() != 40) {
      return false;
    }
    const char *line_buf = line.c_str();
    for (int i = 0; i < 40; ) {
      i += ReadHexUint8(line_buf + i, 2, git_hash + i / 2);
    }
    return true;
  }
  return false;
}

}  // namespace

// Failures here are considered soft, as some callers may wish to ignore
// them and send status anyway.  Callers must check for any field that is
// considered a hard failure if this returns false.
extern "C" bool Q7SlowStatusInit(Q7SlowStatusContext *context) {
  bool success = true;
  memset(context, 0, sizeof(*context));

  success &= GetGitHash(context->git_hash);

  int nprocs = get_nprocs();
  assert(nprocs > 0 && nprocs < 256);
  context->num_cpus = (int8_t)nprocs;

  context->node = GetAioNode();

  GetBuildInfo(&context->build_info);

  return success;
}

extern "C" void Q7SlowStatusInitMessage(const Q7SlowStatusContext *context,
                                        Q7SlowStatusMessage *message) {
  memset(message, 0, sizeof(*message));
  memcpy(&message->git_hash, &context->git_hash, sizeof(message->git_hash));
  memcpy(&message->build_info, &context->build_info,
         sizeof(message->build_info));
  message->sys_info.num_cpus = context->num_cpus;
}

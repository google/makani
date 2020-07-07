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

#include <ctype.h>
#include <netinet/if_ether.h>
#include <netinet/in.h>
#include <netinet/ip.h>
#include <netinet/udp.h>
#include <stdint.h>
#include <string.h>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <pcap/pcap.h>

#include <list>
#include <set>
#include <string>

#include "avionics/common/aio_header.h"
#include "avionics/common/aio_version_code.h"
#include "avionics/common/avionics_messages.h"
#include "avionics/common/network_config.h"
#include "avionics/common/pack_aio_header.h"
#include "lib/hdf5_to_pcap/h5log_reader.h"

DEFINE_string(input_file, "", "The path to the input h5 log file.");
DEFINE_string(output_file, "", "The path to the output pcap log file.");
DEFINE_string(nodes, "", "List of nodes to include.");
DEFINE_string(messages, "", "List of message types to include.");

namespace {

// Parse -nodes and -messages arguments for alpha-numeric tokens, delimited
// by non-alpha-numeric characters.
std::set<std::string> GetTokens(const std::string &s) {
  std::set<std::string> tokens;
  for (size_t start = 0, index = 0; index < s.length(); ++index) {
    if (!isalnum(static_cast<int>(s[index]))) {
      if (start < index) {
        tokens.insert(s.substr(start, index - start));
      }
      start = index + 1;
    } else if (index + 1 == s.length() && start < index) {
      tokens.insert(s.substr(start, index - start + 1));
    }
  }
  return tokens;
}

// Checks that the required file names are non-empty.
bool ValidateFileName(const char * /* flag_name */, const std::string &value) {
  return value.size() > 0;
}

bool ValidateNodes(const char * /* flag_name */, const std::string &value) {
  std::set<std::string> tokens = GetTokens(value);
  for (std::set<std::string>::const_iterator iter = tokens.begin();
       iter != tokens.end(); ++iter) {
    bool found = false;
    for (int32_t i = 0; i < kNumAioNodes && !found; ++i) {
      AioNode aio_node = static_cast<AioNode>(i);
      found = (iter->compare(AioNodeToString(aio_node)) == 0
               || iter->compare(AioNodeToShortString(aio_node)) == 0);
    }
    if (!found) {
      LOG(ERROR) << "Invalid AioNode: " << *iter;
      return false;
    }
  }
  return true;
}

bool ValidateMessages(const char * /* flag_name */, const std::string &value) {
  std::set<std::string> tokens = GetTokens(value);
  for (std::set<std::string>::const_iterator iter = tokens.begin();
       iter != tokens.end(); ++iter) {
    bool found = false;
    for (int32_t i = 0; i < kNumMessageTypes && !found; ++i) {
      MessageType type = static_cast<MessageType>(i);
      found = (iter->compare(MessageTypeToString(type)) == 0
               || iter->compare(MessageTypeToShortString(type)) == 0);
    }
    if (!found) {
      LOG(ERROR) << "Invalid MessageType: " << *iter;
      return false;
    }
  }
  return true;
}

bool validate_input __attribute__((unused)) = (
    google::RegisterFlagValidator(&FLAGS_input_file, &ValidateFileName));
bool validate_output __attribute__((unused)) = (
    google::RegisterFlagValidator(&FLAGS_output_file, &ValidateFileName));
bool validate_nodes __attribute__((unused)) = (
    google::RegisterFlagValidator(&FLAGS_nodes, &ValidateNodes));
bool validate_messages __attribute__((unused)) = (
    google::RegisterFlagValidator(&FLAGS_messages, &ValidateMessages));

void WriteEthHeader(const EthernetAddress &dest, uint8_t *out) {
  struct ethhdr h;
  memset(&h, 0x0, sizeof(h));
  memcpy(h.h_dest, &dest, sizeof(h.h_dest));
  // Source Ethernet address not required (or known).
  h.h_proto = htons(ETHERTYPE_IP);
  memcpy(out, &h, sizeof(h));
}

void WriteIpHeader(const IpAddress &dest, int32_t length, uint8_t *out) {
  struct iphdr h;
  memset(&h, 0x0, sizeof(h));
  h.version = IPVERSION;
#pragma GCC diagnostic ignored "-Wconversion"
  h.ihl = sizeof(h) / sizeof(uint32_t);  // Type unsigned char:4.
  h.frag_off = htons(IP_DF);
  h.ttl = 8;
  CHECK_GE(length, 0);
  h.tot_len = htons(length);
  h.protocol = IPPROTO_UDP;
  h.daddr = htonl(IpAddressToUint32(dest));
  // Source IP address not required (or known).

  // Compute checksum of IP header.
  uint32_t checksum = 0U;
  const uint16_t *raw = reinterpret_cast<const uint16_t *>(&h);
  for (size_t i = 0; i < sizeof(h) / sizeof(uint16_t); ++i) {
    checksum = static_cast<uint32_t>(checksum + raw[i]);
  }
  checksum = (checksum & 0xFFFF) + (checksum >> 16);
  h.check = static_cast<uint16_t>(~checksum);

  memcpy(out, &h, sizeof(h));
}

void WriteUdpHeader(int32_t length, uint8_t *out) {
  struct udphdr h;
  memset(&h, 0x0, sizeof(h));
  h.source = htons(UDP_PORT_AIO);
  h.dest = htons(UDP_PORT_AIO);
  CHECK_GE(length, 0);
  h.len = htons(length);
  memcpy(out, &h, sizeof(h));
}

void WriteAioHeader(const AioHeader *in, AioNode source,
                    MessageType message_type, uint8_t *out) {
  AioHeader h = *in;
  // Update header to the current protocol version and types.
  h.version = AIO_VERSION;
  h.source = source;
  h.type = message_type;
  PackAioHeader(&h, 1U, out);
}

int32_t WritePacket(const h5log_reader::MessageLog &mlog, int64_t index,
                    uint8_t *out) {
  MessageType message_type = mlog.GetMessageType();
  EthernetAddress dest_mac = AioMessageTypeToEthernetAddress(message_type);
  IpAddress dest_ip = AioMessageTypeToIpAddress(message_type);
  int32_t offset = (int32_t)(sizeof(ethhdr) + sizeof(iphdr) + sizeof(udphdr)
                             + PACK_AIOHEADER_SIZE);
  int32_t udp_length = mlog.PackMessageData(index, &out[offset]);
  int32_t total_length = offset + udp_length;

  offset -= PACK_AIOHEADER_SIZE;
  WriteAioHeader(mlog.GetAioHeader(index), mlog.GetAioNode(),
                 message_type, &out[offset]);

  offset -= (int32_t)sizeof(udphdr);
  WriteUdpHeader(total_length - offset, &out[offset]);

  offset -= (int32_t)sizeof(iphdr);
  WriteIpHeader(dest_ip, total_length - offset, &out[offset]);

  offset -= (int32_t)sizeof(ethhdr);
  WriteEthHeader(dest_mac, &out[offset]);
  CHECK_EQ(offset, 0);

  return total_length;
}

void WritePcap(const h5log_reader::MessageLog &mlog, int64_t index,
               pcap_dumper_t *d) {
  uint8_t buf[IP_MAXPACKET];
  int32_t length = WritePacket(mlog, index, buf);

  const CaptureHeader *cap = mlog.GetCaptureHeader(index);
  struct pcap_pkthdr h;
  memset(&h, 0x0, sizeof(h));
  h.ts.tv_sec = cap->tv_sec;
  h.ts.tv_usec = cap->tv_usec;
  h.caplen = length;
  h.len = length;
  pcap_dump(reinterpret_cast<u_char *>(d), &h, buf);
}

typedef std::pair<const h5log_reader::MessageLog &, int64_t> MessageIndexPair;
typedef std::list<MessageIndexPair> MessageIndexList;

void GetCaptureTime(const h5log_reader::MessageLog &mlog, int64_t index,
                    struct timeval *out) {
  const CaptureHeader *cap = mlog.GetCaptureHeader(index);
  out->tv_sec = cap->tv_sec;
  out->tv_usec = cap->tv_usec;
}

MessageIndexList GetPendingMessages(const h5log_reader::MessageLogs &h5log) {
  MessageIndexList pending;
  for (h5log_reader::MessageLogs::const_iterator iter = h5log.begin();
       iter != h5log.end(); ++iter) {
    pending.push_back(MessageIndexPair(*iter, 0));
  }
  return pending;
}

MessageIndexList::iterator GetNextMessage(MessageIndexList *pending) {
  struct timeval min_tv;
  MessageIndexList::iterator next = pending->end();
  for (MessageIndexList::iterator iter = pending->begin();
       iter != pending->end(); ++iter) {
    // Assume data sets contain a monotonically increasing time vector.
    struct timeval tv;
    GetCaptureTime(iter->first, iter->second, &tv);
    if (next == pending->end() || timercmp(&tv, &min_tv, <)) {
      next = iter;
      min_tv = tv;
    }
  }
  return next;
}

void H5LogToPcap(const h5log_reader::MessageLogs &h5log, pcap_dumper_t *d) {
  MessageIndexList pending = GetPendingMessages(h5log);
  while (!pending.empty()) {
    MessageIndexList::iterator next = GetNextMessage(&pending);
    WritePcap(next->first, next->second, d);

    // Increment message index.
    ++next->second;

    // Remove message log from list after processing the last message in log.
    if (next->second >= next->first.GetNumMessages()) {
      pending.erase(next);
    }
  }
}

}  // namespace

int main(int argc, char *argv[]) {
  google::InitGoogleLogging(argv[0]);
  google::SetUsageMessage(
      "hdf5_to_pcap -input_file=input.h5 -output_file=output.pcap");
  google::ParseCommandLineFlags(&argc, &argv, true);

  LOG(INFO) << "Reading " << FLAGS_input_file << "...";
  h5log_reader::MessageLogs h5log = h5log_reader::ReadFile(
      FLAGS_input_file, GetTokens(FLAGS_nodes), GetTokens(FLAGS_messages));

  int exit_code = EXIT_FAILURE;
  pcap_t *p = pcap_open_dead(DLT_EN10MB, 65535);
  if (p != NULL) {
    LOG(INFO) << "Writing " << FLAGS_output_file << "...";
    pcap_dumper_t *d = pcap_dump_open(p, FLAGS_output_file.c_str());
    if (d != NULL) {
      H5LogToPcap(h5log, d);
      pcap_dump_close(d);
      exit_code = EXIT_SUCCESS;
    }
    pcap_close(p);
  }
  return exit_code;
}

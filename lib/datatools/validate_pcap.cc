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

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <limits.h>
#include <math.h>
#include <netinet/if_ether.h>
#include <netinet/in.h>
#include <netinet/ip.h>
#include <netinet/udp.h>
#include <pcap/pcap.h>
#include <pcap/sll.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>

#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <ostream>
#include <set>
#include <string>
#include <vector>

#include "avionics/common/aio_header.h"
#include "avionics/common/network_config.h"
#include "avionics/common/pack_aio_header.h"
#include "avionics/network/aio_node.h"
#include "avionics/network/message_stats.h"
#include "avionics/network/message_type.h"
#include "lib/pcap_reader/pcap_reader.h"

DEFINE_bool(ignore_version, false, "Ignore AIO header version.");
DEFINE_bool(detailed_stats, false, "Dump detailed stats.");

// These flags only have to do with the python data dump.
DEFINE_string(data_file, "", "Output python data file for graphing traffic.");
DEFINE_int64(seq_check_start_ms, 0,
             "Check sequence numbers starting at this time offset in ms.");
DEFINE_int64(seq_check_duration_ms, 1000,
             "Check sequence numbers for this many ms.");
DEFINE_bool(gaps_only, false, "Suppress packet output; only output gaps.");
DEFINE_string(check_message_type, "",
              "Which message types to graph and check.");
DEFINE_string(check_message_source, "",
              "Which message source AIO nodes to graph and check.");

// We don't have libglog on the Q7, so we divert its methods here, as
// they're used by lib/pcap_reader.
namespace google {

LogMessage::LogMessage(const char* file, int line,
                       const CheckOpString& result)
    : allocated_(NULL),
      data_(NULL) {
  fprintf(stderr, "Error in file %s, line %d.\n", file, line);
  fprintf(stderr, "Check failed: '%s'.\n", result.str_->c_str());
}

LogMessage::LogMessage(const char* file, int line)
    : allocated_(NULL),
      data_(NULL) {
  fprintf(stderr, "Message in file %s, line %d.\n", file, line);
}

LogMessage::~LogMessage() {
}

std::ostream& LogMessage::stream() {
  return std::cout;
}

LogMessageFatal::LogMessageFatal(const char* file, int line) :
    LogMessage::LogMessage(file, line) {
}

LogMessageFatal::LogMessageFatal(
    const char* file, int line, const CheckOpString& result) :
    LogMessage::LogMessage(file, line, result) {
}

LogMessageFatal::~LogMessageFatal() {
  fprintf(stderr, "Error was fatal.\n");
  exit(EXIT_FAILURE);
}

}  // namespace google

//
// This program does some basic validations on a set of logs.  There are a
// number we may want to add.
//
// Implemented:
//
// Note any messages that aren't AIO messages.
// Note any corrupted packets.
// Note any messages with bad AIO version.
// Note any unrecognized AIO messages.
// List all nodes and AIO messages seen in the log.
//   Note any unrecognized nodes.
//   Note any that exist only for certain segments of the log; currently we just
//   check each file as a unit, but we could do smaller spot checks.
//   Check packet rates against the expected rates.
//
// Potential future improvements:
//
// Note any fragmented packets.
// Note any nodes or messages that are missing.
// Look inside switch stats packets to see if they match our numbers.
//
// Potential optimization:
//
// Allow skimming through a large file, skipping large segments.
// To do this for really big files, we want to fseek through it and then
// reacquire the packet boundaries.  Here's how:
//
// Look for the header of an AIO message:
//   Bytes  0 -  5 are dest MAC.     Look for 1:0:5E:0:0:X
//   Bytes  6 - 11 are source MAC.   Look for 2:0:0:0:0:Y
//   Bytes 12 - 13 are ethernet len. Check that it's the right len for X.
//   Bytes 16 - 17 are ipv4 len.     Check that it agrees with ethernet len.
//   Bytes 26 - 29 are source IP.    Look for 192.168.1.Y
//   Bytes 30 - 33 are dest IP.      Look for 239.0.0.X
//   Bytes 34 - 35 are source port.  Look for 40000
//   Bytes 36 - 37 are dest port.    Look for 40000
//   Bytes 38 - 39 are udp len.      Check that it agrees with ipv4 len.
//
// Once you've found that, back up to the pcap packet header that precedes the
// packet; it's 16 bytes.  They are:
//
// uint32_t ts_sec
// uint32_t ts_usec
// uint32_t incl_len [length in this file]
// uint32_t orig_len [length as seen on the wire]
//
// incl_len should agree with the ethernet, IPv4, and UDP lengths.
//
// According to
// https://ask.wireshark.org/questions/21900/random-access-read-and-dissection-of-packets-from-a-pcap-file
// you have to call pcap_open_offline on the FILE* pointed at the start of the
// file, and then you can fseek it to where you like.
//

namespace {

int64_t g_start_sequence_ts_us = -1;
int64_t g_end_sequence_ts_us = -1;
std::set<MessageType> g_check_message_types;
std::set<AioNode> g_check_message_sources;

typedef struct {
  // Bad packet reasons.
  int32_t bad_header;
  int32_t bad_protocol;
  int32_t bad_port;
  int32_t bad_source;
  int32_t bad_type;
  int32_t bad_version;
  // Packet counters.
  int32_t total_packets;
  int32_t bad_packets;
  int32_t good_packets;
  int32_t duplicate_packets;
} AioPacketStats;

typedef struct {
  struct timeval capture_start;
  struct timeval capture_end;
  double duration_s;
  // Bad packet reasons.
  int32_t bad_ethernet_header;
  int32_t bad_ethernet_protocol;
  int32_t bad_ip_header;
  int32_t bad_ip_protocol;
  int32_t bad_link_protocol;
  int32_t bad_pcap_header;
  int32_t bad_sll_header;
  int32_t bad_udp_header;
  // Packet counters.
  int32_t total_packets;
  int32_t bad_packets;
  int32_t fragmented_packets;
  int32_t ip_packets;
  int32_t pcap_packets;
  int32_t probe_packets;
  int32_t udp_packets;
  int32_t winch_packets;
  int64_t total_bytes;
  int64_t unique_aio_bytes;
  // AIO layer.
  AioPacketStats aio;
  // File counters.
  int32_t failed_files;
} PacketStats;

typedef std::map<MessageType, int32_t> MessageCount;
typedef std::map<AioNode, MessageCount> NodeRecord;

const char kPacketTypeAio[] = "AIO";
const char kPacketTypeWinch[] = "WINCH";
const char kPacketTypeProbe[] = "PROBE";
const char kPacketTypeFragment[] = "FRAGMENT";
const char kPacketTypeUnknown[] = "UNKNOWN";

bool Within1Percent(double a, double b) {
  DCHECK(a > 0.0);
  return fabs((a - b) / a) <= 0.01;
}

int64_t TsToTimeUs(const struct timeval &ts) {
  return static_cast<int64_t>(ts.tv_sec) * 1000000 + ts.tv_usec;
}

// Compute the diff from t0 to t1 [meaning t1 - t0].
double TimeDiffSeconds(const struct timeval &t0, const struct timeval &t1) {
  struct timeval tv_diff;
  timersub(&t1, &t0, &tv_diff);
  double time_diff_s = static_cast<double>(tv_diff.tv_sec) +
      static_cast<double>(tv_diff.tv_usec) / 1000000.0;
  return time_diff_s;
}

std::string TimeValToString(const struct timeval &t) {
  std::ostringstream output;
  output << "{" << t.tv_sec << ", " << t.tv_usec << "}";
  return output.str();
}

void RecordAioInfo(const lib::pcap_reader::PcapReader &reader,
                   NodeRecord *node_record, std::ostream *error_stream) {
  static std::map<AioNode, uint8_t> octet_record;
  AioNode source = reader.GetAio().GetSource();
  MessageType type = reader.GetAio().GetMessageType();
  ++(*node_record)[source][type];
  uint8_t octet = static_cast<uint8_t>(reader.GetIp().GetSource() >> 24);
  if (octet_record.count(source)) {
    if (octet_record[source] != octet) {
      *error_stream << "ERROR: Got multiple IP addresses for AioNode " <<
          (unsigned)source << ": " << octet_record[source] << " vs " << octet <<
          ".\n";
    }
  } else {
    octet_record[source] = octet;
  }
}

void InsertKeys(const NodeRecord::const_iterator &start,
                const NodeRecord::const_iterator &end,
                std::vector<AioNode> *nodes) {
  for (NodeRecord::const_iterator iter = start; iter != end; ++iter) {
    nodes->push_back(iter->first);
  }
}

std::string NodeVectorToString(const std::vector<AioNode> &vec) {
  std::ostringstream s;
  for (std::vector<AioNode>::const_iterator i = vec.begin(); i != vec.end();
       ++i) {
    s << static_cast<unsigned>(*i) << " ";
  }
  return s.str();
}

void RecordNodeDiffs(const NodeRecord &last_node_record,
                     const NodeRecord &cur_node_record,
                     std::ostream *error_stream) {
  NodeRecord::const_iterator last_iter = last_node_record.begin();
  NodeRecord::const_iterator cur_iter = cur_node_record.begin();
  std::vector<AioNode> lost_nodes;
  std::vector<AioNode> gained_nodes;
  while (last_iter != last_node_record.end() ||
         cur_iter != cur_node_record.end()) {
    if (last_iter == last_node_record.end()) {
      InsertKeys(cur_iter, cur_node_record.end(), &gained_nodes);
      cur_iter = cur_node_record.end();
    } else if (cur_iter == cur_node_record.end()) {
      InsertKeys(last_iter, last_node_record.end(), &lost_nodes);
      last_iter = last_node_record.end();
    } else if (last_iter->first < cur_iter->first) {
      lost_nodes.push_back(last_iter->first);
      ++last_iter;
    } else if (cur_iter->first < last_iter->first) {
      gained_nodes.push_back(cur_iter->first);
      ++cur_iter;
    } else {
      DCHECK(last_iter->first == cur_iter->first);
      ++last_iter;
      ++cur_iter;
    }
  }

  if (!lost_nodes.empty()) {
    *error_stream << "\tNode(s) that went away: " <<
        NodeVectorToString(lost_nodes) << std::endl;
  }
  if (!gained_nodes.empty()) {
    *error_stream << "\tNode(s) that are new: " <<
        NodeVectorToString(gained_nodes) << std::endl;
  }
}

// Returns true on a match.
bool CheckNodeRecord(const NodeRecord &last_node_record,
                     const NodeRecord &cur_node_record,
                     std::ostream *error_stream,
                     const char *last_file_name,
                     const char *cur_file_name) {
  if (last_node_record.size() != cur_node_record.size()) {
    *error_stream << "WARNING: Difference in number of nodes;\n\tfile " <<
        last_file_name << " had " << last_node_record.size() << ";\n\tfile " <<
        cur_file_name << " has " << cur_node_record.size() << ".\n";
    RecordNodeDiffs(last_node_record, cur_node_record, error_stream);
    return false;
  }
  for (NodeRecord::const_iterator iter_n = last_node_record.begin();
       iter_n != last_node_record.end(); ++iter_n) {
    const AioNode node = iter_n->first;
    if (!cur_node_record.count(node)) {
      *error_stream << "cur_node_record is missing node " << (unsigned)node <<
          ".\n";
      return false;
    }
    const MessageCount &last_message_count = iter_n->second;
    const MessageCount &cur_message_count = cur_node_record.at(node);
    if (last_message_count.size() != cur_message_count.size()) {
      *error_stream <<
          "WARNING: Difference in number of message types for node " <<
          (unsigned)node << ";\n\twas " << last_message_count.size() <<
          ", now " << cur_message_count.size() << ".\n";
      return false;
    }
    for (MessageCount::const_iterator iter_m = last_message_count.begin();
         iter_m != last_message_count.end(); ++iter_m) {
      MessageType message_type = iter_m->first;
      if (!cur_message_count.count(message_type)) {
        *error_stream << "cur_node_record is missing message " <<
            (unsigned)message_type << " from node " << (unsigned)node << ".\n";
        return false;
      }
    }
  }
  return true;
}

void DumpStats(const PacketStats &stats, bool detailed_stats,
               std::ostream *info_stream) {
  // Non-AIO layer.
  // Packet counters.
  *info_stream << "Total packets: " << stats.total_packets << std::endl;
  if (detailed_stats) {
    *info_stream << "Mean bandwidth: "
                 << (static_cast<double>(stats.total_bytes * 8) /
                     (1024 * 1024 * stats.duration_s))
                 << " Mb/s" << std::endl;
    *info_stream << "PCAP packets: " << stats.pcap_packets << std::endl;
    *info_stream << "IP packets: " << stats.ip_packets << std::endl;
    *info_stream << "UDP packets: " << stats.udp_packets << std::endl;
    *info_stream << "Fragmented packets: "
                 << stats.fragmented_packets << std::endl;
    *info_stream << "Probe packets: " << stats.probe_packets << std::endl;
    *info_stream << "Winch packets: " << stats.winch_packets << std::endl;
    *info_stream << "Total bytes: " << stats.total_bytes << std::endl;
    *info_stream << "Unique AIO bytes: " << stats.unique_aio_bytes << std::endl;
    *info_stream << "Mean AIO packet rate: "
                 << stats.aio.good_packets / stats.duration_s
                 << " Packets/s" << std::endl;
    *info_stream << "Mean unique AIO bandwidth: "
                 << (static_cast<double>(stats.unique_aio_bytes * 8) /
                     (1024 * 1024 * stats.duration_s))
                 << " Mb/s" << std::endl;
  }

  // Bad packet reasons.
  *info_stream << "Bad packets: " << stats.bad_packets << std::endl;
  if (stats.bad_packets > 0 && detailed_stats) {
    *info_stream << "\tBad pcap header: " << stats.bad_pcap_header << std::endl;
    *info_stream << "\tBad link protocol: "
                 << stats.bad_link_protocol << std::endl;
    *info_stream << "\tBad Ethernet header: "
                 << stats.bad_ethernet_header << std::endl;
    *info_stream << "\tBad Ethernet protocol: "
                 << stats.bad_ethernet_protocol << std::endl;
    *info_stream << "\tBad SLL header: " << stats.bad_sll_header << std::endl;
    *info_stream << "\tBad IP header: " << stats.bad_ip_header << std::endl;
    *info_stream << "\tBad IP protocol: " << stats.bad_ip_protocol << std::endl;
    *info_stream << "\tBad UDP header: " << stats.bad_udp_header << std::endl;
  }

  // AIO layer.
  *info_stream << "AIO packets: " << stats.aio.good_packets << std::endl;
  *info_stream << "Non-AIO packets: "
               << stats.total_packets - stats.aio.total_packets << std::endl;
  if (detailed_stats) {
    *info_stream << "Unique AIO packets: "
                 << stats.aio.good_packets - stats.aio.duplicate_packets
                 << std::endl;
    *info_stream << "Duplicate AIO packets: " << stats.aio.duplicate_packets
                 << std::endl;
    *info_stream << "Non-AIO packets excluding fragments: "
                 << (stats.total_packets - stats.aio.total_packets -
                     stats.fragmented_packets)
                 << std::endl;
  }

  // Bad packet reasons.
  *info_stream << "Bad AIO packets: " << stats.aio.bad_packets << std::endl;
  if (stats.aio.bad_packets > 0 && detailed_stats) {
    *info_stream << "\tBad AIO header: " << stats.aio.bad_header << std::endl;
    *info_stream << "\tBad AIO protocol: "
                 << stats.aio.bad_protocol << std::endl;
    *info_stream << "\tBad AIO port: " << stats.aio.bad_port << std::endl;
    *info_stream << "\tBad AIO source: " << stats.aio.bad_source << std::endl;
    *info_stream << "\tBad AIO type: " << stats.aio.bad_type << std::endl;
    *info_stream << "\tBad AIO version: " << stats.aio.bad_version << std::endl;
  }
}

void ProcessNodeRecords(const NodeRecord &sum_node_record,
                        const PacketStats &stats,
                        std::ostringstream *info_stream,
                        std::ostringstream *error_stream) {
  *info_stream << "Saw " << sum_node_record.size() << " unique AIO Node(s):\n";
  for (NodeRecord::const_iterator iter_n = sum_node_record.begin();
       iter_n != sum_node_record.end(); ++iter_n) {
    const MessageCount &message_count = iter_n->second;
    AioNode sender = iter_n->first;
    if (!IsValidNode(sender) && !IsUnknownNode(sender)) {
      *error_stream << "ERROR: Unrecognized node number " << (unsigned)sender <<
          ".\n";
      continue;
    }
    *info_stream << "\tNode " << AioNodeToString(sender) << " sent " <<
        message_count.size() << " unique message type(s):\n";
    for (MessageCount::const_iterator iter_m = message_count.begin();
         iter_m != message_count.end(); ++iter_m) {
      MessageType message_type = iter_m->first;
      if (!IsValidMessageType(message_type)) {
        *error_stream << "ERROR: Unrecognized message type " <<
            (unsigned)message_type << ".\n";
        continue;
      }
      int32_t count = iter_m->second;
      double expected_frequency = GetMessageFrequency(sender, message_type);
      double frequency = count / stats.duration_s;
      *info_stream << "\t\t" << MessageTypeToString(message_type) << ": " <<
          count << " packets (" << count / stats.duration_s << " Hz)\n";
      if (expected_frequency > 0.0) {
        if (!Within1Percent(expected_frequency, frequency) &&
            (!strstr(MessageTypeToString(message_type), "Rtcm") ||
             frequency <= 0 || frequency > expected_frequency)) {
          *error_stream << "ERROR: Frequency mismatch > 1%.\n";
          *error_stream << "\tSender: " << AioNodeToString(sender) << ";\n";
          *error_stream << "\tMessage: " << MessageTypeToString(message_type) <<
              ".\n";
          *error_stream << "\tSaw " << frequency << " Hz; was expecting " <<
              expected_frequency << " Hz.\n";
        }
      } else if (frequency > 0.0) {
        *error_stream << "WARNING: didn't expect to see those packets.\n";
        *error_stream << "\tSender: " << AioNodeToString(sender) << ";\n";
        *error_stream << "\tMessage: " << MessageTypeToString(message_type) <<
            ".\n";
        *error_stream << "\tSaw " << frequency
                      << " Hz; wasn't expecting any.\n";
      }
    }
  }
  *info_stream << "\nThe logs covered a duration of " <<
      std::fixed << std::setprecision(2) << stats.duration_s << " seconds.\n";
}

void HandleCaptureTime(const pcap_pkthdr &pcap_header, PacketStats *stats,
                       std::ostream *debug_stream) {
  // The pcap timestamps are almost but not completely monotonic.  In one
  // test, I saw about 1 misordering per file, not at the beginning or end.
  // We're assuming for now that the files come into this function in order.
  if (stats->pcap_packets <= 0) {
    stats->capture_start = pcap_header.ts;
    *debug_stream << "First packet received at "
                  << TimeValToString(stats->capture_start) << std::endl;
  } else if (TimeDiffSeconds(stats->capture_end, pcap_header.ts) < 0) {
    *debug_stream << "Time went backwards from "
                  << TimeValToString(stats->capture_end)
                  << " to " << TimeValToString(pcap_header.ts) << "!\n";
  }
  stats->capture_end = pcap_header.ts;
}

bool HandleIp(lib::pcap_reader::PcapReader *reader, bool *is_fragment,
              PacketStats *stats, std::ostream *debug_stream) {
  lib::pcap_reader::IpStatus status = reader->ReadIp();
  if (status != lib::pcap_reader::IpStatus::kBadPcapHeader) {
    HandleCaptureTime(reader->GetHeader(), stats, debug_stream);
    ++stats->pcap_packets;
  }
  switch (status) {
    case lib::pcap_reader::IpStatus::kBadEthernetHeader:
      ++stats->bad_ethernet_header;
      ++stats->bad_packets;
      return false;
    case lib::pcap_reader::IpStatus::kBadEthernetProtocol:
      ++stats->bad_ethernet_protocol;
      ++stats->bad_packets;
      return false;
    case lib::pcap_reader::IpStatus::kBadIpHeader:
      ++stats->bad_ip_header;
      ++stats->bad_packets;
      return false;
    case lib::pcap_reader::IpStatus::kBadLinkProtocol:
      ++stats->bad_link_protocol;
      ++stats->bad_packets;
      return false;
    case lib::pcap_reader::IpStatus::kBadPcapHeader:
      ++stats->bad_pcap_header;
      ++stats->bad_packets;
      return false;
    case lib::pcap_reader::IpStatus::kBadSllHeader:
      ++stats->bad_sll_header;
      ++stats->bad_packets;
      return false;
    case lib::pcap_reader::IpStatus::kFragmented:
      ++stats->fragmented_packets;
      *is_fragment = true;
      return false;
    case lib::pcap_reader::IpStatus::kValid:
      ++stats->ip_packets;
      return true;
    default:
      CHECK(false) << "Unsupported return code.";
      return false;
  }
}

bool HandleUdp(lib::pcap_reader::PcapReader *reader, PacketStats *stats) {
  switch (reader->ReadUdp()) {
    case lib::pcap_reader::UdpStatus::kBadIpProtocol:
      ++stats->bad_ip_protocol;
      ++stats->bad_packets;
      return false;
    case lib::pcap_reader::UdpStatus::kBadUdpHeader:
      ++stats->bad_udp_header;
      ++stats->bad_packets;
      return false;
    case lib::pcap_reader::UdpStatus::kValid:
      ++stats->udp_packets;
      return true;
    default:
      CHECK(false) << "Unsupported return code.";
      return false;
  }
}

bool HandleAio(lib::pcap_reader::PcapReader *reader,
               AioPacketStats *aio_stats, bool *is_duplicate) {
  ++aio_stats->total_packets;
  *is_duplicate = false;
  switch (reader->ReadAio(FLAGS_ignore_version)) {
    case lib::pcap_reader::AioStatus::kBadAioHeader:
      ++aio_stats->bad_header;
      ++aio_stats->bad_packets;
      return false;
    case lib::pcap_reader::AioStatus::kBadAioVersion:
      assert(!FLAGS_ignore_version);
      ++aio_stats->bad_version;
      ++aio_stats->bad_packets;
      return false;
    case lib::pcap_reader::AioStatus::kBadIpProtocol:
      ++aio_stats->bad_protocol;
      ++aio_stats->bad_packets;
      return false;
    case lib::pcap_reader::AioStatus::kBadMessageType:
      ++aio_stats->bad_type;
      ++aio_stats->bad_packets;
      return false;
    case lib::pcap_reader::AioStatus::kBadSource:
      ++aio_stats->bad_source;
      ++aio_stats->bad_packets;
      return false;
    case lib::pcap_reader::AioStatus::kBadUdpPort:
      ++aio_stats->bad_port;
      ++aio_stats->bad_packets;
      return false;
    case lib::pcap_reader::AioStatus::kDuplicate:
      ++aio_stats->duplicate_packets;
      *is_duplicate = true;  // Fall through
    case lib::pcap_reader::AioStatus::kValid:
      ++aio_stats->good_packets;
      return true;
    default:
      CHECK(false) << "Unsupported return code.";
      return false;
  }
}

int32_t AdjustSenderTimestamp(AioNode sender, int32_t timestamp) {
  // We adjust sender timestamps relative to the first one we saw.
  // They're only meaningful within a single source node's data anyway.
  static std::map<AioNode, int32_t> timestamps;
  if (IsValidNode(sender)) {
    if (!timestamps.count(sender)) {
      timestamps[sender] = timestamp;
    }
    timestamp -= timestamps[sender];
  }
  return timestamp;
}

void OutputPacket(int64_t ts_us, int32_t timestamp, uint32_t packet_len,
                  uint32_t source_octet, AioNode node, MessageType type,
                  const char *packet_type, std::ostream *stream) {
  static bool started = false;
  if (!g_check_message_types.empty() &&
      !g_check_message_types.count(type)) {
    return;
  }
  if (!g_check_message_sources.empty() &&
      !g_check_message_sources.count(node)) {
    return;
  }
  timestamp = AdjustSenderTimestamp(node, timestamp);
  if (FLAGS_gaps_only) {
    return;
  }
  if (ts_us >= g_start_sequence_ts_us && ts_us < g_end_sequence_ts_us) {
    if (started) {
      *stream << ",\n";
    } else {
      started = true;
    }
    char output[240];
    snprintf(
        output, sizeof(output),
        "  {'src_ip': %d, 'packet_type': \"%s\", 'node': %d, "
        "'message_type': %d, 'ts_us': %ld, 'timestamp': %d, 'len': %d}",
        source_octet, packet_type, node, type, ts_us, timestamp, packet_len);
    *stream << output;
  }
}

void OutputSequenceGap(uint32_t ip, AioNode node, MessageType type,
                       int64_t ts_us, int32_t timestamp, uint32_t packet_len,
                       std::ostream *stream) {
  static bool started = false;
  if (!g_check_message_types.empty() &&
      !g_check_message_types.count(type)) {
    return;
  }
  if (!g_check_message_sources.empty() &&
      !g_check_message_sources.count(node)) {
    return;
  }
  if (ts_us >= g_start_sequence_ts_us && ts_us < g_end_sequence_ts_us) {
    if (started) {
      *stream << ",\n";
    } else {
      started = true;
    }
    timestamp = AdjustSenderTimestamp(node, timestamp);
    char output[240];
    snprintf(output, sizeof(output),
             "  {'src_ip': %d, 'node': %d, 'message_type': %d, 'ts_us': %ld, "
             "'timestamp': %d, 'len': %d}",
             ip, node, type, ts_us, timestamp, packet_len);
    *stream << output;
  }
}

struct SequenceRecord {
  bool valid;
  int64_t ts_us;
  uint16_t last_sequence;
  int sequence_count;
  int expected_count;

  SequenceRecord() : valid(false), ts_us(0), last_sequence(0),
                     sequence_count(0), expected_count(0) {
  }
};

typedef uint32_t IP;
typedef std::pair<IP, AioNode> Sender;
typedef std::pair<Sender, MessageType> Datum;
typedef std::map<Datum, SequenceRecord> SequenceMap;

int64_t GetPeriod(AioNode sender, MessageType type) {
  if (!IsValidNode(sender) && !IsUnknownNode(sender)) {
    fprintf(stderr, "Saw unrecognized sender %d.\n", sender);
    return 0;
  }
  int32_t expected_frequency = GetMessageFrequency(sender, type);
  if (expected_frequency <= 0) {
    fprintf(stderr, "Saw gap in unexpected message %d %d.\n", sender, type);
    return 0;
  }
  return 1000000 / expected_frequency;
}

void RecordSequenceInfo(
    lib::pcap_reader::PcapReader *reader,
    int64_t ts_us,
    SequenceMap *sequence_map,
    std::ostream *gap_stream) {
  MessageType type = reader->GetAio().GetMessageType();
  AioNode node = reader->GetAio().GetSource();
  if (node == kAioNodeSimulator) {
    return;
  }
  uint32_t source_octet = reader->GetIp().GetSource() & 0x000000ff;
  uint32_t packet_len = reader->GetHeader().len;
  uint16_t cur_sequence = reader->GetAio().GetSequence();
  int32_t timestamp = reader->GetAio().GetHeader().timestamp;

  SequenceRecord& sequence_record =
      (*sequence_map)[Datum(Sender(source_octet, node), type)];
  if (sequence_record.valid) {
    if (cur_sequence == sequence_record.last_sequence) {
      // A duplicated packet, probably due to multiple networks or trunks.
      ++sequence_record.sequence_count;
      sequence_record.expected_count =
          std::max(sequence_record.expected_count,
                   sequence_record.sequence_count);
      assert(sequence_record.sequence_count < 5);
    } else if (cur_sequence + 1 == sequence_record.last_sequence) {
      // Out of order.
      // This isn't ideal; we've probably already counted it as a dropped
      // packet.  But if it's just one now and then, it'll just be a little
      // noise.  Do nothing here and let it go by.
    } else {
      while (sequence_record.sequence_count <
             sequence_record.expected_count) {
        OutputSequenceGap(source_octet, node, type, sequence_record.ts_us,
                          timestamp, packet_len, gap_stream);
        ++sequence_record.sequence_count;
      }
      ++sequence_record.last_sequence;
      sequence_record.ts_us += GetPeriod(node, type);
      while (sequence_record.last_sequence < cur_sequence) {
        for (int i = 0; i < sequence_record.expected_count; ++i) {
          OutputSequenceGap(source_octet, node, type, sequence_record.ts_us,
                            timestamp, packet_len, gap_stream);
        }
        sequence_record.ts_us += GetPeriod(node, type);
        ++sequence_record.last_sequence;
      }
      sequence_record.ts_us = ts_us;
      sequence_record.sequence_count = 1;
    }
  } else {
    sequence_record.last_sequence = cur_sequence;
    sequence_record.ts_us = ts_us;
    sequence_record.sequence_count = 1;
    sequence_record.expected_count = 1;
    sequence_record.valid = true;
  }
}

void HandlePcap(lib::pcap_reader::PcapReader *reader,
                PacketStats *stats,
                NodeRecord *cur_node_record,
                NodeRecord *sum_node_record,
                SequenceMap *sequence_map,
                std::ostream *error_stream,
                std::ostream *debug_stream,
                std::ostream *gap_stream,
                std::ostream *packet_stream) {
  uint32_t packet_len = reader->GetHeader().len;
  int64_t ts_us = TsToTimeUs(reader->GetHeader().ts) -
      TsToTimeUs(stats->capture_start);

  ++stats->total_packets;
  stats->total_bytes += packet_len;
  bool is_fragment = false;

  // Handle IP layer.
  if (!HandleIp(reader, &is_fragment, stats, debug_stream)) {
    if (is_fragment) {  // IP header is valid.
      uint32_t source_octet = reader->GetIp().GetSource() & 0x000000ff;
      uint32_t dest_octet = reader->GetIp().GetDestination() & 0x000000ff;
      uint32_t capture_len = reader->GetHeader().caplen;
      OutputPacket(ts_us, -1, capture_len, source_octet, kAioNodeUnknown,
                   (MessageType)dest_octet, kPacketTypeFragment,
                   packet_stream);
    } else {
      OutputPacket(ts_us, -1, packet_len, -1, kAioNodeUnknown,
                   kNumMessageTypes, kPacketTypeUnknown, packet_stream);
    }
    return;
  }

  uint32_t source_octet = reader->GetIp().GetSource() & 0x000000ff;

  // Handle UDP layer.
  if (!HandleUdp(reader, stats)) {
    uint32_t dest_octet = reader->GetIp().GetDestination() & 0x000000ff;
    OutputPacket(ts_us, -1, packet_len, source_octet, kAioNodeUnknown,
                 (MessageType)dest_octet, kPacketTypeUnknown, packet_stream);
    return;
  }
  if (reader->GetUdp().GetDestination() == UDP_PORT_WINCH) {
    ++stats->winch_packets;
    OutputPacket(ts_us, -1, packet_len, source_octet, kAioNodeUnknown,
                 kNumMessageTypes, kPacketTypeWinch, packet_stream);
    return;
  } else if (reader->GetUdp().GetDestination() == UDP_PORT_NET_PROBE) {
    ++stats->probe_packets;
    OutputPacket(ts_us, -1, packet_len, source_octet, kAioNodeUnknown,
                 kNumMessageTypes, kPacketTypeProbe, packet_stream);
    return;
  } else if (reader->GetUdp().GetDestination() != UDP_PORT_AIO) {
    OutputPacket(ts_us, -1, packet_len, source_octet, kAioNodeUnknown,
                 kNumMessageTypes, kPacketTypeUnknown, packet_stream);
    return;
  }

  // Handle AIO layer.
  bool is_duplicate;
  if (HandleAio(reader, &stats->aio, &is_duplicate)) {
    const lib::pcap_reader::AioReader &aio_reader = reader->GetAio();
    int32_t timestamp = aio_reader.GetHeader().timestamp;
    AioNode node = aio_reader.GetSource();
    MessageType type = aio_reader.GetMessageType();
    OutputPacket(ts_us, timestamp, packet_len, source_octet, node, type,
                 kPacketTypeAio, packet_stream);
    RecordSequenceInfo(reader, ts_us, sequence_map, gap_stream);
    if (!is_duplicate) {
      stats->unique_aio_bytes += reader->GetHeader().len;
      RecordAioInfo(*reader, cur_node_record, error_stream);
      RecordAioInfo(*reader, sum_node_record, error_stream);
    }
  } else {
    OutputPacket(ts_us, -1, packet_len, source_octet, kAioNodeUnknown,
                 kNumMessageTypes, kPacketTypeUnknown, packet_stream);
  }
}

void ProcessPcapFile(const char *filename, lib::pcap_reader::PcapReader *reader,
                     PacketStats *stats, NodeRecord *cur_node_record,
                     NodeRecord *sum_node_record, SequenceMap *sequence_map,
                     std::ostream *error_stream,
                     std::ostream *debug_stream,
                     std::ostream *gap_stream,
                     std::ostream *packet_stream) {
  if (reader->OpenOffline(filename)) {
    while (reader->ReadNext()) {
      HandlePcap(reader, stats, cur_node_record, sum_node_record, sequence_map,
                 error_stream, debug_stream, gap_stream, packet_stream);
    }
    reader->Close();
  } else {
    ++stats->failed_files;
    *error_stream << "Pcap could not open " << filename << ":"
                  << reader->GetError() << std::endl;
  }
}

std::string GetFileInfo(const char *filename) {
  std::ostringstream output;
  struct stat buf;
  if (!stat(filename, &buf)) {
    if (!S_ISREG(buf.st_mode)) {
      output << "Not a regular file; mode was " << buf.st_mode << std::endl;
    }
    output << "Size: " << buf.st_size << std::endl;
    struct tm local_tm;
    char timebuf[80];
    if (!localtime_r(&buf.st_mtime, &local_tm)) {
      output << "Failed to convert mtime to localtime.\n";
    } else if (strftime(timebuf, 80, "%Y%m%d-%H%M%S", &local_tm) == 0) {
      output << "Failed to convert mtime to string.\n";
    } else {
      output << "Mtime: " << timebuf << " (" << buf.st_mtime << ")"
             << std::endl;
    }

  } else {
    output << "Failed to stat file '" << filename << "' with errno " << errno
           << std::endl;
  }
  return output.str();
}

void ProcessPackets(bool detailed_stats, int32_t argc, char *argv[]) {
  lib::pcap_reader::PcapReader pcap_reader;
  NodeRecord last_node_record;
  NodeRecord sum_node_record;
  PacketStats stats;
  std::ostringstream info_stream;
  info_stream.precision(4);
  std::ostringstream error_stream;
  error_stream.precision(4);
  // This is here just to debug a specific fatal error; see b/31801366.
  std::ostringstream debug_stream;
  debug_stream.precision(4);
  std::ostringstream gap_stream;
  std::ostringstream packet_stream;
  SequenceMap sequence_map;

  memset(&stats, 0, sizeof(stats));
  debug_stream << "Invocation:";
  for (int32_t i = 0; i < argc; ++i) {
    debug_stream << " " << argv[i];
  }
  debug_stream << std::endl;
  const char *last_filename = NULL;
  for (int32_t i = 1; i < argc; ++i) {
    const char *filename = argv[i];
    NodeRecord cur_node_record;
    debug_stream << "Processing file '" << filename << "'.\n";
    debug_stream << GetFileInfo(filename);
    ProcessPcapFile(argv[i], &pcap_reader, &stats, &cur_node_record,
                    &sum_node_record, &sequence_map, &error_stream,
                    &debug_stream, &gap_stream, &packet_stream);
    if (last_node_record.size()) {
      CHECK_NOTNULL(last_filename);
      CheckNodeRecord(last_node_record, cur_node_record, &error_stream,
                      last_filename, filename);
    }
    last_node_record = cur_node_record;
    last_filename = filename;
  }
  double time_diff_s = TimeDiffSeconds(stats.capture_start,
                                       stats.capture_end);
  if (time_diff_s <= 0.0) {
    std::ostringstream fatal_errors;
    fatal_errors << "Duration was negative: " << time_diff_s << std::endl;
    fatal_errors << "capture_start: " << TimeValToString(stats.capture_start)
                 << std::endl;
    fatal_errors << "capture_end: " << TimeValToString(stats.capture_end)
                 << std::endl;
    fatal_errors << debug_stream.str() << std::endl;
    fatal_errors << error_stream.str() << std::endl;
    fatal_errors << info_stream.str() << std::endl;
    LOG(FATAL) << fatal_errors.str();
  }
  CHECK(time_diff_s > 0.0);
  stats.duration_s = time_diff_s;

  DumpStats(stats, detailed_stats, &info_stream);
  ProcessNodeRecords(sum_node_record, stats, &info_stream, &error_stream);
  std::cout << info_stream.str() << std::endl;
  std::cout << error_stream.str() << std::endl;
  if (!FLAGS_data_file.empty()) {
    std::ofstream data_file(FLAGS_data_file);
    if (data_file.is_open()) {
      data_file << "gaps = [\n" << gap_stream.str() << "]\n" << std::endl;
      data_file << "packets = [\n" << packet_stream.str() << "]" << std::endl;
    } else {
      fprintf(stderr, "Failed to open data_file '%s'\n",
              FLAGS_data_file.c_str());
    }
  }
}

template <typename T> std::set<T> ConvertCommaSeparatedList(
    const std::string input, T (*convert)(const char *), T invalid,
    std::string *rejected) {
  std::string token;
  std::set<T> output;
  std::istringstream s(input);
  while (std::getline(s, token, ',')) {
    T converted = convert(token.c_str());
    if (converted == invalid) {
      *rejected = token;
      output.clear();
      break;
    } else {
      output.insert(converted);
    }
  }
  return output;
}

}  // namespace

int main(int argc, char *argv[]) {
  google::SetUsageMessage(std::string("\nUsage: ") + argv[0] +
                          " 1.pcap [2.pcap [...]]");
  google::ParseCommandLineFlags(&argc, &argv, true);

  if (argc < 2) {
    google::ShowUsageWithFlagsRestrict(argv[0], "validate_pcap.cc");
    return 1;
  } else if (!FLAGS_data_file.empty()) {
    if (FLAGS_seq_check_start_ms < 0 ||
        FLAGS_seq_check_duration_ms <= 0) {
      google::ShowUsageWithFlagsRestrict(argv[0], "validate_pcap.cc");
      return 1;
    } else {
      g_start_sequence_ts_us = FLAGS_seq_check_start_ms * 1000;
      g_end_sequence_ts_us = g_start_sequence_ts_us +
          FLAGS_seq_check_duration_ms * 1000;
      if (!FLAGS_check_message_type.empty()) {
        std::string error_type;
        g_check_message_types =
            ConvertCommaSeparatedList(FLAGS_check_message_type,
                                      StringToMessageType, kNumMessageTypes,
                                      &error_type);
        if (!g_check_message_types.size()) {
          std::cerr << "Unrecognized message type '" << error_type
                    << "'.\n";
          return 1;
        }
      }
      if (!FLAGS_check_message_source.empty()) {
        std::string error_source;
        g_check_message_sources =
            ConvertCommaSeparatedList(FLAGS_check_message_source,
                                      StringToAioNode, kAioNodeForceSigned,
                                      &error_source);
        if (!g_check_message_sources.size()) {
          std::cerr << "Unrecognized message source '" << error_source
                    << "'.\n";
          return 1;
        }
      }
    }
  } else {
    google::CommandLineFlagInfo seq_start_info =
        google::GetCommandLineFlagInfoOrDie("seq_check_start_ms");
    google::CommandLineFlagInfo seq_time_info =
        google::GetCommandLineFlagInfoOrDie("seq_check_duration_ms");
    google::CommandLineFlagInfo gaps_only_info =
        google::GetCommandLineFlagInfoOrDie("gaps_only");
    if (!seq_start_info.is_default || !seq_time_info.is_default ||
        !gaps_only_info.is_default || !FLAGS_check_message_type.empty()) {
      // Can't set time bracket or specify gaps_only without a filename.
      google::ShowUsageWithFlagsRestrict(argv[0], "validate_pcap.cc");
      return 1;
    }
  }

  ProcessPackets(FLAGS_detailed_stats, argc, argv);
  return 0;
}

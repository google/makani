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
#include <stdint.h>
#include <stdio.h>

#include <queue>
#include <vector>

#include "avionics/common/aio_header.h"
#include "lib/pcap_reader/pcap_reader.h"

DEFINE_string(output_file, "", "The path of the output data file.");
DEFINE_string(node, "", "AioNode to process.");
DEFINE_bool(ignore_aio_version, false, "Whether to ignore the AIO version.");
DEFINE_int32(port, 0, "Port number to process in SerialDebugMessage.");

namespace {

// Checks that the required file names are non-empty.
bool ValidateFileName(const char * /* flag_name */, const std::string &value) {
  return value.size() > 0;
}

bool validate_output_file __attribute__((unused)) = (
    google::RegisterFlagValidator(&FLAGS_output_file, &ValidateFileName));

bool ValidateNode(const char * /* flag_name */, const std::string &value) {
  bool found = false;
  for (int32_t i = 0; i < kNumAioNodes && !found; ++i) {
    AioNode aio_node = static_cast<AioNode>(i);
    found = (value.compare(AioNodeToString(aio_node)) == 0
             || value.compare(AioNodeToShortString(aio_node)) == 0);
  }
  if (!found) {
    LOG(ERROR) << "Invalid AioNode: " << value;
    return false;
  }
  return true;
}

bool validate_node __attribute__((unused)) = (
    google::RegisterFlagValidator(&FLAGS_node, &ValidateNode));

}  // namespace

// Base class for sort comparisons. We use the sequence number in
// aio_header_ to sort messages.
class AioMessageBase {
 public:
  AioMessageBase() : aio_header_() {}
  virtual ~AioMessageBase() {}

  AioHeader aio_header_;
};

class AioMessageCompare {
 public:
  // This compare operation handles rollovers provided that the list contains
  // less than half of the sequence number range. We mitigate this issue by
  // limiting the list size.
  bool operator()(const AioMessageBase &a, const AioMessageBase &b) {
    return (int16_t)(a.aio_header_.sequence - b.aio_header_.sequence) < 0;
  }
};

class SerialDebugContainer : public AioMessageBase {
 public:
  explicit SerialDebugContainer(const lib::pcap_reader::AioReader &aio_reader)
      : message_() {
    AioMessageData data;
    aio_reader.UnpackData(&data);
    aio_header_ = aio_reader.GetHeader();
    message_ = data.serial_debug;
  }
  ~SerialDebugContainer() {}

  SerialDebugMessage message_;
};

class PcapToSerial {
 public:
  PcapToSerial(AioNode node, int32_t port, bool ignore_aio_version)
      : list_(),
        sequence_z1_(),
        sequence_z1_valid_(false),
        node_(node),
        port_(port),
        file_(nullptr),
        ignore_aio_version_(ignore_aio_version) {}
  ~PcapToSerial() { Close(); }

  bool Open(const std::string &filename);
  void Close(void);

  bool HandlePcapMessage(lib::pcap_reader::PcapReader *reader);

  static bool HandlePcapMessageWrapper(lib::pcap_reader::PcapReader *reader,
                                       PcapToSerial *ths) {
    return ths->HandlePcapMessage(reader);
  }

 private:
  typedef std::list<SerialDebugContainer> MessageList;

  void ProcessMessage(const SerialDebugContainer &m);
  void HandleEndOfFile(void);

  MessageList list_;
  uint16_t sequence_z1_;  // For deduplication.
  bool sequence_z1_valid_;
  const AioNode node_;  // AioNode to process.
  const int32_t port_;  // Port number in SerialDebugMessage to process.
  FILE *file_;

  bool ignore_aio_version_;

  // Limit the maximum size of the message list to mitigate potential issues
  // with sequence number rollovers. The list size should handle the maximum
  // unsorted distance in sequence numbers, yet still have a low probability
  // that the list spans no more than half the sequence number range.
  static constexpr size_t kMaxListSize = 1024;

  DISALLOW_COPY_AND_ASSIGN(PcapToSerial);
};

bool PcapToSerial::Open(const std::string &filename) {
  if (file_ != nullptr) {
    Close();
  }
  file_ = fopen(filename.c_str(), "w");

  return file_ != nullptr;
}

void PcapToSerial::Close(void) {
  if (file_ != nullptr) {
    HandleEndOfFile();
    fclose(file_);
    file_ = nullptr;
  }
}

bool PcapToSerial::HandlePcapMessage(lib::pcap_reader::PcapReader *reader) {
  // Accept duplicates and out-of-order messages from AioReader since we
  // sort messages.
  if (!IsBadStatus(reader->ReadAio(ignore_aio_version_))) {
    const lib::pcap_reader::AioReader &aio_reader = reader->GetAio();
    if (aio_reader.GetSource() == node_
        && aio_reader.GetMessageType() == kMessageTypeSerialDebug) {
      list_.push_back(SerialDebugContainer(aio_reader));
      if (list_.size() >= kMaxListSize) {
        list_.sort(AioMessageCompare());
        while (list_.size() > kMaxListSize / 2) {
          ProcessMessage(list_.front());
          list_.pop_front();
        }
      }
    }
  }
  return true;
}

void PcapToSerial::HandleEndOfFile(void) {
  // Process remaining messages in list.
  list_.sort(AioMessageCompare());
  while (!list_.empty()) {
    ProcessMessage(list_.front());
    list_.pop_front();
  }
}

void PcapToSerial::ProcessMessage(const SerialDebugContainer &m) {
  // We only need to deduplicate messages here since they're sorted.
  if (!sequence_z1_valid_ || m.aio_header_.sequence != sequence_z1_) {
    sequence_z1_ = m.aio_header_.sequence;
    sequence_z1_valid_ = true;

    if (m.message_.length > 0 && m.message_.port == port_) {
      if (fwrite(m.message_.data, m.message_.length, 1, file_) != 1) {
        LOG(ERROR) << "Write error: " << strerror(errno);
      }
    }
  }
}

int main(int argc, char *argv[]) {
  google::InitGoogleLogging(argv[0]);
  google::SetUsageMessage(
      "\nUsage: pcap_to_serial -output_file=output.raw 1.pcap [2.pcap] ...\n");
  google::ParseCommandLineFlags(&argc, &argv, true);

  if (FLAGS_output_file.empty() || argc < 2) {
    google::ShowUsageWithFlagsRestrict(argv[0], "pcap_to_serial");
    return EXIT_FAILURE;
  }

  PcapToSerial pcap_to_serial(StringToAioNode(FLAGS_node.c_str()), FLAGS_port,
                              FLAGS_ignore_aio_version);
  if (!pcap_to_serial.Open(FLAGS_output_file)) {
    LOG(ERROR) << "Write error: " << strerror(errno);
    return EXIT_FAILURE;
  }

  // Process consecutive log files.
  lib::pcap_reader::PcapReader pcap_reader;
  if (pcap_reader.ForEach(argc - 1, argv + 1,
                          PcapToSerial::HandlePcapMessageWrapper,
                          &pcap_to_serial) < 0) {
    LOG(FATAL) << "Pcap error: " << pcap_reader.GetError();
  }
  pcap_to_serial.Close();

  return EXIT_SUCCESS;
}

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

#ifndef LIB_PCAP_READER_PCAP_READER_H_
#define LIB_PCAP_READER_PCAP_READER_H_

#include <pcap/pcap.h>
#include <stdint.h>
#include <sys/time.h>

#include <string>

#include "lib/pcap_reader/aio_reader.h"
#include "lib/pcap_reader/ip_reader.h"
#include "lib/pcap_reader/udp_reader.h"

namespace lib {
namespace pcap_reader {

// Class to read and validate PCAP network packets.
class PcapReader {
 public:
  PcapReader(void);
  ~PcapReader(void) { Close(); }

  bool OpenOffline(const std::string &filename);
  void Close(void);

  // Reset stored state. Call this function before parsing unrelated files to
  // clear the internal state.
  void Reset(void);

  // Read next record in PCAP file.
  bool ReadNext(void);
  const pcap_pkthdr &GetHeader(void) const { return pcap_header_; }
  int64_t GetAbsoluteTimestamp(void) const {
    return pcap_header_.ts.tv_sec * 1000000 + pcap_header_.ts.tv_usec;
  }
  int64_t GetRelativeTimestamp(void) const {
    timeval rel_time;
    timersub(&pcap_header_.ts, &min_pcap_time_, &rel_time);
    return rel_time.tv_sec * 1000000 + rel_time.tv_usec;
  }
  const char *GetError(void) const { return pcap_errbuf_; }
  const timeval &GetMinPcapTime(void) const { return min_pcap_time_; }

  // Read IP packet header.
  IpStatus ReadIp(void);
  const IpReader &GetIp(void) const { return ip_reader_; }

  // Read UDP packet header.
  UdpStatus ReadUdp(void);
  const UdpReader &GetUdp(void) const { return udp_reader_; }

  // Read AIO packet header.
  AioStatus ReadAio(bool ignore_aio_version = false);
  const AioReader &GetAio(void) const { return aio_reader_; }

  // Process all messages in a file.
  // Args:
  //   filename: A pcap file to process.
  //   func: A function to process each record. It should return true to
  //     continue processing, or false to terminate processing.
  //    arg: An arbitrary argument to pass to func.
  // Returns:
  //   -1=on error, 0=when terminated early, 1=processed all records.
  template<typename T>
  int32_t ForEach(const std::string &filename,
                  bool (* const func)(PcapReader *reader, T *arg), T *arg) {
    if (OpenOffline(filename)) {
      int32_t processed_all_records = 1;
      while (processed_all_records > 0 && ReadNext()) {
        processed_all_records = func(this, arg);
      }
      Close();
      return processed_all_records;
    }
    return -1;
  }

  // Process all messages in a series of files.
  // Args:
  //   num_filenames: The number of file names in the filenames[] array.
  //   filenames: An array of pcap file names to process.
  //   func: A function to process each record. It should return true to
  //     continue processing, or false to terminate processing.
  //    arg: An arbitrary argument to pass to func.
  // Returns:
  //   -1=on error, 0=when terminated early, 1=processed all records.
  template<typename T>
  int32_t ForEach(int32_t num_filenames, char *filenames[],
                  bool (* const func)(PcapReader *reader, T *arg), T *arg) {
    int32_t processed_all_records =
        CheckIfAllFilesExist(num_filenames, filenames) ? 1 : -1;
    for (int32_t i = 0; i < num_filenames && processed_all_records > 0; ++i) {
      processed_all_records = ForEach(filenames[i], func, arg);
    }
    return processed_all_records;
  }

 private:
  void ResetFlags(void);
  void UpdateMinPcapTime(const timeval &pcap_time);
  bool CheckIfAllFilesExist(int32_t num_filenames, char *filenames[]);

  // Store minimum time for relative time calculations (e.g., log analysis).
  timeval min_pcap_time_;

  // Pcap state.
  pcap_t *pcap_handle_;
  pcap_pkthdr pcap_header_;
  const uint8_t *pcap_payload_;
  char pcap_errbuf_[PCAP_ERRBUF_SIZE];

  // IP state.
  IpReader ip_reader_;
  IpStatus ip_status_;
  bool ip_status_valid_;

  // UDP state.
  UdpReader udp_reader_;
  UdpStatus udp_status_;
  bool udp_status_valid_;

  // AIO state.
  AioReader aio_reader_;
  AioStatus aio_status_;
  bool aio_status_valid_;

  DISALLOW_COPY_AND_ASSIGN(PcapReader);
};

}  // namespace pcap_reader
}  // namespace lib

#endif  // LIB_PCAP_READER_PCAP_READER_H_

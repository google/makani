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

#ifndef LIB_PCAP_TO_HDF5_MESSAGE_LOG_H_
#define LIB_PCAP_TO_HDF5_MESSAGE_LOG_H_

#include <hdf5.h>
#include <stdint.h>

#include <map>
#include <string>
#include <utility>
#include <vector>

#include "avionics/network/aio_node.h"
#include "avionics/network/message_type.h"
#include "lib/pcap_reader/pcap_reader.h"
#include "lib/pcap_to_hdf5/capture_info.h"

namespace lib {

namespace pcap_to_hdf5 {

// ChunkedDataset is responsible for managing a single HDF5 dataset,
// including creating and closing the dataset and buffering packets
// before calls to H5Dwrite.  Earlier versions of this code called
// H5Dwrite with each incoming packet, but a significant speedup was
// gained by writing packets in groups of ~100.  For convenience (and
// at the advice of some HDF5 documentation), the size of this buffer
// is taken to be equal to the size of the dataset's chunks.
class ChunkedDataset {
 public:
  // Setup an expanding dataset.
  //
  // The dataset is not created until Create() is called.
  ChunkedDataset();
  ~ChunkedDataset() {
    if (dataset_id_ >= 0) Close();
  }

  // Creates a chunked (therefore resizeable) dataset.
  //
  // This call creates the HDF5 dataset and allocates memory for the
  // internal buffer data.  It should only be called once.  The
  // ChunkedDataset must be closed *before* type_tid is closed.
  //
  // Args:
  //   group_id: Group or file to add this dataset to.
  //   dataset_name: Name of the dataset to add.
  //   type_id: Type of the dataset.
  //
  // Returns:
  //   True if the dataset was succesfully created.
  bool Create(hid_t group_id, const char *dataset_name, hid_t type_id);

  // Writes any buffered data then closes the HDF5 dataset.
  void Close();

  // Places a datum in the data buffer and writes out that buffer if
  // it is full.
  void Write(const uint8_t *datum);

 private:
  hsize_t GetNumBuffered() const;
  void WriteBuffered();

  // This value sets the chunk size for the HDF5 file.  There is an
  // overhead for chunking. If the chunk size is really small, then
  // your files will be larger and there could be performance issues.
  // The minimum is 1 record -- then there is no padding in the final
  // file.
  static constexpr hsize_t kChunkDim = 250;

  // This constant is used with H5Pset_deflate to set the GNU gzip
  // compression level (see CreateDataset).
  static constexpr uint32_t kCompressionLevel = 2;

  // These next three constants are used for H5Pset_chunk_cache (see
  // CreateDataset).

  // 4 MByte cache for the chunks.
  static constexpr size_t kRawDataChunkCacheNBytes = 4 * 1024 * 1024;

  // Slots value should ideally be a prime number.  As a rule of
  // thumb, this value should be at least 10 times the number of
  // chunks that can fit in kRawDataChunkCacheNBytes bytes.  For
  // maximum performance, this value should be set approximately 100
  // times the number of chunks.
  static constexpr size_t kRawDataChunkCacheNSlots = 15083;

  // If the application only reads or writes data once, this can be
  // safely set to 1.  Otherwise, this should be set lower, depending
  // on how often you re-read or re-write the same data.
  static constexpr double kRawDataChunkCachePremptionPolicy = 1.0;

  // ID of the dataset owned by the object or -1 if no dataset is open.
  hid_t dataset_id_;
  // Copy of the type used to create dataset_id_.
  //
  // This resource is owned by the caller, however comparing types for
  // equality results in a performance hit unless the IDs are
  // identical, so we cache the value here.
  hid_t type_id_;
  // Cached value of H5Tget_size(type_id_).
  hsize_t message_size_;
  // Total number of messages received via Write() (always non-negative).
  hsize_t offset_;
  // Total number of messages written via H5Dwrite (always
  // non-negative and less than or equal to offset_).
  hsize_t offset_last_written_;
  // Buffer of up to kChunkDim messages received but not yet written
  // via H5Dwrite.
  std::vector<uint8_t> buffer_;
};

struct SyncTimeStats {
  // Intermediate values to compute running linear regressions that compute
  // GPS timestamp as functions of the capture header timestamp. E.g,
  // y = ax + b where x is capture time and y is GPS time. `a` is computed as
  // cov(xy) / var(x), and `b` is computed as mean(y) - a*mean(x). All of
  // those components can be computed as Pcap reader scans through the message
  // sequence.
  double mean_x, mean_y, var_x, cov_xy, count;
  std::map<AioNode, double> tow_ms_rollover;
  std::map<AioNode, double> last_tow_ms;
  SyncTimeStats()
      : mean_x(0.0),
        mean_y(0.0),
        var_x(0.0),
        cov_xy(0.0),
        count(0.0),
        tow_ms_rollover(),
        last_tow_ms() {}
  ~SyncTimeStats() {}
};

class MessageLog {
 public:
  explicit MessageLog(const std::string &filename)
      : parameter_tids_(),
        bad_packet_info_tid_(-1),
        bad_packet_info_did_(-1),
        bad_packets_dataset_(),
        log_info_tid_(-1),
        log_info_did_(-1),
        log_info_dataset_(),
        message_type_ids_(),
        message_sizes_(),
        buffers_(),
        file_id_(-1),
        messages_gid_(-1),
        source_gids_(),
        datasets_(),
        gs_sync_time_stats_(),
        wing_sync_time_stats_() {
    SetupTypes();
    CreateFile(filename);
  }
  ~MessageLog();

  void WriteData(const CaptureHeader &capture_header,
                 const AioHeader &aio_header, const uint8_t *data, int32_t len);

  void CalcTimingStats(const AioHeader &aio_header,
                       const CaptureHeader &capture_header,
                       const uint8_t *data);

  // Write data with spoofed capture and AIO headers.
  //
  // Args:
  //   source_node: Source node under which to log the data.
  //   message_type: Type of the msessage.
  //   timestamp_us: Timestamp to use in the capture and AIO headers.
  //   buffer: Buffer containing the packed message only.
  //   length: Length of the buffer.
  void WriteWithSpoofedHeaders(AioNode source_node, MessageType message_type,
                               int64_t timestamp_us, const uint8_t *buffer,
                               int32_t length);

  void WriteBadPacketInfo(const BadPacketInfo &bad_packet);
  void WriteLogInfo(const lib::pcap_reader::PcapReader &reader);

 private:
  void AddParameterTypes(hid_t parameters_gid, const std::string &name,
                         const void *data_ptr, int32_t size);
  void SetupTypes();
  void CreateParameterDatasets();
  void CreateFile(const std::string &filename);

  void UpdateSyncTimeStats(const NovAtelObservationsMessage &message,
                           AioNode source, const CaptureHeader &capture_header,
                           SyncTimeStats *sync_time_stats);

  // Information for making the parameter types.
  std::map<std::string, std::pair<hid_t, const void *>> parameter_tids_;

  // Handles for the bad packet information.
  hid_t bad_packet_info_tid_;
  hid_t bad_packet_info_did_;
  ChunkedDataset bad_packets_dataset_;

  // Handles for the log information.
  hid_t log_info_tid_;
  hid_t log_info_did_;
  ChunkedDataset log_info_dataset_;

  // HDF5 types for each MessageType.
  std::map<MessageType, hid_t> message_type_ids_;
  // Sizes of each message.  These are cached to avoid calling the
  // HDF5 library in the write loop.
  std::map<MessageType, hsize_t> message_sizes_;

  // Buffers for the packet + AioHeader used for each message type.
  std::map<MessageType, std::vector<uint8_t>> buffers_;

  // Handles for the file, /messages group.
  hid_t file_id_, messages_gid_;

  // Handles from the groups /<source name>.
  std::map<AioNode, hid_t> source_gids_;

  // All of the datasets at /<source name>/<message type name>.
  std::map<std::pair<AioNode, MessageType>, ChunkedDataset> datasets_;

  // Stats used to compute running linear fitting from capture time to gps time.
  SyncTimeStats gs_sync_time_stats_, wing_sync_time_stats_;
};

}  // namespace pcap_to_hdf5

}  // namespace lib

#endif  // LIB_PCAP_TO_HDF5_MESSAGE_LOG_H_

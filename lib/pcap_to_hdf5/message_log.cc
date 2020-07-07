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

#include "lib/pcap_to_hdf5/message_log.h"

#include <glog/logging.h>
#include <hdf5.h>
#include <hdf5_hl.h>
#include <stdint.h>

#include <cstring>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "avionics/common/aio_header.h"
#include "avionics/common/aio_version_code.h"
#include "avionics/common/build_info.h"
#include "avionics/common/network_config.h"
#include "avionics/common/pack_aio_header.h"
#include "avionics/common/pack_avionics_messages.h"
#include "avionics/network/aio_node.h"
#include "avionics/network/aio_node_to_ip_address.h"
#include "avionics/network/message_type.h"
#include "control/control_params.h"
#include "control/system_params.h"
#include "lib/pcap_reader/pcap_reader.h"
#include "lib/pcap_to_hdf5/capture_info.h"
#include "lib/pcap_to_hdf5/hdf5_format.h"
#include "sim/sim_params.h"

namespace lib {

namespace pcap_to_hdf5 {

ChunkedDataset::ChunkedDataset()
    : dataset_id_(-1),
      type_id_(-1),
      message_size_(0),
      offset_(0),
      offset_last_written_(0),
      buffer_() {}

bool ChunkedDataset::Create(hid_t group_id, const char *dataset_name,
                            hid_t type_id) {
  CHECK_EQ(-1, dataset_id_);

  // Create the dataspace.
  constexpr int kDatasetRank = 1;
  hsize_t dims[kDatasetRank] = {0};  // Start empty.
  hsize_t max_dims[kDatasetRank] = {H5S_UNLIMITED};
  hid_t dataspace_id = H5Screate_simple(kDatasetRank, dims, max_dims);

  // The dataset needs to be chunked so it can be extended indefinitely during
  // the duration of the recording.
  hid_t dataset_create_pid = H5Pcreate(H5P_DATASET_CREATE);
  CHECK_LE(0, dataset_create_pid);

  // H5Pset_chunk_cache is used to adjust the chunk cache parameters on a
  // per-dataset basis, as opposed to a global setting for the file.
  // This local variable is used as we cannot get the pointer address to a
  // static class variable.
  hsize_t chunk_dims[kDatasetRank] = {kChunkDim};
  CHECK_LE(0, H5Pset_chunk(dataset_create_pid, kDatasetRank, chunk_dims));
  CHECK_LE(0, H5Pset_deflate(dataset_create_pid, kCompressionLevel));

  hid_t dataset_access_pid = H5Pcreate(H5P_DATASET_ACCESS);
  CHECK_LE(0, dataset_access_pid);
  CHECK_LE(0, H5Pset_chunk_cache(dataset_access_pid, kRawDataChunkCacheNSlots,
                                 kRawDataChunkCacheNBytes,
                                 kRawDataChunkCachePremptionPolicy));

  dataset_id_ = H5Dcreate2(group_id, dataset_name, type_id, dataspace_id,
                           H5P_DEFAULT, dataset_create_pid, dataset_access_pid);

  // Close these resources.
  H5Pclose(dataset_access_pid);
  H5Pclose(dataset_create_pid);
  H5Sclose(dataspace_id);

  if (dataset_id_ < 0) return false;

  type_id_ = type_id;
  message_size_ = H5Tget_size(type_id);
  offset_ = 0U;
  offset_last_written_ = 0U;
  buffer_.resize(message_size_ * kChunkDim);

  return true;
}

void ChunkedDataset::Close() {
  CHECK_LE(0, dataset_id_);
  WriteBuffered();
  H5Dclose(dataset_id_);
  dataset_id_ = -1;
}

void ChunkedDataset::Write(const uint8_t *datum) {
  CHECK_LE(0, dataset_id_);
  hsize_t num_buffered = GetNumBuffered();
  CHECK_GT(kChunkDim, num_buffered);
  memcpy(buffer_.data() + message_size_ * num_buffered, datum, message_size_);
  offset_++;
  if (GetNumBuffered() == kChunkDim) WriteBuffered();
}

hsize_t ChunkedDataset::GetNumBuffered() const {
  CHECK_LE(offset_last_written_, offset_);
  hsize_t num_buffered = offset_ - offset_last_written_;
  return num_buffered;
}

// Write all buffered data.
void ChunkedDataset::WriteBuffered() {
  CHECK_LE(0, dataset_id_);
  hsize_t num_buffered = GetNumBuffered();
  CHECK_GE(kChunkDim, num_buffered);
  if (num_buffered > 0) {
    constexpr int kDatasetRank = 1;
    hid_t file_dataspace_id = H5Dget_space(dataset_id_);
    CHECK_LE(0, file_dataspace_id);
    CHECK_EQ(kDatasetRank, H5Sget_simple_extent_ndims(file_dataspace_id));

    hsize_t dims[kDatasetRank];
    CHECK_LE(0, H5Sget_simple_extent_dims(file_dataspace_id, dims, NULL));
    // Expanding the dataset requires closing the dataspace.
    H5Sclose(file_dataspace_id);

    dims[0] += num_buffered;
    H5Dset_extent(dataset_id_, dims);

    // Re-open the dataspace.
    file_dataspace_id = H5Dget_space(dataset_id_);
    CHECK_LE(0, file_dataspace_id);

    // Re-use dims here for the source dataspace.
    dims[0] = num_buffered;
    hid_t memory_dataspace_id = H5Screate_simple(kDatasetRank, dims, NULL);
    CHECK_LE(0, memory_dataspace_id);

    CHECK_LE(0, H5Sselect_hyperslab(file_dataspace_id, H5S_SELECT_SET,
                                    &offset_last_written_, NULL, dims, NULL));

    // Write this part of the (expanding) data set.
    CHECK_LE(0, H5Dwrite(dataset_id_, type_id_, memory_dataspace_id,
                         file_dataspace_id, H5P_DEFAULT, buffer_.data()));
    H5Sclose(memory_dataspace_id);
    H5Sclose(file_dataspace_id);

    offset_last_written_ = offset_;
  }
}

// Write a buffer to a dataset if possible.
//
// Args:
//   capture_header: Additional information about the data capture.
//   aio_header: AioHeader for the incoming data.
//   data: Data to be written.  Should consist of the AioHeader and payload.
//   len: Combined length of the AioHeader and payload.
void MessageLog::WriteData(const CaptureHeader &capture_header,
                           const AioHeader &aio_header, const uint8_t *data,
                           int32_t len) {
  MessageType type = static_cast<MessageType>(aio_header.type);
  AioNode source = static_cast<AioNode>(aio_header.source);
  BadPacketInfo bad_packet = {kBadPacketReasonNone, capture_header, aio_header,
                              len};

  // Compare destination multicast address to IpAddress.
  if (IpAddressToUint32(AioMessageTypeToIpAddress(type)) !=
      capture_header.destination_address) {
    bad_packet.reason = kBadPacketReasonAioAddress;
    WriteBadPacketInfo(bad_packet);
    return;
  }

  if (source_gids_.find(source) == source_gids_.end()) {
    bad_packet.reason = kBadPacketReasonAioSource;
    WriteBadPacketInfo(bad_packet);
    return;
  }

  if (message_type_ids_.find(type) == message_type_ids_.end()) {
    bad_packet.reason = kBadPacketReasonAioType;
    WriteBadPacketInfo(bad_packet);
    return;
  }

  hsize_t message_size = message_sizes_[type];

  if (message_size != len + sizeof(capture_header)) {
    bad_packet.reason = kBadPacketReasonAioLength;
    WriteBadPacketInfo(bad_packet);
    return;
  }

  CHECK_EQ(sizeof(capture_header) + len, buffers_[type].size());
  memcpy(buffers_[type].data(), &capture_header, sizeof(capture_header));
  memcpy(buffers_[type].data() + sizeof(capture_header), data, len);

  std::pair<AioNode, MessageType> key(source, type);
  // Create the dataset if it does not exist.
  if (datasets_.find(key) == datasets_.end()) {
    datasets_[key] = ChunkedDataset();
    CHECK(datasets_[key].Create(source_gids_[source], MessageTypeToString(type),
                                message_type_ids_[type]));
  }

  datasets_[key].Write(buffers_[type].data());
}

void MessageLog::CalcTimingStats(const AioHeader &aio_header,
                                 const CaptureHeader &capture_header,
                                 const uint8_t *data) {
  if (aio_header.type != kMessageTypeNovAtelObservations) {
    return;
  }

  NovAtelObservationsMessage message;
  UnpackNovAtelObservationsMessage(data, 1, &message);
  AioNode source = static_cast<AioNode>(aio_header.source);

  if (source == kAioNodeGpsBaseStation) {  // GS GPS
    UpdateSyncTimeStats(message, source, capture_header, &gs_sync_time_stats_);
  } else {  // Wing GPS.
    UpdateSyncTimeStats(message, source, capture_header,
                        &wing_sync_time_stats_);
  }
}

void MessageLog::UpdateSyncTimeStats(const NovAtelObservationsMessage &message,
                                     AioNode source,
                                     const CaptureHeader &capture_header,
                                     SyncTimeStats *sync_time_stats) {
  uint8_t time_status = message.range.timestamp.time_status;
  double time_of_week_ms = static_cast<double>(message.range.timestamp.tow);
  double pps_latency_us = static_cast<double>(message.pps_latency_usec);
  const double gps_week_ms = 7.0 * 24 * 3600 * 1000;

  // See NovAtel OEM6 docs for "GPS Reference Time Status" on page 35.
  if (time_status < 100) {
    return;
  }
  // Validate time-of-week range.
  if (time_of_week_ms >= gps_week_ms) {
    return;
  }
  // Validate PPS latency range.
  if (pps_latency_us <= 0.0 || pps_latency_us > 1000000.0) {
    return;
  }

  // Handle GPS week rollovers.
  if (sync_time_stats->last_tow_ms.find(source) !=
      sync_time_stats->last_tow_ms.end()) {
    time_of_week_ms += sync_time_stats->tow_ms_rollover[source];
    if (sync_time_stats->last_tow_ms[source] - time_of_week_ms >
        gps_week_ms * 0.5) {
      sync_time_stats->tow_ms_rollover[source] += gps_week_ms;
      time_of_week_ms += gps_week_ms;
      assert(time_of_week_ms > sync_time_stats->last_tow_ms[source]);
    }
  } else {
    sync_time_stats->tow_ms_rollover[source] = 0;
  }
  sync_time_stats->last_tow_ms[source] = time_of_week_ms;

  double transport_delay_us =
      pps_latency_us -
      static_cast<double>((static_cast<uint64_t>(time_of_week_ms) % 1000) *
                          1000);
  double capture_time = static_cast<double>(capture_header.tv_sec) +
                        static_cast<double>(capture_header.tv_usec) * 1e-6;
  double gps_time = time_of_week_ms * 1e-3 + transport_delay_us * 1e-6;

  // Compute variables for a running linear regression.
  // https://stats.stackexchange.com/questions/23481/are-there-algorithms-for-computing-running-linear-or-logistic-regression-param
  double dx, dy;
  sync_time_stats->count += 1.0;
  double n = sync_time_stats->count;
  dx = capture_time - sync_time_stats->mean_x;
  dy = gps_time - sync_time_stats->mean_y;
  sync_time_stats->mean_x += dx / n;
  sync_time_stats->mean_y += dy / n;
  sync_time_stats->var_x +=
      (((n - 1.0) / n) * dx * dx - sync_time_stats->var_x) / n;
  sync_time_stats->cov_xy +=
      (((n - 1.0) / n) * dx * dy - sync_time_stats->cov_xy) / n;
}

void MessageLog::WriteWithSpoofedHeaders(AioNode source_node,
                                         MessageType message_type,
                                         int64_t timestamp_us,
                                         const uint8_t *buffer,
                                         int32_t length) {
  CaptureHeader capture_header;
  capture_header.tv_sec = timestamp_us / 1000000;
  capture_header.tv_usec = timestamp_us % 1000000;
  capture_header.source_address =
      IpAddressToUint32(AioNodeToIpAddress(source_node));
  capture_header.destination_address =
      IpAddressToUint32(AioMessageTypeToIpAddress(message_type));

  AioHeader aio_header;
  aio_header.version = AIO_VERSION;
  aio_header.source = static_cast<int8_t>(source_node);
  aio_header.type = static_cast<int8_t>(message_type);
  aio_header.sequence = 0U;
  aio_header.timestamp = static_cast<uint32_t>(timestamp_us);

  std::vector<uint8_t> new_buffer(PACK_AIOHEADER_SIZE + length, 0U);
  CHECK_EQ(PACK_AIOHEADER_SIZE,
           PackAioHeader(&aio_header, 1U, new_buffer.data()));
  memcpy(&new_buffer[PACK_AIOHEADER_SIZE], buffer, length);
  WriteData(capture_header, aio_header, new_buffer.data(),
            static_cast<int32_t>(new_buffer.size()));
}

// Write out information about a packet that was not logged.
//
// Args:
//   info: Bad packet information to write.
void MessageLog::WriteBadPacketInfo(const BadPacketInfo &info) {
  bad_packets_dataset_.Write(reinterpret_cast<const uint8_t *>(&info));
}

// Write log info dataset.
void MessageLog::WriteLogInfo(const lib::pcap_reader::PcapReader &reader) {
  LogInfo info;
  memset(&info, 0, sizeof(info));

  // Store minimum capture time for log analysis (e.g., to allow time-sync'd
  // relative time calculations in the web log viewer).
  info.min_tv_sec = reader.GetMinPcapTime().tv_sec;
  info.min_tv_usec = reader.GetMinPcapTime().tv_usec;

  info.gs_gps_time_coeff =
      gs_sync_time_stats_.cov_xy / gs_sync_time_stats_.var_x;
  info.gs_gps_time_bias = gs_sync_time_stats_.mean_y -
                          info.gs_gps_time_coeff * gs_sync_time_stats_.mean_x;

  info.wing_gps_time_coeff =
      wing_sync_time_stats_.cov_xy / wing_sync_time_stats_.var_x;
  info.wing_gps_time_bias =
      wing_sync_time_stats_.mean_y -
      info.wing_gps_time_coeff * wing_sync_time_stats_.mean_x;

  // Write dataset.
  log_info_dataset_.Write(reinterpret_cast<const uint8_t *>(&info));
}

// Release all the HDF5 resources we've acquired.
MessageLog::~MessageLog() {
  bad_packets_dataset_.Close();
  CHECK_LE(0, H5Tclose(bad_packet_info_tid_));
  CHECK_LE(0, H5Dclose(bad_packet_info_did_));

  log_info_dataset_.Close();
  CHECK_LE(0, H5Tclose(log_info_tid_));
  CHECK_LE(0, H5Dclose(log_info_did_));

  for (const std::pair<std::string, std::pair<hid_t, const void *>> &item :
       parameter_tids_) {
    H5Tclose(item.second.first);
  }

  // Note that these dataset must be closed before the
  // message_type_ids_ are closed.
  for (std::pair<const std::pair<AioNode, MessageType>, ChunkedDataset> &item :
       datasets_) {
    item.second.Close();
  }

  for (const std::pair<AioNode, hid_t> &item : source_gids_) {
    H5Gclose(item.second);
  }

  for (const std::pair<MessageType, hid_t> &item : message_type_ids_) {
    H5Tclose(item.second);
  }

  if (messages_gid_ >= 0) H5Gclose(messages_gid_);
  if (file_id_ >= 0) H5Fclose(file_id_);
}

void MessageLog::AddParameterTypes(hid_t parameters_gid,
                                   const std::string &name,
                                   const void *data_ptr, int32_t size) {
  hid_t dataset_id = H5Dopen2(parameters_gid, name.c_str(), H5P_DEFAULT);
  CHECK_LE(0, dataset_id);
  hid_t type_id = H5Dget_type(dataset_id);
  CHECK_LE(0, type_id);
  CHECK_EQ(H5Tget_size(type_id), size);

  parameter_tids_[name] =
      std::pair<hid_t, const void *>(H5Tcopy(type_id), data_ptr);

  H5Tclose(type_id);
  H5Dclose(dataset_id);
}

// Create the HDF5 types used for each message type.
//
// These types are imported from an example HDF5 file provided via
// generate_hdf5_format.py.
void MessageLog::SetupTypes() {
  // Load remaining format from the format file.
  hid_t format_file_id = H5LTopen_file_image(
      H5FormatFileImage(), H5FormatFileImageSize(),
      H5LT_FILE_IMAGE_DONT_COPY | H5LT_FILE_IMAGE_DONT_RELEASE);
  CHECK_LE(0, format_file_id);

  // Get parameter types.
  hid_t parameters_gid = H5Gopen2(format_file_id, "parameters", H5P_DEFAULT);
  CHECK_LE(0, parameters_gid);

  BuildInfo build_info;
  GetBuildInfo(&build_info);
  AddParameterTypes(parameters_gid, "build_info",
                    reinterpret_cast<const void *>(&build_info),
                    sizeof(BuildInfo));
  AddParameterTypes(parameters_gid, "system_params",
                    reinterpret_cast<const void *>(GetSystemParams()),
                    sizeof(SystemParams));
  AddParameterTypes(parameters_gid, "control_params",
                    reinterpret_cast<const void *>(GetControlParams()),
                    sizeof(ControlParams));
  AddParameterTypes(parameters_gid, "sim_params",
                    reinterpret_cast<const void *>(GetSimParams()),
                    sizeof(SimParams));

  H5Gclose(parameters_gid);

  // Fetch the AIO header type (stores the type for AioHeader).
  hid_t aio_header_did = H5Dopen2(format_file_id, "aio_header", H5P_DEFAULT);
  CHECK_LE(0, aio_header_did) << "Key 'aio_header' missing from format file.";
  hid_t aio_header_tid = H5Dget_type(aio_header_did);
  CHECK_LE(0, aio_header_tid);
  CHECK_EQ(H5Tget_size(aio_header_tid), sizeof(AioHeader));

  // Fetch the capture header type (stores the type for CaptureHeader).
  hid_t capture_header_did =
      H5Dopen2(format_file_id, "capture_header", H5P_DEFAULT);
  CHECK_LE(0, capture_header_did) << "Key 'capture_header' missing from"
                                  << " format file.";
  hid_t capture_header_tid = H5Dget_type(capture_header_did);
  CHECK_LE(0, capture_header_tid);
  CHECK_EQ(H5Tget_size(capture_header_tid), sizeof(CaptureHeader));

  // Open the group whose elements have the types for each message.
  hid_t messages_gid = H5Gopen2(format_file_id, "messages", H5P_DEFAULT);
  CHECK_LE(0, messages_gid) << "Key 'messages' missing from format file.";

  // Load the message types.
  hsize_t num_objs;
  CHECK_LE(0, H5Gget_num_objs(messages_gid, &num_objs));
  for (int32_t i = 0; i < kNumMessageTypes; ++i) {
    const char *message_name = MessageTypeToString(static_cast<MessageType>(i));

    // Temporarily disable automatic error printing in order to probe
    // if a dataset exists.
    H5E_auto1_t old_func;
    void *old_client_data;
    CHECK_LE(0, H5Eget_auto1(&old_func, &old_client_data));
    CHECK_LE(0, H5Eset_auto1(NULL, NULL));

    hid_t message_did = H5Dopen2(messages_gid, message_name, H5P_DEFAULT);

    // Re-enable error printing.
    CHECK_LE(0, H5Eset_auto1(old_func, old_client_data));

    if (message_did < 0) continue;

    // Combine the header type and the message type.
    hid_t message_tid = H5Dget_type(message_did);
    CHECK_LE(0, message_tid);

    hid_t compound_tid =
        H5Tcreate(H5T_COMPOUND, H5Tget_size(capture_header_tid) +
                                    H5Tget_size(aio_header_tid) +
                                    H5Tget_size(message_tid));
    CHECK_LE(0, compound_tid);
    CHECK_LE(0,
             H5Tinsert(compound_tid, "capture_header", 0, capture_header_tid));
    CHECK_LE(0, H5Tinsert(compound_tid, "aio_header",
                          H5Tget_size(capture_header_tid), aio_header_tid));
    CHECK_LE(
        0, H5Tinsert(compound_tid, "message", H5Tget_size(capture_header_tid) +
                                                  H5Tget_size(aio_header_tid),
                     message_tid));
    CHECK_LE(0, H5Tpack(compound_tid));

    message_type_ids_[static_cast<MessageType>(i)] = compound_tid;
    message_sizes_[static_cast<MessageType>(i)] = H5Tget_size(compound_tid);
    buffers_.emplace(
        std::make_pair(static_cast<MessageType>(i),
                       std::vector<uint8_t>(H5Tget_size(compound_tid))));

    CHECK_LE(0, H5Tclose(message_tid));
    CHECK_LE(0, H5Dclose(message_did));
  }

  // Close the message group.
  CHECK_LE(0, H5Gclose(messages_gid));

  // Fetch the capture header type (stores the type for BadPacketInfo).
  bad_packet_info_did_ =
      H5Dopen2(format_file_id, "bad_packet_info", H5P_DEFAULT);
  CHECK_LE(0, bad_packet_info_did_) << "Key 'bad_packet_info' missing from"
                                    << " format file.";
  bad_packet_info_tid_ = H5Dget_type(bad_packet_info_did_);
  CHECK_LE(0, bad_packet_info_tid_);
  CHECK_EQ(H5Tget_size(bad_packet_info_tid_), sizeof(BadPacketInfo));

  // Fetch the log info type (stores the type for LogInfo).
  log_info_did_ = H5Dopen2(format_file_id, "info", H5P_DEFAULT);
  CHECK_LE(0, log_info_did_) << "Key 'info' missing from format file.";
  log_info_tid_ = H5Dget_type(log_info_did_);
  CHECK_LE(0, log_info_tid_);
  CHECK_EQ(H5Tget_size(log_info_tid_), sizeof(LogInfo));

  CHECK_LE(0, H5Tclose(aio_header_tid));
  CHECK_LE(0, H5Dclose(aio_header_did));
  CHECK_LE(0, H5Tclose(capture_header_tid));
  CHECK_LE(0, H5Dclose(capture_header_did));
  CHECK_LE(0, H5Fclose(format_file_id));
}

// Create the /parameters group and populate its members.
void MessageLog::CreateParameterDatasets() {
  hid_t parameters_gid =
      H5Gcreate2(file_id_, "parameters", H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
  CHECK_LE(0, parameters_gid);

  for (const std::pair<std::string, std::pair<hid_t, const void *>> &item :
       parameter_tids_) {
    const std::string &name = item.first;
    hid_t type_id = item.second.first;
    const void *data_ptr = item.second.second;

    hsize_t dim[1] = {1};
    hid_t memory_dataspace_id = H5Screate_simple(1, dim, NULL);
    CHECK_LE(0, memory_dataspace_id);

    hid_t dataset_id =
        H5Dcreate2(parameters_gid, name.c_str(), type_id, memory_dataspace_id,
                   H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
    CHECK_LE(0, dataset_id);

    hid_t file_dataspace_id = H5Dget_space(dataset_id);
    CHECK_LE(0, file_dataspace_id);

    CHECK_LE(0, H5Dwrite(dataset_id, type_id, memory_dataspace_id,
                         file_dataspace_id, H5P_DEFAULT, data_ptr));

    H5Sclose(file_dataspace_id);
    H5Dclose(dataset_id);
    H5Sclose(memory_dataspace_id);
  }

  H5Gclose(parameters_gid);
}

// Create an HDF5 log file.
//
// Args:
//   filename: Name of the file to create.
void MessageLog::CreateFile(const std::string &filename) {
  CHECK_EQ(-1, file_id_) << "CreateFile must be called only once.";

  file_id_ =
      H5Fcreate(filename.c_str(), H5F_ACC_TRUNC, H5P_DEFAULT, H5P_DEFAULT);
  CHECK_LE(0, file_id_);

  CreateParameterDatasets();

  // Create the dataset for storing bad packet info.
  bad_packets_dataset_ = ChunkedDataset();
  CHECK(bad_packets_dataset_.Create(file_id_, "bad_packets",
                                    bad_packet_info_tid_));

  // Create the dataset for storing log info.
  log_info_dataset_ = ChunkedDataset();
  CHECK(log_info_dataset_.Create(file_id_, "info", log_info_tid_));

  // Create the group that will store all messages.
  messages_gid_ =
      H5Gcreate2(file_id_, "messages", H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
  CHECK_LE(0, messages_gid_);

  // Create one group for each node.
  for (int32_t i = 0; i < kNumAioNodes; ++i) {
    source_gids_[static_cast<AioNode>(i)] =
        H5Gcreate(messages_gid_, AioNodeToString(static_cast<AioNode>(i)),
                  H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
    CHECK_LE(0, source_gids_[static_cast<AioNode>(i)]);
  }
}

}  // namespace pcap_to_hdf5

}  // namespace lib

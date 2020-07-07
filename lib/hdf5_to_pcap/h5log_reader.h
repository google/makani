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

#ifndef LIB_HDF5_TO_PCAP_H5LOG_READER_H_
#define LIB_HDF5_TO_PCAP_H5LOG_READER_H_

#include <stddef.h>
#include <stdint.h>

#include <hdf5.h>

#include <list>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "avionics/network/aio_node.h"
#include "avionics/network/message_type.h"
#include "common/macros.h"
#include "lib/hdf5_to_pcap/type_info.h"
#include "lib/pcap_to_hdf5/capture_info.h"

namespace h5log_reader {

// This class provides a simple HDF5 file interface abstraction.
class File {
 public:
  File() : file_id_(-1) {}
  ~File() { Close(); }

  bool IsOpen() const { return file_id_ >= 0; }
  bool Open(const std::string &file);
  void Close();

  // Determine if an HDF5 dataset exists within the file.
  bool DatasetExists(const std::string &dataset_path) const;

  // Export the file_id descriptor for the Dataset class.
  hid_t file_id() const { return file_id_; }

 private:
  hid_t file_id_;  // Valid when >= 0.

  // We do not support copying HDF5 descriptors.
  DISALLOW_COPY_AND_ASSIGN(File);
};

// This class stores a dataset in the data format described within the HDF5
// file. This format may not correspond to that of avionics_messages.h. When
// we read the HDF5 file, we convert all types to native types (e.g., little
// or big endian, depending on the host architecture).
class Dataset {
 public:
  Dataset()
      : fields_(),
        types_(),
        data_(nullptr),
        num_objects_(0),
        object_size_(0) {}
  ~Dataset() { Clear(); }

  // Read the fields, number of objects, and object size.
  bool ReadTableOfContents(const File &file, const std::string &path);

  // Read the entire dataset.
  bool ReadData(const File &file, const std::string &path);

  // Determine if a field exists in the dataset. Call ReadTableOfContents or
  // ReadData first.
  bool FieldExists(const std::string &field_path) const;

  // Get total number of objects (product of all dimensions) in dataset. Call
  // ReadTableOfContents or ReadData first.
  int64_t GetNumObjects() const { return num_objects_; }

  // Get the number of bytes in each object. Call ReadTableOfContents or
  // ReadData first.
  size_t GetObjectSize() const { return object_size_; }

  // Export the packed HDF5 data, from the original file version, to an unpacked
  // data structure of the current version. Unknown fields are set to zero.
  template <typename T>
  std::vector<T> Export() const {
    std::vector<T> v(num_objects_);
    ExportData(GetTypeInfo<T>(), v.size(), v.data());
    return v;
  }

  // Export the packed HDF5 data, from the original file version, to an unpacked
  // data structure of the current version. Unknown fields are set to zero.
  template <typename T>
  int64_t Export(int64_t count, T objects[]) const {
    memset(objects, 0, count * sizeof(*objects));
    return ExportData(GetTypeInfo<T>(), count, objects);
  }

  // Export the packed HDF5 data, from the original file version, to an unpacked
  // data structure of the current version. Unknown fields are set to zero.
  std::unique_ptr<uint8_t> Export(const TypeInfo *type_info) const;

 private:
  // Container to store information about each field.
  struct FieldInfo {
   public:
    FieldInfo() : type_id(0), offset(0) {}
    FieldInfo(hid_t field_type_id, size_t field_offset)
        : type_id(field_type_id), offset(field_offset) {}

    hid_t type_id;
    size_t offset;
  };

  // This structure maps a flattened field path to the FieldInfo structure.
  typedef std::map<std::string, FieldInfo> FieldInfoMap;

  // Open/close a dataset.
  hid_t Open(const File &file, const std::string &path);
  void Close(hid_t dataset_id) const;

  // Clear stored memory associated with the dataset.
  void Clear();

  // Read information from an open dataset.
  int64_t ReadNumObjects(hid_t dataset_id) const;
  size_t ReadObjectSize(hid_t dataset_id) const;
  bool ReadTableOfContents(hid_t dataset_id);
  bool ReadData(hid_t dataset_id);

  // These functions create the fields_ map by recursively iterating
  // through the HDF5 dataset types.
  void AppendType(hid_t type_id, const std::string &path, size_t offset);
  void AppendCompound(hid_t type_id, const std::string &path, size_t offset);
  void AppendArray(hid_t type_id, const std::string &path, size_t offset);
  void AppendArrayDim(hid_t type_id, const std::string &path, size_t offset,
                      size_t size, const hsize_t *dims, int32_t ndims);

  // Helper function to export packed data from the original file version to an
  // unpacked data structure of the current version.
  int64_t ExportData(const TypeInfo *type_info, int64_t count,
                     void *data) const;

  // Store a mapping of field paths to HDF5 types. We use this map to determine
  // if fields exist and to understand their type when exporting to another
  // type.
  FieldInfoMap fields_;

  // Store custom HDF5 types. Call H5Tclose() to free each type.
  std::list<hid_t> types_;

  // Data stored with original data structure in native format.
  std::unique_ptr<uint8_t> data_;
  int64_t num_objects_;
  size_t object_size_;

  // We do not support copying HDF5 descriptors.
  DISALLOW_COPY_AND_ASSIGN(Dataset);
};

// This class stores arrays of the CaptureHeader, AioHeader, and message in
// the current message structure as defined in avionics_messages.h.
class MessageLog {
 public:
  MessageLog() :
      aio_node_(), message_type_(), capture_header_(), aio_header_(),
      message_(), message_info_(), num_messages_(0) {}
  ~MessageLog() {}

  bool LoadDataset(AioNode aio_node, MessageType message_type,
                   const Dataset &dataset);

  AioNode GetAioNode(void) const { return aio_node_; }
  MessageType GetMessageType(void) const { return message_type_; }

  int64_t GetNumMessages(void) const { return num_messages_; }
  size_t GetPackedSize(void) const { return message_info_.pack_size; }
  size_t GetUnpackedSize(void) const { return message_info_.unpack_size; }

  const CaptureHeader *GetCaptureHeader(int64_t index) const;
  const AioHeader *GetAioHeader(int64_t index) const;
  const uint8_t *GetMessageData(int64_t index) const;
  size_t PackMessageData(int64_t index, uint8_t *out) const;

 private:
  AioNode aio_node_;
  MessageType message_type_;
  std::vector<CaptureHeader> capture_header_;
  std::vector<AioHeader> aio_header_;
  std::shared_ptr<uint8_t> message_;
  TypeInfo message_info_;
  int64_t num_messages_;
};

// Get dataset group path.
// /messages/kAioNodeFcA/kMessageTypeFlightComputerSensor.
std::string GetDatasetPath(AioNode aio_node, MessageType message_type);

// Read an HDF5 file. Sets nodes and messages specify the AioNodes and
// MessageTypes to process, each represented as strings. To process all
// nodes or messages, specify an empty set. The return value contains
// a list of message logs, each representing a single dataset within the
// HDF5 file. Each dataset contains an array of messages.
typedef std::list<MessageLog> MessageLogs;
MessageLogs ReadFile(const std::string &file,
                     const std::set<std::string> &nodes,
                     const std::set<std::string> &messages);

}  // namespace h5log_reader

#endif  // LIB_HDF5_TO_PCAP_H5LOG_READER_H_

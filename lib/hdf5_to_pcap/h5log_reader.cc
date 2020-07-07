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

#include "lib/hdf5_to_pcap/h5log_reader.h"

#include <stddef.h>
#include <stdint.h>

#include <glog/logging.h>
#include <hdf5.h>
#include <hdf5_hl.h>

#include "common/macros.h"
#include "lib/hdf5_to_pcap/type_info.h"

namespace h5log_reader {
namespace {

// HDF5 defines individual elements in a character array as a string type.
bool IsStringChar(hid_t dtype) {
  hid_t s1_type = H5LTtext_to_dtype(
      "H5T_STRING {"
      "  STRSIZE 1;"
      "  STRPAD H5T_STR_NULLPAD;"
      "  CSET H5T_CSET_ASCII;"
      "  CTYPE H5T_C_S1;"
      "}", H5LT_DDL);
  bool equal = H5Tequal(dtype, s1_type);
  H5Tclose(s1_type);
  return equal;
}

bool IsInt8BoolEnum(hid_t dtype) {
  hid_t s1_type = H5LTtext_to_dtype(
      "H5T_ENUM {"
      "  H5T_STD_I8LE;"
      "  \"FALSE\" 0;"
      "  \"TRUE\"  1;"
      "}", H5LT_DDL);
  bool equal = H5Tequal(dtype, s1_type);
  H5Tclose(s1_type);
  return equal;
}

bool ConvertFromH5(int64_t n, hid_t src_type, size_t src_offset,
                   size_t src_stride, const void *src_data, DataType dst_type,
                   size_t dst_offset, size_t dst_stride, void *dst_data) {
  // We assume H5Dread() converts H5 types to native endianess.
  if (H5Tequal(src_type, H5T_NATIVE_CHAR) || IsStringChar(src_type)) {
    ConvertDataType(n, kDataTypeChar, src_offset, src_stride, src_data,
                    dst_type, dst_offset, dst_stride, dst_data);
  } else if (H5Tequal(src_type, H5T_NATIVE_UINT8)) {
    ConvertDataType(n, kDataTypeUint8T, src_offset, src_stride, src_data,
                    dst_type, dst_offset, dst_stride, dst_data);
  } else if (H5Tequal(src_type, H5T_NATIVE_UINT16)) {
    ConvertDataType(n, kDataTypeUint16T, src_offset, src_stride, src_data,
                    dst_type, dst_offset, dst_stride, dst_data);
  } else if (H5Tequal(src_type, H5T_NATIVE_UINT32)) {
    ConvertDataType(n, kDataTypeUint32T, src_offset, src_stride, src_data,
                    dst_type, dst_offset, dst_stride, dst_data);
  } else if (H5Tequal(src_type, H5T_NATIVE_UINT64)) {
    ConvertDataType(n, kDataTypeUint64T, src_offset, src_stride, src_data,
                    dst_type, dst_offset, dst_stride, dst_data);
  } else if (H5Tequal(src_type, H5T_NATIVE_INT8) || IsInt8BoolEnum(src_type)) {
    ConvertDataType(n, kDataTypeInt8T, src_offset, src_stride, src_data,
                    dst_type, dst_offset, dst_stride, dst_data);
  } else if (H5Tequal(src_type, H5T_NATIVE_INT16)) {
    ConvertDataType(n, kDataTypeInt16T, src_offset, src_stride, src_data,
                    dst_type, dst_offset, dst_stride, dst_data);
  } else if (H5Tequal(src_type, H5T_NATIVE_INT32)) {
    ConvertDataType(n, kDataTypeInt32T, src_offset, src_stride, src_data,
                    dst_type, dst_offset, dst_stride, dst_data);
  } else if (H5Tequal(src_type, H5T_NATIVE_INT64)) {
    ConvertDataType(n, kDataTypeInt64T, src_offset, src_stride, src_data,
                    dst_type, dst_offset, dst_stride, dst_data);
  } else if (H5Tequal(src_type, H5T_NATIVE_FLOAT)) {
    ConvertDataType(n, kDataTypeFloat, src_offset, src_stride, src_data,
                    dst_type, dst_offset, dst_stride, dst_data);
  } else if (H5Tequal(src_type, H5T_NATIVE_DOUBLE)) {
    ConvertDataType(n, kDataTypeDouble, src_offset, src_stride, src_data,
                    dst_type, dst_offset, dst_stride, dst_data);
  } else {
    // Not supported.
    return false;
  }
  return true;
}

}  // namespace

bool File::Open(const std::string &file) {
  Close();
  file_id_ = H5Fopen(file.c_str(), H5F_ACC_RDONLY, H5P_DEFAULT);
  return IsOpen();
}

void File::Close() {
  if (IsOpen()) {
    H5Fclose(file_id_);
    file_id_ = -1;  // Invalid.
  }
}

bool File::DatasetExists(const std::string &dataset_path) const {
  CHECK(IsOpen());

  // Verify dataset path to mitigate H5Dopen() error messages.
  size_t pos = 0;
  do {
    pos = dataset_path.find("/", pos + 1);
    const std::string partial_path = dataset_path.substr(0, pos);
    if (H5Lexists(file_id_, partial_path.c_str(), H5P_DEFAULT) <= 0) {
      return false;
    }
  } while (pos != std::string::npos);
  return true;
}

bool Dataset::ReadTableOfContents(const File &file, const std::string &path) {
  const hid_t dataset_id = Open(file, path);
  if (dataset_id >= 0) {
    const bool success = ReadTableOfContents(dataset_id);
    Close(dataset_id);
    return success;
  }
  return false;
}

bool Dataset::ReadData(const File &file, const std::string &path) {
  const hid_t dataset_id = Open(file, path);
  if (dataset_id >= 0) {
    LOG(INFO) << "Reading " << std::to_string(num_objects_)
              << " records from " << path << "...\n";
    const bool success =
        ReadTableOfContents(dataset_id) && ReadData(dataset_id);
    Close(dataset_id);
    return success;
  }
  return false;
}

bool Dataset::FieldExists(const std::string &field_path) const {
  return fields_.find(field_path) != fields_.end();
}

hid_t Dataset::Open(const File &file, const std::string &path) {
  Clear();
  hid_t dataset_id = -1;  // Invalid.
  if (file.DatasetExists(path)) {
    dataset_id = H5Dopen2(file.file_id(), path.c_str(), H5P_DEFAULT);
    num_objects_ = ReadNumObjects(dataset_id);
    object_size_ = ReadObjectSize(dataset_id);
  }
  return dataset_id;
}

void Dataset::Close(hid_t dataset_id) const {
  H5Dclose(dataset_id);
}

void Dataset::Clear() {
  // Clear memory associated with the table of contents.
  for (auto& type_id : types_) {
    H5Tclose(type_id);
  }
  types_.clear();
  fields_.clear();

  // Clear memory associated with the data.
  data_.reset(nullptr);
  object_size_ = 0;
  num_objects_ = 0;
}

int64_t Dataset::ReadNumObjects(hid_t dataset_id) const {
  hsize_t dims[H5S_MAX_RANK] = {0};
  const hid_t dataspace_id = H5Dget_space(dataset_id);
  const int32_t ndims = H5Sget_simple_extent_dims(dataspace_id, dims, nullptr);
  hsize_t objects = 0;
  if (ndims > 0) {
    objects = 1;
    for (int32_t d = 0; d < ndims; ++d) {
      objects *= dims[d];
    }
  }
  return objects;
}

size_t Dataset::ReadObjectSize(hid_t dataset_id) const {
  const hid_t storage_id = H5Dget_type(dataset_id);
  const hid_t native_id = H5Tget_native_type(storage_id, H5T_DIR_ASCEND);
  const size_t size = H5Tget_size(native_id);
  H5Tclose(native_id);
  H5Tclose(storage_id);
  return size;
}

bool Dataset::ReadTableOfContents(hid_t dataset_id) {
  const hid_t storage_id = H5Dget_type(dataset_id);
  const hid_t native_id = H5Tget_native_type(storage_id, H5T_DIR_ASCEND);
  AppendType(native_id, "", 0);
  H5Tclose(native_id);
  H5Tclose(storage_id);
  return fields_.size() > 0;
}

bool Dataset::ReadData(hid_t dataset_id) {
  data_.reset(new uint8_t[object_size_ * num_objects_]);
  if (data_ != nullptr) {
    const hid_t storage_id = H5Dget_type(dataset_id);
    const hid_t native_id = H5Tget_native_type(storage_id, H5T_DIR_ASCEND);
    if (H5Dread(dataset_id, native_id, H5S_ALL, H5S_ALL, H5P_DEFAULT,
                data_.get()) < 0) {
      data_.reset(nullptr);
    }
    H5Tclose(native_id);
    H5Tclose(storage_id);
  }
  return data_ != nullptr;
}

void Dataset::AppendType(hid_t type_id, const std::string &path,
                         size_t offset) {
  // Recursively append primitive data types to fields_ structure. The final
  // fields_ structure will contain a list of paths similar to:
  //    .capture_header.tv_sec
  //    .capture_header.tv_usec
  //    .capture_header.source_address
  //    .capture_header.destination_address
  //    .aio_header.version
  //    .aio_header.source
  //    .aio_header.type
  //    .aio_header.sequence
  //    .aio_header.reserved
  //    .message.imu.status
  //    .message.imu.error
  //    .message.imu.cs_seq_num
  //    .message.imu.raw_seq_num
  //    .message.imu.cs[0].latency
  //    .message.imu.cs[0].dt
  //    .message.imu.cs[0].phi[0]
  //    .message.imu.cs[0].phi[1]
  //    .message.imu.cs[0].phi[2]
  //    .message.imu.cs[0].dvsf[0]
  //    .message.imu.cs[0].dvsf[1]
  //    .message.imu.cs[0].dvsf[2]
  //    .message.imu.cs[0].alpha[0]
  //    .message.imu.cs[0].alpha[1]
  //    .message.imu.cs[0].alpha[2]
  //    .message.imu.cs[0].nu[0]
  //    .message.imu.cs[0].nu[1]
  //    .message.imu.cs[0].nu[2]
  //    ...
  const H5T_class_t class_id = H5Tget_class(type_id);
  if (class_id == H5T_COMPOUND) {
    AppendCompound(type_id, path, offset);
  } else if (class_id == H5T_ARRAY) {
    AppendArray(type_id, path, offset);
  } else {
    fields_[path] = FieldInfo(type_id, offset);
  }

  // TODO: Remove. This hack works around a bug in pcap_to_hdf5 where
  // single element arrays do not appear as arrays, but rather compound or
  // primitive types.
  if (!path.empty()) {
    if (class_id == H5T_COMPOUND) {
      AppendCompound(type_id, path + "[0]", offset);
    } else if (class_id != H5T_ARRAY) {
      fields_[path + "[0]"] = FieldInfo(type_id, offset);
    }
  }
}

void Dataset::AppendCompound(hid_t type_id, const std::string &path,
                             size_t offset) {
  // Append each member in compound (structure) type.
  const int32_t nmembers = H5Tget_nmembers(type_id);
  for (int32_t i = 0; i < nmembers; ++i) {
    const char *member_name = H5Tget_member_name(type_id, i);
    const size_t member_offset = H5Tget_member_offset(type_id, i);
    const std::string member_path = path + "." + std::string(member_name);
    const hid_t member_type = H5Tget_member_type(type_id, i);
    AppendType(member_type, member_path, offset + member_offset);
    types_.push_back(member_type);
  }
}

void Dataset::AppendArray(hid_t type_id, const std::string &path,
                          size_t offset) {
  // Get array dimensions, then append each index.
  const size_t size = H5Tget_size(type_id);
  const int32_t ndim = H5Tget_array_ndims(type_id);
  CHECK_LE(ndim, H5S_MAX_RANK);
  hsize_t dims[H5S_MAX_RANK];
  H5Tget_array_dims2(type_id, dims);
  AppendArrayDim(type_id, path, offset, size, dims, ndim);
}

void Dataset::AppendArrayDim(hid_t type_id, const std::string &path,
                             size_t offset, size_t size,
                             const hsize_t *dims, int32_t ndims) {
  // Append each array member using name[index] syntax. This function iterates
  // for each element in the array to create paths similar to:
  //    .message.imu.raw.acc[0]
  //    .message.imu.raw.acc[1]
  //    .message.imu.raw.acc[2]
  //    ...
  // For complex types, including multi-dimensional arrays, this function
  // evaluates each array member recursively to create paths similar to:
  //    .message.imu.cs[0].latency
  //    .message.imu.cs[0].dt
  //    .message.imu.cs[0].phi[0]
  //    .message.imu.cs[0].phi[1]
  //    .message.imu.cs[0].phi[2]
  //    .message.imu.cs[0].matrix[0][0]
  //    .message.imu.cs[0].matrix[0][1]
  //    .message.imu.cs[0].matrix[0][2]
  //    .message.imu.cs[0].matrix[1][0]
  //    .message.imu.cs[0].matrix[1][1]
  //    .message.imu.cs[0].matrix[1][2]
  //    .message.imu.cs[0].matrix[2][0]
  //    .message.imu.cs[0].matrix[2][1]
  //    .message.imu.cs[0].matrix[2][2]
  //    ...
  CHECK_LT(0, ndims);
  CHECK_LT(0, *dims);
  const size_t step = size / *dims;
  for (hsize_t i = 0; i < *dims; ++i) {
    // TODO: Replace index with enumeration name when applicable.
    const std::string p = path + "[" + std::to_string(i) + "]";
    const size_t addr = offset + i * step;
    if (ndims > 1) {
      AppendArrayDim(type_id, p, addr, step, dims + 1, ndims - 1);
    } else {
      const hid_t super_type = H5Tget_super(type_id);
      AppendType(super_type, p, addr);
      types_.push_back(super_type);
    }
  }
}

std::unique_ptr<uint8_t> Dataset::Export(const TypeInfo *type_info) const {
  CHECK(type_info != nullptr);
  CHECK_GE(num_objects_, 0);

  const size_t size = type_info->unpack_size * num_objects_;
  std::unique_ptr<uint8_t> out(new uint8_t[size]);
  if (out != nullptr) {
    memset(out.get(), 0, size);
    ExportData(type_info, num_objects_, out.get());
  }
  return out;
}

int64_t Dataset::ExportData(const TypeInfo *type_info, int64_t count,
                            void *data) const {
  CHECK(type_info != nullptr);
  CHECK_GE(count, 0);
  CHECK(data != nullptr);

  if (count > num_objects_) {
    count = num_objects_;
  }

  // Output matching fields using string comparisons.
  for (int32_t f = 0; f < type_info->num_fields; ++f) {
    const TypeInfoField *field = &type_info->field[f];
    FieldInfoMap::const_iterator src = fields_.find(field->path);
    if (src == fields_.end()) {
      LOG(WARNING) << "Field not found: " << field->path;
    } else if (!ConvertFromH5(count, src->second.type_id,
                              src->second.offset, object_size_, data_.get(),
                              field->type, field->offset,
                              type_info->unpack_size, data)) {
      char text[2048];
      size_t len = sizeof(text) - 1;
      H5LTdtype_to_text(src->second.type_id, text, H5LT_DDL, &len);
      LOG(WARNING) << "Unsupported field type: " << field->path << "=" << text;
    }
  }
  return count;
}

bool MessageLog::LoadDataset(AioNode aio_node, MessageType message_type,
                             const Dataset &dataset) {
  // Returns nullptr for unsupported types.
  const TypeInfo *info = GetMessageTypeInfo(message_type);
  if (info != nullptr) {
    aio_node_ = aio_node;
    message_type_ = message_type;
    num_messages_ = dataset.GetNumObjects();

    capture_header_ = dataset.Export<CaptureHeader>();
    aio_header_ = dataset.Export<AioHeader>();

    message_info_ = *info;
    message_.reset(dataset.Export(&message_info_).release());
    return true;
  }
  return false;
}

const CaptureHeader *MessageLog::GetCaptureHeader(int64_t index) const {
  if (0 <= index && index < num_messages_) {
    return &capture_header_[index];
  }
  return nullptr;
}

const AioHeader *MessageLog::GetAioHeader(int64_t index) const {
  if (0 <= index && index < num_messages_) {
    return &aio_header_[index];
  }
  return nullptr;
}

const uint8_t *MessageLog::GetMessageData(int64_t index) const {
  if (message_ != nullptr && 0 <= index && index < num_messages_) {
    size_t offset = index * message_info_.unpack_size;
    const uint8_t *data = message_.get();
    return &data[offset];
  }
  return nullptr;
}

size_t MessageLog::PackMessageData(int64_t index, uint8_t *out) const {
  const uint8_t *data = GetMessageData(index);
  if (data != nullptr && message_info_.pack) {
    return message_info_.pack(data, 1, out);
  }
  return 0;
}

std::string GetDatasetPath(AioNode aio_node, MessageType message_type) {
  return "/messages/" + std::string(AioNodeToString(aio_node)) + "/"
      + std::string(MessageTypeToString(message_type));
}

bool IsAioNodeInSet(const std::set<std::string> &set, AioNode node) {
  return (set.find(AioNodeToString(node)) != set.end()
          || set.find(AioNodeToShortString(node)) != set.end());
}

bool IsMessageInSet(const std::set<std::string> &set, MessageType type) {
  return (set.find(MessageTypeToString(type)) != set.end()
          || set.find(MessageTypeToShortString(type)) != set.end());
}

MessageLogs ReadFile(const std::string &file,
                     const std::set<std::string> &nodes,
                     const std::set<std::string> &messages) {
  MessageLogs log;
  File h5;
  if (h5.Open(file)) {
    for (int32_t m = 0; m < kNumMessageTypes; ++m) {
      MessageType message_type = static_cast<MessageType>(m);

      // Only process specified message types, or all types if empty.
      if (!messages.empty() && !IsMessageInSet(messages, message_type)) {
        continue;
      }

      // Skip unsupported types.
      if (GetMessageTypeInfo(message_type) == nullptr) {
        LOG(WARNING) << "Message type " << MessageTypeToString(message_type)
                     << " not supported.";
        continue;
      }

      for (int32_t n = 0; n < kNumAioNodes; ++n) {
        AioNode aio_node = static_cast<AioNode>(n);

        // Only process specified nodes, or all nodes if empty.
        if (!nodes.empty() && !IsAioNodeInSet(nodes, aio_node)) {
          continue;
        }

        // Process dataset.
        Dataset dataset;
        if (dataset.ReadData(h5, GetDatasetPath(aio_node, message_type))) {
          MessageLog message;
          message.LoadDataset(aio_node, message_type, dataset);
          log.push_back(message);
        }
      }
    }
  }
  return log;
}

}  // namespace h5log_reader

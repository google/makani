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

#include "lib/datatools/load_pcap_hdf5_dataset.h"

#include <glog/logging.h>
#include <hdf5.h>
#include <hdf5_hl.h>
#include <stddef.h>
#include <stdint.h>

#include <functional>
#include <string>
#include <vector>

PcapToHdf5DatasetRawData::PcapToHdf5DatasetRawData(
    const std::string &filename, const std::string &aio_node_str,
    const std::string &message_type_str, int32_t first_message_to_read,
    int32_t last_message_to_read)
    : num_messages_(0),
      message_and_header_size_(0U),
      message_size_(0U),
      message_offset_(0U),
      message_and_header_raw_data_() {
  // Open the dataset containing the header information and message payloads.
  // This involves opening the HDF5 file and several groups, all released at the
  // end of this function.
  hid_t file_id = H5Fopen(filename.c_str(), H5F_ACC_RDONLY, H5P_DEFAULT);
  CHECK_LE(0, file_id) << "Could not open file.";

  hid_t messages_group_id = H5Gopen2(file_id, "messages", H5P_DEFAULT);
  CHECK_LE(0, messages_group_id) << "Could not open /messages.";

  hid_t node_group_id = H5Gopen2(messages_group_id, aio_node_str.c_str(),
                                 H5P_DEFAULT);
  CHECK_LE(0, node_group_id) << "Could not open " << aio_node_str << ".";

  hid_t message_and_header_dataset_id = H5Dopen2(node_group_id,
                                                 message_type_str.c_str(),
                                                 H5P_DEFAULT);
  CHECK_LE(0, message_and_header_dataset_id)
      << "Could not open " << message_type_str << ".";

  LoadFromDataset(message_and_header_dataset_id, first_message_to_read,
                  last_message_to_read);

  H5Dclose(message_and_header_dataset_id);
  H5Gclose(node_group_id);
  H5Gclose(messages_group_id);
  H5Fclose(file_id);
}

void PcapToHdf5DatasetRawData::LoadFromDataset(
    hid_t message_and_header_dataset_id,
    int32_t first_message_to_read,
    int32_t last_message_to_read) {
  hid_t message_and_header_type_id = H5Dget_type(message_and_header_dataset_id);
  CHECK_LE(0, message_and_header_type_id) << "Could not get message type.";
  message_and_header_size_ = H5Tget_size(message_and_header_type_id);

  // Get information about the message on its own.
  int32_t message_index
      = H5Tget_member_index(message_and_header_type_id, "message");
  CHECK_LE(0, message_index) << "The field 'message' was missing.";

  message_offset_ = H5Tget_member_offset(message_and_header_type_id,
                                         message_index);

  hid_t message_type_id = H5Tget_member_type(message_and_header_type_id,
                                             message_index);
  CHECK_LE(0, message_type_id);
  message_size_ = H5Tget_size(message_type_id);
  H5Tclose(message_type_id);

  // Determine dimensions.
  hid_t message_and_header_space_id
      = H5Dget_space(message_and_header_dataset_id);
  CHECK_LE(0, message_and_header_space_id);

  const hsize_t rank = 1;
  hsize_t dims[1];
  int32_t ndims = H5Sget_simple_extent_ndims(message_and_header_space_id);
  CHECK_EQ(rank, ndims);
  H5Sget_simple_extent_dims(message_and_header_space_id, dims, NULL);

  // Reassign dims to contain the number of messages to read.
  CHECK_LE(0, first_message_to_read);
  if (last_message_to_read == -1) {
    dims[0] -= first_message_to_read;
  } else {
    CHECK_LE(first_message_to_read, last_message_to_read);
    CHECK_LT(last_message_to_read, dims[0]);
    dims[0] = last_message_to_read - first_message_to_read;
  }

  // Select the portion of the file to read.
  hsize_t first_message_to_read_hsize = first_message_to_read;
  CHECK_LE(0, H5Sselect_hyperslab(message_and_header_space_id, H5S_SELECT_SET,
                                  &first_message_to_read_hsize,
                                  NULL, dims, NULL));

  // Create the data location to write to.
  message_and_header_raw_data_.resize(dims[0] * message_and_header_size_);
  hid_t target_space_id = H5Screate_simple(rank, dims, NULL);
  CHECK_LE(0, target_space_id);

  CHECK_LE(0, H5Dread(message_and_header_dataset_id, message_and_header_type_id,
                      target_space_id, message_and_header_space_id, H5P_DEFAULT,
                      message_and_header_raw_data_.data()));

  H5Sclose(target_space_id);
  H5Sclose(message_and_header_space_id);
  H5Tclose(message_and_header_type_id);

  num_messages_ = static_cast<int32_t>(dims[0]);
}

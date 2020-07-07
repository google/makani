#!/bin/bash -e
# Copyright 2020 Makani Technologies LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

#
# Post-processing tcpdump pcap files in the log folder.
#
# Usage:
#   ./log_process.sh <path_to_the_pcap_file_or_directory> [tag [pcap_to_hdf5]]
#
# Circularly remove old pcap files, convert pcap to HDF5, and gzip the pcap
# file. If `tag` is provided, the *.pcap.gz and *.h5 files in the log directory
# will have `tag` appended to their names.

source "${MAKANI_HOME}/lib/scripts/mbash.sh"

if [[ "$#" = 0 ]]; then
  exit 1
fi

# Convert pcap file to HDF5, and gzip the pcap file
#
# Args:
#   $1: File to be processed.
#   $2: path to pcap_to_hdf5 binary.
function pcap_to_hdf5() {
  # If we are running out of room for more log files, start deleting the
  # oldest log files in the current log directory.
  readonly DISK_FREE_THRESHOLD_MB=1000

  while [[ "$(df --block-size=1M --output=avail $1 | tail -1)" \
    -lt "${DISK_FREE_THRESHOLD_MB}" \
    && "$(ls *.pcap | wc -l)" -gt 1 ]]; do
    rm "$(ls -t *.pcap | tail -1)"
  done

  # Convert the log into an HDF5 file.
  readonly H5_FILE="$(echo "$1" | sed 's/.pcap$/.h5/')"

  if [[ -n "$2" ]]; then
    readonly pcap_to_hdf5_binary="$2"
  else
    readonly pcap_to_hdf5_binary="${BUILD_DIR}/lib/pcap_to_hdf5/pcap_to_hdf5"
  fi

  "${pcap_to_hdf5_binary}" "$1" --output_file="${H5_FILE}"

  # Compress the pcap file into a .gz file.
  gzip "$1"
}

# Moves files in the current directory with names 'foo$2' to 'foo-$1$2'.
#
# Args:
#   $1: Log name.
#   $2: Extension.
function tag_files() {
  for file in $(ls | grep $2'$'); do
    local len="$((${#file} - ${#2}))"
    # Use '-' as the delimiter to differentiate from the default name.
    mv "${file}" "${file:0:len}-$1$2"
  done
}

# Tag files in a log directory.
#
# Args:
#   $1: Tag name.
function tag_directory() {
  # Fill in log_name field in the description file.
  sed -i 's/"log_name": ""/"log_name": "'$1'"/' "log_desc.json"

  # Append log name to the end of the directory name and of the individual
  # log files.

  # Rename the HDF5 log as well.
  tag_files "$1" '.h5'

  # Rename the PCAP file as well.
  tag_files "$1" '.pcap.gz'
}

if [[ -d "$1" ]]; then
  # Handle the case where there is no last log.
  # We may still have to tag files in the directory.
  readonly LOG_DIR="$1"
  readonly FILE_NAME=""
elif [[ -e "$1" ]]; then
  readonly LOG_DIR="$(dirname $1)"
  readonly FILE_NAME="$(basename $1)"
else
  echo "Invalid directory or file \"$1\"."
  exit 1
fi

pushd "${LOG_DIR}" >> /dev/null

if [[ -n "${FILE_NAME}" ]]; then
  pcap_to_hdf5 "${FILE_NAME}" "$3"
fi

if [[ -n "$2" ]]; then
  tag_directory "$2"
fi

popd >> /dev/null

#!/bin/bash -x
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
# Startup script for a batch simulation worker.  This depends on the following
# metadata attributes:
#   apt_dependencies: Space-separated list of packages to install via
#     "apt-get install".
#   worker_package_path: Cloud Storage path to the worker's client-generated package.
#   config_package_path: Cloud Storage path to the worker's config package.
#   error_log_dir: Cloud Storage directory for error logs.
#   exec_cmd: Main executable command for the worker to run.

function get_metadata_value() {
  curl -H 'Metadata-Flavor: Google' \
    "http://metadata.google.internal/computeMetadata/v1/instance/$1"
}

# Terminate after 1 minute if the worker is still running, as the client has
# probably errored out.
function eventual_shutdown() {
  sleep 60
  local readonly ZONE="$(get_metadata_value zone | awk -F'/' '{print $NF}')"
  gcloud compute instances delete "$(get_metadata_value name)" \
    --zone="${ZONE}" --quiet
}

function export_log_file() {
  gsutil cp /var/log/syslog "${error_log_dir}/$(get_metadata_value name).LOG"
}


function run_with_retries() {
  local attempts=$(($1 + 1))
  shift 1

  while [[ $attempts > 0 ]]; do
    if "$@"; then
      return;
    fi
    attempts=$(($attempts - 1))
    sleep 1
  done
}

# Query metadata for required values.
readonly apt_packages="$(get_metadata_value attributes/apt_dependencies)"
readonly config_package_path="$(get_metadata_value attributes/config_package_path)"
readonly worker_package_path="$(get_metadata_value attributes/worker_package_path)"
readonly error_log_dir="$(get_metadata_value attributes/error_log_dir)"
readonly exec_cmd="$(get_metadata_value attributes/exec_cmd)"

# On error, export our log file, and then shut down after a while in case the
# client doesn't do it for us.
function on_error() {
  export_log_file
  eventual_shutdown
}
trap on_error ERR

# Download apt dependencies.
if [[ -n "${apt_packages}" ]]; then
  # An update failure is probably recoverable.
  trap - ERR
  apt-get -y update
  trap on_error ERR

  # Try to install/upgrade all of the required packages.  If the
  # apt-get fails, continue if at least some version of each of the
  # packages is already installed.
  apt-get -y install ${apt_packages} \
    || dpkg -s ${apt_packages} > /dev/null
fi

# Download worker package.
# Downloads via gsutil fail occasionally due to a bad checksum, so retry a few
# times.
run_with_retries 3 gsutil cp "${worker_package_path}" .
chmod +x "$(basename ${worker_package_path})"

# Download and unpack config package.
run_with_retries 3 gsutil cp "${config_package_path}" .
tar -xzf "$(basename ${config_package_path})"

# Run the main program.
${exec_cmd}

# Don't trigger on_error once command execution is complete.
trap - ERR

eventual_shutdown

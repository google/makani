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


# Runs tcpdump with filtering for desktop simulation.  Usage:
#     ./lib/scripts/sim_tcpdump <pcap_file_name> [<interface>]

# TODO(b/141934868): Load scripts using bazel runfiles wrapper script.
if [[ ! -d "${RUNFILES_DIR}" ]]; then
  RUNFILES_DIR="$0.runfiles"
fi

if [[ -d "${RUNFILES_DIR}" ]]; then
  MAKANI_HOME="${RUNFILES_DIR}/makani"
fi

source "${MAKANI_HOME}/lib/scripts/mbash.sh"
source "${MAKANI_HOME}/lib/scripts/network_config.sh"

if [[ "$#" -lt 2 ]]; then
  readonly INTERFACE=lo
else
  readonly INTERFACE="${2}"
fi

exec tcpdump -B 65536 -i "${INTERFACE}" -U -w "${1}" \
  net "${M600_MULTICAST_SUBNET}" or net "${M600_UNICAST_SUBNET}" \
  >/dev/null 2>&1

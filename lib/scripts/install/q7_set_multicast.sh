#!/bin/bash -u
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
# Sets the multicast max memberships to the makani system default.
#
# Usage:
# ./q7_set_multicast --node [node_name]
# or
# ./q7_set_multicast --ip [ip]

source "${MAKANI_HOME}/lib/scripts/mbash.sh"
source "${MAKANI_HOME}/lib/scripts/network_config.sh"

# Source shflags.
source /opt/shflags-1.0.3/src/shflags

DEFINE_string 'node' '' \
  'Fix a particular node. Specified by name (e.g. M600_CONTROLLER_A).'
DEFINE_string 'ip' '' \
  'Fix a particular node. Specified by ip.'

FLAGS "$@" || exit $?
eval set -- "${FLAGS_ARGV}"
if [ $# -ne 0 ]; then
  echo "Unprocessed trailing arguments: '$@'"
  exit 1
fi

trap 'echo -e "\nERROR: q7_set_multicast script terminated prematurely."' ERR
set -o errexit

function fix_node {
  ssh "root@$1" "mount -o remount,rw,sync /"
  scp "${MAKANI_HOME}/lib/scripts/install/data/99-igmp-max-memberships.conf" \
    "root@$1:/etc/sysctl.conf"
  ssh "root@$1" "/sbin/sysctl -p"
  ssh "root@$1" "mount -o remount,ro /"
}

if (([[ -z "${FLAGS_node}" ]] && [[ -z "${FLAGS_ip}" ]]) ||
    ([[ -n "${FLAGS_node}" ]] && [[ -n "${FLAGS_ip}" ]])); then
  echo "Exactly one of --node or --ip is required."
  exit 1
fi

if ([[ -n "${FLAGS_node}" ]]); then
  fix_node "${!FLAGS_node}"
fi
if ([[ -n "${FLAGS_ip}" ]]); then
  fix_node "${FLAGS_ip}"
fi
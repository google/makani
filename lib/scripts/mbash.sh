#!/bin/bash
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
# This is a library of common functions used by Makani's Bash scripts.

# Given a path to an exectuable relative to MAKANI_HOME (e.g. sim/sim)
# finds all processes run by the current user with that path and kills
# them.  Note that this cannot be used for Python files.
function mbash::kill_makani_proc_by_name() {
  if [[ "$#" -ne 1 ]]; then
    echo "kill_makani_proc_by_name takes one argument."
    exit 1
  fi

  # Using killall with a filename does not support the case where
  # the binary has been deleted.  Instead, we manually inspect /proc.
  filename="$(basename "$1")"

  # Without -f, pgrep matching is limited to 15 characters. Ick! So we use a
  # very imprecise match and compensate by carefully inspecting filenames and
  # paths below.
  local readonly PIDS="$(pgrep -f "${filename}")"

  local zombie_pids=''
  for pid in ${PIDS}; do
    # The awk command removes '(deleted)' from the end of readlink's output.
    local pid_filename="$(readlink "/proc/${pid}/exe" | awk '{print $1}')"

    if [[ -n "${pid_filename}" \
      && "$(basename ${pid_filename})" = "${filename}" ]]; then
      if [[ "${pid_filename}" = "${MAKANI_HOME}/$1" ]] \
        || [[ "${pid_filename}" \
              = "$(readlink -f ${MAKANI_HOME}/bazel-bin/$1)" ]] \
        || [[ "${pid_filename}" \
              = "$(readlink -f ${MAKANI_HOME}/bazel-out/k8-py2-fastbuild/bin/$1)" ]]; then
        kill -9 "${pid}"
      else
        zombie_pids="${pid} ${zombie_pids}"
      fi
    fi
  done

  if [[ -n "${zombie_pids}" ]]; then
    zombie_info="$(ps -fp ${zombie_pids})"
    if [[ -n "$(echo "${zombie_info}" | sed 1d)" ]]; then
      echo 'Warning: Zombie PIDs are still running:'
      echo "${zombie_info}"
      exit 1
    fi
  fi
}

# Kill all programs that we might have started.
function mbash::kill_all_makani_procs() {
  # Use "|| true" to suppress nonzero return codes.
  pkill -f 'joystick.py' || true
  pkill -f 'valgrind.bin' || true
  mbash::kill_makani_proc_by_name gs/aio_snapshot/aio_snapshot
  mbash::kill_makani_proc_by_name control/control
  mbash::kill_makani_proc_by_name sim/sim
  mbash::kill_makani_proc_by_name vis/vis
  mbash::kill_makani_proc_by_name gs/monitor/monitor
  mbash::kill_makani_proc_by_name lib/flight_gear/flight_gear_interface

  # Bazel fixups.
  mbash::kill_makani_proc_by_name control/hitl_controller
  mbash::kill_makani_proc_by_name control/sim_controller
  mbash::kill_makani_proc_by_name control/sim_ground_estimator
  mbash::kill_makani_proc_by_name control/hitl_ground_estimator
  mbash::kill_makani_proc_by_name lib/joystick/joystick
}

# Prints a message to stderr and exits with an error code 1.
function mbash::die() {
  echo "$@" >&2
  exit 1
}

# Prints the top comment of the file ignoring the first two lines,
# which are reserved for the shebang line.
function mbash::print_usage() {
  local head_num
  head_num="$(grep -n "^$" $0 | cut -f1 -d: | head -n 1)"
  head -n "${head_num}" $0 | tail -n +3 | sed 's/^# \?//g'
}

# Execute a command on a remote computer.  Note that the "bash -lc" is
# used to get the normal environment variables when you execute the
# command.  Also, the "-t" is necessary; I don't understand why.
#
# Usage:
#     mbash::exec_on_remote 192.168.4.xxx 'command1' 'command2'
function mbash::exec_on_remote() {
  for cmd in "${@:2}"; do
    ssh -tq "makani@$1" "bash -lc '${cmd}'" > /dev/null \
      || mbash::die "Error: The following remote command failed on $1:" \
      "    ${cmd}"
  done
}

# Checks if multicast packets are routed to the specified interface for AIO.
# Exits with an error if not.
function mbash::check_multicast_route() {
  route -n | grep "^239.0.0.0.*$1\$" > /dev/null
}

# A utility function to join arrays of strings with a delimiter.
#
# Args:
#   $1: string delimiter.
#   $*: strings to be joined.
#
# Example:
# join , a "b c" d     ==>    a,b c,d
# join / var local tmp ==>    var/local/tmp
# FOO=( a b c )
# join , "${FOO[@]}"   ==>    a,b,c
#
# Reference:
# http://stackoverflow.com/questions/1527049/bash-join-elements-of-an-array
function mbash::join() {
  local IFS="${1}"; shift; echo "$*";
}

readonly FLIGHTGEAR_IP_ADDR='192.168.4.130'
readonly TAU_IP_ADDR='10.0.1.251'

readonly TAU_TESTING_LOGS_BACKUP_DIR='/vol/network/TestingLogs/'

# Set the location of build outputs.
readonly BUILD_DIR="${MAKANI_HOME}/bazel-bin"
readonly GENFILES_DIR="${MAKANI_HOME}/bazel-genfiles"
readonly Q7_BUILD_DIR="${MAKANI_HOME}/q7-bin"
readonly TMS570_BUILD_DIR="${MAKANI_HOME}/tms570-bin"

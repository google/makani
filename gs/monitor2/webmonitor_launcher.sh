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


# Copy-pasted from the Bazel Bash runfiles library v2.
set -uo pipefail; f=bazel_tools/tools/bash/runfiles/runfiles.bash
  source "${RUNFILES_DIR:-/dev/null}/$f" 2>/dev/null || \
  source "$(grep -sm1 "^$f " "${RUNFILES_MANIFEST_FILE:-/dev/null}" | cut -f2- -d' ')" 2>/dev/null || \
  source "$0.runfiles/$f" 2>/dev/null || \
  source "$(grep -sm1 "^$f " "$0.runfiles_manifest" | cut -f2- -d' ')" 2>/dev/null || \
  source "$(grep -sm1 "^$f " "$0.exe.runfiles_manifest" | cut -f2- -d' ')" 2>/dev/null || \
  { echo>&2 "ERROR: cannot find $f"; exit 1; }; f=; set -e

# Unset -u since this script accesses unset positional args.
set +u

readonly CHROME_PATH='google-chrome'
readonly CHROME_FLAGS='--disk-cache-dir=/dev/null'
readonly URL='http://localhost:8000'

function list_layouts() {
  "$(rlocation makani/gs/monitor2/commands/list)"
}

# Start the web monitor.
# Args:
#   $1: true if running the server in background.
#   $2: The name of the monitor to open after starting the server.
#   $3: true if the AIO messages should be populated from the simulator.
#   $4: true if a web browser should be started.
function start_server() {
  readonly MONITOR="$(rlocation makani/gs/monitor2/webmonitor)"

  # Set monitor mode to bring back the server to foreground.
  set -m

  local server_pid
  # Check if server is already running and start it if not.
  if [[ -z "$(ps aux | grep '[r]unserver')" ]]; then
    # Synchronize the database.
    "${MONITOR}" syncdb --noinput
    # Set flags.
    if [[ "$3" == true ]]; then
      export MAKANI_WEBMON_FROM_SIM=True
    fi
    # Start the web server.
    if [[ "$1" == true ]]; then
      "${MONITOR}" runserver &> /dev/null &
    else
      # Execute the server in background to continue and start client.
      "${MONITOR}" runserver &
    fi
    server_pid="$!"
    # Wait for the server to start.
    sleep 3
  elif [[ "$1" == false ]]; then
    echo "A server thread is already running."
    return
  fi

  local url="${URL}"
  if [[ -n "$2" ]]; then
    url="${URL}/dashboard/aio/layout/$2"
  fi

  if [[ "$4" == true ]]; then
    # Open a web browser tab.
    "${CHROME_PATH}" "${CHROME_FLAGS}" "${url}" &> /dev/null &
  fi

  if [[ "$1" == false ]]; then
    # Bring back the server to foreground.
    fg %1
  fi
}

function stop_server() {
  # Look for existing monitor instances.
  # Prevent catching the grep process itself.
  while [[ -n "$(ps aux | grep '[r]unserver')" ]]; do
    local result
    result="$(ps aux | grep '[r]unserver')"
    if [[ -n "${result}" ]]; then
      local items
      # Split the output text into an array, using space as the delimiter.
      items=(${result// / })
      # `items[1]` is the pid to kill.
      kill -9 "${items[1]}"
    fi
  done
}

function open_monitor() {
  local url="${URL}/dashboard/aio/layout/$1"
  if [[ -z "$(ps aux | grep '[r]unserver')" ]]; then
    start_server true "${url}"
  else
    "${CHROME_PATH}" "${CHROME_FLAGS}" "${URL}/dashboard/aio/layout/$1" &> /dev/null &
  fi
}

function print_usage() {
  echo 'To start the webmonitor: "./webmonitor start[ <layout_name>]"'
  echo 'To start the webmonitor which populates sim messages: "./webmonitor sim[ <layout_name>]"'
  echo 'To stop the webmonitor: "./webmonitor stop"'
  echo 'To restart the webmonitor: "./webmonitor restart"'
  echo 'To debug the webmonitor: "./webmonitor debug[ <layout_name>]"'
  echo 'To debug the webmonitor which populates sim messages: "./webmonitor debug_sim[ <layout_name>]"'
  echo 'To list all layouts: "./webmonitor list"'
}

if [[ "$1" = 'restart' && "$#" -eq 1 ]]; then
  stop_server
  start_server true "" false false
elif [[ "$1" = 'start' && "$#" -le 2 ]]; then
  start_server true "$2" false true
elif [[ "$1" = 'sim' && "$#" -le 2 ]]; then
  start_server true "$2" true true
elif [[ "$1" = 'stop' && "$#" == 1 ]]; then
  stop_server
elif [[ "$1" = 'debug' && "$#" -le 2 ]]; then
  start_server false "$2" false true
elif [[ "$1" = 'debug_sim' && "$#" -le 2 ]]; then
  start_server false "$2" true true
elif [[ "$1" = 'list' ]]; then
  list_layouts
else
  print_usage
fi

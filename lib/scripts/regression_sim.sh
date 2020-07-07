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
# Clone the repository located at $MAKANI_HOME, check out two commits,
# run a simulation for each, and compare the generated log files.

function run_simulation() {
  git checkout "$1"

  ./lib/scripts/developer/run_sim -l \
    --overrides "${FLAGS_overrides}" --log_file "$(basename $2)" \
    -- --time "${FLAGS_time}"
}

source "${MAKANI_HOME}/lib/scripts/mbash.sh"

# Source shflags.
source /opt/shflags-1.0.3/src/shflags

# Define flags.
DEFINE_string 'overrides' '' 'JSON string of override parameters.' 'o'
DEFINE_float 'time' '500' 'Duration of simulation (in seconds).'
DEFINE_boolean 'dirty' false 'Whether to allow uncommitted changes.' 'd'

FLAGS "$@" || exit "$?"
eval set -- "${FLAGS_ARGV}"

if [[ "$#" -ne 2 ]]; then
  echo "Usage: lib/scripts/regression_simulation.sh [options] COMMIT_A COMMIT_B"
  exit -1
fi

set -o errexit  # Exit if any command returns a failure.
set -o xtrace  # Print each command when it is run.

cd "${MAKANI_HOME}"

if [[ "${FLAGS_dirty}" -eq "${FLAGS_FALSE}" ]]; then
  # Any lines from "git status --porcelain" that do not start with "??"
  # indicate uncommitted changes.
  if git status --porcelain | grep -q -v '^??'; then
    echo 'Uncommitted changes detected. Aborting.'
    exit 1
  fi
fi

readonly INITIAL_BRANCH="$(git rev-parse --abbrev-ref HEAD)"
trap "git checkout ${INITIAL_BRANCH}" EXIT

readonly COMMIT_1="$(git rev-parse --short=8 $1)"
readonly COMMIT_2="$(git rev-parse --short=8 $2)"

readonly LOG_FILE_1="logs/regression-lhs-${COMMIT_1}.h5"
readonly LOG_FILE_2="logs/regression-rhs-${COMMIT_2}.h5"
if [[ -e "${LOG_FILE_1}" ]]; then
  rm "${LOG_FILE_1}"
fi
if [[ -e "${LOG_FILE_2}" ]]; then
  rm "${LOG_FILE_2}"
fi

run_simulation "${COMMIT_1}" "${LOG_FILE_1}"
run_simulation "${COMMIT_2}" "${LOG_FILE_2}"
bazel build //analysis/util:h5comp
bazel-out/k8-py2-fastbuild/bin/analysis/util/h5comp --truncate \
  --config analysis/util/h5comp_default_config.json \
  "${LOG_FILE_1}" "${LOG_FILE_2}"

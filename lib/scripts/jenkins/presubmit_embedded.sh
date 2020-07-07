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


# Jenkins integration.
export MAKANI_HOME="${WORKSPACE}"
source "${MAKANI_HOME}/lib/scripts/jenkins/test_command.sh"

# Define build directories: BUILD_DIR, Q7_BUILD_DIR, TMS570_BUILD_DIR.
source "$(dirname $0)/../mbash.sh"

# Define regression tests to execute.
readonly CONFIG="$(dirname $0)/embedded/regression.yaml"

# Define arguments for process_commit.
readonly ARGS="$(echo \
  "--workpath=\"${MAKANI_HOME}\"" \
  "--config=\"${CONFIG}\"" \
  "--q7_bin=\"${Q7_BUILD_DIR}\"" \
  "--tms570_bin=\"${TMS570_BUILD_DIR}\"" \
  "$@")"

# Execute and post results to Jenkins.
test_command_embedded "bazel_build lib/python/embedded:process_commit \
  && bazel-out/k8-py2-fastbuild/bin/lib/python/embedded/process_commit ${ARGS}"

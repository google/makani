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

# Generates build_info.h.
#
# TODO: Generate the build info header as a proper build step.
function generate_build_info_header() {
  BUILD_INFO_SCRIPT="${MAKANI_HOME}/lib/scripts/generate_build_info_header"
  if [[ -e "${BUILD_INFO_SCRIPT}" ]]; then
    if ! "${BUILD_INFO_SCRIPT}"; then
      exit 1
    fi
  fi
}

# Exits the calling process on failure.
function run_preflight_actions() {
  # We must not output to stdout on success, as not to pollute the output of
  # "bazel query".
  if ! (generate_build_info_header) 1>&2 ; then
    exit 1
  fi
}

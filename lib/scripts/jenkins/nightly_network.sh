#!/bin/bash -xu
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
# Run nightly batch simulation tests.
#
# The "-xe" flags above make Bash behave similarly to Jenkin's default
# behavior; it runs Bash in debug mode and errors out on the first
# error.  The "-u" option makes it an error to try to expand an unset
# variable (for example, if WORKSPACE isn't set).

export MAKANI_HOME="${WORKSPACE}"
declare -i result=0
log_dir="${MAKANI_HOME}/logs"
git clean -dxf

( set -e
  output_dir="${log_dir}/network_dashboard"
  mkdir -p "${output_dir}"

  # TODO: We're ignoring controllers b & c right now.  This should
  # be re-enabled once we're using redundant control.
  bazel_build //avionics/network:network_bandwidth \
    //avionics/network:network_dashboard
  ./bazel-out/k8-py2-fastbuild/bin/avionics/network/network_bandwidth \
    --export_json \
    --ignore_node=controller_b \
    --ignore_node=controller_c \
    --ignore_location=test_fixture \
    --network_file ./avionics/network/network.yaml \
    | ./bazel-out/k8-py2-fastbuild/bin/avionics/network/network_dashboard \
      --output_dir="${output_dir}"

  # Export HTML to Cloud Storage for easy access.
  gsutil -m rm -R gs://makani/dashboard/network_dashboard || true
  gsutil -m cp -R "${output_dir}" gs://makani/dashboard/network_dashboard

  # TODO: Upload stats to cloud storage/db.
)
(( result |= $? ))

exit ${result}

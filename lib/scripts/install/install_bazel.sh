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
# Download and install Bazel.
#
# This script currently assumes that install_packages.sh has been run at least
# once to create /opt/makani with appropriate permissions.

# TODO: Remove bazel bootstrap, install wrapper only.
source "${MAKANI_HOME}/lib/scripts/mbash.sh"
source "${MAKANI_HOME}/lib/scripts/system.sh"

readonly WRAPPER_SCRIPT='/usr/local/bin/bazel'

sudo apt-get -y install openjdk-8-jdk
readonly JAVA_DIR='/usr/lib/jvm/java-8-openjdk-amd64'

sudo cp "${MAKANI_HOME}/lib/scripts/install/data/bazel_wrapper.sh" \
  "${WRAPPER_SCRIPT}"
sudo sed -i "s|JAVA_HOME_PLACEHOLDER|${JAVA_DIR}|" "${WRAPPER_SCRIPT}"
sudo chmod a+rx "${WRAPPER_SCRIPT}"

gsutil cp gs://gcp-public-data-makani-deps/deps/machine_setup/bazel-complete.bash .
sudo mv bazel-complete.bash /etc/bash_completion.d/
sudo chmod 0555 /etc/bash_completion.d/bazel-complete.bash

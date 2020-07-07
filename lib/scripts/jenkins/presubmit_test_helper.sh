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

source "${MAKANI_HOME}/lib/scripts/jenkins/test_command.sh"

test_command "bazel_run //lib/bazel:check_test_rules"

function presubmit_test() {
  test_command "${MAKANI_HOME}/lib/scripts/developer/btest_all ${BAZEL_OPTS}" "$@"

  # Some "manual" tests can not by run by bazel test.  If we have more of these
  # we could use a "manual_test" tag and do the enumeration automatically.
  # Hopefully this "manual" test is short lived.
  test_command \
    $(echo "bazel_run $@ --copt -Werror" \
      "//lib/python:shell_interfaces_test_manual")
}

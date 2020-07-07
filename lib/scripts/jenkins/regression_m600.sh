#!/bin/bash -xe
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
# Checks if a simulation of M600 with the current commit performs
# exactly the same as the previous commit.  Exits successfully if the
# two commits are the same, otherwise posts a +1 to Gerrit and exits
# with a failure.

export MAKANI_HOME="${WORKSPACE}"
source "${MAKANI_HOME}/lib/scripts/jenkins/test_command.sh"

test_command_regression 'COMMON_FLAGS="-Werror -O2" \
  ./lib/scripts/developer/run_sim -l -w m600 -- --time 1500'

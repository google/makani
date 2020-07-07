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
# Checks if a commit passes lint.  This is separated from other
# source-level checks as gpylint is only available on corp.

export MAKANI_HOME="${WORKSPACE}"
source "${MAKANI_HOME}/lib/scripts/jenkins/test_command.sh"

readonly ARGS=$(echo '--norun_cpplint' \
  '--norun_gjslint' \
  '--norun_gpylint' \
  '--norun_pack2_lint' \
  '--norun_splint' \
  '--norun_buildifier' \
  '--run_style_checker' \
  '--noverbose')

test_command "bazel run //lib/scripts/style:run_lint -- ${ARGS}"

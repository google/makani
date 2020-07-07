#!/bin/bash -eu
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
# Executes a command with a MAKANI_HOME that allows importing of Bazel-built
# Python dependencies.  See the BUILD file for usage.

readonly RUNFILES_DIR="$(readlink -f "$0.runfiles")"
export MAKANI_HOME="${RUNFILES_DIR}/makani"
exec "$@"

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


source /opt/shflags-1.0.3/src/shflags
DEFINE_boolean 'verbose' true 'Print message for each file.'
FLAGS "$@" || exit $?

readonly P2FMT="${MAKANI_HOME}/bazel-out/k8-py2-fastbuild/bin/lib/python/pack2/tools/p2fmt"

if [[ ! -x "${P2FMT}" ]]; then
  echo "${P2FMT} is not built."
  exit 255
fi

num_errors=0
for f in "$@"; do
  if [[ -e "${f}" ]]; then
    if [[ "${FLAGS_verbose}" -eq "${FLAGS_TRUE}" ]]; then
      echo "Checking style in ${f}"
    fi

    OUTPUT="$(${P2FMT} < ${f} | diff ${f} - | colordiff)"
    if [[ ! -z "${OUTPUT}" ]]; then
      let num_errors++
      echo "Style error in file ${f}"
      echo "${OUTPUT%x}"
    fi
  fi
done

exit "${num_errors}"

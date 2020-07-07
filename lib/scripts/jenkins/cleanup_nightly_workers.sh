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


# For instructions on installing gcloud, visit
#     https://cloud.google.com/sdk/downloads.
readonly INSTANCES="$(gcloud compute instances list)"

function get_instances() {
  SIM_NAME="$1"
  echo "${INSTANCES}" \
    | grep -E "batch-sim-nightly-${SIM_NAME}-worker-[0-9]+" | cut -f 1 -d ' '
}

SIM_NAMES=(
  power-curve
  hover-disturbances
  onshore-cw-sweeps
  onshore-cw-sweeps-monte-carlo
  offshore-cw-sweeps-monte-carlo
)

for sim_name in ${SIM_NAMES[@]}; do
  instances="$(get_instances ${sim_name})"
  # The test below checks whether the LHS string contains non-whitespace characters.
  if [[ "${instances}" = *[![:space:]]* ]]; then
    cmd="gcloud compute instances delete --zone us-central1-f --quiet ${instances}"
    ${cmd}
  fi
done

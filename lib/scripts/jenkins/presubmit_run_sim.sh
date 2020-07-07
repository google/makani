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
# Checks that simulation gives no errors under a variety of input
# options.

export MAKANI_HOME="${WORKSPACE}"
source "${MAKANI_HOME}/lib/scripts/jenkins/test_command.sh"

readonly RUN_SIM=./lib/scripts/developer/run_sim

test_command "${RUN_SIM} -o '{\"sim\": {\"sim_opt\": 0}}' -- --time 1 \
  && ${RUN_SIM} -o '{\"sim\": {\"sim_opt\": 32767}}' -- --time 1 \
  && ${RUN_SIM} -o '{\"system\": {\"flight_plan\": 0}}' -- --time 1 \
  && ${RUN_SIM} -o '{\"system\": {\"flight_plan\": 1}}' -- --time 1 \
  && ${RUN_SIM} -o '{\"system\": {\"flight_plan\": 2}}' -- --time 1 \
  && ${RUN_SIM} -o '{\"system\": {\"flight_plan\": 3}}' -- --time 1 \
  && ${RUN_SIM} -o '{\"system\": {\"flight_plan\": 4}}' -- --time 1"

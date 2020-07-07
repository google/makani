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

#
# Contains the network setup for M600 HITL tests.  Used by scripts in
# lib/scripts/operator/.
#
# Must be kept in sync with avionics/common/network_config.c.

readonly M600_MULTICAST_SUBNET='239.0.0.0/8'
readonly M600_UNICAST_SUBNET='192.168.1.0/24'

# List of IP addresses referenced by operator scripts.
readonly M600_CONTROLLER_A='192.168.1.1'
readonly M600_CONTROLLER_B='192.168.1.2'
readonly M600_CONTROLLER_C='192.168.1.3'
readonly M600_GS_COMPASS='192.168.1.37'
readonly M600_RECORDER_Q7_PLATFORM='192.168.1.42'
readonly M600_RECORDER_Q7_WING='192.168.1.43'
readonly M600_JOYSTICK='192.168.1.100'
readonly M600_GROUND_POWER_Q7='192.168.1.69'
readonly M600_GROUND_ESTIMATOR_Q7='192.168.1.74'

# List of command center computers.
readonly M600_HOST_SIM='192.168.1.0'
readonly M600_LOGGER_A='192.168.1.91'
readonly M600_LOGGER_B='192.168.1.92'
readonly M600_FLIGHT_SPARE='192.168.1.93'
readonly M600_WEBMONITOR='192.168.1.94'

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
# Runs FlightGear with settings to display W7 and the tether.

/usr/games/fgfs \
  --ai-scenario=with_tether_scenario \
  --aircraft=w7 \
  --generic=socket,in,50,127.0.0.1,40022,udp,wing_and_tether_prot \
  --fdm=null \
  --prop:/sim/current-view/view-number=3 \
  --control=none \
  --enable-fullscreen \
  --timeofday=noon \
  --disable-random-objects \
  --disable-random-vegetation

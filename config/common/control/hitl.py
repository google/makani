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

"""Options related to HITL."""
from makani.config import mconfig


@mconfig.Config
def MakeParams():
  return {
      # Maximum time [s] to wait for the first update from the simulator.
      'sim_init_timeout': 15.0,

      # Maximum time [s] to run the controller without updates from
      # the simulator.
      'sim_update_timeout': 0.1
  }

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

"""Fault schedule."""

from makani.config import mconfig
from makani.sim import sim_types as m


@mconfig.Config
def MakeParams():
  # Example events:
  #
  #   motor_out_event = {
  #       't_start': 100.0,
  #       't_end': 110.0,
  #       'component': 'PowerSys/motor_connections[0]',
  #       'type': m.kSimFaultActuatorZero,
  #       'num_parameters': 0,
  #       'parameters': [0.0] * m.MAX_FAULT_EVENT_PARAMETERS,
  #   }

  none_event = {
      't_start': 0.0,
      't_end': 0.0,
      'component': '',
      'type': m.kSimFaultNoFault,
      'num_parameters': 0,
      'parameters': [0.0] * m.MAX_FAULT_EVENT_PARAMETERS,
  }

  return {
      'num_fault_events': 0,
      'fault_events': [none_event] * m.MAX_FAULT_EVENTS
  }

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

"""Monitor parameters."""

from makani.config import mconfig
from makani.gs.monitor import monitor_types as m
import numpy as np


@mconfig.Config(deps={
    'avionics_monitor': 'common.monitor.avionics_monitor',
    'comms_monitor': 'common.monitor.comms_monitor',
    'control_monitor': 'common.monitor.control_monitor',
    'environment': 'common.monitor.environment',
    'estimator_monitor': 'm600.monitor.estimator_monitor',
    'filter_params': 'common.monitor.filter_params',
    'power_monitor': 'common.monitor.power_monitor',
    'sensor_checks': 'common.monitor.sensor_checks',
    'temperature': 'common.monitor.temperature',
    'landing_zone': 'm600.monitor.landing_zone',
    'tether': 'm600.monitor.tether',
    'thermal': 'common.monitor.thermal',
})
def MakeParams(params):
  monitor_params = {
      'comms': params['comms_monitor'],
      'avionics': params['avionics_monitor'],
      'power': params['power_monitor'],

      # Monitor parameters for the rotors.
      'rotors': {
          # Limits on (absolute value of) speed [rad/s].
          'speed': {
              'very_low': np.finfo('d').min,
              'low': np.finfo('d').min,
              'high': 240.0,
              'very_high': 250.0,
          }
      },

      'temperature': params['temperature'],
      'environment': params['environment'],
      'est': params['estimator_monitor'],
      'control': params['control_monitor'],
      'autochecks': params['sensor_checks'],
      'filter': params['filter_params'],
      'landing_zone': params['landing_zone'],
      'tether': params['tether'],
      'thermal': params['thermal'],
      'v1_monitor_use_sim_sensors': False,
  }
  assert mconfig.MatchesCStruct(monitor_params, m.MonitorParams)
  return monitor_params

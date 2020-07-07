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

"""Controller parameters."""

from makani.config import mconfig
from makani.control import control_types as m


@mconfig.Config(deps={
    'control_opt': 'common.control.control_options',
    'control_output': 'm600.control.control_output',
    'crosswind': mconfig.WING_MODEL + '.control.crosswind',
    'estimator': 'm600.control.estimator',
    'fault_detection': 'm600.control.fault_detection',
    'flight_plan': 'common.flight_plan',
    'hitl': 'common.control.hitl',
    'hover': mconfig.WING_MODEL + '.control.hover',
    'joystick_control': 'common.control.joystick_control',
    'manual': 'm600.control.manual',
    'planner': 'm600.control.planner',
    'rotor_control': 'm600.control.rotor_control',
    'sensor_limits': 'm600.control.sensor_limits',
    'ground_sensor_limits': 'base_station.control.ground_sensor_limits',
    'simple_aero_model': mconfig.WING_MODEL + '.control.simple_aero_model',
    'trans_in': 'm600.control.trans_in',
})
def MakeParams(params):
  control_params = {
      'flight_plan': params['flight_plan'],
      'control_opt': params['control_opt'],
      'simple_aero_model': params['simple_aero_model'],
      'rotor_control': params['rotor_control'],
      'joystick_control': params['joystick_control'],
      'sensor_limits': params['sensor_limits'],
      'ground_sensor_limits': params['ground_sensor_limits'],
      'hover': params['hover'],
      'trans_in': params['trans_in'],
      'crosswind': params['crosswind'],
      'manual': params['manual'],
      'planner': params['planner'],
      'control_output': params['control_output'],
      'estimator': params['estimator'],
      'fault_detection': params['fault_detection'],
      'hitl': params['hitl'],
  }
  assert mconfig.MatchesCStruct(control_params, m.ControlParams)
  return control_params
